// AquaChirp — STM32G474RE firmware for the SYSTEM PAYLOAD architecture:
// Sensor Input -> STM32G474RE -> IR2104 Driver -> MOSFET Half-Bridge
// -> (Op-Amp buffer, hardware only) -> Low-Pass Filter -> PHW-500-03AB
//
// This is a DIFFERENT approach from the internal-DAC firmware given
// previously -- that one is for a DAC+linear-amp path with no half-bridge.
// This one matches your actual PPT diagram: a hardware timer directly
// toggles a GPIO (feeding IR2104's logic input), and DMA feeds a table of
// PERIOD values into the timer so the toggle frequency itself sweeps,
// sample by sample, with zero CPU involvement per sample. This reaches
// real 100-500kHz cleanly, since it isn't limited by DAC settling time.
//
// VERIFY BEFORE TRUSTING BLIND:
//  - TIM2_CH1 -> PA15 alternate-function mapping, against your exact
//    G474 package datasheet (PA15 is also the default JTDI/JTAG pin;
//    confirm it's free for GPIO/AF reuse if you're using SWD only, which
//    is standard, but worth a first-principles check on your exact setup)
//  - Toggle-mode output frequency formula assumes timerClock/(2*(ARR+1));
//    confirm against your actual timer clock tree
//  - DMA transfer-complete callback firing behavior for TIM_DMA_ID_UPDATE
//    bursts -- verify it fires once per full buffer, not per sample

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int potTemp      = PA0;
const int potSalinity  = PA1;
const int potDepth     = PA5;
const int potTurbidity = PA6;
const int potBattery   = PA7;
const int btnWaveform  = PB0;
const int btnWindow    = PB1;
// PA15 = TIM2_CH1 output -> IR2104 driver input, HAL-managed below.
// PA2/PA3 = ST-LINK VCP (Serial), reserved.

const float TEMP_MIN  = 3.0,   TEMP_MAX  = 30.0;
const float SAL_MIN   = 0.0,   SAL_MAX   = 40.0;
const float DEPTH_MIN = 0.0,   DEPTH_MAX = 500.0;
const float TURB_MIN  = 0.0,   TURB_MAX  = 100.0;
const float BATT_MIN  = 0.0,   BATT_MAX  = 100.0;

const float FREQ_MUDDY_HZ = 100000.0f;
const float FREQ_CLEAR_HZ = 500000.0f;
const float SWEEP_SPAN_FRACTION = 0.2f;

const float TURB_AMP_MIN = 0.4f;  // NOTE: this switching topology can't vary
const float TURB_AMP_MAX = 1.0f;  // amplitude in firmware the way a DAC can --
                                   // see the amplitude note after this file.

const float BATT_LOW_THRESHOLD  = 30.0f;
const float DUR_BATT_FLOOR_FRACTION = 0.5f;
const unsigned long IDLE_PAUSE_MAX_MS = 1000;

const int N_SAMPLES = 200;
uint32_t periodTable[N_SAMPLES]; // ARR values, one per output cycle

const int8_t BARKER13[13] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
const int BARKER_LEN = 13;

enum WaveformType { WF_LFM_CHIRP = 0, WF_GEOMETRIC_SWEEP = 1, WF_PHASE_CODED = 2, WF_COUNT = 3 };
WaveformType currentWaveform = WF_LFM_CHIRP;

bool lastWaveBtnState = false;
unsigned long lastWaveDebounce = 0;
const unsigned long DEBOUNCE_MS = 30;

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
}
float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

bool debouncedPress(int pin, bool &lastState, unsigned long &lastDebounce) {
  bool reading = digitalRead(pin);
  bool pressed = false;
  if (reading != lastState && (millis() - lastDebounce) > DEBOUNCE_MS) {
    lastDebounce = millis();
    if (reading == HIGH) pressed = true;
    lastState = reading;
  }
  return pressed;
}

const char* waveformName(WaveformType w) {
  switch (w) { case WF_LFM_CHIRP: return "LFM Chirp"; case WF_GEOMETRIC_SWEEP: return "GeoSweep"; case WF_PHASE_CODED: return "PhaseCode"; }
  return "?";
}

// --- Instantaneous frequency at sample index n of N, for each mode ---
float instFreqLFM(int n, int N, float f0, float f1) {
  float ratio = (float)n / (float)(N - 1);
  return f0 + (f1 - f0) * ratio; // linear sweep
}
float instFreqGeometric(int n, int N, float f0, float f1) {
  float ratio = (float)n / (float)(N - 1);
  return f0 * pow(f1 / f0, ratio); // exponential sweep
}
float instFreqPhaseCoded(int n, int N, float fc) {
  return fc; // carrier stays fixed; phase-coding a toggled square wave
             // is a firmware extension not implemented in this version --
             // see note below the file.
}

// --- HAL: TIM2 toggle-mode output + DMA-fed frequency sweep ---
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_up;
volatile bool pulseDone = true;
uint32_t timerClockHz = 0;

extern "C" void DMA1_Channel2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_tim2_up);
}

void onSweepComplete(DMA_HandleTypeDef *hdma) {
  HAL_TIM_Base_Stop(&htim2);
  pulseDone = true;
}

void TIM2_GPIO_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF1_TIM2; // VERIFY against your exact package datasheet
  HAL_GPIO_Init(GPIOA, &gpio);
}

void TIM2_Init() {
  __HAL_RCC_TIM2_CLK_ENABLE();
  timerClockHz = HAL_RCC_GetPCLK1Freq(); // adjust if your APB1 prescaler > 1 (then real clock is 2x this)

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF; // overwritten per-sample via DMA below
  HAL_TIM_OC_Init(&htim2);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

void DMA_Init() {
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_tim2_up.Instance = DMA1_Channel2;
  hdma_tim2_up.Init.Request = DMA_REQUEST_TIM2_UP; // VERIFY exact enum name
  hdma_tim2_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_up.Init.Mode = DMA_NORMAL;
  hdma_tim2_up.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_tim2_up);
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_UPDATE], hdma_tim2_up);
  hdma_tim2_up.XferCpltCallback = onSweepComplete;
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

void startSweep(int nSamples) {
  pulseDone = false;
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
  HAL_DMA_Start_IT(&hdma_tim2_up, (uint32_t)periodTable, (uint32_t)&(TIM2->ARR), nSamples);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(btnWaveform, INPUT);

  TIM2_GPIO_Init();
  TIM2_Init();
  DMA_Init();

  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found at 0x3C");
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  if (debouncedPress(btnWaveform, lastWaveBtnState, lastWaveDebounce)) {
    currentWaveform = (WaveformType)((currentWaveform + 1) % WF_COUNT);
  }

  if (!pulseDone) return;

  int rawTemp = analogRead(potTemp);
  int rawDep  = analogRead(potDepth);
  int rawTurb = analogRead(potTurbidity);
  int rawBatt = analogRead(potBattery);

  float temperature = mapFloat(rawTemp, 0, 4095, TEMP_MIN,  TEMP_MAX);
  float depth       = mapFloat(rawDep,  0, 4095, DEPTH_MIN, DEPTH_MAX);
  float turbidity   = mapFloat(rawTurb, 0, 4095, TURB_MIN,  TURB_MAX);
  float battery     = mapFloat(rawBatt, 0, 4095, BATT_MIN,  BATT_MAX);

  float turbFraction  = clamp01((turbidity - TURB_MIN) / (TURB_MAX - TURB_MIN));
  float depthFraction = clamp01((depth - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN));
  float tempColdness  = clamp01(1.0f - (temperature - TEMP_MIN) / (TEMP_MAX - TEMP_MIN));
  float envDifficulty = clamp01(0.6f * turbFraction + 0.25f * depthFraction + 0.15f * tempColdness);

  float baseFreqHz = FREQ_CLEAR_HZ + (FREQ_MUDDY_HZ - FREQ_CLEAR_HZ) * envDifficulty;
  float f0 = baseFreqHz * (1.0f - SWEEP_SPAN_FRACTION);
  float f1 = baseFreqHz * (1.0f + SWEEP_SPAN_FRACTION);

  bool lowBattery = battery < BATT_LOW_THRESHOLD;
  float battDurScale = 1.0f;
  unsigned long idlePauseMs = 0;
  if (lowBattery) {
    float bf = battery / BATT_LOW_THRESHOLD;
    battDurScale = DUR_BATT_FLOOR_FRACTION + (1.0f - DUR_BATT_FLOOR_FRACTION) * bf;
    idlePauseMs = (unsigned long)((1.0f - bf) * IDLE_PAUSE_MAX_MS);
  }
  int nSamples = (int)(N_SAMPLES * battDurScale);
  if (nSamples < 8) nSamples = 8;

  for (int n = 0; n < nSamples; n++) {
    float f;
    switch (currentWaveform) {
      case WF_LFM_CHIRP:       f = instFreqLFM(n, nSamples, f0, f1); break;
      case WF_GEOMETRIC_SWEEP: f = instFreqGeometric(n, nSamples, f0, f1); break;
      case WF_PHASE_CODED:
      default:                 f = instFreqPhaseCoded(n, nSamples, baseFreqHz); break;
    }
    uint32_t arr = (uint32_t)(timerClockHz / (2.0f * f)) - 1;
    periodTable[n] = arr;
  }

  Serial.print("Temp:"); Serial.print(temperature, 1);
  Serial.print(" Depth:"); Serial.print(depth, 1);
  Serial.print(" Turb:"); Serial.print(turbidity, 1);
  Serial.print(" Batt:"); Serial.print(battery, 0);
  Serial.print(" Freq:"); Serial.print(baseFreqHz / 1000.0f, 0); Serial.print("kHz");
  Serial.print(" N:"); Serial.print(nSamples);
  Serial.print(" WF:"); Serial.println(waveformName(currentWaveform));

  display.clearDisplay();
  display.setCursor(0, 0);  display.print("Temp:");  display.print(temperature, 1); display.print("C");
  display.setCursor(0, 16); display.print("Depth:"); display.print(depth, 1); display.print("m");
  display.setCursor(0, 32); display.print("Freq:");  display.print(baseFreqHz / 1000.0f, 0); display.print("kHz");
  display.setCursor(0, 48); display.print(waveformName(currentWaveform));
  display.display();

  if (idlePauseMs > 0) delay(idlePauseMs);

  startSweep(nSamples);
}
