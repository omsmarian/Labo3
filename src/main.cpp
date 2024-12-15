#include <Arduino.h>
#include <driver/timer.h>
#include <thread>
#include <chrono>

#define TIMER_TICK_PIN 26 // Define GPIO 26 for square wave output

#define BUTTON_RECORD 23    // Record button
#define BUTTON_EFFECT_1 16  // Flanger
#define BUTTON_EFFECT_2 17  // Echo
#define BUTTON_EFFECT_3 18  // Reverb
#define BUTTON_EFFECT_4 19  // Tremolo
#define BUTTON_EFFECT_5 22  // Distortion

#define LED_EFFECT_1 33
#define LED_EFFECT_2 32
#define LED_EFFECT_3 15
#define LED_EFFECT_4 4
#define LED_EFFECT_5 2

#define AUDIO_IN_PIN 34     // ADC
#define AUDIO_OUT_PIN 25    // DAC

const int sampleRate = 30000; // Sampling rate in Hz
const int bufferSize = 1024 * 32;
int16_t audioBuffer[bufferSize];
enum SystemState { IDLE, RECORDING, PLAYBACK };
volatile SystemState systemState = IDLE;

volatile int sampleIndex = 0;
volatile bool timerFlag = false;
unsigned long buttonPressStartTime = 0;
bool isLongPress = false;

bool effect1Enabled = false;
bool effect2Enabled = false;
bool effect3Enabled = false;
bool effect4Enabled = false;
bool effect5Enabled = false;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastDebounceTime[6] = {0};
const unsigned long debounceDelay = 200; // Debounce delay in milliseconds
const unsigned long longPressThreshold = 1000; // Long press threshold in milliseconds

void IRAM_ATTR onTimer() {
  static bool toggle = false; // Static variable to maintain state

  // Toggle GPIO 26 for square wave output
  digitalWrite(TIMER_TICK_PIN, toggle ? HIGH : LOW);
  toggle = !toggle;

  portENTER_CRITICAL_ISR(&timerMux);
  timerFlag = true; // Set flag for main loop processing
  portEXIT_CRITICAL_ISR(&timerMux);
}

void startTimer() {
  timer = timerBegin(0, 80, true); // 80 MHz clock, prescaler 80 -> 1 MHz timer frequency
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000 / sampleRate, true); // Set timer frequency to sampleRate
  timerAlarmEnable(timer);
}

void processAudio() {
  if (systemState == RECORDING) {
    if (sampleIndex < bufferSize) {
      audioBuffer[sampleIndex++] = analogRead(AUDIO_IN_PIN);
    } else {
      systemState = IDLE;
      sampleIndex = 0;
      Serial.println("Recording complete.");
    }
  } else if (systemState == PLAYBACK) {
    if (sampleIndex < bufferSize) {
      int16_t sample = audioBuffer[sampleIndex++];

      if (effect1Enabled) {
        int delayIndex = (sampleIndex - 10 + bufferSize) % bufferSize;
        sample += (audioBuffer[delayIndex] / 2);
      }
      if (effect2Enabled) {
        int delayIndex = (sampleIndex - 100 + bufferSize) % bufferSize;
        sample += (audioBuffer[delayIndex] / 2);
      }
      if (effect3Enabled) {
        int delayIndex = (sampleIndex - 50 + bufferSize) % bufferSize;
        sample = (sample + audioBuffer[delayIndex]) / 2;
        audioBuffer[delayIndex] = sample;
      }
      if (effect4Enabled) {
        float tremolo = 0.5 + 0.5 * sin(2 * PI * 5 * sampleIndex / sampleRate);
        sample *= tremolo;
      }
      if (effect5Enabled) {
        if (sample > 1000) sample = 1000;
        else if (sample < -1000) sample = -1000;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(20));

      dacWrite(AUDIO_OUT_PIN, (sample >> 8) + 128);
    } else {
      systemState = IDLE;
      sampleIndex = 0;
      Serial.println("Playback complete.");
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TIMER_TICK_PIN, OUTPUT); // Set GPIO 26 as output for timer ticks

  pinMode(AUDIO_IN_PIN, INPUT);
  pinMode(AUDIO_OUT_PIN, OUTPUT);

  pinMode(LED_EFFECT_1, OUTPUT);
  pinMode(LED_EFFECT_2, OUTPUT);
  pinMode(LED_EFFECT_3, OUTPUT);
  pinMode(LED_EFFECT_4, OUTPUT);
  pinMode(LED_EFFECT_5, OUTPUT);

  pinMode(BUTTON_RECORD, INPUT_PULLDOWN);
  pinMode(BUTTON_EFFECT_1, INPUT_PULLDOWN);
  pinMode(BUTTON_EFFECT_2, INPUT_PULLDOWN);
  pinMode(BUTTON_EFFECT_3, INPUT_PULLDOWN);
  pinMode(BUTTON_EFFECT_4, INPUT_PULLDOWN);
  pinMode(BUTTON_EFFECT_5, INPUT_PULLDOWN);

  startTimer();
}

void loop() {
  if (timerFlag) {
    portENTER_CRITICAL(&timerMux);
    timerFlag = false;
    portEXIT_CRITICAL(&timerMux);
    processAudio();
  }

  unsigned long currentTime = millis();

  // Handle Record button for short and long press
  if (digitalRead(BUTTON_RECORD) == HIGH) {
    if (buttonPressStartTime == 0) {
      buttonPressStartTime = currentTime; // Start timing the button press
    }

    if ((currentTime - buttonPressStartTime) > longPressThreshold && !isLongPress) {
      // Long press detected
      if (systemState == IDLE) {
        Serial.println("Recording started...");
        systemState = RECORDING;
        sampleIndex = 0;
        isLongPress = true;
      }
    }
  } else {
    if (buttonPressStartTime > 0 && !isLongPress) {
      // Short press detected
      if (systemState == IDLE) {
        Serial.println("Playback started...");
        systemState = PLAYBACK;
        sampleIndex = 0;
      }
    }
    buttonPressStartTime = 0;
    isLongPress = false;
  }

  if (digitalRead(BUTTON_EFFECT_1) == HIGH && (currentTime - lastDebounceTime[1]) > debounceDelay) {
    effect1Enabled = !effect1Enabled;
    digitalWrite(LED_EFFECT_1, effect1Enabled);
    Serial.println("Effect 1 toggled");
    lastDebounceTime[1] = currentTime;
  }

  if (digitalRead(BUTTON_EFFECT_2) == HIGH && (currentTime - lastDebounceTime[2]) > debounceDelay) {
    effect2Enabled = !effect2Enabled;
    digitalWrite(LED_EFFECT_2, effect2Enabled);
    Serial.println("Effect 2 toggled");
    lastDebounceTime[2] = currentTime;
  }

  if (digitalRead(BUTTON_EFFECT_3) == HIGH && (currentTime - lastDebounceTime[3]) > debounceDelay) {
    effect3Enabled = !effect3Enabled;
    digitalWrite(LED_EFFECT_3, effect3Enabled);
    Serial.println("Effect 3 toggled");
    lastDebounceTime[3] = currentTime;
  }

  if (digitalRead(BUTTON_EFFECT_4) == HIGH && (currentTime - lastDebounceTime[4]) > debounceDelay) {
    effect4Enabled = !effect4Enabled;
    digitalWrite(LED_EFFECT_4, effect4Enabled);
    Serial.println("Effect 4 toggled");
    lastDebounceTime[4] = currentTime;
  }

  if (digitalRead(BUTTON_EFFECT_5) == HIGH && (currentTime - lastDebounceTime[5]) > debounceDelay) {
    effect5Enabled = !effect5Enabled;
    digitalWrite(LED_EFFECT_5, effect5Enabled);
    Serial.println("Effect 5 toggled");
    lastDebounceTime[5] = currentTime;
  }
}
