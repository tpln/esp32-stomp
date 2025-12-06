#include <Arduino.h>

// ----- Pins -----
const int DAC_OUT1 = 25;  // DAC1 output (sine)
const int DAC_OUT2 = 26;  // DAC2 output (processed)
const int ADC_IN   = 34;  // ADC input

// ----- Sine table -----
const int   TABLE_SIZE = 256;
float       sineTable[TABLE_SIZE];

// ----- Timing / sample rate -----
const uint32_t SAMPLE_RATE_HZ   = 20000;     // 20 kHz
const uint32_t TIMER_BASE_HZ    = 1000000;   // 1 MHz timer base
const uint32_t TICKS_PER_SAMPLE = TIMER_BASE_HZ / SAMPLE_RATE_HZ; // 1e6 / 2e4 = 50

// ----- Test signal frequency -----
const float TEST_FREQ_HZ = 5.0f;           // A4 tone, change freely

// Phase accumulator stuff
float     g_phase       = 0.0f;
float     g_phaseStep   = 0.0f;              // computed in setup()

// ----- Plotting throttle -----
const uint16_t PLOT_DECIMATION = 200;        // 20k/200 = 100 Hz plot rate

// ----- Soft-clip parameters -----
const float CLIP_LIMIT = 0.3f;

// ----- Timer & shared state -----
hw_timer_t* timer = nullptr;

// ISR → loop tick flag
volatile bool g_sampleTick = false;

// Plotting shared state
volatile uint8_t g_plotSample      = 0;
volatile bool    g_plotSampleReady = false;


// -----------------------------------------------------
// Generate LUT [-1, 1]
// -----------------------------------------------------
void generateSineTable() {
    for (int i = 0; i < TABLE_SIZE; ++i) {
        float angle = 2.0f * PI * (float)i / (float)TABLE_SIZE;
        sineTable[i] = sinf(angle);
    }
}


// -----------------------------------------------------
// Generate sine at TEST_FREQ_HZ
// -----------------------------------------------------
float generateSine() {
    // Fractional index
    int   index      = (int)g_phase;
    float sample     = sineTable[index];

    // Advance phase accumulator
    g_phase += g_phaseStep;
    if (g_phase >= TABLE_SIZE) {
        g_phase -= TABLE_SIZE;
    }

    return sample;
}


// -----------------------------------------------------
float adcToNorm(uint16_t raw) {
    return ((float)raw - 2048.0f) / 2048.0f;
}

uint8_t normToDac(float x) {
    if (x > 1.0f)  x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    float shifted = (x + 1.0f) * 0.5f;
    return (uint8_t)(shifted * 255.0f + 0.5f);
}

float softClip(float x) {
    float y = x / (1.0f + fabsf(x));  // smooth compress
    if (y >  CLIP_LIMIT) y =  CLIP_LIMIT;
    if (y < -CLIP_LIMIT) y = -CLIP_LIMIT;
    return y;
}


// -----------------------------------------------------
void ARDUINO_ISR_ATTR onTimer() {
    g_sampleTick = true;
}


// -----------------------------------------------------
void processOneSample() {
    float   sineNorm = generateSine();
    uint8_t sineDac  = normToDac(sineNorm);
    dacWrite(DAC_OUT1, sineDac);

    uint16_t adcRaw = analogRead(ADC_IN);
    float    inNorm = adcToNorm(adcRaw);

    float   outNorm = softClip(inNorm);
    uint8_t outDac  = normToDac(outNorm);

    dacWrite(DAC_OUT2, outDac);

    static uint16_t plotCounter = 0;
    plotCounter++;
    if (plotCounter >= PLOT_DECIMATION) {
        plotCounter = 0;

        noInterrupts();
        g_plotSample      = outDac;
        g_plotSampleReady = true;
        interrupts();
    }
}


// -----------------------------------------------------
void plotSampleIfReady() {
    uint8_t sample = 0;
    bool    ready  = false;

    noInterrupts();
    if (g_plotSampleReady) {
        ready             = true;
        sample            = g_plotSample;
        g_plotSampleReady = false;
    }
    interrupts();

    if (ready) {
        Serial.println(sample);
    }
}


// -----------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("20 kHz DSP loop with test tone frequency control...");

    generateSineTable();

    // Compute fractional phase step for arbitrary frequency
    g_phase       = 0.0f;
    g_phaseStep   = (float)TABLE_SIZE * TEST_FREQ_HZ / (float)SAMPLE_RATE_HZ;

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    timer = timerBegin(TIMER_BASE_HZ);  // 1 MHz timer
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, TICKS_PER_SAMPLE, true, 0);  // 50 ticks = 20 kHz
}


// -----------------------------------------------------
void loop() {
    if (g_sampleTick) {
        g_sampleTick = false;
        processOneSample();
    }
    plotSampleIfReady();
}
