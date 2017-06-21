#include "Arduino.h"
#include <pgmspace.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <Adafruit_PWMServoDriver.h>

#define countof(a) (sizeof(a) / sizeof(a[0]))

#define PWM_FREQ 1600
#define PWM_PINS 6
#define PWM_MAX_VALUE 4096

#define DEFAULT_LIGHT_PERCENT 70


const uint8_t STATUS_TRANSITION_START = 0;
const uint8_t STATUS_TRANSITION_END = 1;

void printDateTime(const RtcDateTime& dt);
float easeInOut(float t,float b, float c, float d);
void light(uint16_t initialValue, uint16_t destinationValue, uint16_t durationMils = 2000);
void lightOn();
float intensityValue(uint8_t percent);


template <typename T, size_t N> size_t ArraySize (T (&) [N]){ return N; }
template <typename T> void PROGMEM_load (const T * sce, T& dest){memcpy_P (&dest, sce, sizeof (T));}


RtcDS3231<TwoWire> Rtc(Wire);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint16_t lightIntensity;

uint8_t status;

unsigned long previousMillis = 0;
uint8_t previousMinute = 0;
const long interval = 59000; // 59sec

struct Task {
    uint8_t hours;
    uint8_t minutes;
    uint8_t intensityPercent;
};

const Task Scheduler [6] PROGMEM = {
    {23, 24, 0},
    {23, 25, 70},
    {23, 26, 10},
    {23, 27, 80},
    {23, 28, 10},
    {23, 29, 100},
};

void setup() {
        Serial.begin(115200);

        Rtc.Begin();

        Wire.begin(D1, D2);

        RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
        printDateTime(compiled);

        pwm.begin();
        pwm.setPWMFreq(PWM_FREQ);

        for (uint8_t pwmnum=0; pwmnum < PWM_PINS; pwmnum++) {
            pwm.setPWM(pwmnum, 0, 0);
        }

        lightIntensity = intensityValue(DEFAULT_LIGHT_PERCENT);

        delay(2000);

        lightOn();
}

void loop() {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        RtcDateTime now = Rtc.GetDateTime();

        uint8_t hour = now.Hour();
        uint8_t minute = now.Minute();

        if (previousMinute != minute) {
            previousMinute = minute;

            for (uint8_t i = 0; i < 6; i++) {
                Task task;
                PROGMEM_load(&Scheduler[i], task);
                if (hour == task.hours && minute == task.minutes) {
                    uint16_t destIntensity = intensityValue(task.intensityPercent);
                    light(lightIntensity, destIntensity);
                }
            }
        }
    }
}

void printDateTime(const RtcDateTime& dt)
{
        char datestring[20];

        snprintf_P(datestring,
                   countof(datestring),
                   PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
                   dt.Month(),
                   dt.Day(),
                   dt.Year(),
                   dt.Hour(),
                   dt.Minute(),
                   dt.Second() );
        Serial.print(datestring);
        Serial.println();
}

float easeInOut(float t,float b, float c, float d)
{
	if ((t/=d/2) < 1) return c/2*t*t*t + b;
	return c/2*((t-=2)*t*t + 2) + b;
}

void light(uint16_t initialValue, uint16_t destinationValue, uint16_t durationMils)
{
    int16_t amountOfChange = destinationValue - initialValue;
    uint16_t elapsed = 0;
    uint8_t stepTimeout = 17;
    float result;

    while(elapsed <= durationMils) {
        yield();
        result = easeInOut(elapsed, initialValue, amountOfChange, durationMils);
        for (uint8_t pwmnum=0; pwmnum < PWM_PINS; pwmnum++) {
            pwm.setPWM(pwmnum, 0, result);
        }
        delay(stepTimeout);
        elapsed += stepTimeout;
    }

    lightIntensity = result;
}

void lightOn()
{
    light(0, lightIntensity);
}

float intensityValue(uint8_t percent)
{
    return (float)percent / 100.0 * PWM_MAX_VALUE;
}
