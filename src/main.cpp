#include <pgmspace.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <Adafruit_PWMServoDriver.h>
#include "main.h"


#define PWM_FREQ 200
#define PWM_PINS 6
#define PWM_MAX_VALUE 4096

RtcDS3231<TwoWire> Rtc(Wire);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t initialIntensity = 0;

unsigned long previousMillis = 0;
uint8_t previousMinute = 0;
const long interval = 30000; // 30sec

struct LightTask {
        uint8_t hour;
        uint8_t minute;
        uint8_t intensity;
};
const int SCHEDULER_SIZE = 10;
const LightTask Scheduler [SCHEDULER_SIZE] PROGMEM = {
        {7, 30, 3}, //1%
        {7, 40, 26}, //10%
        {7, 50, 52}, //20%
        {7, 55, 128}, //50%
        {8, 0, 205}, //80%
        {21, 0, 154}, //60%
        {21, 20, 103}, //40%
        {21, 40, 52}, //20%
        {22, 0, 26}, //10%
        {22, 30, 0}, //0%
};

// const LightTask Scheduler [SCHEDULER_SIZE] PROGMEM = {
//         {23, 0, 3}, //1%
//         {23, 1, 26}, //10%
//         {23, 2, 52}, //20%
//         {23, 4, 128}, //50%
//         {23, 5, 205}, //80%
//         {23, 6, 154}, //60%
//         {23, 7, 103}, //40%
//         {23, 8, 52}, //20%
//         {23, 9, 26}, //10%
//         {23, 10, 0}, //0%
// };

void setup() {
        //Serial.begin(9600);
        Wire.begin(D1, D2);

        Rtc.Begin();
        //RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

        pwm.begin();
        pwm.setPWMFreq(PWM_FREQ);

        for (uint8_t pwmnum=0; pwmnum < PWM_PINS; pwmnum++) {
                pwm.setPWM(pwmnum, 0, 0);
        }

        uint8_t destIntensity = getCurrentLightTaskIntensity();

        // Serial.print(compiled.Hour());
        // Serial.print(':');
        // Serial.print(compiled.Minute());
        // Serial.println();
        // Serial.println(destIntensity);

        if (destIntensity != 0) {
                light(0, destIntensity);
        }
}

void loop() {

        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= interval) {

                previousMillis = currentMillis;


                // int val =  analogRead(A0);
                // val = map(val, 0, 1023, 0,255);
                //
                // if (initialIntensity != val) {
                //     light(initialIntensity, val);
                // }


                RtcDateTime now = Rtc.GetDateTime();

                uint8_t hour = now.Hour();
                uint8_t minute = now.Minute();

                if (previousMinute != minute) {
                        previousMinute = minute;

                        for (uint8_t i = 0; i < SCHEDULER_SIZE; i++) {
                                LightTask task;
                                PROGMEM_load(&Scheduler[i], task);
                                if (hour == task.hour && minute == task.minute) {
                                        //Serial.println(task.intensity);
                                        light(initialIntensity, task.intensity);
                                }
                        }
                }
        }
}

// void printDateTime(const RtcDateTime& dt)
// {
//         char datestring[20];
//
//         snprintf_P(datestring,
//                    countof(datestring),
//                    PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
//                    dt.Month(),
//                    dt.Day(),
//                    dt.Year(),
//                    dt.Hour(),
//                    dt.Minute(),
//                    dt.Second() );
//         Serial.print(datestring);
//         Serial.println();
// }

float easeInOut(float t,float b, float c, float d)
{
        if ((t/=d/2) < 1) return c/2*t*t*t + b;
        return c/2*((t-=2)*t*t + 2) + b;
}

void light(uint8_t initialValue, uint8_t destinationValue, uint16_t durationMils)
{
        int16_t amountOfChange = destinationValue - initialValue;
        uint16_t elapsed = 0;
        uint8_t stepTimeout = 17; // 60FPS = 1sec/60 = 1000/60 = 16.6 ~= 17
        float result;

        unsigned long previousMillis = millis();

        // Serial.print("Initial: ");
        // Serial.print(initialValue);
        // Serial.print(", destinationValue: ");
        // Serial.print(destinationValue);
        // Serial.print(", amountOfChange: ");
        // Serial.println(amountOfChange);

        uint16_t prevPwmVal;

        while(elapsed <= durationMils) {
                yield();
                unsigned long currentMillis = millis();
                unsigned long timeDiff = currentMillis - previousMillis;
                if (elapsed > 0 && timeDiff < stepTimeout) {
                        continue;
                }
                previousMillis = currentMillis;

                result = easeInOut(elapsed, initialValue, amountOfChange, durationMils);
                uint16_t pwmVal = CIELab8to12(result);
                if (prevPwmVal == pwmVal) {
                        elapsed += (uint16_t)timeDiff;
                        continue;
                }
                prevPwmVal = pwmVal;
                //Serial.print(" ");Serial.print((int)result);Serial.print(" ");
                for (uint8_t pwmNum=0; pwmNum < PWM_PINS; pwmNum++) {
                        pwm.setPWM(pwmNum, 0, pwmVal);
                }
                elapsed += (uint16_t)timeDiff;
        }

        initialIntensity = destinationValue;
}

uint8_t getCurrentLightTaskIntensity()
{
        RtcDateTime now = Rtc.GetDateTime();
        uint8_t hour = now.Hour();
        uint8_t minute = now.Minute();

        uint8_t prevTaskIntensity = Scheduler[SCHEDULER_SIZE - 1].intensity;
        // Serial.print("prev");
        // Serial.println(prevTaskIntensityPercent);

        for (uint8_t i = 0; i < SCHEDULER_SIZE; i++) {
                LightTask task;
                PROGMEM_load(&Scheduler[i], task);

                if (hour > task.hour) {
                        prevTaskIntensity = task.intensity;
                        continue;
                }

                if (hour == task.hour) {
                        if (minute > task.minute) {
                                prevTaskIntensity = task.intensity;
                                continue;
                        } else {
                                return prevTaskIntensity;
                        }
                }

                return prevTaskIntensity;
        }
        return prevTaskIntensity;
}

uint16_t CIELab8to12(uint8_t a) {
        return pgm_read_word_near(CIEL_8_12 + a);
}
