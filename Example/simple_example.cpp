#include <Arduino.h>
#include "PIDF.h"

#define INPUT_PIN A0
#define OUTPUT_PIN D5
#define PID_LPF_FREQ 20.0f

float Kp = 5.0f;
float Ki = 0.2f;
float Kd = 0.01f;
float Kf = 0.3f;
float IMax = 100.0f;

PID myController{Kp, Ki, Kd, Kf, iMax, PID_LPF_FREQ}; // Constructor with gain values set

float setPoint = 72;
float actual = 0;
float output = 0;

void setup()
{
    pinMode(INPUT_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
}

void loop()
{
    actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 255);

    output = myController.Compute(setPoint, actual);
    analogWrite(OUTPUT_PIN, output);

    Serial.print("Output is: ");
    Serial.println(output);
}