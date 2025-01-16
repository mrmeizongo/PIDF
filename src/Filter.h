/*
MIT License

Copyright (c) 2025 Jamal Meizongo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#ifndef FILTER_H
#define FILTER_H
#include <Arduino.h>

#define SQRT2 1.4142135623730950488f // Hard coded value of sqrt(2) to save execution cycles

class Filter
{
public:
    virtual float Process(float, float) = 0;
    virtual ~Filter() = default;
};

class FirstOrderLPF : public Filter
{
public:
    FirstOrderLPF(float cutoffFrequency)
    {
        rc = 1.0f / (2.0f * M_PI * cutoffFrequency);
        prevOutput = 0.0f;
    }

    float Process(float input, float samplingFrequency) override
    {
        // Calculate alpha based on the cuttoff frequency and sampling frequency
        prevOutput = prevOutput + ((samplingFrequency / (rc + samplingFrequency)) * (input - prevOutput));
        return prevOutput;
    }

private:
    float prevOutput; // Previous output value
    float rc;
};

class SecondOrderLPF : public Filter
{
public:
    SecondOrderLPF(float cutoffFrequency)
        : cutoffFrequency(cutoffFrequency), prevInput1(0.0f), prevInput2(0.0f), prevOutput1(0.0f), prevOutput2(0.0f) {}

    void CalculateCoEfficients(float samplingFrequency)
    {
        // Calculate normalized cutoff frequency
        float omega = 2.0f * M_PI * (cutoffFrequency * samplingFrequency);
        float sinOmega = sin(omega);
        float cosOmega = cos(omega);

        // Compute Butterworth coefficients
        float alpha = sinOmega / SQRT2;
        float scale = 1.0f / (1.0f + alpha);

        b0 = (1.0f - cosOmega) / (2.0f * scale);
        b1 = (1.0f - cosOmega) * scale;
        b2 = b0;
        a1 = -2.0f * cosOmega * scale;
        a2 = (1.0f - alpha) * scale;
    }

    float Process(float input, float samplingFrequency) override
    {
        CalculateCoEfficients(samplingFrequency);
        float output = (b0 * input) + (b1 * prevInput1) + (b2 * prevInput2) - (a1 * prevOutput1) - (a2 * prevOutput2);

        // Update previous values
        prevInput2 = prevInput1;
        prevInput1 = input;
        prevOutput2 = prevOutput1;
        prevOutput1 = output;

        return output;
    }

private:
    float cutoffFrequency;
    float a1, a2, b0, b1, b2;                               // Filter coefficients
    float prevInput1, prevInput2, prevOutput1, prevOutput2; // Previous input and output values
};
#endif // FILTER_H