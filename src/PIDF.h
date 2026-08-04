/* ============================================
Copyright (C) 2024 Jamal Meizongo
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
===============================================
*/

#ifndef _PIDF_H
#define _PIDF_H
#include <Arduino.h>
#include "../lib/LowPassFilter/src/LowPassFilter.h"

template <typename T>
class PIDF
{
public:
    PIDF(float _Kp, float _Ki, float _Kd, float _Kf, float _IMax, float _deltaTime, uint16_t _filterCutoffFrequency)
        : Kp{_Kp}, Ki{_Ki}, Kd{_Kd}, Kf{_Kf}, IMax{_IMax}, deltaTime{_deltaTime},
          integrator{0.f}, previousError{0.f}, previousTime{0},
          currentPointFilter{SecondOrderLPF<float>(_filterCutoffFrequency, deltaTime)},
          derivativeFilter{SecondOrderLPF<float>(_filterCutoffFrequency, deltaTime)}
    {
    }

    void reset(void) { previousTime = 0; } // Reset PIDF controller

    T Compute(T setPoint, T currentPoint)
    {
        uint32_t currentTime = millis();
        uint32_t dt = currentTime - previousTime;
        T output = T{};

        /*
         * If this PIDF just started, reset or hasn't been used for two seconds then reset the PIDF.
         * It prevents I buildup from a previous fight mode
         * from causing a massive return before the integrator gets a chance to correct itself
         */
        if (previousTime == 0 || dt > 2000)
        {
            dt = 0;
            integrator = 0.f;
            currentPointFilter.Reset();
            derivativeFilter.Reset();
        }

        previousTime = currentTime;

        // Compute proportional component
        currentPoint = currentPointFilter.Process(currentPoint);
        T currentError = setPoint - currentPoint;
        output += currentError * Kp;

        // Compute integral component if time has elapsed
        if ((fabsf(Ki) > 0) && (dt > 0))
        {
            integrator += (currentError * Ki) * deltaTime;
            // Limit integrator wind up
            integrator = constrain(integrator, -IMax, IMax);
            output += integrator;
        }

        // Compute derivative component if time has elapsed
        if ((fabsf(Kd) > 0) && (dt > 0))
        {
            // Calculate new derivative
            float derivative = (currentError - previousError) / deltaTime;
            // Apply low pass filter to eliminate high frequency noise in the derivative term
            derivative = derivativeFilter.Process(derivative);
            // Update state
            previousError = currentError;
            // Add in derivative component
            output += derivative * Kd;
        }

        // Compute feedforward component if time has elapsed
        if ((fabsf(Kf) > 0) && (dt > 0))
            output += setPoint * Kf;

        return output;
    }

    float getKp(void) { return Kp; }
    float getKi(void) { return Ki; }
    float getKd(void) { return Kd; }
    float getKf(void) { return Kf; }
    float getIMax(void) { return IMax; }

    void setKp(float _Kp) { Kp = _Kp; }
    void setKi(float _Ki) { Ki = _Ki; }
    void setKd(float _Kd) { Kd = _Kd; }
    void setKf(float _Kf) { Kf = _Kf; }
    void setIMax(float _IMax) { IMax = _IMax; }

private:
    // Gains
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float IMax;

    float deltaTime;

    float integrator;
    float previousError;
    uint32_t previousTime;

    // First order low pass filter for measured current point
    SecondOrderLPF<float> currentPointFilter;
    // First order low pass filter for derivative
    SecondOrderLPF<float> derivativeFilter;
};
#endif //_PIDF_H