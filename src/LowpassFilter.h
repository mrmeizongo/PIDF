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
#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H
#include "Filter.h"

enum class FilterType : uint8_t
{
    FIRST_ORDER = 0U,
    SECOND_ORDER
};

class LowPassFilter
{
public:
    // Use default filter of first order if no filter type is specified
    LowPassFilter(float cutoffFrequency, FilterType _filterType = FilterType::FIRST_ORDER)
    {
        filterType = _filterType;
        switch (filterType)
        {
        case FilterType::FIRST_ORDER:
            lpf = new FirstOrderLPF(cutoffFrequency);
            break;
        case FilterType::SECOND_ORDER:
            lpf = new SecondOrderLPF(cutoffFrequency);
            break;
        default:
            break;
        }
    }

    float Process(float input, float samplingFrequency)
    {
        return lpf->Process(input, samplingFrequency);
    }

    FilterType getFilterType() { return filterType; }

    ~LowPassFilter()
    {
        delete lpf;
    }

private:
    Filter *lpf;
    FilterType filterType;
};

#endif // LOWPASSFILTER_H