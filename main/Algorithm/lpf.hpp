#pragma once

#include <cmath>

class LPF
{
public:
    LPF(float alpha = 0.7f) : _alpha(alpha), _initialized(false), _value(0.0f)
    {
    }

    void setAlpha(float alpha)
    {
        _alpha = alpha;
    }

    float update(float input)
    {
        if (!_initialized) {
            _value = input;
            _initialized = true;
            return _value;
        }

        _value = _alpha * _value + (1.0f - _alpha) * input;
        return _value;
    }

    float value() const
    {
        return _value;
    }

    void reset(float value = 0.0f)
    {
        _value = value;
        _initialized = false;
    }

private:
    float _alpha;
    bool _initialized;
    float _value;
};
