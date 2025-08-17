#pragma once

#include <cmath>
#include "MathConstants.hpp"

namespace Engine2D
{
    Real ClearError(Real value, Real digit = Math::EPSILON)
    {
        return (Real)std::round(value / digit) * digit;
    }
}
