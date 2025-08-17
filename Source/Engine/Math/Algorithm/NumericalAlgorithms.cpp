#include "NumericalAlgorithms.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Engine2D
{
    // ========================================
    // 보간법 (Interpolation)
    // ========================================

    Vector2 NumericalAlgorithms::LinearInterpolation(const Vector2& a, const Vector2& b, Real t)
    {
        t = std::clamp(t, Real(0), Real(1));
        return a + (b - a) * t;
    }

    Vector2 NumericalAlgorithms::QuadraticInterpolation(const Vector2& a, const Vector2& b, const Vector2& c, Real t)
    {
        t = std::clamp(t, Real(0), Real(1));
        Real t2 = t * t;
        Real oneMinusT = 1 - t;
        Real oneMinusT2 = oneMinusT * oneMinusT;

        return a * oneMinusT2 + b * (2 * oneMinusT * t) + c * t2;
    }

    Vector2 NumericalAlgorithms::CubicInterpolation(const Vector2& a, const Vector2& b,
                                                   const Vector2& c, const Vector2& d, Real t)
    {
        t = std::clamp(t, Real(0), Real(1));
        Real t2 = t * t;
        Real t3 = t2 * t;
        Real oneMinusT = 1 - t;
        Real oneMinusT2 = oneMinusT * oneMinusT;
        Real oneMinusT3 = oneMinusT2 * oneMinusT;

        return a * oneMinusT3 + b * (3 * oneMinusT2 * t) +
               c * (3 * oneMinusT * t2) + d * t3;
    }

    Real NumericalAlgorithms::LinearInterpolation(Real a, Real b, Real t)
    {
        t = std::clamp(t, Real(0), Real(1));
        return a + (b - a) * t;
    }

    Real NumericalAlgorithms::QuadraticInterpolation(Real a, Real b, Real c, Real t)
    {
        t = std::clamp(t, Real(0), Real(1));
        Real t2 = t * t;
        Real oneMinusT = 1 - t;
        Real oneMinusT2 = oneMinusT * oneMinusT;

        return a * oneMinusT2 + b * (2 * oneMinusT * t) + c * t2;
    }

    Real NumericalAlgorithms::CubicInterpolation(Real a, Real b, Real c, Real d, Real t)
    {
        t = std::clamp(t, Real(0), Real(1));
        Real t2 = t * t;
        Real t3 = t2 * t;
        Real oneMinusT = 1 - t;
        Real oneMinusT2 = oneMinusT * oneMinusT;
        Real oneMinusT3 = oneMinusT2 * oneMinusT;

        return a * oneMinusT3 + b * (3 * oneMinusT2 * t) +
               c * (3 * oneMinusT * t2) + d * t3;
    }

    // ========================================
    // 근사 계산 (Approximation)
    // ========================================

    Real NumericalAlgorithms::ApproximateIntegral(const std::function<Real(Real)>& func, Real a, Real b, int segments)
    {
        if (segments <= 0) return 0;

        Real h = (b - a) / segments;
        Real sum = (func(a) + func(b)) / 2;

        for (int i = 1; i < segments; ++i) {
            sum += func(a + i * h);
        }

        return sum * h;
    }

    Real NumericalAlgorithms::ApproximateIntegralSimpson(const std::function<Real(Real)>& func, Real a, Real b, int segments)
    {
        if (segments <= 0 || segments % 2 != 0) return 0;

        Real h = (b - a) / segments;
        Real sum = func(a) + func(b);

        for (int i = 1; i < segments; ++i) {
            Real x = a + i * h;
            sum += (i % 2 == 0 ? 2 : 4) * func(x);
        }

        return sum * h / 3;
    }

    Real NumericalAlgorithms::ApproximateDerivative(const std::function<Real(Real)>& func, Real x, Real h)
    {
        return (func(x + h) - func(x - h)) / (2 * h);
    }

    Vector2 NumericalAlgorithms::ApproximateDerivative(const std::function<Vector2(Real)>& func, Real t, Real h)
    {
        return (func(t + h) - func(t - h)) / (2 * h);
    }

    Real NumericalAlgorithms::ApproximateSecondDerivative(const std::function<Real(Real)>& func, Real x, Real h)
    {
        return (func(x + h) - 2 * func(x) + func(x - h)) / (h * h);
    }

    // ========================================
    // 수치 안정성 (Numerical Stability)
    // ========================================

    Real NumericalAlgorithms::Clamp(Real value, Real min, Real max)
    {
        return std::clamp(value, min, max);
    }

    bool NumericalAlgorithms::IsNearlyEqual(Real a, Real b, Real tolerance)
    {
        return std::abs(a - b) <= tolerance;
    }

    Real NumericalAlgorithms::SafeDivide(Real numerator, Real denominator, Real defaultValue)
    {
        if (std::abs(denominator) < Math::EPSILON) {
            return defaultValue;
        }
        return numerator / denominator;
    }

    Real NumericalAlgorithms::SafeSqrt(Real value, Real defaultValue)
    {
        if (value < 0) {
            return defaultValue;
        }
        return std::sqrt(value);
    }

    Real NumericalAlgorithms::SafeInverse(Real value, Real defaultValue)
    {
        if (std::abs(value) < Math::EPSILON) {
            return defaultValue;
        }
        return 1.0 / value;
    }

    Real NumericalAlgorithms::Normalize(Real value, Real min, Real max)
    {
        if (std::abs(max - min) < Math::EPSILON) return 0;
        return Clamp((value - min) / (max - min), 0, 1);
    }

    Real NumericalAlgorithms::Denormalize(Real normalizedValue, Real min, Real max)
    {
        return min + normalizedValue * (max - min);
    }

    // ========================================
    // 수치 최적화 (Numerical Optimization)
    // ========================================

    Real NumericalAlgorithms::BisectionMethod(const std::function<Real(Real)>& func, Real a, Real b,
                                             Real tolerance, int maxIterations)
    {
        if (func(a) * func(b) > 0) {
            return (a + b) / 2; // 해가 없을 수 있음
        }

        for (int i = 0; i < maxIterations; ++i) {
            Real c = (a + b) / 2;
            Real fc = func(c);

            if (std::abs(fc) < tolerance) {
                return c;
            }

            if (func(a) * fc < 0) {
                b = c;
            } else {
                a = c;
            }
        }

        return (a + b) / 2;
    }

    Real NumericalAlgorithms::NewtonRaphsonMethod(const std::function<Real(Real)>& func,
                                                 const std::function<Real(Real)>& derivative,
                                                 Real initialGuess, Real tolerance,
                                                 int maxIterations)
    {
        Real x = initialGuess;

        for (int i = 0; i < maxIterations; ++i) {
            Real fx = func(x);
            Real dfx = derivative(x);

            if (std::abs(dfx) < Math::EPSILON) {
                break; // 미분이 0에 가까움
            }

            Real xNew = x - fx / dfx;

            if (std::abs(xNew - x) < tolerance) {
                return xNew;
            }

            x = xNew;
        }

        return x;
    }

    Real NumericalAlgorithms::GoldenSectionSearch(const std::function<Real(Real)>& func, Real a, Real b,
                                                 Real tolerance, int maxIterations)
    {
        const Real goldenRatio = (1 + std::sqrt(5)) / 2;
        const Real goldenRatioInv = 1 / goldenRatio;

        Real c = b - goldenRatioInv * (b - a);
        Real d = a + goldenRatioInv * (b - a);

        for (int i = 0; i < maxIterations; ++i) {
            if (std::abs(c - d) < tolerance) {
                return (c + d) / 2;
            }

            if (func(c) < func(d)) {
                b = d;
                d = c;
                c = b - goldenRatioInv * (b - a);
            } else {
                a = c;
                c = d;
                d = a + goldenRatioInv * (b - a);
            }
        }

        return (c + d) / 2;
    }

    // ========================================
    // 수치 필터링 (Numerical Filtering)
    // ========================================

    Real NumericalAlgorithms::MovingAverage(const std::vector<Real>& values, int windowSize)
    {
        if (values.empty() || windowSize <= 0) return 0;

        int actualWindowSize = std::min(windowSize, static_cast<int>(values.size()));
        Real sum = 0;

        for (int i = 0; i < actualWindowSize; ++i) {
            sum += values[values.size() - 1 - i];
        }

        return sum / actualWindowSize;
    }

    Real NumericalAlgorithms::ExponentialMovingAverage(Real currentValue, Real previousEMA, Real alpha)
    {
        alpha = std::clamp(alpha, Real(0), Real(1));
        return alpha * currentValue + (1 - alpha) * previousEMA;
    }

    Real NumericalAlgorithms::MedianFilter(const std::vector<Real>& values)
    {
        if (values.empty()) return 0;

        std::vector<Real> sorted = values;
        std::sort(sorted.begin(), sorted.end());

        if (sorted.size() % 2 == 0) {
            return (sorted[sorted.size() / 2 - 1] + sorted[sorted.size() / 2]) / 2;
        } else {
            return sorted[sorted.size() / 2];
        }
    }

    // ========================================
    // 수치 변환 (Numerical Conversion)
    // ========================================

    Real NumericalAlgorithms::DegreesToRadians(Real degrees)
    {
        return degrees * PI / 180.0;
    }

    Real NumericalAlgorithms::RadiansToDegrees(Real radians)
    {
        return radians * 180.0 / PI;
    }

    Real NumericalAlgorithms::NormalizeAngle(Real angle)
    {
        angle = std::fmod(angle, 360.0);
        if (angle < 0) angle += 360.0;
        return angle;
    }

    Real NumericalAlgorithms::NormalizeRadians(Real radians)
    {
        radians = std::fmod(radians, TWO_PI);
        if (radians < 0) radians += TWO_PI;
        return radians;
    }

    // ========================================
    // 수치 검증 (Numerical Validation)
    // ========================================

    bool NumericalAlgorithms::IsValidNumber(Real value)
    {
        return !std::isnan(value) && !std::isinf(value);
    }

    bool NumericalAlgorithms::IsValidVector(const Vector2& vector)
    {
        return IsValidNumber(vector.x) && IsValidNumber(vector.y);
    }

    Vector2 NumericalAlgorithms::SafeNormalize(const Vector2& vector, const Vector2& defaultValue)
    {
        Real length = vector.Length();
        if (length < Math::EPSILON) {
            return defaultValue;
        }
        return vector / length;
    }

    Vector2 NumericalAlgorithms::RotateVector(const Vector2& vector, Real angle)
    {
        Real cosAngle = std::cos(angle);
        Real sinAngle = std::sin(angle);
        return Vector2(
            vector.x * cosAngle - vector.y * sinAngle,
            vector.x * sinAngle + vector.y * cosAngle
        );
    }

    // ========================================
    // 성능 측정 (Performance Measurement)
    // ========================================

    Real NumericalAlgorithms::CalculateAccuracy(const std::vector<Real>& computed, const std::vector<Real>& expected)
    {
        if (computed.size() != expected.size() || computed.empty()) return 0;

        Real totalError = 0;
        for (size_t i = 0; i < computed.size(); ++i) {
            totalError += std::abs(computed[i] - expected[i]);
        }

        return totalError / computed.size();
    }

    Real NumericalAlgorithms::CalculateRelativeError(Real computed, Real expected)
    {
        if (std::abs(expected) < Math::EPSILON) return 0;
        return std::abs(computed - expected) / std::abs(expected);
    }

    Real NumericalAlgorithms::CalculateAbsoluteError(Real computed, Real expected)
    {
        return std::abs(computed - expected);
    }
}
