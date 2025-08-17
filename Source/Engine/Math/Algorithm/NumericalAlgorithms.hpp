#pragma once

#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <limits>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"

namespace Engine2D
{
    // ========================================
    // 수치 계산 알고리즘 클래스
    // ========================================

    class NumericalAlgorithms
    {
    public:
        // ========================================
        // 보간법 (Interpolation)
        // ========================================

        // 선형 보간
        static Vector2 LinearInterpolation(const Vector2& a, const Vector2& b, Real t);

        // 이차 보간 (3점)
        static Vector2 QuadraticInterpolation(const Vector2& a, const Vector2& b, const Vector2& c, Real t);

        // 삼차 보간 (4점)
        static Vector2 CubicInterpolation(const Vector2& a, const Vector2& b,
                                         const Vector2& c, const Vector2& d, Real t);

        // 스칼라 선형 보간
        static Real LinearInterpolation(Real a, Real b, Real t);

        // 스칼라 이차 보간
        static Real QuadraticInterpolation(Real a, Real b, Real c, Real t);

        // 스칼라 삼차 보간
        static Real CubicInterpolation(Real a, Real b, Real c, Real d, Real t);

        // ========================================
        // 근사 계산 (Approximation)
        // ========================================

        // 수치 적분 (사다리꼴 법칙)
        static Real ApproximateIntegral(const std::function<Real(Real)>& func, Real a, Real b, int segments);

        // 수치 적분 (Simpson 법칙)
        static Real ApproximateIntegralSimpson(const std::function<Real(Real)>& func, Real a, Real b, int segments);

        // 수치 미분 (중앙 차분)
        static Real ApproximateDerivative(const std::function<Real(Real)>& func, Real x, Real h = 1e-6);

        // 벡터 함수의 수치 미분
        static Vector2 ApproximateDerivative(const std::function<Vector2(Real)>& func, Real t, Real h = 1e-6);

        // 이차 미분 (중앙 차분)
        static Real ApproximateSecondDerivative(const std::function<Real(Real)>& func, Real x, Real h = 1e-6);

        // ========================================
        // 수치 안정성 (Numerical Stability)
        // ========================================

        // 값 클램핑
        static Real Clamp(Real value, Real min, Real max);

        // 값이 거의 같은지 확인
        static bool IsNearlyEqual(Real a, Real b, Real tolerance = Math::EPSILON);

        // 안전한 나눗셈
        static Real SafeDivide(Real numerator, Real denominator, Real defaultValue = 0);

        // 안전한 제곱근
        static Real SafeSqrt(Real value, Real defaultValue = 0);

        // 안전한 역수
        static Real SafeInverse(Real value, Real defaultValue = 0);

        // 값 정규화 (0-1 범위로)
        static Real Normalize(Real value, Real min, Real max);

        // 값 비정규화 (0-1 범위에서 원래 범위로)
        static Real Denormalize(Real normalizedValue, Real min, Real max);

        // ========================================
        // 수치 최적화 (Numerical Optimization)
        // ========================================

        // 이분법 (Bisection Method)
        static Real BisectionMethod(const std::function<Real(Real)>& func, Real a, Real b,
                                   Real tolerance = Math::EPSILON, int maxIterations = 100);

        // 뉴턴-랩슨 방법 (Newton-Raphson Method)
        static Real NewtonRaphsonMethod(const std::function<Real(Real)>& func,
                                       const std::function<Real(Real)>& derivative,
                                       Real initialGuess, Real tolerance = Math::EPSILON,
                                       int maxIterations = 100);

        // 황금 분할 탐색 (Golden Section Search)
        static Real GoldenSectionSearch(const std::function<Real(Real)>& func, Real a, Real b,
                                       Real tolerance = Math::EPSILON, int maxIterations = 100);

        // ========================================
        // 수치 필터링 (Numerical Filtering)
        // ========================================

        // 이동 평균 필터
        static Real MovingAverage(const std::vector<Real>& values, int windowSize);

        // 지수 이동 평균
        static Real ExponentialMovingAverage(Real currentValue, Real previousEMA, Real alpha);

        // 중간값 필터
        static Real MedianFilter(const std::vector<Real>& values);

        // ========================================
        // 수치 변환 (Numerical Conversion)
        // ========================================

        // 도를 라디안으로 변환
        static Real DegreesToRadians(Real degrees);

        // 라디안을 도로 변환
        static Real RadiansToDegrees(Real radians);

        // 각도 정규화 (0-360도)
        static Real NormalizeAngle(Real angle);

        // 라디안 정규화 (0-2π)
        static Real NormalizeRadians(Real radians);

        // ========================================
        // 수치 검증 (Numerical Validation)
        // ========================================

        // 유효한 숫자인지 확인
        static bool IsValidNumber(Real value);

        // 유효한 벡터인지 확인
        static bool IsValidVector(const Vector2& vector);

        // 벡터 정규화 (안전한 버전)
        static Vector2 SafeNormalize(const Vector2& vector, const Vector2& defaultValue = Vector2(1, 0));

        // 벡터 회전
        static Vector2 RotateVector(const Vector2& vector, Real angle);

        // ========================================
        // 성능 측정 (Performance Measurement)
        // ========================================

        // 수치 계산의 정확도 측정
        static Real CalculateAccuracy(const std::vector<Real>& computed, const std::vector<Real>& expected);

        // 수치 계산의 상대 오차 측정
        static Real CalculateRelativeError(Real computed, Real expected);

        // 수치 계산의 절대 오차 측정
        static Real CalculateAbsoluteError(Real computed, Real expected);
    };
}
