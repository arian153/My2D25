#include "Hermite.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // Hermite 곡선 계산 함수들
    // ========================================

    Vector2 Hermite::Point(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) return Vector2(0, 0);

        // Hermite 곡선은 4개 제어점: p0, p1, t0, t1
        Vector2 p0 = controlPoints[0];
        Vector2 p1 = controlPoints[1];
        Vector2 t0 = controlPoints[2];
        Vector2 t1 = controlPoints[3];

        return CalculateHermitePoint(p0, p1, t0, t1, t);
    }

    Vector2 Hermite::Derivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) return Vector2(0, 0);

        // Hermite 곡선은 4개 제어점: p0, p1, t0, t1
        Vector2 p0 = controlPoints[0];
        Vector2 p1 = controlPoints[1];
        Vector2 t0 = controlPoints[2];
        Vector2 t1 = controlPoints[3];

        // Hermite 미분 계산
        Real t2 = t * t;

        Real h0 = 6 * t2 - 6 * t;
        Real h1 = -6 * t2 + 6 * t;
        Real h2 = 3 * t2 - 4 * t + 1;
        Real h3 = 3 * t2 - 2 * t;

        return p0 * h0 + p1 * h1 + t0 * h2 + t1 * h3;
    }

    Vector2 Hermite::SecondDerivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) return Vector2(0, 0);

        // Hermite 곡선은 4개 제어점: p0, p1, t0, t1
        Vector2 p0 = controlPoints[0];
        Vector2 p1 = controlPoints[1];
        Vector2 t0 = controlPoints[2];
        Vector2 t1 = controlPoints[3];

        // Hermite 2차 미분 계산
        Real h0 = 12 * t - 6;
        Real h1 = -12 * t + 6;
        Real h2 = 6 * t - 4;
        Real h3 = 6 * t - 2;

        return p0 * h0 + p1 * h1 + t0 * h2 + t1 * h3;
    }

    Real Hermite::Curvature(const Curve& curve, Real t)
    {
        Vector2 firstDerivative = Derivative(curve, t);
        Vector2 secondDerivative = SecondDerivative(curve, t);

        Real cross = firstDerivative.x * secondDerivative.y - firstDerivative.y * secondDerivative.x;
        Real firstDerivativeLength = firstDerivative.Length();

        if (firstDerivativeLength < Math::EPSILON) return 0;

        return std::abs(cross) / (firstDerivativeLength * firstDerivativeLength * firstDerivativeLength);
    }

    Vector2 Hermite::CurvatureComb(const Curve& curve, Real t)
    {
        Vector2 point = Point(curve, t);
        Vector2 normal = Normal(curve, t);
        Real curvature = Curvature(curve, t);

        return point + normal * curvature;
    }

    Vector2 Hermite::Tangent(const Curve& curve, Real t)
    {
        Vector2 derivative = Derivative(curve, t);
        return Normalize(derivative);
    }

    Vector2 Hermite::Normal(const Curve& curve, Real t)
    {
        Vector2 tangent = Tangent(curve, t);
        return Vector2(-tangent.y, tangent.x);
    }

    Vector2 Hermite::Binormal(const Curve& curve, Real t)
    {
        // 2D에서는 항상 (0,0,1)
        return Vector2(0, 0);
    }

    // ========================================
    // 곡선 길이 관련 함수들
    // ========================================

    Real Hermite::ArcLength(const Curve& curve)
    {
        return ArcLength(curve, 1.0);
    }

    Real Hermite::ArcLength(const Curve& curve, Real t)
    {
        // 수치적 적분으로 호 길이 계산
        const int segments = 100;
        Real step = t / segments;
        Real length = 0;

        for (int i = 0; i < segments; ++i) {
            Real t1 = i * step;
            Real t2 = (i + 1) * step;

            Vector2 p1 = Point(curve, t1);
            Vector2 p2 = Point(curve, t2);

            length += Length(p2 - p1);
        }

        return length;
    }

    Real Hermite::InverseArcLength(const Curve& curve, Real s)
    {
        Real totalLength = ArcLength(curve);
        if (s >= totalLength) return 1.0;
        if (s <= 0) return 0.0;

        // 이분 탐색
        Real left = 0.0, right = 1.0;
        Real epsilon = 1e-6;
        int maxIterations = 50;

        for (int i = 0; i < maxIterations; ++i) {
            Real mid = (left + right) * 0.5;
            Real currentLength = ArcLength(curve, mid);

            if (std::abs(currentLength - s) < epsilon) {
                return mid;
            }

            if (currentLength < s) {
                left = mid;
            } else {
                right = mid;
            }
        }

        return (left + right) * 0.5;
    }

    // ========================================
    // 충돌 검출을 위한 함수들
    // ========================================

    Real Hermite::DistanceToPoint(const Curve& curve, const Vector2& point)
    {
        Vector2 closestPoint = ClosestPoint(curve, point);
        return Length(point - closestPoint);
    }

    Vector2 Hermite::ClosestPoint(const Curve& curve, const Vector2& point)
    {
        Real t = ClosestParameter(curve, point);
        return Point(curve, t);
    }

    Real Hermite::ClosestParameter(const Curve& curve, const Vector2& point)
    {
        // 뉴턴법으로 최단 거리 매개변수 찾기
        Real t = 0.5; // 초기 추정값
        Real epsilon = 1e-6;
        int maxIterations = 20;

        for (int i = 0; i < maxIterations; ++i) {
            Vector2 curvePoint = Point(curve, t);
            Vector2 derivative = Derivative(curve, t);
            Vector2 secondDerivative = SecondDerivative(curve, t);

            Vector2 diff = curvePoint - point;
            Real derivativeLength = Length(derivative);

            if (derivativeLength < Math::EPSILON) break;

            Real numerator = DotProduct(diff, derivative);
            Real denominator = DotProduct(derivative, derivative) + DotProduct(diff, secondDerivative);

            if (std::abs(denominator) < Math::EPSILON) break;

            Real delta = -numerator / denominator;
            t += delta;

            if (std::abs(delta) < epsilon) break;
        }

        return ClampParameter(t);
    }

    // ========================================
    // Hermite 특화 함수들
    // ========================================

    std::pair<Curve, Curve> Hermite::Split(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) {
            return std::make_pair(Curve(), Curve());
        }

        // Hermite 곡선 분할
        Vector2 p0 = controlPoints[0];
        Vector2 p1 = controlPoints[1];
        Vector2 t0 = controlPoints[2];
        Vector2 t1 = controlPoints[3];

        // 중간점과 접선 계산
        Vector2 midPoint = Point(curve, t);
        Vector2 midTangent = Derivative(curve, t);

        // 왼쪽 곡선: p0 -> midPoint, t0 -> midTangent
        std::vector<Vector2> leftPoints = {p0, midPoint, t0, midTangent};

        // 오른쪽 곡선: midPoint -> p1, midTangent -> t1
        std::vector<Vector2> rightPoints = {midPoint, p1, midTangent, t1};

        return std::make_pair(Curve(leftPoints), Curve(rightPoints));
    }

    Curve Hermite::ElevateDegree(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.size() < 4) return Curve();

        // Hermite는 이미 3차 곡선이므로 차수 승적은 단순히 제어점 추가
        Vector2 p0 = controlPoints[0];
        Vector2 p1 = controlPoints[1];
        Vector2 t0 = controlPoints[2];
        Vector2 t1 = controlPoints[3];

        // 중간점 추가
        Vector2 midPoint = (p0 + p1) * 0.5;
        Vector2 midTangent = (t0 + t1) * 0.5;

        std::vector<Vector2> newControlPoints = {p0, midPoint, p1, t0, midTangent, t1};
        return Curve(newControlPoints);
    }

    std::vector<Vector2> Hermite::GetConvexHull(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        return CalculateConvexHull(controlPoints);
    }

    std::vector<Vector2> Hermite::GetControlPolygon(const Curve& curve)
    {
        return curve.GetControlPoints();
    }

    Curve Hermite::Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale)
    {
        const auto& controlPoints = curve.GetControlPoints();
        std::vector<Vector2> transformedPoints;
        transformedPoints.reserve(controlPoints.size());

        Real cosAngle = std::cos(rotation);
        Real sinAngle = std::sin(rotation);

        for (const auto& point : controlPoints) {
            Vector2 scaled = point * scale;
            Vector2 rotated(
                scaled.x * cosAngle - scaled.y * sinAngle,
                scaled.x * sinAngle + scaled.y * cosAngle
            );
            transformedPoints.push_back(rotated + translation);
        }

        return Curve(transformedPoints);
    }

    // ========================================
    // Hermite 곡선 생성 함수들
    // ========================================

    Curve Hermite::CreateHermiteCurve(const Vector2& p0, const Vector2& p1,
                                     const Vector2& t0, const Vector2& t1)
    {
        std::vector<Vector2> controlPoints = {p0, p1, t0, t1};
        return Curve(controlPoints);
    }

    Curve Hermite::CreateHermiteCurve(const std::vector<Vector2>& controlPoints)
    {
        if (controlPoints.size() < 4) return Curve();
        return Curve(controlPoints);
    }

    Curve Hermite::CreateHermiteCurve(const Vector2& p0, const Vector2& p1,
                                     const Vector2& t0, const Vector2& t1, Real tension)
    {
        // 장력 매개변수를 사용한 Hermite 곡선 생성
        Vector2 adjustedT0 = t0 * tension;
        Vector2 adjustedT1 = t1 * tension;

        std::vector<Vector2> controlPoints = {p0, p1, adjustedT0, adjustedT1};
        return Curve(controlPoints);
    }

    // ========================================
    // Hermite 곡선 계산 함수들 (CurveAlgorithms에서 통합)
    // ========================================

    Vector2 Hermite::CalculateHermitePoint(const Vector2& p0, const Vector2& p1,
                                          const Vector2& t0, const Vector2& t1, Real t)
    {
        Real t2 = t * t;
        Real t3 = t2 * t;

        // Hermite 기저 함수
        Real h0 = 2 * t3 - 3 * t2 + 1;
        Real h1 = -2 * t3 + 3 * t2;
        Real h2 = t3 - 2 * t2 + t;
        Real h3 = t3 - t2;

        return p0 * h0 + p1 * h1 + t0 * h2 + t1 * h3;
    }

    std::vector<Vector2> Hermite::GenerateHermiteCurve(const Vector2& p0, const Vector2& p1,
                                                       const Vector2& t0, const Vector2& t1, int segments)
    {
        std::vector<Vector2> curve;
        curve.reserve(segments + 1);

        for (int i = 0; i <= segments; ++i) {
            Real t = static_cast<Real>(i) / segments;
            curve.push_back(CalculateHermitePoint(p0, p1, t0, t1, t));
        }

        return curve;
    }

    // ========================================
    // 내부 구현 함수들
    // ========================================

    Real Hermite::ClampParameter(Real t)
    {
        return std::max(0.0, std::min(1.0, t));
    }

    Vector2 Hermite::Normalize(const Vector2& v)
    {
        Real length = Length(v);
        if (length < Math::EPSILON) return Vector2(0, 0);
        return v / length;
    }

    Real Hermite::DotProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    Real Hermite::Length(const Vector2& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    std::vector<Vector2> Hermite::CalculateConvexHull(const std::vector<Vector2>& points)
    {
        if (points.size() < 3) return points;

        // Graham scan 알고리즘
        std::vector<Vector2> hull;

        // 가장 아래쪽 점 찾기
        size_t lowest = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].y < points[lowest].y ||
                (points[i].y == points[lowest].y && points[i].x < points[lowest].x)) {
                lowest = i;
            }
        }

        // 각도로 정렬
        std::vector<std::pair<Real, Vector2>> sorted;
        Vector2 pivot = points[lowest];

        for (size_t i = 0; i < points.size(); ++i) {
            if (i == lowest) continue;
            Vector2 diff = points[i] - pivot;
            Real angle = std::atan2(diff.y, diff.x);
            sorted.push_back({angle, points[i]});
        }

        std::sort(sorted.begin(), sorted.end());

        // Graham scan
        hull.push_back(pivot);
        hull.push_back(sorted[0].second);

        for (size_t i = 1; i < sorted.size(); ++i) {
            while (hull.size() > 1 &&
                   CrossProduct(hull[hull.size() - 1] - hull[hull.size() - 2],
                               sorted[i].second - hull[hull.size() - 2]) <= 0) {
                hull.pop_back();
            }
            hull.push_back(sorted[i].second);
        }

        return hull;
    }

    Real Hermite::CrossProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.y - a.y * b.x;
    }
}
