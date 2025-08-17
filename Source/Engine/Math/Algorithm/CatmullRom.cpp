#include "CatmullRom.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // Catmull-Rom 곡선 계산 함수들
    // ========================================

    Vector2 CatmullRom::Point(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) return Vector2(0, 0);

        // 매개변수 t를 [0, 1] 범위로 정규화
        int segment = static_cast<int>(t * (controlPoints.size() - 3));
        Real localT = t * (controlPoints.size() - 3) - segment;

        // 4개의 제어점 선택
        Vector2 p0 = controlPoints[std::max(0, segment - 1)];
        Vector2 p1 = controlPoints[segment];
        Vector2 p2 = controlPoints[segment + 1];
        Vector2 p3 = controlPoints[std::min(static_cast<int>(controlPoints.size()) - 1, segment + 2)];

        return CalculateCatmullRomPoint({p0, p1, p2, p3}, localT);
    }

    Vector2 CatmullRom::Derivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) return Vector2(0, 0);

        // 매개변수 t를 [0, 1] 범위로 정규화
        int segment = static_cast<int>(t * (controlPoints.size() - 3));
        Real localT = t * (controlPoints.size() - 3) - segment;

        // 4개의 제어점 선택
        Vector2 p0 = controlPoints[std::max(0, segment - 1)];
        Vector2 p1 = controlPoints[segment];
        Vector2 p2 = controlPoints[segment + 1];
        Vector2 p3 = controlPoints[std::min(static_cast<int>(controlPoints.size()) - 1, segment + 2)];

        // Catmull-Rom 미분 계산
        Real t2 = localT * localT;

        Real c0 = -1.5f * t2 + 2 * localT - 0.5f;
        Real c1 = 4.5f * t2 - 5 * localT;
        Real c2 = -4.5f * t2 + 4 * localT + 0.5f;
        Real c3 = 1.5f * t2 - localT;

        return p0 * c0 + p1 * c1 + p2 * c2 + p3 * c3;
    }

    Vector2 CatmullRom::SecondDerivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 4) return Vector2(0, 0);

        // 매개변수 t를 [0, 1] 범위로 정규화
        int segment = static_cast<int>(t * (controlPoints.size() - 3));
        Real localT = t * (controlPoints.size() - 3) - segment;

        // 4개의 제어점 선택
        Vector2 p0 = controlPoints[std::max(0, segment - 1)];
        Vector2 p1 = controlPoints[segment];
        Vector2 p2 = controlPoints[segment + 1];
        Vector2 p3 = controlPoints[std::min(static_cast<int>(controlPoints.size()) - 1, segment + 2)];

        // Catmull-Rom 2차 미분 계산
        Real c0 = -3 * localT + 2;
        Real c1 = 9 * localT - 5;
        Real c2 = -9 * localT + 4;
        Real c3 = 3 * localT - 1;

        return p0 * c0 + p1 * c1 + p2 * c2 + p3 * c3;
    }

    Real CatmullRom::Curvature(const Curve& curve, Real t)
    {
        Vector2 firstDerivative = Derivative(curve, t);
        Vector2 secondDerivative = SecondDerivative(curve, t);

        Real cross = firstDerivative.x * secondDerivative.y - firstDerivative.y * secondDerivative.x;
        Real firstDerivativeLength = firstDerivative.Length();

        if (firstDerivativeLength < Math::EPSILON) return 0;

        return std::abs(cross) / (firstDerivativeLength * firstDerivativeLength * firstDerivativeLength);
    }

    Vector2 CatmullRom::CurvatureComb(const Curve& curve, Real t)
    {
        Vector2 point = Point(curve, t);
        Vector2 normal = Normal(curve, t);
        Real curvature = Curvature(curve, t);

        return point + normal * curvature;
    }

    Vector2 CatmullRom::Tangent(const Curve& curve, Real t)
    {
        Vector2 derivative = Derivative(curve, t);
        return Normalize(derivative);
    }

    Vector2 CatmullRom::Normal(const Curve& curve, Real t)
    {
        Vector2 tangent = Tangent(curve, t);
        return Vector2(-tangent.y, tangent.x);
    }

    Vector2 CatmullRom::Binormal(const Curve& curve, Real t)
    {
        // 2D에서는 항상 (0,0,1)
        return Vector2(0, 0);
    }

    // ========================================
    // 곡선 길이 관련 함수들
    // ========================================

    Real CatmullRom::ArcLength(const Curve& curve)
    {
        return ArcLength(curve, 1.0);
    }

    Real CatmullRom::ArcLength(const Curve& curve, Real t)
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

    Real CatmullRom::InverseArcLength(const Curve& curve, Real s)
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

    Real CatmullRom::DistanceToPoint(const Curve& curve, const Vector2& point)
    {
        Vector2 closestPoint = ClosestPoint(curve, point);
        return Length(point - closestPoint);
    }

    Vector2 CatmullRom::ClosestPoint(const Curve& curve, const Vector2& point)
    {
        Real t = ClosestParameter(curve, point);
        return Point(curve, t);
    }

    Real CatmullRom::ClosestParameter(const Curve& curve, const Vector2& point)
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
    // Catmull-Rom 특화 함수들
    // ========================================

    std::pair<Curve, Curve> CatmullRom::Split(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.empty()) {
            return std::make_pair(Curve(), Curve());
        }

        // 중간점에서 분할
        size_t midPoint = static_cast<size_t>(t * (controlPoints.size() - 1));

        std::vector<Vector2> leftControlPoints(controlPoints.begin(), controlPoints.begin() + midPoint + 1);
        std::vector<Vector2> rightControlPoints(controlPoints.begin() + midPoint, controlPoints.end());

        return std::make_pair(Curve(leftControlPoints), Curve(rightControlPoints));
    }

    Curve CatmullRom::ElevateDegree(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.empty()) return Curve();

        // Catmull-Rom은 이미 3차 곡선이므로 차수 승적은 단순히 제어점 추가
        std::vector<Vector2> newControlPoints;
        newControlPoints.reserve(controlPoints.size() + 1);

        newControlPoints.push_back(controlPoints[0]);

        for (size_t i = 1; i < controlPoints.size(); ++i) {
            Vector2 midPoint = (controlPoints[i - 1] + controlPoints[i]) * 0.5;
            newControlPoints.push_back(midPoint);
            newControlPoints.push_back(controlPoints[i]);
        }

        return Curve(newControlPoints);
    }

    std::vector<Vector2> CatmullRom::GetConvexHull(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        return CalculateConvexHull(controlPoints);
    }

    std::vector<Vector2> CatmullRom::GetControlPolygon(const Curve& curve)
    {
        return curve.GetControlPoints();
    }

    Curve CatmullRom::Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale)
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
    // Catmull-Rom 곡선 생성 함수들
    // ========================================

    Curve CatmullRom::CreateCatmullRomCurve(const Vector2& p0, const Vector2& p1,
                                           const Vector2& p2, const Vector2& p3)
    {
        std::vector<Vector2> controlPoints = {p0, p1, p2, p3};
        return Curve(controlPoints);
    }

    Curve CatmullRom::CreateCatmullRomCurve(const std::vector<Vector2>& controlPoints)
    {
        return Curve(controlPoints);
    }

    Curve CatmullRom::CreateCatmullRomCurve(const std::vector<Vector2>& controlPoints, Real tension)
    {
        // 장력 매개변수를 사용한 Catmull-Rom 곡선 생성
        if (controlPoints.size() < 4) return Curve();

        std::vector<Vector2> adjustedPoints;
        adjustedPoints.reserve(controlPoints.size());

        // 첫 번째 점 추가 (장력 적용)
        adjustedPoints.push_back(controlPoints[0]);

        // 중간 점들에 장력 적용
        for (size_t i = 1; i < controlPoints.size() - 1; ++i) {
            Vector2 prev = controlPoints[i - 1];
            Vector2 curr = controlPoints[i];
            Vector2 next = controlPoints[i + 1];

            // 장력에 따른 조정된 점 계산
            Vector2 adjusted = curr + (prev + next - 2 * curr) * tension * 0.5;
            adjustedPoints.push_back(adjusted);
        }

        // 마지막 점 추가
        adjustedPoints.push_back(controlPoints.back());

        return Curve(adjustedPoints);
    }

    // ========================================
    // Catmull-Rom 곡선 계산 함수들 (CurveAlgorithms에서 통합)
    // ========================================

    Vector2 CatmullRom::CalculateCatmullRomPoint(const std::vector<Vector2>& points, Real t)
    {
        if (points.size() < 4) return Vector2(0, 0);

        // 4개의 제어점 선택
        Vector2 p0 = points[0];
        Vector2 p1 = points[1];
        Vector2 p2 = points[2];
        Vector2 p3 = points[3];

        // Catmull-Rom 행렬 계수
        Real t2 = t * t;
        Real t3 = t2 * t;

        Real c0 = -0.5f * t3 + t2 - 0.5f * t;
        Real c1 = 1.5f * t3 - 2.5f * t2 + 1.0f;
        Real c2 = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
        Real c3 = 0.5f * t3 - 0.5f * t2;

        return p0 * c0 + p1 * c1 + p2 * c2 + p3 * c3;
    }

    std::vector<Vector2> CatmullRom::GenerateCatmullRomCurve(const std::vector<Vector2>& points, int segments)
    {
        std::vector<Vector2> curve;
        curve.reserve(segments + 1);

        for (int i = 0; i <= segments; ++i) {
            Real t = static_cast<Real>(i) / segments;
            curve.push_back(CalculateCatmullRomPoint(points, t));
        }

        return curve;
    }

    // ========================================
    // 내부 구현 함수들
    // ========================================

    Real CatmullRom::ClampParameter(Real t)
    {
        return std::max(0.0, std::min(1.0, t));
    }

    Vector2 CatmullRom::Normalize(const Vector2& v)
    {
        Real length = Length(v);
        if (length < Math::EPSILON) return Vector2(0, 0);
        return v / length;
    }

    Real CatmullRom::DotProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    Real CatmullRom::Length(const Vector2& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    std::vector<Vector2> CatmullRom::CalculateConvexHull(const std::vector<Vector2>& points)
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

    Real CatmullRom::CrossProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.y - a.y * b.x;
    }
}
