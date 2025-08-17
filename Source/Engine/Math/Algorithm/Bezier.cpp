#include "Bezier.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 베지어 곡선 계산 함수들
    // ========================================

    Vector2 Bezier::Point(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        return DeCasteljau(curve, t);
    }

    Vector2 Bezier::Derivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 2) return Vector2(0, 0);

        int n = static_cast<int>(controlPoints.size()) - 1;
        std::vector<Vector2> derivativePoints;
        derivativePoints.reserve(n);

        // 미분된 제어점들 계산
        for (int i = 0; i < n; ++i) {
            derivativePoints.push_back((controlPoints[i + 1] - controlPoints[i]) * n);
        }

        // 미분된 베지어 곡선 계산
        return CalculateBezierPoint(derivativePoints, t);
    }

    Vector2 Bezier::SecondDerivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.size() < 3) return Vector2(0, 0);

        int n = static_cast<int>(controlPoints.size()) - 1;
        std::vector<Vector2> secondDerivativePoints;
        secondDerivativePoints.reserve(n - 1);

        // 2차 미분된 제어점들 계산
        for (int i = 0; i < n - 1; ++i) {
            Vector2 diff1 = controlPoints[i + 1] - controlPoints[i];
            Vector2 diff2 = controlPoints[i + 2] - controlPoints[i + 1];
            secondDerivativePoints.push_back((diff2 - diff1) * n * (n - 1));
        }

        return CalculateBezierPoint(secondDerivativePoints, t);
    }

    Real Bezier::Curvature(const Curve& curve, Real t)
    {
        Vector2 firstDerivative = Derivative(curve, t);
        Vector2 secondDerivative = SecondDerivative(curve, t);

        Real cross = CrossProduct(firstDerivative, secondDerivative);
        Real firstDerivativeLength = Length(firstDerivative);

        if (firstDerivativeLength < Math::EPSILON) return 0;

        return std::abs(cross) / (firstDerivativeLength * firstDerivativeLength * firstDerivativeLength);
    }

    Vector2 Bezier::CurvatureComb(const Curve& curve, Real t)
    {
        Vector2 point = Point(curve, t);
        Vector2 normal = Normal(curve, t);
        Real curvature = Curvature(curve, t);

        return point + normal * curvature;
    }

    Vector2 Bezier::Tangent(const Curve& curve, Real t)
    {
        Vector2 derivative = Derivative(curve, t);
        return Normalize(derivative);
    }

    Vector2 Bezier::Normal(const Curve& curve, Real t)
    {
        Vector2 tangent = Tangent(curve, t);
        return Vector2(-tangent.y, tangent.x);
    }

    Vector2 Bezier::Binormal(const Curve& curve, Real t)
    {
        // 2D에서는 항상 (0,0,1)
        return Vector2(0, 0);
    }

    // ========================================
    // 곡선 길이 관련 함수들
    // ========================================

    Real Bezier::ArcLength(const Curve& curve)
    {
        return ArcLength(curve, 1.0);
    }

    Real Bezier::ArcLength(const Curve& curve, Real t)
    {
        return CalculateArcLength(curve, t);
    }

    Real Bezier::InverseArcLength(const Curve& curve, Real s)
    {
        return FindParameterByArcLength(curve, s);
    }

    // ========================================
    // 충돌 검출을 위한 함수들
    // ========================================

    Real Bezier::DistanceToPoint(const Curve& curve, const Vector2& point)
    {
        Vector2 closestPoint = ClosestPoint(curve, point);
        return Length(point - closestPoint);
    }

    Vector2 Bezier::ClosestPoint(const Curve& curve, const Vector2& point)
    {
        Real t = ClosestParameter(curve, point);
        return Point(curve, t);
    }

    Real Bezier::ClosestParameter(const Curve& curve, const Vector2& point)
    {
        return FindClosestParameterNewton(curve, point);
    }

    // ========================================
    // 베지어 곡선 특화 함수들
    // ========================================

    std::pair<Curve, Curve> Bezier::Split(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();

        if (controlPoints.empty()) {
            return std::make_pair(Curve(), Curve());
        }

        std::vector<Vector2> leftPoints, rightPoints;
        std::vector<Vector2> tempPoints = controlPoints;
        int n = static_cast<int>(tempPoints.size()) - 1;

        // de Casteljau 알고리즘으로 분할
        for (int i = 0; i <= n; ++i) {
            leftPoints.push_back(tempPoints[0]);
            rightPoints.insert(rightPoints.begin(), tempPoints[n - i]);

            for (int j = 0; j < n - i; ++j) {
                tempPoints[j] = tempPoints[j] * (1 - t) + tempPoints[j + 1] * t;
            }
        }

        return std::make_pair(Curve(leftPoints), Curve(rightPoints));
    }

    Curve Bezier::ElevateDegree(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.empty()) return Curve();

        int n = static_cast<int>(controlPoints.size()) - 1;
        std::vector<Vector2> newControlPoints;
        newControlPoints.reserve(n + 2);

        newControlPoints.push_back(controlPoints[0]);

        for (int i = 1; i <= n; ++i) {
            Real alpha = static_cast<Real>(i) / (n + 1);
            Vector2 newPoint = controlPoints[i - 1] * (1 - alpha) + controlPoints[i] * alpha;
            newControlPoints.push_back(newPoint);
        }

        newControlPoints.push_back(controlPoints[n]);

        return Curve(newControlPoints);
    }

    std::vector<Vector2> Bezier::GetConvexHull(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        return CalculateConvexHull(controlPoints);
    }

    std::vector<Vector2> Bezier::GetControlPolygon(const Curve& curve)
    {
        return curve.GetControlPoints();
    }

    Curve Bezier::Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale)
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
    // 베지어 곡선 생성 함수들
    // ========================================

    Curve Bezier::CreateBezierCurve(const Vector2& p0, const Vector2& p1,
                                   const Vector2& p2, const Vector2& p3)
    {
        std::vector<Vector2> controlPoints = {p0, p1, p2, p3};
        return Curve(controlPoints);
    }

    Curve Bezier::CreateBezierCurve(const std::vector<Vector2>& controlPoints)
    {
        return Curve(controlPoints);
    }

    Curve Bezier::CreateBezierCurve(int degree, const std::vector<Vector2>& controlPoints)
    {
        if (controlPoints.size() != degree + 1) {
            return Curve();
        }
        return Curve(controlPoints);
    }

    // ========================================
    // 베지어 곡선 계산 함수들 (CurveAlgorithms에서 통합)
    // ========================================

    Vector2 Bezier::CalculateBezierPoint(const std::vector<Vector2>& controlPoints, Real t)
    {
        if (controlPoints.empty()) return Vector2(0, 0);
        if (controlPoints.size() == 1) return controlPoints[0];

        std::vector<Vector2> points = controlPoints;
        int n = static_cast<int>(points.size()) - 1;

        // de Casteljau 알고리즘
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n - i; ++j) {
                points[j] = points[j] * (1 - t) + points[j + 1] * t;
            }
        }

        return points[0];
    }

    std::vector<Vector2> Bezier::GenerateBezierCurve(const std::vector<Vector2>& controlPoints, int segments)
    {
        std::vector<Vector2> curve;
        curve.reserve(segments + 1);

        for (int i = 0; i <= segments; ++i) {
            Real t = static_cast<Real>(i) / segments;
            curve.push_back(CalculateBezierPoint(controlPoints, t));
        }

        return curve;
    }

    Vector2 Bezier::CalculateBezierDerivative(const std::vector<Vector2>& controlPoints, Real t)
    {
        if (controlPoints.size() < 2) return Vector2(0, 0);

        int n = static_cast<int>(controlPoints.size()) - 1;
        std::vector<Vector2> derivativePoints;
        derivativePoints.reserve(n);

        // 미분된 제어점들 계산
        for (int i = 0; i < n; ++i) {
            derivativePoints.push_back((controlPoints[i + 1] - controlPoints[i]) * n);
        }

        // 미분된 베지어 곡선 계산
        return CalculateBezierPoint(derivativePoints, t);
    }

    Real Bezier::CalculateBezierCurvature(const std::vector<Vector2>& controlPoints, Real t)
    {
        Vector2 firstDerivative = CalculateBezierDerivative(controlPoints, t);
        Vector2 secondDerivative = CalculateBezierDerivative(GetDerivativePoints(controlPoints), t);

        Real cross = firstDerivative.x * secondDerivative.y - firstDerivative.y * secondDerivative.x;
        Real firstDerivativeLength = firstDerivative.Length();

        if (firstDerivativeLength < Math::EPSILON) return 0;

        return std::abs(cross) / (firstDerivativeLength * firstDerivativeLength * firstDerivativeLength);
    }

    std::vector<Vector2> Bezier::GetDerivativePoints(const std::vector<Vector2>& controlPoints)
    {
        if (controlPoints.size() < 2) return std::vector<Vector2>();

        std::vector<Vector2> derivativePoints;
        derivativePoints.reserve(controlPoints.size() - 1);

        int n = static_cast<int>(controlPoints.size()) - 1;
        for (int i = 0; i < n; ++i) {
            derivativePoints.push_back((controlPoints[i + 1] - controlPoints[i]) * n);
        }

        return derivativePoints;
    }

    // ========================================
    // 베지어 곡선 분석 함수들
    // ========================================

    bool Bezier::IsSmooth(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.size() < 3) return true;

        for (size_t i = 1; i < controlPoints.size() - 1; ++i) {
            Vector2 prev = controlPoints[i] - controlPoints[i - 1];
            Vector2 next = controlPoints[i + 1] - controlPoints[i];

            if (Length(prev) > Math::EPSILON && Length(next) > Math::EPSILON) {
                Real angle = std::acos(DotProduct(Normalize(prev), Normalize(next)));
                if (angle > Math::PI * 0.1) return false; // 18도 이상 꺾이면 부드럽지 않음
            }
        }
        return true;
    }

    bool Bezier::HasSelfIntersection(const Curve& curve)
    {
        // 간단한 구현: 제어점들이 자기 교차하는지 확인
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.size() < 4) return false;

        for (size_t i = 0; i < controlPoints.size() - 3; ++i) {
            for (size_t j = i + 2; j < controlPoints.size() - 1; ++j) {
                if (DoLineSegmentsIntersect(controlPoints[i], controlPoints[i + 1],
                                          controlPoints[j], controlPoints[j + 1])) {
                    return true;
                }
            }
        }
        return false;
    }

    Curve Bezier::Optimize(const Curve& curve, Real tolerance)
    {
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.size() <= 2) return curve;

        std::vector<Vector2> optimizedPoints;
        optimizedPoints.push_back(controlPoints[0]);

        for (size_t i = 1; i < controlPoints.size() - 1; ++i) {
            Vector2 prev = controlPoints[i] - controlPoints[i - 1];
            Vector2 next = controlPoints[i + 1] - controlPoints[i];

            if (Length(prev) > Math::EPSILON && Length(next) > Math::EPSILON) {
                Real angle = std::acos(DotProduct(Normalize(prev), Normalize(next)));
                if (angle > tolerance) {
                    optimizedPoints.push_back(controlPoints[i]);
                }
            }
        }

        optimizedPoints.push_back(controlPoints.back());
        return Curve(optimizedPoints);
    }

    Real Bezier::CalculateComplexity(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        if (controlPoints.size() < 2) return 0;

        Real complexity = 0;
        for (size_t i = 1; i < controlPoints.size(); ++i) {
            Vector2 segment = controlPoints[i] - controlPoints[i - 1];
            complexity += Length(segment);
        }

        return complexity;
    }

    // ========================================
    // 내부 구현 함수들
    // ========================================

    Real Bezier::BernsteinBasis(int i, int n, Real t)
    {
        if (i < 0 || i > n) return 0;

        Real result = 1.0;
        for (int j = 1; j <= i; ++j) {
            result *= (n - j + 1) * t / j;
        }
        for (int j = i + 1; j <= n; ++j) {
            result *= (1 - t);
        }
        return result;
    }

    Real Bezier::BernsteinBasisDerivative(int i, int n, Real t)
    {
        if (i < 0 || i > n) return 0;

        if (n == 0) return 0;

        Real term1 = (i > 0) ? BernsteinBasis(i - 1, n - 1, t) : 0;
        Real term2 = (i < n) ? BernsteinBasis(i, n - 1, t) : 0;

        return n * (term1 - term2);
    }

    Real Bezier::BernsteinBasisSecondDerivative(int i, int n, Real t)
    {
        if (i < 0 || i > n) return 0;

        if (n <= 1) return 0;

        Real term1 = (i > 1) ? BernsteinBasis(i - 2, n - 2, t) : 0;
        Real term2 = (i > 0 && i < n) ? BernsteinBasis(i - 1, n - 2, t) : 0;
        Real term3 = (i < n - 1) ? BernsteinBasis(i, n - 2, t) : 0;

        return n * (n - 1) * (term1 - 2 * term2 + term3);
    }

    Vector2 Bezier::DeCasteljau(const Curve& curve, Real t)
    {
        const auto& controlPoints = curve.GetControlPoints();
        return CalculateBezierPoint(controlPoints, t);
    }

    Real Bezier::CalculateArcLength(const Curve& curve, Real t)
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

    Real Bezier::FindParameterByArcLength(const Curve& curve, Real targetLength)
    {
        Real totalLength = ArcLength(curve);
        if (targetLength >= totalLength) return 1.0;
        if (targetLength <= 0) return 0.0;

        // 이분 탐색
        Real left = 0.0, right = 1.0;
        Real epsilon = 1e-6;
        int maxIterations = 50;

        for (int i = 0; i < maxIterations; ++i) {
            Real mid = (left + right) * 0.5;
            Real currentLength = ArcLength(curve, mid);

            if (std::abs(currentLength - targetLength) < epsilon) {
                return mid;
            }

            if (currentLength < targetLength) {
                left = mid;
            } else {
                right = mid;
            }
        }

        return (left + right) * 0.5;
    }

    Real Bezier::FindClosestParameterNewton(const Curve& curve, const Vector2& point, Real initialGuess)
    {
        Real t = ClampParameter(initialGuess);
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

    std::vector<Vector2> Bezier::CalculateConvexHull(const std::vector<Vector2>& points)
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

    Real Bezier::ClampParameter(Real t)
    {
        return std::max(0.0, std::min(1.0, t));
    }

    Vector2 Bezier::Normalize(const Vector2& v)
    {
        Real length = Length(v);
        if (length < Math::EPSILON) return Vector2(0, 0);
        return v / length;
    }

    Real Bezier::CrossProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    Real Bezier::DotProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    Real Bezier::Length(const Vector2& v)
    {
        return std::sqrt(LengthSquared(v));
    }

    Real Bezier::LengthSquared(const Vector2& v)
    {
        return v.x * v.x + v.y * v.y;
    }

    bool Bezier::DoLineSegmentsIntersect(const Vector2& p1, const Vector2& q1,
                                        const Vector2& p2, const Vector2& q2)
    {
        Vector2 r = q1 - p1;
        Vector2 s = q2 - p2;
        Vector2 pq = p2 - p1;

        Real crossRS = CrossProduct(r, s);
        Real crossPQ = CrossProduct(pq, s);
        Real crossR = CrossProduct(pq, r);

        if (std::abs(crossRS) < Math::EPSILON) {
            // 평행한 경우
            if (std::abs(crossPQ) < Math::EPSILON) {
                // 같은 직선 위에 있는 경우
                Real t0 = DotProduct(pq, r) / DotProduct(r, r);
                Real t1 = t0 + DotProduct(s, r) / DotProduct(r, r);
                return (t0 <= 1 && t1 >= 0) || (t1 <= 1 && t0 >= 0);
            }
            return false;
        }

        Real t = crossPQ / crossRS;
        Real u = crossR / crossRS;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }
}
