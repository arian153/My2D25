#include "NURBS.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // NURBS 곡선 계산 함수들
    // ========================================

    Vector2 NURBS::Point(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.empty() || controlPoints.size() != weights.size()) {
            return Vector2(0, 0);
        }

        return CalculateNURBSPoint(controlPoints, weights, t);
    }

    Vector2 NURBS::Derivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.size() < 2 || controlPoints.size() != weights.size()) {
            return Vector2(0, 0);
        }

        // NURBS 미분 계산
        int degree = 3; // 3차 B-스플라인
        std::vector<Real> knots = GenerateUniformKnots(controlPoints.size(), degree);

        Vector2 numerator(0, 0);
        Real denominator = 0;

        for (size_t i = 0; i < controlPoints.size(); ++i) {
            Real basis = CalculateBSplineBasis(i, degree, knots, t);
            Real basisDerivative = CalculateBSplineBasisDerivative(i, degree, knots, t);
            Real weight = weights[i];

            numerator += controlPoints[i] * (basisDerivative * weight);
            denominator += basisDerivative * weight;
        }

        if (std::abs(denominator) < Math::EPSILON) return Vector2(0, 0);
        return numerator / denominator;
    }

    Vector2 NURBS::SecondDerivative(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.size() < 3 || controlPoints.size() != weights.size()) {
            return Vector2(0, 0);
        }

        // NURBS 2차 미분 계산
        int degree = 3;
        std::vector<Real> knots = GenerateUniformKnots(controlPoints.size(), degree);

        Vector2 numerator(0, 0);
        Real denominator = 0;

        for (size_t i = 0; i < controlPoints.size(); ++i) {
            Real basis = CalculateBSplineBasis(i, degree, knots, t);
            Real basisSecondDerivative = CalculateBSplineBasisSecondDerivative(i, degree, knots, t);
            Real weight = weights[i];

            numerator += controlPoints[i] * (basisSecondDerivative * weight);
            denominator += basisSecondDerivative * weight;
        }

        if (std::abs(denominator) < Math::EPSILON) return Vector2(0, 0);
        return numerator / denominator;
    }

    Real NURBS::Curvature(const Curve& curve, Real t)
    {
        Vector2 firstDerivative = Derivative(curve, t);
        Vector2 secondDerivative = SecondDerivative(curve, t);

        Real cross = firstDerivative.x * secondDerivative.y - firstDerivative.y * secondDerivative.x;
        Real firstDerivativeLength = firstDerivative.Length();

        if (firstDerivativeLength < Math::EPSILON) return 0;

        return std::abs(cross) / (firstDerivativeLength * firstDerivativeLength * firstDerivativeLength);
    }

    Vector2 NURBS::CurvatureComb(const Curve& curve, Real t)
    {
        Vector2 point = Point(curve, t);
        Vector2 normal = Normal(curve, t);
        Real curvature = Curvature(curve, t);

        return point + normal * curvature;
    }

    Vector2 NURBS::Tangent(const Curve& curve, Real t)
    {
        Vector2 derivative = Derivative(curve, t);
        return Normalize(derivative);
    }

    Vector2 NURBS::Normal(const Curve& curve, Real t)
    {
        Vector2 tangent = Tangent(curve, t);
        return Vector2(-tangent.y, tangent.x);
    }

    Vector2 NURBS::Binormal(const Curve& curve, Real t)
    {
        // 2D에서는 항상 (0,0,1)
        return Vector2(0, 0);
    }

    // ========================================
    // 곡선 길이 관련 함수들
    // ========================================

    Real NURBS::ArcLength(const Curve& curve)
    {
        return ArcLength(curve, 1.0);
    }

    Real NURBS::ArcLength(const Curve& curve, Real t)
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

    Real NURBS::InverseArcLength(const Curve& curve, Real s)
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

    Real NURBS::DistanceToPoint(const Curve& curve, const Vector2& point)
    {
        Vector2 closestPoint = ClosestPoint(curve, point);
        return Length(point - closestPoint);
    }

    Vector2 NURBS::ClosestPoint(const Curve& curve, const Vector2& point)
    {
        Real t = ClosestParameter(curve, point);
        return Point(curve, t);
    }

    Real NURBS::ClosestParameter(const Curve& curve, const Vector2& point)
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
    // NURBS 특화 함수들
    // ========================================

    std::pair<Curve, Curve> NURBS::Split(const Curve& curve, Real t)
    {
        t = ClampParameter(t);
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.empty() || controlPoints.size() != weights.size()) {
            return std::make_pair(Curve(), Curve());
        }

        // Knot 삽입을 통한 분할
        Curve insertedCurve = InsertKnot(curve, t);

        // 중간점에서 분할
        size_t midPoint = controlPoints.size() / 2;

        std::vector<Vector2> leftControlPoints(controlPoints.begin(), controlPoints.begin() + midPoint + 1);
        std::vector<Vector2> rightControlPoints(controlPoints.begin() + midPoint, controlPoints.end());

        std::vector<Real> leftWeights(weights.begin(), weights.begin() + midPoint + 1);
        std::vector<Real> rightWeights(weights.begin() + midPoint, weights.end());

        return std::make_pair(Curve(leftControlPoints, leftWeights), Curve(rightControlPoints, rightWeights));
    }

    Curve NURBS::ElevateDegree(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.empty() || controlPoints.size() != weights.size()) {
            return Curve();
        }

        // NURBS 차수 승적
        int n = static_cast<int>(controlPoints.size()) - 1;
        std::vector<Vector2> newControlPoints;
        std::vector<Real> newWeights;

        newControlPoints.reserve(n + 2);
        newWeights.reserve(n + 2);

        newControlPoints.push_back(controlPoints[0]);
        newWeights.push_back(weights[0]);

        for (int i = 1; i <= n; ++i) {
            Real alpha = static_cast<Real>(i) / (n + 1);
            Vector2 newPoint = controlPoints[i - 1] * (1 - alpha) + controlPoints[i] * alpha;
            Real newWeight = weights[i - 1] * (1 - alpha) + weights[i] * alpha;

            newControlPoints.push_back(newPoint);
            newWeights.push_back(newWeight);
        }

        newControlPoints.push_back(controlPoints[n]);
        newWeights.push_back(weights[n]);

        return Curve(newControlPoints, newWeights);
    }

    Curve NURBS::Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale)
    {
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

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

        return Curve(transformedPoints, weights);
    }

    Curve NURBS::InsertKnot(const Curve& curve, Real u, int multiplicity)
    {
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.empty() || controlPoints.size() != weights.size()) {
            return Curve();
        }

        // Knot 삽입 알고리즘
        std::vector<Vector2> newControlPoints = controlPoints;
        std::vector<Real> newWeights = weights;

        // 단순화: 제어점에 새 점 추가
        size_t insertIndex = static_cast<size_t>(u * (controlPoints.size() - 1));
        if (insertIndex < controlPoints.size() - 1) {
            Vector2 newPoint = controlPoints[insertIndex] * (1 - u) + controlPoints[insertIndex + 1] * u;
            Real newWeight = weights[insertIndex] * (1 - u) + weights[insertIndex + 1] * u;

            newControlPoints.insert(newControlPoints.begin() + insertIndex + 1, newPoint);
            newWeights.insert(newWeights.begin() + insertIndex + 1, newWeight);
        }

        return Curve(newControlPoints, newWeights);
    }

    Curve NURBS::RemoveKnot(const Curve& curve, Real u, int multiplicity)
    {
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.size() <= 2) return curve;

        // Knot 제거 알고리즘 (단순화)
        std::vector<Vector2> newControlPoints = controlPoints;
        std::vector<Real> newWeights = weights;

        // 단순화: 중간 제어점 제거
        if (controlPoints.size() > 2) {
            size_t removeIndex = controlPoints.size() / 2;
            newControlPoints.erase(newControlPoints.begin() + removeIndex);
            newWeights.erase(newWeights.begin() + removeIndex);
        }

        return Curve(newControlPoints, newWeights);
    }

    Curve NURBS::NormalizeKnots(const Curve& curve)
    {
        const auto& controlPoints = curve.GetControlPoints();
        const auto& weights = curve.GetWeights();

        if (controlPoints.empty() || controlPoints.size() != weights.size()) {
            return Curve();
        }

        // 가중치 정규화
        std::vector<Real> normalizedWeights = weights;
        Real sum = 0;
        for (Real weight : weights) {
            sum += weight;
        }

        if (sum > Math::EPSILON) {
            for (Real& weight : normalizedWeights) {
                weight /= sum;
            }
        }

        return Curve(controlPoints, normalizedWeights);
    }

    // ========================================
    // NURBS 곡선 생성 함수들
    // ========================================

    Curve NURBS::CreateNURBSCurve(const Vector2& p0, const Vector2& p1,
                                 const Vector2& p2, const Vector2& p3)
    {
        std::vector<Vector2> controlPoints = {p0, p1, p2, p3};
        std::vector<Real> weights = {1.0, 1.0, 1.0, 1.0}; // 균등 가중치
        return Curve(controlPoints, weights);
    }

    Curve NURBS::CreateNURBSCurve(const std::vector<Vector2>& controlPoints)
    {
        std::vector<Real> weights(controlPoints.size(), 1.0); // 균등 가중치
        return Curve(controlPoints, weights);
    }

    Curve NURBS::CreateNURBSCurve(const std::vector<Vector2>& controlPoints, const std::vector<Real>& weights)
    {
        if (controlPoints.size() != weights.size()) {
            return Curve();
        }
        return Curve(controlPoints, weights);
    }

    // ========================================
    // NURBS 곡선 계산 함수들 (CurveAlgorithms에서 통합)
    // ========================================

    Vector2 NURBS::CalculateNURBSPoint(const std::vector<Vector2>& controlPoints,
                                      const std::vector<Real>& weights, Real t)
    {
        if (controlPoints.empty() || controlPoints.size() != weights.size()) {
            return Vector2(0, 0);
        }

        // 단순화된 NURBS 구현 (균등 매듭 벡터 사용)
        int degree = 3; // 3차 B-스플라인
        std::vector<Real> knots = GenerateUniformKnots(controlPoints.size(), degree);

        Vector2 numerator(0, 0);
        Real denominator = 0;

        for (size_t i = 0; i < controlPoints.size(); ++i) {
            Real basis = CalculateBSplineBasis(i, degree, knots, t);
            Real weight = weights[i];
            numerator += controlPoints[i] * (basis * weight);
            denominator += basis * weight;
        }

        if (std::abs(denominator) < Math::EPSILON) return Vector2(0, 0);
        return numerator / denominator;
    }

    std::vector<Vector2> NURBS::GenerateNURBSCurve(const std::vector<Vector2>& controlPoints,
                                                  const std::vector<Real>& weights, int segments)
    {
        std::vector<Vector2> curve;
        curve.reserve(segments + 1);

        for (int i = 0; i <= segments; ++i) {
            Real t = static_cast<Real>(i) / segments;
            curve.push_back(CalculateNURBSPoint(controlPoints, weights, t));
        }

        return curve;
    }

    std::vector<Real> NURBS::GenerateUniformKnots(int numControlPoints, int degree)
    {
        std::vector<Real> knots;
        int numKnots = numControlPoints + degree + 1;

        // 시작 매듭들 (degree + 1개)
        for (int i = 0; i <= degree; ++i) {
            knots.push_back(0);
        }

        // 중간 매듭들
        for (int i = degree + 1; i < numKnots - degree - 1; ++i) {
            knots.push_back(static_cast<Real>(i - degree) / (numKnots - 2 * degree - 1));
        }

        // 끝 매듭들 (degree + 1개)
        for (int i = 0; i <= degree; ++i) {
            knots.push_back(1);
        }

        return knots;
    }

    Real NURBS::CalculateBSplineBasis(int i, int k, const std::vector<Real>& knots, Real t)
    {
        if (k == 1) {
            return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
        }

        Real d1 = knots[i + k - 1] - knots[i];
        Real d2 = knots[i + k] - knots[i + 1];

        Real c1 = (d1 > Math::EPSILON) ? (t - knots[i]) / d1 : 0;
        Real c2 = (d2 > Math::EPSILON) ? (knots[i + k] - t) / d2 : 0;

        return c1 * CalculateBSplineBasis(i, k - 1, knots, t) +
               c2 * CalculateBSplineBasis(i + 1, k - 1, knots, t);
    }

    Real NURBS::CalculateBSplineBasisDerivative(int i, int k, const std::vector<Real>& knots, Real t)
    {
        if (k == 1) return 0.0;

        Real d1 = knots[i + k - 1] - knots[i];
        Real d2 = knots[i + k] - knots[i + 1];

        Real c1 = (d1 > Math::EPSILON) ? (k - 1) / d1 : 0;
        Real c2 = (d2 > Math::EPSILON) ? (k - 1) / d2 : 0;

        return c1 * CalculateBSplineBasis(i, k - 1, knots, t) -
               c2 * CalculateBSplineBasis(i + 1, k - 1, knots, t);
    }

    Real NURBS::CalculateBSplineBasisSecondDerivative(int i, int k, const std::vector<Real>& knots, Real t)
    {
        if (k <= 2) return 0.0;

        Real d1 = knots[i + k - 1] - knots[i];
        Real d2 = knots[i + k] - knots[i + 1];

        Real c1 = (d1 > Math::EPSILON) ? (k - 1) * (k - 2) / (d1 * d1) : 0;
        Real c2 = (d2 > Math::EPSILON) ? (k - 1) * (k - 2) / (d2 * d2) : 0;

        return c1 * CalculateBSplineBasis(i, k - 2, knots, t) +
               c2 * CalculateBSplineBasis(i + 1, k - 2, knots, t);
    }

    // ========================================
    // 내부 구현 함수들
    // ========================================

    Real NURBS::ClampParameter(Real t)
    {
        return std::max(0.0, std::min(1.0, t));
    }

    Vector2 NURBS::Normalize(const Vector2& v)
    {
        Real length = Length(v);
        if (length < Math::EPSILON) return Vector2(0, 0);
        return v / length;
    }

    Real NURBS::DotProduct(const Vector2& a, const Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    Real NURBS::Length(const Vector2& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }
}
