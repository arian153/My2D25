#include "Ellipse.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 생성자
    // ========================================

    Ellipse::Ellipse() : center(0, 0), radii(1, 1), rotation(0) {}

    Ellipse::Ellipse(const Vector2& center, const Vector2& radii) : center(center), radii(radii), rotation(0) {}

    Ellipse::Ellipse(const Vector2& center, const Vector2& radii, Real rotation) : center(center), radii(radii), rotation(rotation) {}

    Ellipse::Ellipse(Real x, Real y, Real radiusX, Real radiusY) : center(x, y), radii(radiusX, radiusY), rotation(0) {}

    // ========================================
    // Static 함수들 (핵심 알고리즘)
    // ========================================

    // 1. Support Function - GJK/EPA용
    Vector2 Ellipse::SupportFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& direction)
    {
        if (direction.LengthSquared() < 1e-12) {
            return center + Vector2(radii.x, 0);
        }

        // 회전된 방향 벡터 계산
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);
        Vector2 rotatedDir(
            direction.x * cosRot + direction.y * sinRot,
            -direction.x * sinRot + direction.y * cosRot
        );

        // 타원의 서포트 점 계산
        Real denom = (rotatedDir.x * rotatedDir.x) / (radii.x * radii.x) +
                    (rotatedDir.y * rotatedDir.y) / (radii.y * radii.y);

        if (denom < 1e-12) {
            return center + Vector2(radii.x, 0);
        }

        Real scale = std::sqrt(denom);
        Vector2 support(
            rotatedDir.x / scale,
            rotatedDir.y / scale
        );

        // 원래 좌표계로 변환
        Vector2 result(
            support.x * cosRot - support.y * sinRot,
            support.x * sinRot + support.y * cosRot
        );

        return center + result;
    }

    // 2. Area Function - 면적 계산
    Real Ellipse::AreaFunction(const Vector2& radii)
    {
        return PI * radii.x * radii.y;
    }

    // 3. Perimeter Function - 둘레 계산 (근사)
    Real Ellipse::PerimeterFunction(const Vector2& radii)
    {
        // Ramanujan 근사 공식
        Real a = std::max(radii.x, radii.y);
        Real b = std::min(radii.x, radii.y);
        Real h = ((a - b) * (a - b)) / ((a + b) * (a + b));

        return PI * (a + b) * (1 + (3 * h) / (10 + std::sqrt(4 - 3 * h)));
    }

    // 4. SDF Function - 부호가 있는 거리 함수
    SDFResult Ellipse::SDFFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point)
    {
        SDFResult result;

        // 점을 타원의 로컬 좌표계로 변환
        Vector2 localPoint = point - center;

        // 회전 변환
        Real cosRot = std::cos(-rotation);
        Real sinRot = std::sin(-rotation);
        Vector2 rotatedPoint(
            localPoint.x * cosRot - localPoint.y * sinRot,
            localPoint.x * sinRot + localPoint.y * cosRot
        );

        // 타원의 정규화된 좌표
        Vector2 normalizedPoint = rotatedPoint / radii;

        // 타원의 SDF 계산
        Real k0 = normalizedPoint.Length();
        Real k1 = normalizedPoint.Length();

        if (k1 < 1e-6) {
            result.distance = -std::min(radii.x, radii.y);
            result.isInside = true;
            result.closestPoint = center;
            result.gradient = Vector2(1, 0);
            return result;
        }

        Real k2 = k0 * k0;
        Real k3 = k2 * k0;

        // 반복적 근사
        for (int i = 0; i < 4; ++i) {
            k1 = k0;
            k0 = k2 / k1;
            k2 = k0 * k0;
        }

        Real distance = (k0 - 1) * std::min(radii.x, radii.y);
        result.distance = distance;
        result.isInside = distance < 0;

        // 가장 가까운 점 계산
        Vector2 closestNormalized = normalizedPoint / k0;
        Vector2 closestRotated = closestNormalized * radii;
        Vector2 closestPoint(
            closestRotated.x * cosRot + closestRotated.y * sinRot,
            -closestRotated.x * sinRot + closestRotated.y * cosRot
        );
        result.closestPoint = center + closestPoint;

        // 그래디언트 계산
        if (std::abs(distance) > 1e-6) {
            result.gradient = (point - result.closestPoint).Normalized();
        } else {
            result.gradient = Vector2(1, 0);
        }

        return result;
    }

    // 5. Moment of Inertia Function - 관성 모멘트
    Real Ellipse::MomentOfInertiaFunction(const Vector2& radii)
    {
        return (radii.x * radii.x + radii.y * radii.y) / Real(4);
    }

    // 6. Curvature Function - 곡률 계산
    Real Ellipse::CurvatureFunction(const Vector2& radii, Real angle)
    {
        Real a = radii.x;
        Real b = radii.y;
        Real cosAngle = std::cos(angle);
        Real sinAngle = std::sin(angle);

        Real numerator = a * b;
        Real denominator = std::pow(b * cosAngle, 2) + std::pow(a * sinAngle, 2);
        denominator = std::pow(denominator, 1.5);

        return numerator / denominator;
    }

    // 7. Point Containment Function - 점 포함 확인
    bool Ellipse::ContainsPointFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point)
    {
        // 점을 타원의 로컬 좌표계로 변환
        Vector2 localPoint = point - center;

        // 회전 변환
        Real cosRot = std::cos(-rotation);
        Real sinRot = std::sin(-rotation);
        Vector2 rotatedPoint(
            localPoint.x * cosRot - localPoint.y * sinRot,
            localPoint.x * sinRot + localPoint.y * cosRot
        );

        // 타원 방정식 검사
        Real normalizedX = rotatedPoint.x / radii.x;
        Real normalizedY = rotatedPoint.y / radii.y;

        return (normalizedX * normalizedX + normalizedY * normalizedY) <= 1;
    }

    // 8. Distance to Point Function - 점까지의 거리
    Real Ellipse::DistanceToPointFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point)
    {
        SDFResult sdf = SDFFunction(center, radii, rotation, point);
        return std::abs(sdf.distance);
    }

    // 9. Closest Point Function - 가장 가까운 점
    Vector2 Ellipse::ClosestPointFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point)
    {
        SDFResult sdf = SDFFunction(center, radii, rotation, point);
        return sdf.closestPoint;
    }

    // ========================================
    // IShape 인터페이스 구현
    // ========================================

    ShapeType Ellipse::GetType() const
    {
        return ShapeType::ELLIPSE;
    }

    Vector2 Ellipse::GetCenter() const
    {
        return center;
    }

    Real Ellipse::GetBoundingRadius() const
    {
        return std::max(radii.x, radii.y);
    }

    // SAT를 위한 축 투영
    std::vector<Vector2> Ellipse::GetAxes() const
    {
        std::vector<Vector2> axes;
        axes.reserve(2);

        // 회전된 축들
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);

        axes.push_back(Vector2(cosRot, sinRot));      // 장축
        axes.push_back(Vector2(-sinRot, cosRot));     // 단축

        return axes;
    }

    std::pair<Real, Real> Ellipse::ProjectOnAxis(const Vector2& axis) const
    {
        if (axis.LengthSquared() < 1e-12) {
            return {0, 0};
        }

        // 회전된 축 벡터 계산
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);
        Vector2 rotatedAxis(
            axis.x * cosRot + axis.y * sinRot,
            -axis.x * sinRot + axis.y * cosRot
        );

        // 타원의 투영 계산
        Real centerProj = center.Dot(axis);
        Real radiusProj = std::sqrt(
            (rotatedAxis.x * rotatedAxis.x * radii.x * radii.x) +
            (rotatedAxis.y * rotatedAxis.y * radii.y * radii.y)
        );

        return {centerProj - radiusProj, centerProj + radiusProj};
    }

    // SDF를 위한 거리 함수
    SDFResult Ellipse::GetSDF(const Vector2& point) const
    {
        return SDFFunction(center, radii, rotation, point);
    }

    // GJK를 위한 지지점 함수
    Vector2 Ellipse::GetSupportPoint(const Vector2& direction) const
    {
        return SupportFunction(center, radii, rotation, direction);
    }

    // ========================================
    // 기본 멤버 함수들 (Static 함수 사용)
    // ========================================

    // 면적 계산
    Real Ellipse::GetArea() const
    {
        return AreaFunction(radii);
    }

    // 둘레 계산
    Real Ellipse::GetPerimeter() const
    {
        return PerimeterFunction(radii);
    }

    // 관성 모멘트
    Real Ellipse::GetMomentOfInertia() const
    {
        return MomentOfInertiaFunction(radii);
    }

    // 곡률
    Real Ellipse::GetCurvature(Real angle) const
    {
        return CurvatureFunction(radii, angle);
    }

    // 점 포함 확인
    bool Ellipse::ContainsPoint(const Vector2& point) const
    {
        return ContainsPointFunction(center, radii, rotation, point);
    }

    // 점까지의 거리
    Real Ellipse::GetDistanceToPoint(const Vector2& point) const
    {
        return DistanceToPointFunction(center, radii, rotation, point);
    }

    // 가장 가까운 점
    Vector2 Ellipse::GetClosestPoint(const Vector2& point) const
    {
        return ClosestPointFunction(center, radii, rotation, point);
    }

    // ========================================
    // 변형 함수들
    // ========================================

    // 타원 이동
    void Ellipse::Translate(const Vector2& offset)
    {
        center += offset;
    }

    // 타원 크기 조정
    void Ellipse::Scale(Real scale)
    {
        radii *= scale;
    }

    // 타원 회전
    void Ellipse::Rotate(Real angle)
    {
        rotation += angle;
    }

    // ========================================
    // 충돌 검출 함수들
    // ========================================

    // 두 타원의 충돌 검출
    bool Ellipse::IntersectsWith(const Ellipse& other) const
    {
        // 간단한 원형 근사로 충돌 검출
        Vector2 distance = center - other.center;
        Real minDistance = GetBoundingRadius() + other.GetBoundingRadius();
        return distance.LengthSquared() <= minDistance * minDistance;
    }

    // 두 타원의 충돌 정보
    CollisionResult Ellipse::GetCollisionInfo(const Ellipse& other) const
    {
        // 간단한 구현 - 실제로는 더 정교한 접촉점 계산이 필요
        if (IntersectsWith(other)) {
            Vector2 normal = (other.center - center).Normalized();
            Real overlap = 1.0f; // 실제로는 정확한 겹침 계산 필요
            Vector2 contactPoint = center;
            return CollisionResult::Collision(normal, overlap, contactPoint);
        } else {
            return CollisionResult::NoCollision();
        }
    }

    // ========================================
    // 기하학적 속성 함수들
    // ========================================

    // 타원의 바운딩 박스
    BoundingBox Ellipse::GetBoundingBox() const
    {
        // 회전된 타원의 바운딩 박스 계산
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);

        Real maxX = std::abs(radii.x * cosRot) + std::abs(radii.y * sinRot);
        Real maxY = std::abs(radii.x * sinRot) + std::abs(radii.y * cosRot);

        Vector2 min = center - Vector2(maxX, maxY);
        Vector2 max = center + Vector2(maxX, maxY);

        return BoundingBox(min, max);
    }

    // 타원의 경계 길이 (둘레와 동일)
    Real Ellipse::GetBoundaryLength() const
    {
        return GetPerimeter();
    }

    // 타원의 중심점 (무게중심과 동일)
    Vector2 Ellipse::GetCentroid() const
    {
        return center;
    }

    // 타원의 외심 (중심과 동일)
    Vector2 Ellipse::GetCircumcenter() const
    {
        return center;
    }

    // 타원의 내심 (중심과 동일)
    Vector2 Ellipse::GetIncenter() const
    {
        return center;
    }

    // 타원의 수심 (중심과 동일)
    Vector2 Ellipse::GetOrthocenter() const
    {
        return center;
    }

    // 타원의 볼록성 (항상 볼록)
    bool Ellipse::IsConvex() const
    {
        return true;
    }

    // 타원의 정규성 (원인지 확인)
    bool Ellipse::IsRegular() const
    {
        return std::abs(radii.x - radii.y) < 1e-6 && std::abs(rotation) < 1e-6;
    }

    // 타원의 대칭성
    bool Ellipse::IsSymmetric() const
    {
        return std::abs(rotation) < 1e-6 || std::abs(rotation - PI * 0.5f) < 1e-6;
    }

    // 타원의 장축
    Real Ellipse::GetMajorAxis() const
    {
        return std::max(radii.x, radii.y);
    }

    // 타원의 단축
    Real Ellipse::GetMinorAxis() const
    {
        return std::min(radii.x, radii.y);
    }

    // 타원의 이심률
    Real Ellipse::GetEccentricity() const
    {
        Real a = GetMajorAxis();
        Real b = GetMinorAxis();
        return std::sqrt(1 - (b * b) / (a * a));
    }

    // 타원의 초점들
    std::vector<Vector2> Ellipse::GetFoci() const
    {
        Real a = GetMajorAxis();
        Real b = GetMinorAxis();
        Real c = std::sqrt(a * a - b * b);

        Vector2 focus1, focus2;
        if (radii.x >= radii.y) {
            // 장축이 x축에 평행
            focus1 = center + Vector2(c * std::cos(rotation), c * std::sin(rotation));
            focus2 = center + Vector2(-c * std::cos(rotation), -c * std::sin(rotation));
        } else {
            // 장축이 y축에 평행
            focus1 = center + Vector2(-c * std::sin(rotation), c * std::cos(rotation));
            focus2 = center + Vector2(c * std::sin(rotation), -c * std::cos(rotation));
        }

        return {focus1, focus2};
    }

    // 타원의 꼭지점들
    std::vector<Vector2> Ellipse::GetVertices() const
    {
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);

        std::vector<Vector2> vertices = {
            center + Vector2(radii.x * cosRot, radii.x * sinRot),      // 우측
            center + Vector2(-radii.x * cosRot, -radii.x * sinRot),    // 좌측
            center + Vector2(-radii.y * sinRot, radii.y * cosRot),     // 상단
            center + Vector2(radii.y * sinRot, -radii.y * cosRot)      // 하단
        };

        return vertices;
    }
}
