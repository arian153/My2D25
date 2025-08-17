#include "Circle.hpp"
#include <cmath>
#include <algorithm>

namespace Engine2D
{
    // ========================================
    // 생성자
    // ========================================

    Circle::Circle() : center(0, 0), radius(1) {}

    Circle::Circle(const Vector2& center, Real radius) : center(center), radius(radius) {}

    Circle::Circle(Real x, Real y, Real radius) : center(x, y), radius(radius) {}

    // ========================================
    // Static 함수들 (핵심 알고리즘)
    // ========================================

    // 1. Support Function - GJK/EPA용
    Vector2 Circle::SupportFunction(const Vector2& center, Real radius, const Vector2& direction)
    {
        if (direction.LengthSquared() < 1e-12) {
            return center + Vector2(radius, 0);
        }
        Vector2 normalizedDir = direction.Normalized();
        return center + normalizedDir * radius;
    }

    // 2. SDF Function - 부호가 있는 거리 함수
    SDFResult Circle::SDFFunction(const Vector2& center, Real radius, const Vector2& point)
    {
        SDFResult result;
        Vector2 toPoint = point - center;
        Real distance = toPoint.Length();
        result.distance = distance - radius;
        result.isInside = distance < radius;

        if (distance > 1e-6) {
            result.gradient = toPoint / distance;
            result.closestPoint = center + result.gradient * radius;
        } else {
            result.gradient = Vector2(1, 0);
            result.closestPoint = center + Vector2(radius, 0);
        }

        return result;
    }

    // 3. Area Function - 면적 계산
    Real Circle::AreaFunction(Real radius)
    {
        return PI * radius * radius;
    }

    // 4. Perimeter Function - 둘레 계산
    Real Circle::PerimeterFunction(Real radius)
    {
        return Real(2) * PI * radius;
    }

    // 5. Moment of Inertia Function - 관성 모멘트
    Real Circle::MomentOfInertiaFunction(Real radius)
    {
        return Real(0.5) * radius * radius;
    }

    // 6. Curvature Function - 곡률 계산
    Real Circle::CurvatureFunction(Real radius)
    {
        return Real(1) / radius;
    }

    // 7. Point Containment Function - 점 포함 확인
    bool Circle::ContainsPointFunction(const Vector2& center, Real radius, const Vector2& point)
    {
        Vector2 toPoint = point - center;
        return toPoint.LengthSquared() <= radius * radius;
    }

    // 8. Distance to Point Function - 점까지의 거리
    Real Circle::DistanceToPointFunction(const Vector2& center, Real radius, const Vector2& point)
    {
        Vector2 toPoint = point - center;
        return std::abs(toPoint.Length() - radius);
    }

    // 9. Closest Point Function - 가장 가까운 점
    Vector2 Circle::ClosestPointFunction(const Vector2& center, Real radius, const Vector2& point)
    {
        Vector2 toPoint = point - center;
        Real distance = toPoint.Length();

        if (distance < 1e-6) {
            return center + Vector2(radius, 0);
        }

        return center + (toPoint / distance) * radius;
    }

    // ========================================
    // IShape 인터페이스 구현
    // ========================================

    ShapeType Circle::GetType() const
    {
        return ShapeType::CIRCLE;
    }

    Vector2 Circle::GetCenter() const
    {
        return center;
    }

    Real Circle::GetBoundingRadius() const
    {
        return radius;
    }

    // SAT를 위한 축 투영 (원은 모든 방향에서 동일)
    std::vector<Vector2> Circle::GetAxes() const
    {
        return {Vector2(1, 0), Vector2(0, 1)};
    }

    std::pair<Real, Real> Circle::ProjectOnAxis(const Vector2& axis) const
    {
        if (axis.LengthSquared() < 1e-12) {
            return {0, 0};
        }

        Real centerProj = center.Dot(axis);
        return {centerProj - radius, centerProj + radius};
    }

    // SDF를 위한 거리 함수
    SDFResult Circle::GetSDF(const Vector2& point) const
    {
        return SDFFunction(center, radius, point);
    }

    // GJK를 위한 지지점 함수
    Vector2 Circle::GetSupportPoint(const Vector2& direction) const
    {
        return SupportFunction(center, radius, direction);
    }

    // ========================================
    // 기본 멤버 함수들 (Static 함수 사용)
    // ========================================

    // 면적 계산
    Real Circle::GetArea() const
    {
        return AreaFunction(radius);
    }

    // 둘레 계산
    Real Circle::GetPerimeter() const
    {
        return PerimeterFunction(radius);
    }

    // 관성 모멘트
    Real Circle::GetMomentOfInertia() const
    {
        return MomentOfInertiaFunction(radius);
    }

    // 곡률
    Real Circle::GetCurvature() const
    {
        return CurvatureFunction(radius);
    }

    // 점 포함 확인
    bool Circle::ContainsPoint(const Vector2& point) const
    {
        return ContainsPointFunction(center, radius, point);
    }

    // 점까지의 거리
    Real Circle::GetDistanceToPoint(const Vector2& point) const
    {
        return DistanceToPointFunction(center, radius, point);
    }

    // 가장 가까운 점
    Vector2 Circle::GetClosestPoint(const Vector2& point) const
    {
        return ClosestPointFunction(center, radius, point);
    }

    // ========================================
    // 변형 함수들
    // ========================================

    // 원 이동
    void Circle::Translate(const Vector2& offset)
    {
        center += offset;
    }

    // 원 크기 조정
    void Circle::Scale(Real scale)
    {
        radius *= scale;
    }

    // 원 회전 (원은 회전에 불변)
    void Circle::Rotate(Real angle)
    {
        // 원은 회전에 불변하므로 아무것도 하지 않음
    }

    // ========================================
    // 충돌 검출 함수들
    // ========================================

    // 두 원의 충돌 검출
    bool Circle::IntersectsWith(const Circle& other) const
    {
        Vector2 distance = center - other.center;
        Real minDistance = radius + other.radius;
        return distance.LengthSquared() <= minDistance * minDistance;
    }

    // 두 원의 충돌 정보
    CollisionResult Circle::GetCollisionInfo(const Circle& other) const
    {
        Vector2 distance = center - other.center;
        Real distanceLength = distance.Length();
        Real minDistance = radius + other.radius;

        if (distanceLength <= minDistance) {
            Vector2 normal = distanceLength > 1e-6 ? distance / distanceLength : Vector2(1, 0);
            Real overlap = minDistance - distanceLength;
            Vector2 contactPoint = center - normal * radius;
            return CollisionResult::Collision(normal, overlap, contactPoint);
        } else {
            return CollisionResult::NoCollision();
        }
    }

    // ========================================
    // 기하학적 속성 함수들
    // ========================================

    // 원의 바운딩 박스
    BoundingBox Circle::GetBoundingBox() const
    {
        Vector2 min = center - Vector2(radius, radius);
        Vector2 max = center + Vector2(radius, radius);
        return BoundingBox(min, max);
    }

    // 원의 경계 길이 (둘레와 동일)
    Real Circle::GetBoundaryLength() const
    {
        return GetPerimeter();
    }

    // 원의 중심점 (무게중심과 동일)
    Vector2 Circle::GetCentroid() const
    {
        return center;
    }

    // 원의 외심 (중심과 동일)
    Vector2 Circle::GetCircumcenter() const
    {
        return center;
    }

    // 원의 내심 (중심과 동일)
    Vector2 Circle::GetIncenter() const
    {
        return center;
    }

    // 원의 수심 (중심과 동일)
    Vector2 Circle::GetOrthocenter() const
    {
        return center;
    }

    // 원의 볼록성 (항상 볼록)
    bool Circle::IsConvex() const
    {
        return true;
    }

    // 원의 정규성 (항상 정규)
    bool Circle::IsRegular() const
    {
        return true;
    }

    // 원의 대칭성 (모든 방향에서 대칭)
    bool Circle::IsSymmetric() const
    {
        return true;
    }
}
