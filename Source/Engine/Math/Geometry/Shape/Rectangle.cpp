#include "Rectangle.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 생성자
    // ========================================

    Rectangle::Rectangle() : center(0, 0), size(1, 1), rotation(0) {}

    Rectangle::Rectangle(const Vector2& center, const Vector2& size) : center(center), size(size), rotation(0) {}

    Rectangle::Rectangle(const Vector2& center, const Vector2& size, Real rotation) : center(center), size(size), rotation(rotation) {}

    Rectangle::Rectangle(Real x, Real y, Real width, Real height) : center(x, y), size(width, height), rotation(0) {}

    // ========================================
    // 1. SupportFunction - For GJK, EPA
    // ========================================

    // GJK/EPA를 위한 서포트 함수
    Vector2 Rectangle::GetSupportPoint(const Vector2& direction) const
    {
        if (direction.LengthSquared() < 1e-12) {
            return center + Vector2(size.x * 0.5f, 0);
        }

        // 회전된 방향 벡터 계산
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);
        Vector2 rotatedDir(
            direction.x * cosRot + direction.y * sinRot,
            -direction.x * sinRot + direction.y * cosRot
        );

        // 회전된 좌표계에서 서포트 점 계산
        Vector2 halfSize = size * Real(0.5);
        Vector2 support(
            rotatedDir.x > 0 ? halfSize.x : -halfSize.x,
            rotatedDir.y > 0 ? halfSize.y : -halfSize.y
        );

        // 원래 좌표계로 변환
        Vector2 result(
            support.x * cosRot - support.y * sinRot,
            support.x * sinRot + support.y * cosRot
        );

        return center + result;
    }

    // ========================================
    // 2. Area
    // ========================================

    // 면적 계산
    Real Rectangle::GetArea() const
    {
        return size.x * size.y;
    }

    // ========================================
    // 3. Perimeter
    // ========================================

    // 둘레 계산
    Real Rectangle::GetPerimeter() const
    {
        return Real(2) * (size.x + size.y);
    }

    // ========================================
    // 4. SDF
    // ========================================

    // 부호가 있는 거리 함수
    SDFResult Rectangle::GetSDF(const Vector2& point) const
    {
        SDFResult result;

        // 점을 로컬 좌표계로 변환
        Vector2 localPoint = point - center;

        // 회전 변환
        Real cosRot = std::cos(-rotation);
        Real sinRot = std::sin(-rotation);
        Vector2 rotatedPoint(
            localPoint.x * cosRot - localPoint.y * sinRot,
            localPoint.x * sinRot + localPoint.y * cosRot
        );

        // 반사각형의 절반 크기
        Vector2 halfSize = size * Real(0.5);

        // 반사각형 내부에서의 거리 계산
        Vector2 d = Vector2(std::abs(rotatedPoint.x), std::abs(rotatedPoint.y)) - halfSize;
        Real insideDist = std::max(d.x, d.y);

        if (insideDist <= 0) {
            // 내부에 있는 경우
            result.distance = insideDist;
            result.isInside = true;
            result.closestPoint = point;
        } else {
            // 외부에 있는 경우
            Vector2 closestLocal = Vector2(
                std::clamp(rotatedPoint.x, -halfSize.x, halfSize.x),
                std::clamp(rotatedPoint.y, -halfSize.y, halfSize.y)
            );

            // 원래 좌표계로 변환
            Vector2 closestPoint(
                closestLocal.x * cosRot - closestLocal.y * sinRot,
                closestLocal.x * sinRot + closestLocal.y * cosRot
            );

            result.distance = (point - (center + closestPoint)).Length();
            result.isInside = false;
            result.closestPoint = center + closestPoint;
        }

        // 그래디언트 계산
        if (std::abs(result.distance) > 1e-6) {
            result.gradient = (point - result.closestPoint).Normalized();
        } else {
            result.gradient = Vector2(1, 0);
        }

        return result;
    }

    // ========================================
    // IShape 인터페이스 구현
    // ========================================

    ShapeType Rectangle::GetType() const
    {
        return ShapeType::RECTANGLE;
    }

    Vector2 Rectangle::GetCenter() const
    {
        return center;
    }

    Real Rectangle::GetBoundingRadius() const
    {
        return size.Length() * 0.5f;
    }

    // SAT를 위한 축 투영
    std::vector<Vector2> Rectangle::GetAxes() const
    {
        std::vector<Vector2> axes;
        axes.reserve(2);

        // 회전된 축들
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);

        axes.push_back(Vector2(cosRot, sinRot));      // x축
        axes.push_back(Vector2(-sinRot, cosRot));     // y축

        return axes;
    }

    std::pair<Real, Real> Rectangle::ProjectOnAxis(const Vector2& axis) const
    {
        if (axis.LengthSquared() < 1e-12) {
            return {0, 0};
        }

        // 네 꼭지점을 계산
        Vector2 halfSize = size * Real(0.5);
        std::vector<Vector2> vertices = {
            center + Vector2(-halfSize.x, -halfSize.y),
            center + Vector2(halfSize.x, -halfSize.y),
            center + Vector2(halfSize.x, halfSize.y),
            center + Vector2(-halfSize.x, halfSize.y)
        };

        // 회전 적용
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);
        for (auto& vertex : vertices) {
            Vector2 relative = vertex - center;
            vertex = center + Vector2(
                relative.x * cosRot - relative.y * sinRot,
                relative.x * sinRot + relative.y * cosRot
            );
        }

        // 투영 계산
        Real minProj = vertices[0].Dot(axis);
        Real maxProj = minProj;

        for (size_t i = 1; i < vertices.size(); ++i) {
            Real proj = vertices[i].Dot(axis);
            minProj = std::min(minProj, proj);
            maxProj = std::max(maxProj, proj);
        }

        return {minProj, maxProj};
    }

    // SDF를 위한 거리 함수
    SDFResult Rectangle::GetSDF(const Vector2& point) const
    {
        return GetSDF(point);
    }

    // GJK를 위한 지지점 함수
    Vector2 Rectangle::GetSupportPoint(const Vector2& direction) const
    {
        return GetSupportPoint(direction);
    }

    // ========================================
    // 기본 멤버 함수들
    // ========================================

    // 점이 사각형 내부에 있는지 확인
    bool Rectangle::ContainsPoint(const Vector2& point) const
    {
        // 점을 로컬 좌표계로 변환
        Vector2 localPoint = point - center;

        // 회전 변환
        Real cosRot = std::cos(-rotation);
        Real sinRot = std::sin(-rotation);
        Vector2 rotatedPoint(
            localPoint.x * cosRot - localPoint.y * sinRot,
            localPoint.x * sinRot + localPoint.y * cosRot
        );

        // 반사각형의 절반 크기
        Vector2 halfSize = size * Real(0.5);

        return std::abs(rotatedPoint.x) <= halfSize.x && std::abs(rotatedPoint.y) <= halfSize.y;
    }

    // 사각형과 점 사이의 거리
    Real Rectangle::GetDistanceToPoint(const Vector2& point) const
    {
        SDFResult sdf = GetSDF(point);
        return std::abs(sdf.distance);
    }

    // 사각형의 경계에서 가장 가까운 점
    Vector2 Rectangle::GetClosestPointOnBoundary(const Vector2& point) const
    {
        SDFResult sdf = GetSDF(point);
        return sdf.closestPoint;
    }

    // 사각형의 법선 벡터 (경계상의 점에서)
    Vector2 Rectangle::GetNormalAt(const Vector2& boundaryPoint) const
    {
        SDFResult sdf = GetSDF(boundaryPoint);
        return sdf.gradient;
    }

    // ========================================
    // 변형 함수들
    // ========================================

    // 사각형 이동
    void Rectangle::Translate(const Vector2& offset)
    {
        center += offset;
    }

    // 사각형 크기 조정 (중심점 기준)
    void Rectangle::Scale(Real scale)
    {
        size *= scale;
    }

    // 사각형 회전 (중심점 기준)
    void Rectangle::Rotate(Real angle)
    {
        rotation += angle;
    }

    // ========================================
    // 충돌 검출 함수들
    // ========================================

    // 두 사각형의 충돌 검출
    bool Rectangle::IntersectsWith(const Rectangle& other) const
    {
        std::vector<Vector2> axes = GetAxes();
        std::vector<Vector2> otherAxes = other.GetAxes();

        // 모든 축에 대해 투영 검사
        for (const auto& axis : axes) {
            auto proj1 = ProjectOnAxis(axis);
            auto proj2 = other.ProjectOnAxis(axis);

            if (proj1.second < proj2.first || proj2.second < proj1.first) {
                return false;
            }
        }

        for (const auto& axis : otherAxes) {
            auto proj1 = ProjectOnAxis(axis);
            auto proj2 = other.ProjectOnAxis(axis);

            if (proj1.second < proj2.first || proj2.second < proj1.first) {
                return false;
            }
        }

        return true;
    }

    // 두 사각형의 충돌 정보
    CollisionResult Rectangle::GetCollisionInfo(const Rectangle& other) const
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

    // 사각형의 바운딩 박스
    BoundingBox Rectangle::GetBoundingBox() const
    {
        // 회전된 사각형의 바운딩 박스 계산
        Vector2 halfSize = size * Real(0.5);
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);

        // 네 꼭지점
        std::vector<Vector2> vertices = {
            Vector2(-halfSize.x, -halfSize.y),
            Vector2(halfSize.x, -halfSize.y),
            Vector2(halfSize.x, halfSize.y),
            Vector2(-halfSize.x, halfSize.y)
        };

        // 회전 적용
        for (auto& vertex : vertices) {
            Vector2 rotated = Vector2(
                vertex.x * cosRot - vertex.y * sinRot,
                vertex.x * sinRot + vertex.y * cosRot
            );
            vertex = center + rotated;
        }

        // 최소/최대 계산
        Vector2 min = vertices[0];
        Vector2 max = vertices[0];

        for (const auto& vertex : vertices) {
            min.x = std::min(min.x, vertex.x);
            min.y = std::min(min.y, vertex.y);
            max.x = std::max(max.x, vertex.x);
            max.y = std::max(max.y, vertex.y);
        }

        return BoundingBox(min, max);
    }

    // 사각형의 경계 길이 (둘레와 동일)
    Real Rectangle::GetBoundaryLength() const
    {
        return GetPerimeter();
    }

    // 사각형의 중심점 (무게중심과 동일)
    Vector2 Rectangle::GetCentroid() const
    {
        return center;
    }

    // 사각형의 관성 모멘트
    Real Rectangle::GetMomentOfInertia() const
    {
        return (size.x * size.x + size.y * size.y) / Real(12);
    }

    // 사각형의 볼록성 (항상 볼록)
    bool Rectangle::IsConvex() const
    {
        return true;
    }

    // 사각형의 정규성 (정사각형인지 확인)
    bool Rectangle::IsRegular() const
    {
        return std::abs(size.x - size.y) < 1e-6 && std::abs(rotation) < 1e-6;
    }

    // 사각형의 대칭성
    bool Rectangle::IsSymmetric() const
    {
        return std::abs(rotation) < 1e-6 || std::abs(rotation - PI * 0.5f) < 1e-6;
    }

    // 사각형의 꼭지점들
    std::vector<Vector2> Rectangle::GetVertices() const
    {
        Vector2 halfSize = size * Real(0.5);
        Real cosRot = std::cos(rotation);
        Real sinRot = std::sin(rotation);

        std::vector<Vector2> vertices = {
            Vector2(-halfSize.x, -halfSize.y),
            Vector2(halfSize.x, -halfSize.y),
            Vector2(halfSize.x, halfSize.y),
            Vector2(-halfSize.x, halfSize.y)
        };

        // 회전 적용
        for (auto& vertex : vertices) {
            Vector2 rotated = Vector2(
                vertex.x * cosRot - vertex.y * sinRot,
                vertex.x * sinRot + vertex.y * cosRot
            );
            vertex = center + rotated;
        }

        return vertices;
    }

    // 사각형의 모서리들
    std::vector<std::pair<Vector2, Vector2>> Rectangle::GetEdges() const
    {
        std::vector<Vector2> vertices = GetVertices();
        return {
            {vertices[0], vertices[1]},
            {vertices[1], vertices[2]},
            {vertices[2], vertices[3]},
            {vertices[3], vertices[0]}
        };
    }
}
