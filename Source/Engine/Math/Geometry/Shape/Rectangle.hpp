#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../Algorithm/GJK_EPA.hpp"
#include "../Algorithm/SAT.hpp"
#include "IShape.hpp"
#include "../../Structure/Geometry.hpp"

namespace Engine2D
{
    // ========================================
    // 사각형 클래스
    // ========================================

    class Rectangle : public IShape
    {
    public:
        Vector2 center;      // 중심점
        Vector2 size;        // 크기 (width, height)
        Real rotation;       // 회전 각도 (라디안)

        // 생성자
        Rectangle();
        Rectangle(const Vector2& center, const Vector2& size);
        Rectangle(const Vector2& center, const Vector2& size, Real rotation);
        Rectangle(Real x, Real y, Real width, Real height);

        // ========================================
        // IShape 인터페이스 구현
        // ========================================

        ShapeType GetType() const override;
        Vector2 GetCenter() const override;
        Real GetBoundingRadius() const override;

        // SAT를 위한 축 투영
        std::vector<Vector2> GetAxes() const override;
        std::pair<Real, Real> ProjectOnAxis(const Vector2& axis) const override;

        // SDF를 위한 거리 함수
        SDFResult GetSDF(const Vector2& point) const override;

        // GJK를 위한 지지점 함수
        Vector2 GetSupportPoint(const Vector2& direction) const override;

        // ========================================
        // 기본 멤버 함수들
        // ========================================

        // 면적 계산
        Real GetArea() const;

        // 둘레 계산
        Real GetPerimeter() const;

        // 점이 사각형 내부에 있는지 확인
        bool ContainsPoint(const Vector2& point) const;

        // 사각형과 점 사이의 거리
        Real GetDistanceToPoint(const Vector2& point) const;

        // 사각형의 경계에서 가장 가까운 점
        Vector2 GetClosestPointOnBoundary(const Vector2& point) const;

        // 사각형의 법선 벡터 (경계상의 점에서)
        Vector2 GetNormalAt(const Vector2& boundaryPoint) const;

        // ========================================
        // 변형 함수들
        // ========================================

        // 사각형 이동
        void Translate(const Vector2& offset);

        // 사각형 크기 조정 (중심점 기준)
        void Scale(Real scale);

        // 사각형 회전 (중심점 기준)
        void Rotate(Real angle);

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 두 사각형의 충돌 검출
        bool IntersectsWith(const Rectangle& other) const;

        // 두 사각형의 충돌 정보
        CollisionResult GetCollisionInfo(const Rectangle& other) const;

        // ========================================
        // 기하학적 속성 함수들
        // ========================================

        // 사각형의 바운딩 박스
        BoundingBox GetBoundingBox() const;

        // 사각형의 경계 길이 (둘레와 동일)
        Real GetBoundaryLength() const;

        // 사각형의 중심점 (무게중심과 동일)
        Vector2 GetCentroid() const;

        // 사각형의 관성 모멘트
        Real GetMomentOfInertia() const;

        // 사각형의 볼록성 (항상 볼록)
        bool IsConvex() const;

        // 사각형의 정규성 (정사각형인지 확인)
        bool IsRegular() const;

        // 사각형의 대칭성
        bool IsSymmetric() const;

        // 사각형의 꼭지점들
        std::vector<Vector2> GetVertices() const;

        // 사각형의 모서리들
        std::vector<std::pair<Vector2, Vector2>> GetEdges() const;
    };
}
