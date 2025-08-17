#pragma once

#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../Algorithm/GJK_EPA.hpp"
#include "IShape.hpp"
#include "../../Structure/Geometry.hpp"

namespace Engine2D
{
    // ========================================
    // 원 클래스
    // ========================================

    class Circle : public IShape
    {
    public:
        Vector2 center;  // 중심점
        Real radius;     // 반지름

        // 생성자
        Circle();
        Circle(const Vector2& center, Real radius);
        Circle(Real x, Real y, Real radius);

        // ========================================
        // Static 함수들 (핵심 알고리즘)
        // ========================================

        // 1. Support Function - GJK/EPA용
        static Vector2 SupportFunction(const Vector2& center, Real radius, const Vector2& direction);

        // 2. SDF Function - 부호가 있는 거리 함수
        static SDFResult SDFFunction(const Vector2& center, Real radius, const Vector2& point);

        // 3. Area Function - 면적 계산
        static Real AreaFunction(Real radius);

        // 4. Perimeter Function - 둘레 계산
        static Real PerimeterFunction(Real radius);

        // 5. Moment of Inertia Function - 관성 모멘트
        static Real MomentOfInertiaFunction(Real radius);

        // 6. Curvature Function - 곡률 계산
        static Real CurvatureFunction(Real radius);

        // 7. Point Containment Function - 점 포함 확인
        static bool ContainsPointFunction(const Vector2& center, Real radius, const Vector2& point);

        // 8. Distance to Point Function - 점까지의 거리
        static Real DistanceToPointFunction(const Vector2& center, Real radius, const Vector2& point);

        // 9. Closest Point Function - 가장 가까운 점
        static Vector2 ClosestPointFunction(const Vector2& center, Real radius, const Vector2& point);

        // ========================================
        // IShape 인터페이스 구현
        // ========================================

        ShapeType GetType() const override;
        Vector2 GetCenter() const override;
        Real GetBoundingRadius() const override;

        // SAT를 위한 축 투영 (원은 모든 방향에서 동일)
        std::vector<Vector2> GetAxes() const override;
        std::pair<Real, Real> ProjectOnAxis(const Vector2& axis) const override;

        // SDF를 위한 거리 함수
        SDFResult GetSDF(const Vector2& point) const override;

        // GJK를 위한 지지점 함수
        Vector2 GetSupportPoint(const Vector2& direction) const override;

        // ========================================
        // 기본 멤버 함수들 (Static 함수 사용)
        // ========================================

        // 면적 계산
        Real GetArea() const;

        // 둘레 계산
        Real GetPerimeter() const;

        // 관성 모멘트
        Real GetMomentOfInertia() const;

        // 곡률
        Real GetCurvature() const;

        // 점 포함 확인
        bool ContainsPoint(const Vector2& point) const;

        // 점까지의 거리
        Real GetDistanceToPoint(const Vector2& point) const;

        // 가장 가까운 점
        Vector2 GetClosestPoint(const Vector2& point) const;

        // ========================================
        // 변형 함수들
        // ========================================

        // 원 이동
        void Translate(const Vector2& offset);

        // 원 크기 조정
        void Scale(Real scale);

        // 원 회전 (원은 회전에 불변)
        void Rotate(Real angle);

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 두 원의 충돌 검출
        bool IntersectsWith(const Circle& other) const;

        // 두 원의 충돌 정보
        CollisionResult GetCollisionInfo(const Circle& other) const;

        // ========================================
        // 기하학적 속성 함수들
        // ========================================

        // 원의 바운딩 박스
        BoundingBox GetBoundingBox() const;

        // 원의 경계 길이 (둘레와 동일)
        Real GetBoundaryLength() const;

        // 원의 중심점 (무게중심과 동일)
        Vector2 GetCentroid() const;

        // 원의 외심 (중심과 동일)
        Vector2 GetCircumcenter() const;

        // 원의 내심 (중심과 동일)
        Vector2 GetIncenter() const;

        // 원의 수심 (중심과 동일)
        Vector2 GetOrthocenter() const;

        // 원의 볼록성 (항상 볼록)
        bool IsConvex() const;

        // 원의 정규성 (항상 정규)
        bool IsRegular() const;

        // 원의 대칭성 (모든 방향에서 대칭)
        bool IsSymmetric() const;
    };
}
