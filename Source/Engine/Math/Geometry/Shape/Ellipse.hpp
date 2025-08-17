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
    // 타원 클래스
    // ========================================

    class Ellipse : public IShape
    {
    public:
        Vector2 center;      // 중심점
        Vector2 radii;       // 반지름 (x축, y축)
        Real rotation;       // 회전 각도 (라디안)

        // 생성자
        Ellipse();
        Ellipse(const Vector2& center, const Vector2& radii);
        Ellipse(const Vector2& center, const Vector2& radii, Real rotation);
        Ellipse(Real x, Real y, Real radiusX, Real radiusY);

        // ========================================
        // Static 함수들 (핵심 알고리즘)
        // ========================================

        // 1. Support Function - GJK/EPA용
        static Vector2 SupportFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& direction);

        // 2. Area Function - 면적 계산
        static Real AreaFunction(const Vector2& radii);

        // 3. Perimeter Function - 둘레 계산 (근사)
        static Real PerimeterFunction(const Vector2& radii);

        // 4. SDF Function - 부호가 있는 거리 함수
        static SDFResult SDFFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point);

        // 5. Moment of Inertia Function - 관성 모멘트
        static Real MomentOfInertiaFunction(const Vector2& radii);

        // 6. Curvature Function - 곡률 계산
        static Real CurvatureFunction(const Vector2& radii, Real angle);

        // 7. Point Containment Function - 점 포함 확인
        static bool ContainsPointFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point);

        // 8. Distance to Point Function - 점까지의 거리
        static Real DistanceToPointFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point);

        // 9. Closest Point Function - 가장 가까운 점
        static Vector2 ClosestPointFunction(const Vector2& center, const Vector2& radii, Real rotation, const Vector2& point);

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
        // 기본 멤버 함수들 (Static 함수 사용)
        // ========================================

        // 면적 계산
        Real GetArea() const;

        // 둘레 계산
        Real GetPerimeter() const;

        // 관성 모멘트
        Real GetMomentOfInertia() const;

        // 곡률
        Real GetCurvature(Real angle) const;

        // 점 포함 확인
        bool ContainsPoint(const Vector2& point) const;

        // 점까지의 거리
        Real GetDistanceToPoint(const Vector2& point) const;

        // 가장 가까운 점
        Vector2 GetClosestPoint(const Vector2& point) const;

        // ========================================
        // 변형 함수들
        // ========================================

        // 타원 이동
        void Translate(const Vector2& offset);

        // 타원 크기 조정
        void Scale(Real scale);

        // 타원 회전
        void Rotate(Real angle);

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 두 타원의 충돌 검출
        bool IntersectsWith(const Ellipse& other) const;

        // 두 타원의 충돌 정보
        CollisionResult GetCollisionInfo(const Ellipse& other) const;

        // ========================================
        // 기하학적 속성 함수들
        // ========================================

        // 타원의 바운딩 박스
        BoundingBox GetBoundingBox() const;

        // 타원의 경계 길이 (둘레와 동일)
        Real GetBoundaryLength() const;

        // 타원의 중심점 (무게중심과 동일)
        Vector2 GetCentroid() const;

        // 타원의 외심 (중심과 동일)
        Vector2 GetCircumcenter() const;

        // 타원의 내심 (중심과 동일)
        Vector2 GetIncenter() const;

        // 타원의 수심 (중심과 동일)
        Vector2 GetOrthocenter() const;

        // 타원의 볼록성 (항상 볼록)
        bool IsConvex() const;

        // 타원의 정규성 (원인지 확인)
        bool IsRegular() const;

        // 타원의 대칭성
        bool IsSymmetric() const;

        // 타원의 장축
        Real GetMajorAxis() const;

        // 타원의 단축
        Real GetMinorAxis() const;

        // 타원의 이심률
        Real GetEccentricity() const;

        // 타원의 초점들
        std::vector<Vector2> GetFoci() const;

        // 타원의 꼭지점들
        std::vector<Vector2> GetVertices() const;
    };
}
