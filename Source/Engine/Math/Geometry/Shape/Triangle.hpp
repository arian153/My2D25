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
    // 삼각형 클래스
    // ========================================

    class Triangle : public IShape
    {
    public:
        Vector2 vertices[3];  // 세 정점

        // 생성자
        Triangle();
        Triangle(const Vector2& a, const Vector2& b, const Vector2& c);
        Triangle(Real x1, Real y1, Real x2, Real y2, Real x3, Real y3);

        // ========================================
        // Static 함수들 (핵심 알고리즘)
        // ========================================

        // 1. Support Function - GJK/EPA용
        static Vector2 SupportFunction(const Vector2& vertices[3], const Vector2& direction);

        // 2. Area Function - 면적 계산
        static Real AreaFunction(const Vector2& vertices[3]);

        // 3. Perimeter Function - 둘레 계산
        static Real PerimeterFunction(const Vector2& vertices[3]);

        // 4. SDF Function - 부호가 있는 거리 함수
        static SDFResult SDFFunction(const Vector2& vertices[3], const Vector2& point);

        // 5. Centroid Function - 무게중심 계산
        static Vector2 CentroidFunction(const Vector2& vertices[3]);

        // 6. Circumcenter Function - 외심 계산
        static Vector2 CircumcenterFunction(const Vector2& vertices[3]);

        // 7. Incenter Function - 내심 계산
        static Vector2 IncenterFunction(const Vector2& vertices[3]);

        // 8. Orthocenter Function - 수심 계산
        static Vector2 OrthocenterFunction(const Vector2& vertices[3]);

        // 9. BoundingBox Function - 바운딩 박스 계산
        static BoundingBox BoundingBoxFunction(const Vector2& vertices[3]);

        // 10. Moment of Inertia Function - 관성 모멘트 계산
        static Real MomentOfInertiaFunction(const Vector2& vertices[3]);

        // 11. IsConvex Function - 볼록성 확인
        static bool IsConvexFunction(const Vector2& vertices[3]);

        // 12. IsEquilateral Function - 정삼각형 확인
        static bool IsEquilateralFunction(const Vector2& vertices[3]);

        // 13. IsIsosceles Function - 이등변삼각형 확인
        static bool IsIsoscelesFunction(const Vector2& vertices[3]);

        // 14. IsRight Function - 직각삼각형 확인
        static bool IsRightFunction(const Vector2& vertices[3]);

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

        // 무게중심
        Vector2 GetCentroid() const;

        // 외심
        Vector2 GetCircumcenter() const;

        // 내심
        Vector2 GetIncenter() const;

        // 수심
        Vector2 GetOrthocenter() const;

        // 바운딩 박스
        BoundingBox GetBoundingBox() const;

        // 관성 모멘트
        Real GetMomentOfInertia() const;

        // 볼록성 확인
        bool IsConvex() const;

        // 정삼각형 확인
        bool IsEquilateral() const;

        // 이등변삼각형 확인
        bool IsIsosceles() const;

        // 직각삼각형 확인
        bool IsRight() const;

        // ========================================
        // 추가 멤버 함수들
        // ========================================

        // 정점들 가져오기
        std::vector<Vector2> GetVertices() const;

        // 모서리들 가져오기
        std::vector<std::pair<Vector2, Vector2>> GetEdges() const;

        // 점이 삼각형 내부에 있는지 확인 (면적 기반)
        bool ContainsPoint(const Vector2& point) const;

        // 삼각형과 점 사이의 거리
        Real GetDistanceToPoint(const Vector2& point) const;

        // 삼각형의 경계에서 가장 가까운 점
        Vector2 GetClosestPointOnBoundary(const Vector2& point) const;

        // 삼각형의 법선 벡터 (경계상의 점에서)
        Vector2 GetNormalAt(const Vector2& boundaryPoint) const;

        // ========================================
        // 변형 함수들
        // ========================================

        // 삼각형 이동
        void Translate(const Vector2& offset);

        // 삼각형 크기 조정 (중심점 기준)
        void Scale(Real scale);

        // 삼각형 회전 (중심점 기준)
        void Rotate(Real angle);

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 두 삼각형의 충돌 검출
        bool IntersectsWith(const Triangle& other) const;

        // 두 삼각형의 충돌 정보
        CollisionResult GetCollisionInfo(const Triangle& other) const;

        // ========================================
        // 기하학적 속성 함수들
        // ========================================

        // 삼각형의 각도들 계산
        std::vector<Real> GetAngles() const;

        // 삼각형의 변 길이들
        std::vector<Real> GetSideLengths() const;

        // 삼각형의 높이들
        std::vector<Real> GetHeights() const;

        // 삼각형의 중선들
        std::vector<Vector2> GetMedians() const;

        // 삼각형의 각이등분선들
        std::vector<Vector2> GetAngleBisectors() const;

        // 삼각형의 경계 길이 (둘레와 동일)
        Real GetBoundaryLength() const;
    };
}
