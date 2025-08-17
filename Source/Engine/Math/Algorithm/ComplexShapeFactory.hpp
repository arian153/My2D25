#pragma once

#include <vector>
#include <memory>
#include <functional>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../Shape/IShape.hpp"
#include "../Shape/ComplexShape.hpp"
#include "Minkowski.hpp"

namespace Engine2D
{
    // ========================================
    // 복잡한 도형 팩토리 클래스
    // ========================================

    class ComplexShapeFactory
    {
    public:
        // ========================================
        // 스윕 도형 생성
        // ========================================

        // 원이 곡선을 따라 스윕된 도형 생성
        static std::unique_ptr<ComplexShape> CreateSweptCircle(
            CurveType curveType, const std::vector<Vector2>& controlPoints, Real radius);

        // 사각형이 곡선을 따라 스윕된 도형 생성
        static std::unique_ptr<ComplexShape> CreateSweptRectangle(
            CurveType curveType, const std::vector<Vector2>& controlPoints, const Vector2& size);

        // 다각형이 곡선을 따라 스윕된 도형 생성
        static std::unique_ptr<ComplexShape> CreateSweptPolygon(
            CurveType curveType, const std::vector<Vector2>& controlPoints, const std::vector<Vector2>& vertices);

        // 커스텀 프로파일로 스윕된 도형 생성
        static std::unique_ptr<ComplexShape> CreateSweptShape(
            CurveType curveType, const std::vector<Vector2>& controlPoints,
            std::unique_ptr<ISweepProfile> profile);

        // ========================================
        // 민코스키 합 도형 생성
        // ========================================

        // 두 도형의 민코스키 합 생성
        static std::unique_ptr<ComplexShape> CreateMinkowskiSum(
            const IShape* shapeA, const IShape* shapeB);

        // ========================================
        // SDF 블렌딩 도형 생성
        // ========================================

        // 두 SDF의 블렌딩 (합집합, 교집합, 차집합)
        enum class BlendOperation
        {
            UNION,      // 합집합 (OR)
            INTERSECTION, // 교집합 (AND)
            SUBTRACTION   // 차집합 (A - B)
        };

        static std::unique_ptr<ComplexShape> CreateSDFBlend(
            const IShape* shapeA, const IShape* shapeB, BlendOperation operation);

        // ========================================
        // 사용자 정의 SDF 도형 생성
        // ========================================

        // 함수형 SDF 도형 생성
        static std::unique_ptr<ComplexShape> CreateCustomSDF(
            std::function<SDFResult(const Vector2&)> sdfFunction,
            const std::pair<Vector2, Vector2>& boundingBox);

        // ========================================
        // 편의 함수들
        // ========================================

        // 원형 프로파일 생성
        static std::unique_ptr<ISweepProfile> CreateCircleProfile(Real radius);

        // 사각형 프로파일 생성
        static std::unique_ptr<ISweepProfile> CreateRectangleProfile(const Vector2& size);

        // 다각형 프로파일 생성
        static std::unique_ptr<ISweepProfile> CreatePolygonProfile(const std::vector<Vector2>& vertices);

        // ========================================
        // 고급 블렌딩 함수들
        // ========================================

        // 부드러운 합집합 (Smooth Union)
        static std::unique_ptr<ComplexShape> CreateSmoothUnion(
            const IShape* shapeA, const IShape* shapeB, Real smoothness);

        // 부드러운 교집합 (Smooth Intersection)
        static std::unique_ptr<ComplexShape> CreateSmoothIntersection(
            const IShape* shapeA, const IShape* shapeB, Real smoothness);

        // 부드러운 차집합 (Smooth Subtraction)
        static std::unique_ptr<ComplexShape> CreateSmoothSubtraction(
            const IShape* shapeA, const IShape* shapeB, Real smoothness);

        // 원형 블렌딩 (Round Blend)
        static std::unique_ptr<ComplexShape> CreateRoundBlend(
            const IShape* shapeA, const IShape* shapeB, Real radius);

        // ========================================
        // 변형 함수들
        // ========================================

        // 도형의 윤곽선 생성 (두께가 있는 선)
        static std::unique_ptr<ComplexShape> CreateOutline(
            const IShape* shape, Real thickness);

        // 도형의 그림자 생성
        static std::unique_ptr<ComplexShape> CreateShadow(
            const IShape* shape, const Vector2& lightDirection, Real shadowLength);

        // 도형의 반사 생성
        static std::unique_ptr<ComplexShape> CreateReflection(
            const IShape* shape, const Vector2& mirrorNormal, Real mirrorDistance);

        // ========================================
        // 패턴 생성 함수들
        // ========================================

        // 격자 패턴 생성
        static std::unique_ptr<ComplexShape> CreateGridPattern(
            const IShape* baseShape, Real spacing, Real lineThickness);

        // 점 패턴 생성
        static std::unique_ptr<ComplexShape> CreateDotPattern(
            const IShape* baseShape, Real spacing, Real dotRadius);

        // 줄무늬 패턴 생성
        static std::unique_ptr<ComplexShape> CreateStripePattern(
            const IShape* baseShape, Real stripeWidth, Real spacing);

        // ========================================
        // 유틸리티 함수들
        // ========================================

        // 도형의 복사본 생성
        static std::unique_ptr<ComplexShape> Clone(const ComplexShape* shape);

        // 도형의 변형된 버전 생성
        static std::unique_ptr<ComplexShape> Transform(
            const ComplexShape* shape, const Vector2& translation, Real rotation, Real scale);

        // 도형의 조합 생성
        static std::unique_ptr<ComplexShape> Combine(
            const std::vector<const IShape*>& shapes, BlendOperation operation);

    private:
        // 내부 헬퍼 함수들
        static std::vector<Vector2> GenerateCurvePoints(
            CurveType curveType, const std::vector<Vector2>& controlPoints, int numPoints = 100);

        static SDFResult BlendSDFs(
            const SDFResult& sdfA, const SDFResult& sdfB, BlendOperation operation);

        static SDFResult SmoothBlendSDFs(
            const SDFResult& sdfA, const SDFResult& sdfB, BlendOperation operation, Real smoothness);

        static SDFResult RoundBlendSDFs(
            const SDFResult& sdfA, const SDFResult& sdfB, Real radius);
    };
}
