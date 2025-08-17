#pragma once

#include <vector>
#include <memory>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "GJK_EPA.hpp"
#include "SAT.hpp"
#include "ComplexShapeFactory.hpp"
#include "../Shape/IShape.hpp"

namespace Engine2D
{
    // ========================================
    // 충돌 검출 알고리즘 열거형
    // ========================================

    enum class CollisionAlgorithm
    {
        GJK_EPA,    // Gilbert-Johnson-Keerthi + Expanding Polytope Algorithm
        SAT,         // Separating Axis Theorem
        SDF,         // Signed Distance Function
        AUTO         // 자동 선택 (도형에 따라 최적의 알고리즘 선택)
    };



    // ========================================
    // 충돌 검출 결과
    // ========================================

    struct CollisionResult
    {
        bool hasCollision;           // 충돌 여부
        Vector2 normal;              // 충돌 법선 벡터
        Real depth;                  // 침투 깊이
        Vector2 contactPoint;        // 접촉점
        CollisionAlgorithm algorithm; // 사용된 알고리즘
        Real distance;               // 거리 (충돌이 없을 때)

        CollisionResult()
            : hasCollision(false), normal(0, 0), depth(0), contactPoint(0, 0),
              algorithm(CollisionAlgorithm::AUTO), distance(0) {}
    };



    // ========================================
    // 충돌 검출 시스템 클래스
    // ========================================

    class CollisionDetection
    {
    public:
        CollisionDetection();
        ~CollisionDetection();

        // ========================================
        // GJK/EPA 알고리즘
        // ========================================

        // GJK를 사용한 충돌 검출
        static bool DetectCollisionGJK(const IShape* shapeA, const IShape* shapeB);

        // EPA를 사용한 침투 정보 계산
        static CollisionInfo CalculatePenetrationEPA(const IShape* shapeA, const IShape* shapeB);

        // ========================================
        // SAT 알고리즘
        // ========================================

        // SAT를 사용한 충돌 검출 (간단한 버전)
        static bool DetectCollisionSAT(const std::vector<Vector2>& polygonA,
                                      const std::vector<Vector2>& polygonB);

        // SAT를 사용한 상세한 충돌 검출
        static SATResult DetectCollisionSATDetailed(const std::vector<Vector2>& polygonA,
                                                   const std::vector<Vector2>& polygonB);

        // 원과 다각형 간의 SAT 충돌 검출
        static bool DetectCollisionSATCirclePolygon(const Vector2& circleCenter, Real radius,
                                                   const std::vector<Vector2>& polygon);

        // 두 원 간의 SAT 충돌 검출
        static bool DetectCollisionSATCircles(const Vector2& centerA, Real radiusA,
                                             const Vector2& centerB, Real radiusB);

        // ========================================
        // SDF 기반 알고리즘
        // ========================================

        // SDF를 사용한 거리 계산
        static Real CalculateSDFDistance(const IShape* shapeA, const IShape* shapeB);

        // SDF를 사용한 충돌 검출
        static bool DetectCollisionSDF(const IShape* shapeA, const IShape* shapeB);

        // SDF를 사용한 접촉점 찾기
        static Vector2 FindSDFContactPoint(const IShape* shapeA, const IShape* shapeB);

        // ========================================
        // 최적 알고리즘 선택
        // ========================================

        // 도형에 따른 최적 알고리즘 선택
        static CollisionAlgorithm SelectOptimalAlgorithm(const IShape* shapeA, const IShape* shapeB);

        // ========================================
        // 통합 충돌 검출
        // ========================================

        // 기본 충돌 검출 (자동 알고리즘 선택)
        CollisionResult DetectCollision(const IShape* shapeA, const IShape* shapeB);

        // 특정 알고리즘으로 충돌 검출
        CollisionResult DetectCollision(const IShape* shapeA, const IShape* shapeB,
                                       CollisionAlgorithm algorithm);

        // 자동 알고리즘 선택으로 충돌 검출
        static bool DetectCollision(const IShape* shapeA, const IShape* shapeB);

        // 특정 알고리즘으로 충돌 검출
        static bool DetectCollision(const IShape* shapeA, const IShape* shapeB,
                                   CollisionAlgorithm algorithm);

        // ========================================
        // 거리 계산
        // ========================================

        // 거리 계산 (충돌이 없을 때)
        Real CalculateDistance(const IShape* shapeA, const IShape* shapeB);

        // 두 도형 간의 최단 거리 계산
        static Real CalculateMinimumDistance(const IShape* shapeA, const IShape* shapeB);

        // 도형에서 점까지의 최단 거리
        static Real CalculateDistanceToPoint(const IShape* shape, const Vector2& point);

        // ========================================
        // 접촉점 계산
        // ========================================

        // 두 도형의 접촉점들 계산
        static std::vector<Vector2> CalculateContactPoints(const IShape* shapeA, const IShape* shapeB);

        // ========================================
        // 성능 측정
        // ========================================

        // 성능 통계
        struct PerformanceStats
        {
            int totalTests;
            int gjkEpaTests;
            int satTests;
            int sdfTests;
            Real averageTime;

            PerformanceStats() : totalTests(0), gjkEpaTests(0), satTests(0), sdfTests(0), averageTime(0) {}
        };

        const PerformanceStats& GetStats() const { return stats; }
        void ResetStats() { stats = PerformanceStats(); }

        static PerformanceStats& GetStats()
        {
            static PerformanceStats stats;
            return stats;
        }

        static void ResetStats()
        {
            GetStats() = PerformanceStats();
        }

        // ========================================
        // 유틸리티 함수
        // ========================================

        // 도형이 볼록한지 확인
        static bool IsConvexShape(const IShape* shape);

        // 도형이 오목한지 확인
        static bool IsConcaveShape(const IShape* shape);

        // 두 도형이 같은 타입인지 확인
        static bool AreSameShapeType(const IShape* shapeA, const IShape* shapeB);

        // 도형의 복잡도 평가 (정점 수 기반)
        static int EvaluateShapeComplexity(const IShape* shape);

    private:
        // 개별 알고리즘 구현
        CollisionResult DetectCollisionGJK_EPA(const IShape* shapeA, const IShape* shapeB);
        CollisionResult DetectCollisionSAT(const IShape* shapeA, const IShape* shapeB);
        CollisionResult DetectCollisionSDF(const IShape* shapeA, const IShape* shapeB);

        // 성능 측정
        PerformanceStats stats;

        // 알고리즘별 적합성 판단
        bool IsGJK_EPASuitable(const IShape* shapeA, const IShape* shapeB);
        bool IsSATSuitable(const IShape* shapeA, const IShape* shapeB);
        bool IsSDFSuitable(const IShape* shapeA, const IShape* shapeB);
    };

    // ========================================
    // 편의 함수들
    // ========================================

    // 전역 충돌 검출 인스턴스
    extern CollisionDetection g_CollisionDetection;

    // 간편한 충돌 검출 함수들
    inline CollisionResult DetectCollision(const IShape* shapeA, const IShape* shapeB)
    {
        return g_CollisionDetection.DetectCollision(shapeA, shapeB);
    }

    inline CollisionResult DetectCollision(const IShape* shapeA, const IShape* shapeB,
                                          CollisionAlgorithm algorithm)
    {
        return g_CollisionDetection.DetectCollision(shapeA, shapeB, algorithm);
    }

    inline Real CalculateDistance(const IShape* shapeA, const IShape* shapeB)
    {
        return g_CollisionDetection.CalculateDistance(shapeA, shapeB);
    }

    // ========================================
    // 알고리즘별 특화 함수들
    // ========================================

    namespace CollisionUtils
    {
        // SAT 헬퍼 함수들
        bool CheckAxisSeparation(const IShape* shapeA, const IShape* shapeB, const Vector2& axis);
        Vector2 FindMinimumTranslationVector(const IShape* shapeA, const IShape* shapeB);

        // SDF 헬퍼 함수들
        Real CalculateSDFDistance(const IShape* shapeA, const IShape* shapeB);
        Vector2 FindSDFContactPoint(const IShape* shapeA, const IShape* shapeB);

        // GJK-EPA 헬퍼 함수들
        bool IsConvexShape(const IShape* shape);
        bool IsConcaveShape(const IShape* shape);
    }
}
