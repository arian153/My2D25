#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "GeometricCalculations.hpp"

namespace Engine2D
{
    // ========================================
    // Minkowski 연산 결과 구조체
    // ========================================

    struct MinkowskiResult
    {
        std::vector<Vector2> vertices;  // 결과 다각형의 정점들
        std::vector<std::pair<int, int>> edges;  // 모서리들 (정점 인덱스 쌍)

        MinkowskiResult() = default;

        // 결과가 유효한지 확인
        bool IsValid() const { return vertices.size() >= 3; }

        // 결과 다각형의 면적 계산
        Real CalculateArea() const
        {
            return GeometricCalculations::CalculatePolygonArea(vertices);
        }

        // 결과 다각형의 둘레 계산
        Real CalculatePerimeter() const
        {
            return GeometricCalculations::CalculatePolygonPerimeter(vertices);
        }
    };

    // ========================================
    // Minkowski 연산 알고리즘 클래스
    // ========================================

    class MinkowskiAlgorithms
    {
    public:
        // ========================================
        // Minkowski 합 (Minkowski Sum)
        // ========================================

        // 두 볼록 다각형의 Minkowski 합 계산
        static MinkowskiResult CalculateSum(const std::vector<Vector2>& polygonA,
                                           const std::vector<Vector2>& polygonB);

        // 두 원의 Minkowski 합 계산
        static MinkowskiResult CalculateSumCircles(const Vector2& centerA, Real radiusA,
                                                  const Vector2& centerB, Real radiusB);

        // 원과 다각형의 Minkowski 합 계산
        static MinkowskiResult CalculateSumCirclePolygon(const Vector2& circleCenter, Real radius,
                                                        const std::vector<Vector2>& polygon);

        // ========================================
        // Minkowski 차이 (Minkowski Difference)
        // ========================================

        // 두 볼록 다각형의 Minkowski 차이 계산
        static MinkowskiResult CalculateDifference(const std::vector<Vector2>& polygonA,
                                                  const std::vector<Vector2>& polygonB);

        // 두 원의 Minkowski 차이 계산
        static MinkowskiResult CalculateDifferenceCircles(const Vector2& centerA, Real radiusA,
                                                         const Vector2& centerB, Real radiusB);

        // 원과 다각형의 Minkowski 차이 계산
        static MinkowskiResult CalculateDifferenceCirclePolygon(const Vector2& circleCenter, Real radius,
                                                               const std::vector<Vector2>& polygon);

        // ========================================
        // 충돌 검출을 위한 Minkowski 차이
        // ========================================

        // Minkowski 차이에서 원점 포함 여부 확인 (충돌 검출)
        static bool ContainsOrigin(const MinkowskiResult& result)
        {
            if (!result.IsValid()) return false;

            // 원점이 결과 다각형 내부에 있는지 확인
            return GeometricCalculations::IsPointInsidePolygon(Vector2(0, 0), result.vertices);
        }

        // 두 다각형의 충돌 검출 (Minkowski 차이 사용)
        static bool DetectCollision(const std::vector<Vector2>& polygonA,
                                   const std::vector<Vector2>& polygonB)
        {
            MinkowskiResult diff = CalculateDifference(polygonA, polygonB);
            return ContainsOrigin(diff);
        }

        // ========================================
        // 유틸리티 함수
        // ========================================

        // 두 다각형 병합 (Minkowski 합의 내부 구현)
        static std::vector<Vector2> MergePolygons(const std::vector<Vector2>& polygonA,
                                                 const std::vector<Vector2>& polygonB);

        // 정점들을 각도에 따라 정렬
        static std::vector<Vector2> SortVerticesByAngle(const std::vector<Vector2>& vertices);

        // Minkowski 연산 결과의 유효성 검사
        static bool ValidateMinkowskiResult(const MinkowskiResult& result);

        // Minkowski 연산 결과의 단순화 (불필요한 정점 제거)
        static MinkowskiResult SimplifyResult(const MinkowskiResult& result, Real tolerance = Math::EPSILON);

        // ========================================
        // 고급 Minkowski 연산
        // ========================================

        // Minkowski 합의 경계 계산
        static std::vector<Vector2> CalculateMinkowskiSumBoundary(const std::vector<Vector2>& polygonA,
                                                                 const std::vector<Vector2>& polygonB)
        {
            MinkowskiResult sum = CalculateSum(polygonA, polygonB);
            return sum.vertices;
        }

        // Minkowski 차이의 경계 계산
        static std::vector<Vector2> CalculateMinkowskiDifferenceBoundary(const std::vector<Vector2>& polygonA,
                                                                        const std::vector<Vector2>& polygonB)
        {
            MinkowskiResult diff = CalculateDifference(polygonA, polygonB);
            return diff.vertices;
        }

        // Minkowski 연산의 성능 측정
        struct PerformanceMetrics
        {
            Real computationTime;
            int vertexCount;
            int edgeCount;
            bool isValid;

            PerformanceMetrics() : computationTime(0), vertexCount(0), edgeCount(0), isValid(false) {}
        };

        static PerformanceMetrics GetPerformanceMetrics(const MinkowskiResult& result)
        {
            PerformanceMetrics metrics;
            metrics.vertexCount = static_cast<int>(result.vertices.size());
            metrics.edgeCount = static_cast<int>(result.edges.size());
            metrics.isValid = result.IsValid();
            return metrics;
        }
    };
}
