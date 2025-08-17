#include "MinkowskiAlgorithms.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // Minkowski 합 (Minkowski Sum)
    // ========================================

    MinkowskiResult MinkowskiAlgorithms::CalculateSum(const std::vector<Vector2>& polygonA,
                                                     const std::vector<Vector2>& polygonB)
    {
        MinkowskiResult result;

        if (polygonA.size() < 3 || polygonB.size() < 3) {
            return result; // 유효하지 않은 다각형
        }

        // 두 다각형의 모든 정점 쌍을 더해서 새로운 정점들 생성
        std::vector<Vector2> sumVertices;
        for (const auto& vertexA : polygonA) {
            for (const auto& vertexB : polygonB) {
                sumVertices.push_back(vertexA + vertexB);
            }
        }

        // 볼록 껍질 계산
        result.vertices = GeometricCalculations::CalculateConvexHull(sumVertices);

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    MinkowskiResult MinkowskiAlgorithms::CalculateSumCircles(const Vector2& centerA, Real radiusA,
                                                            const Vector2& centerB, Real radiusB)
    {
        MinkowskiResult result;

        // 두 원의 Minkowski 합은 중심이 더해지고 반지름이 더해진 원
        Vector2 newCenter = centerA + centerB;
        Real newRadius = radiusA + radiusB;

        // 원을 다각형으로 근사 (32개의 정점)
        const int numVertices = 32;
        for (int i = 0; i < numVertices; ++i) {
            Real angle = Real(2) * PI * i / numVertices;
            Vector2 vertex = newCenter + Vector2(std::cos(angle), std::sin(angle)) * newRadius;
            result.vertices.push_back(vertex);
        }

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    // ========================================
    // Minkowski 차 (Minkowski Difference)
    // ========================================

    MinkowskiResult MinkowskiAlgorithms::CalculateDifference(const std::vector<Vector2>& polygonA,
                                                            const std::vector<Vector2>& polygonB)
    {
        MinkowskiResult result;

        if (polygonA.size() < 3 || polygonB.size() < 3) {
            return result; // 유효하지 않은 다각형
        }

        // 두 다각형의 모든 정점 쌍을 빼서 새로운 정점들 생성
        std::vector<Vector2> diffVertices;
        for (const auto& vertexA : polygonA) {
            for (const auto& vertexB : polygonB) {
                diffVertices.push_back(vertexA - vertexB);
            }
        }

        // 볼록 껍질 계산
        result.vertices = GeometricCalculations::CalculateConvexHull(diffVertices);

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    MinkowskiResult MinkowskiAlgorithms::CalculateDifferenceCircles(const Vector2& centerA, Real radiusA,
                                                                   const Vector2& centerB, Real radiusB)
    {
        MinkowskiResult result;

        // 두 원의 Minkowski 차는 중심이 빼지고 반지름이 더해진 원
        Vector2 newCenter = centerA - centerB;
        Real newRadius = radiusA + radiusB;

        // 원을 다각형으로 근사 (32개의 정점)
        const int numVertices = 32;
        for (int i = 0; i < numVertices; ++i) {
            Real angle = Real(2) * PI * i / numVertices;
            Vector2 vertex = newCenter + Vector2(std::cos(angle), std::sin(angle)) * newRadius;
            result.vertices.push_back(vertex);
        }

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    // ========================================
    // 고급 Minkowski 연산
    // ========================================

    MinkowskiResult MinkowskiAlgorithms::CalculateConvexHull(const std::vector<Vector2>& points)
    {
        MinkowskiResult result;

        if (points.size() < 3) {
            return result; // 볼록 껍질을 만들기에는 점이 부족
        }

        // Graham scan 알고리즘으로 볼록 껍질 계산
        result.vertices = GeometricCalculations::CalculateConvexHull(points);

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    bool MinkowskiAlgorithms::IsPointInside(const Vector2& point, const MinkowskiResult& minkowskiResult)
    {
        if (!minkowskiResult.IsValid()) return false;

        // 점이 볼록 다각형 내부에 있는지 확인
        return GeometricCalculations::IsPointInsidePolygon(point, minkowskiResult.vertices);
    }

    Real MinkowskiAlgorithms::CalculateDistance(const Vector2& point, const MinkowskiResult& minkowskiResult)
    {
        if (!minkowskiResult.IsValid()) return std::numeric_limits<Real>::max();

        // 점에서 볼록 다각형까지의 최단 거리 계산
        Real minDistance = std::numeric_limits<Real>::max();

        for (const auto& edge : minkowskiResult.edges) {
            Vector2 v1 = minkowskiResult.vertices[edge.first];
            Vector2 v2 = minkowskiResult.vertices[edge.second];

            // 선분에서 점까지의 거리 계산
            Vector2 edgeVector = v2 - v1;
            Vector2 toPoint = point - v1;

            Real t = toPoint.Dot(edgeVector) / edgeVector.Dot(edgeVector);
            t = std::clamp(t, Real(0), Real(1));

            Vector2 closestPoint = v1 + edgeVector * t;
            Real distance = (point - closestPoint).Length();

            if (distance < minDistance) {
                minDistance = distance;
            }
        }

        return minDistance;
    }

    // ========================================
    // 추가 구현 함수들
    // ========================================

    MinkowskiResult MinkowskiAlgorithms::CalculateSumCirclePolygon(const Vector2& circleCenter, Real radius,
                                                                  const std::vector<Vector2>& polygon)
    {
        MinkowskiResult result;

        if (polygon.size() < 3) {
            return result; // 유효하지 않은 다각형
        }

        // 원을 다각형으로 근사 (16개의 정점)
        const int numCircleVertices = 16;
        std::vector<Vector2> circleVertices;
        for (int i = 0; i < numCircleVertices; ++i) {
            Real angle = Real(2) * PI * i / numCircleVertices;
            Vector2 vertex = circleCenter + Vector2(std::cos(angle), std::sin(angle)) * radius;
            circleVertices.push_back(vertex);
        }

        // 원 다각형과 입력 다각형의 Minkowski 합 계산
        return CalculateSum(circleVertices, polygon);
    }

    MinkowskiResult MinkowskiAlgorithms::CalculateDifference(const std::vector<Vector2>& polygonA,
                                                            const std::vector<Vector2>& polygonB)
    {
        MinkowskiResult result;

        if (polygonA.size() < 3 || polygonB.size() < 3) {
            return result; // 유효하지 않은 다각형
        }

        // 두 다각형의 모든 정점 쌍을 빼서 새로운 정점들 생성
        std::vector<Vector2> diffVertices;
        for (const auto& vertexA : polygonA) {
            for (const auto& vertexB : polygonB) {
                diffVertices.push_back(vertexA - vertexB);
            }
        }

        // 볼록 껍질 계산
        result.vertices = GeometricCalculations::CalculateConvexHull(diffVertices);

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    MinkowskiResult MinkowskiAlgorithms::CalculateDifferenceCircles(const Vector2& centerA, Real radiusA,
                                                                   const Vector2& centerB, Real radiusB)
    {
        MinkowskiResult result;

        // 두 원의 Minkowski 차이는 중심이 빼지고 반지름이 빼진 원
        Vector2 newCenter = centerA - centerB;
        Real newRadius = std::abs(radiusA - radiusB);

        // 원을 다각형으로 근사 (32개의 정점)
        const int numVertices = 32;
        for (int i = 0; i < numVertices; ++i) {
            Real angle = Real(2) * PI * i / numVertices;
            Vector2 vertex = newCenter + Vector2(std::cos(angle), std::sin(angle)) * newRadius;
            result.vertices.push_back(vertex);
        }

        // 모서리 정보 생성
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            size_t j = (i + 1) % result.vertices.size();
            result.edges.push_back({static_cast<int>(i), static_cast<int>(j)});
        }

        return result;
    }

    MinkowskiResult MinkowskiAlgorithms::CalculateDifferenceCirclePolygon(const Vector2& circleCenter, Real radius,
                                                                         const std::vector<Vector2>& polygon)
    {
        MinkowskiResult result;

        if (polygon.size() < 3) {
            return result; // 유효하지 않은 다각형
        }

        // 원을 다각형으로 근사 (16개의 정점)
        const int numCircleVertices = 16;
        std::vector<Vector2> circleVertices;
        for (int i = 0; i < numCircleVertices; ++i) {
            Real angle = Real(2) * PI * i / numCircleVertices;
            Vector2 vertex = circleCenter + Vector2(std::cos(angle), std::sin(angle)) * radius;
            circleVertices.push_back(vertex);
        }

        // 원 다각형과 입력 다각형의 Minkowski 차이 계산
        return CalculateDifference(circleVertices, polygon);
    }

    std::vector<Vector2> MinkowskiAlgorithms::MergePolygons(const std::vector<Vector2>& polygonA,
                                                           const std::vector<Vector2>& polygonB)
    {
        if (polygonA.empty()) return polygonB;
        if (polygonB.empty()) return polygonA;

        std::vector<Vector2> merged;
        merged.reserve(polygonA.size() + polygonB.size());

        // 모든 정점 쌍을 더해서 새로운 정점들 생성
        for (const auto& vertexA : polygonA) {
            for (const auto& vertexB : polygonB) {
                merged.push_back(vertexA + vertexB);
            }
        }

        return merged;
    }

    std::vector<Vector2> MinkowskiAlgorithms::SortVerticesByAngle(const std::vector<Vector2>& vertices)
    {
        if (vertices.size() < 3) return vertices;

        // 중심점 계산
        Vector2 center = GeometricCalculations::CalculatePolygonCentroid(vertices);

        // 중심점을 기준으로 각도 정렬
        std::vector<Vector2> sorted = vertices;
        std::sort(sorted.begin(), sorted.end(),
            [center](const Vector2& a, const Vector2& b) {
                Vector2 va = a - center;
                Vector2 vb = b - center;
                return std::atan2(va.y, va.x) < std::atan2(vb.y, vb.x);
            });

        return sorted;
    }

    bool MinkowskiAlgorithms::ValidateMinkowskiResult(const MinkowskiResult& result)
    {
        if (!result.IsValid()) return false;

        // 정점 수와 모서리 수의 일관성 확인
        if (result.edges.size() != result.vertices.size()) return false;

        // 모든 모서리가 유효한 정점 인덱스를 참조하는지 확인
        for (const auto& edge : result.edges) {
            if (edge.first < 0 || edge.first >= static_cast<int>(result.vertices.size()) ||
                edge.second < 0 || edge.second >= static_cast<int>(result.vertices.size())) {
                return false;
            }
        }

        // 면적이 양수인지 확인
        if (result.CalculateArea() <= 0) return false;

        return true;
    }

    MinkowskiResult MinkowskiAlgorithms::SimplifyResult(const MinkowskiResult& result, Real tolerance)
    {
        if (!result.IsValid()) return result;

        MinkowskiResult simplified;
        std::vector<bool> keepVertex(result.vertices.size(), true);

        // 거의 같은 위치의 정점들을 제거
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            for (size_t j = i + 1; j < result.vertices.size(); ++j) {
                if ((result.vertices[i] - result.vertices[j]).LengthSquared() < tolerance * tolerance) {
                    keepVertex[j] = false;
                }
            }
        }

        // 유지할 정점들만 복사
        std::vector<int> newIndices(result.vertices.size(), -1);
        int newIndex = 0;
        for (size_t i = 0; i < result.vertices.size(); ++i) {
            if (keepVertex[i]) {
                simplified.vertices.push_back(result.vertices[i]);
                newIndices[i] = newIndex++;
            }
        }

        // 모서리 정보 업데이트
        for (const auto& edge : result.edges) {
            if (keepVertex[edge.first] && keepVertex[edge.second]) {
                simplified.edges.push_back({newIndices[edge.first], newIndices[edge.second]});
            }
        }

        return simplified;
    }
}
