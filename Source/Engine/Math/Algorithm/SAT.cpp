#include "SAT.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Engine2D
{
    // ========================================
    // Projection 구조체 구현
    // ========================================

    bool Projection::Overlaps(const Projection& other) const
    {
        return !(max < other.min || other.max < min);
    }

    Real Projection::GetOverlap(const Projection& other) const
    {
        if (!Overlaps(other)) return 0;

        Real overlap1 = max - other.min;
        Real overlap2 = other.max - min;

        return std::min(overlap1, overlap2);
    }

    // ========================================
    // SAT 알고리즘 구현
    // ========================================

    SATResult SAT::DetectCollision(const std::vector<Vector2>& polygonA,
                                  const std::vector<Vector2>& polygonB)
    {
        SATResult result;

        if (polygonA.size() < 3 || polygonB.size() < 3) {
            return result; // 유효하지 않은 다각형
        }

        // 모든 축에 대해 테스트
        std::vector<Vector2> axes = GetAxes(polygonA);
        std::vector<Vector2> axesB = GetAxes(polygonB);
        axes.insert(axes.end(), axesB.begin(), axesB.end());

        // 중복 제거 (정규화된 벡터 비교)
        std::sort(axes.begin(), axes.end(), [](const Vector2& a, const Vector2& b) {
            if (std::abs(a.x - b.x) < 1e-6) return a.y < b.y;
            return a.x < b.x;
        });
        axes.erase(std::unique(axes.begin(), axes.end(), [](const Vector2& a, const Vector2& b) {
            return (a - b).LengthSquared() < 1e-12;
        }), axes.end());

        Real minOverlap = std::numeric_limits<Real>::max();
        Vector2 minAxis(0, 0);

        for (const auto& axis : axes) {
            if (TestAxis(polygonA, polygonB, axis, result)) {
                if (result.overlap < minOverlap) {
                    minOverlap = result.overlap;
                    minAxis = axis;
                }
            } else {
                // 분리축 발견
                result.hasCollision = false;
                result.separationAxis = axis;
                return result;
            }
        }

        // 모든 축에서 겹침이 발견됨
        result.hasCollision = true;
        result.overlap = minOverlap;
        result.normal = minAxis;

        return result;
    }

    SATResult SAT::DetectCirclePolygonCollision(const Vector2& circleCenter, Real radius,
                                               const std::vector<Vector2>& polygon)
    {
        SATResult result;

        if (polygon.size() < 3) return result;

        // 다각형의 축들
        std::vector<Vector2> axes = GetAxes(polygon);

        // 원의 중심에서 다각형의 각 정점으로의 축 추가
        for (const auto& vertex : polygon) {
            Vector2 axis = (vertex - circleCenter).Normalized();
            axes.push_back(axis);
        }

        Real minOverlap = std::numeric_limits<Real>::max();
        Vector2 minAxis(0, 0);

        for (const auto& axis : axes) {
            if (TestCirclePolygonAxis(circleCenter, radius, polygon, axis, result)) {
                if (result.overlap < minOverlap) {
                    minOverlap = result.overlap;
                    minAxis = axis;
                }
            } else {
                // 분리축 발견
                result.hasCollision = false;
                result.separationAxis = axis;
                return result;
            }
        }

        // 모든 축에서 겹침이 발견됨
        result.hasCollision = true;
        result.overlap = minOverlap;
        result.normal = minAxis;

        return result;
    }

    SATResult SAT::DetectCircleCollision(const Vector2& centerA, Real radiusA,
                                        const Vector2& centerB, Real radiusB)
    {
        SATResult result;

        // 두 원 사이의 거리
        Vector2 distance = centerB - centerA;
        Real distanceLength = distance.Length();

        // 반지름의 합
        Real radiusSum = radiusA + radiusB;

        if (distanceLength < radiusSum) {
            // 충돌 발생
            result.hasCollision = true;
            result.overlap = radiusSum - distanceLength;
            result.normal = distance.Normalized();
        } else {
            // 충돌 없음
            result.hasCollision = false;
            result.separationAxis = distance.Normalized();
        }

        return result;
    }

    std::vector<Vector2> SAT::GetAxes(const std::vector<Vector2>& polygon)
    {
        std::vector<Vector2> axes;

        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            Vector2 edge = polygon[j] - polygon[i];
            Vector2 normal = SATUtils::GetPerpendicular(edge);
            axes.push_back(SATUtils::SafeNormalize(normal));
        }

        return axes;
    }

    Projection SAT::ProjectPolygon(const std::vector<Vector2>& polygon, const Vector2& axis)
    {
        if (polygon.empty()) return Projection();

        Real min = polygon[0].Dot(axis);
        Real max = min;

        for (size_t i = 1; i < polygon.size(); ++i) {
            Real projection = polygon[i].Dot(axis);
            min = std::min(min, projection);
            max = std::max(max, projection);
        }

        return Projection(min, max);
    }

    Projection SAT::ProjectCircle(const Vector2& center, Real radius, const Vector2& axis)
    {
        Real centerProjection = center.Dot(axis);
        return Projection(centerProjection - radius, centerProjection + radius);
    }

    bool SAT::TestAxis(const std::vector<Vector2>& polygonA, const std::vector<Vector2>& polygonB,
                       const Vector2& axis, SATResult& result)
    {
        Projection projA = ProjectPolygon(polygonA, axis);
        Projection projB = ProjectPolygon(polygonB, axis);

        if (!projA.Overlaps(projB)) {
            return false; // 분리축 발견
        }

        result.overlap = projA.GetOverlap(projB);
        return true;
    }

    bool SAT::TestCirclePolygonAxis(const Vector2& circleCenter, Real radius,
                                    const std::vector<Vector2>& polygon, const Vector2& axis,
                                    SATResult& result)
    {
        Projection circleProj = ProjectCircle(circleCenter, radius, axis);
        Projection polyProj = ProjectPolygon(polygon, axis);

        if (!circleProj.Overlaps(polyProj)) {
            return false; // 분리축 발견
        }

        result.overlap = circleProj.GetOverlap(polyProj);
        return true;
    }

    // ========================================
    // 고급 SAT 기능 구현
    // ========================================

    Vector2 SATAdvanced::CalculateMTV(const std::vector<Vector2>& polygonA,
                                     const std::vector<Vector2>& polygonB)
    {
        SATResult result = SAT::DetectCollision(polygonA, polygonB);

        if (!result.hasCollision) {
            return Vector2(0, 0);
        }

        return result.normal * result.overlap;
    }

    Vector2 SATAdvanced::CalculateSeparationVector(const std::vector<Vector2>& polygonA,
                                                  const std::vector<Vector2>& polygonB)
    {
        SATResult result = SAT::DetectCollision(polygonA, polygonB);

        if (result.hasCollision) {
            return Vector2(0, 0); // 충돌이 있으면 분리 벡터 없음
        }

        // 분리축을 따라 최소 분리 거리 계산
        Projection projA = SAT::ProjectPolygon(polygonA, result.separationAxis);
        Projection projB = SAT::ProjectPolygon(polygonB, result.separationAxis);

        Real separationDistance = 0;
        if (projA.max < projB.min) {
            separationDistance = projB.min - projA.max;
        } else {
            separationDistance = projA.min - projB.max;
        }

        return result.separationAxis * separationDistance;
    }

    std::vector<Vector2> SATAdvanced::CalculateContactPoints(const std::vector<Vector2>& polygonA,
                                                            const std::vector<Vector2>& polygonB)
    {
        std::vector<Vector2> contactPoints;

        // 각 다각형의 정점이 다른 다각형 내부에 있는지 확인
        for (const auto& vertex : polygonA) {
            if (SATUtils::IsPointInsidePolygon(vertex, polygonB)) {
                contactPoints.push_back(vertex);
            }
        }

        for (const auto& vertex : polygonB) {
            if (SATUtils::IsPointInsidePolygon(vertex, polygonA)) {
                contactPoints.push_back(vertex);
            }
        }

        // 모서리 교차점 찾기
        std::vector<Vector2> intersectionPoints = FindIntersectionPoints(polygonA, polygonB);
        contactPoints.insert(contactPoints.end(), intersectionPoints.begin(), intersectionPoints.end());

        return contactPoints;
    }

    Real SATAdvanced::CalculatePenetrationDepth(const std::vector<Vector2>& polygonA,
                                               const std::vector<Vector2>& polygonB)
    {
        SATResult result = SAT::DetectCollision(polygonA, polygonB);
        return result.hasCollision ? result.overlap : 0;
    }

    Vector2 SATAdvanced::FindClosestPointOnPolygon(const Vector2& point, const std::vector<Vector2>& polygon)
    {
        if (polygon.empty()) return point;

        Vector2 closestPoint = polygon[0];
        Real minDistance = (point - polygon[0]).LengthSquared();

        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            Vector2 edgeStart = polygon[i];
            Vector2 edgeEnd = polygon[j];

            // 선분에서 가장 가까운 점 계산
            Vector2 edge = edgeEnd - edgeStart;
            Vector2 toPoint = point - edgeStart;

            Real t = toPoint.Dot(edge) / edge.Dot(edge);
            t = std::clamp(t, Real(0), Real(1));

            Vector2 closestOnEdge = edgeStart + edge * t;
            Real distance = (point - closestOnEdge).LengthSquared();

            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = closestOnEdge;
            }
        }

        return closestPoint;
    }

    std::vector<Vector2> SATAdvanced::FindIntersectionPoints(const std::vector<Vector2>& polygonA,
                                                            const std::vector<Vector2>& polygonB)
    {
        std::vector<Vector2> intersectionPoints;

        for (size_t i = 0; i < polygonA.size(); ++i) {
            size_t nextI = (i + 1) % polygonA.size();
            Vector2 a1 = polygonA[i];
            Vector2 a2 = polygonA[nextI];

            for (size_t j = 0; j < polygonB.size(); ++j) {
                size_t nextJ = (j + 1) % polygonB.size();
                Vector2 b1 = polygonB[j];
                Vector2 b2 = polygonB[nextJ];

                if (SATUtils::DoLinesIntersect(a1, a2, b1, b2)) {
                    Vector2 intersection = SATUtils::CalculateIntersectionPoint(a1, a2, b1, b2);
                    intersectionPoints.push_back(intersection);
                }
            }
        }

        return intersectionPoints;
    }

    // ========================================
    // SAT 유틸리티 함수 구현
    // ========================================

    bool SATUtils::IsConvex(const std::vector<Vector2>& polygon)
    {
        if (polygon.size() < 3) return false;

        int sign = 0;
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            size_t k = (i + 2) % polygon.size();

            Vector2 v1 = polygon[j] - polygon[i];
            Vector2 v2 = polygon[k] - polygon[j];

            Real cross = v1.x * v2.y - v1.y * v2.x;

            if (cross != 0) {
                if (sign == 0) {
                    sign = (cross > 0) ? 1 : -1;
                } else if ((cross > 0 && sign < 0) || (cross < 0 && sign > 0)) {
                    return false; // 볼록하지 않음
                }
            }
        }

        return true;
    }

    std::vector<Vector2> SATUtils::CalculateNormals(const std::vector<Vector2>& polygon)
    {
        std::vector<Vector2> normals;

        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            Vector2 edge = polygon[j] - polygon[i];
            Vector2 normal = GetPerpendicular(edge);
            normals.push_back(SafeNormalize(normal));
        }

        return normals;
    }

    bool SATUtils::IsPointInsidePolygon(const Vector2& point, const std::vector<Vector2>& polygon)
    {
        if (polygon.size() < 3) return false;

        bool inside = false;
        size_t j = polygon.size() - 1;

        for (size_t i = 0; i < polygon.size(); ++i) {
            if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
                (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
                          (polygon[j].y - polygon[i].y) + polygon[i].x)) {
                inside = !inside;
            }
            j = i;
        }

        return inside;
    }

    bool SATUtils::DoLinesIntersect(const Vector2& p1, const Vector2& p2,
                                   const Vector2& p3, const Vector2& p4)
    {
        Real denominator = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);

        if (std::abs(denominator) < 1e-9) return false; // 평행한 선분

        Real ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / denominator;
        Real ub = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / denominator;

        return (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1);
    }

    Vector2 SATUtils::CalculateIntersectionPoint(const Vector2& p1, const Vector2& p2,
                                                const Vector2& p3, const Vector2& p4)
    {
        Real denominator = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);

        if (std::abs(denominator) < 1e-9) {
            return p1; // 평행한 경우 첫 번째 점 반환
        }

        Real ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / denominator;

        return p1 + (p2 - p1) * ua;
    }

    Vector2 SATUtils::GetPerpendicular(const Vector2& vector)
    {
        return Vector2(-vector.y, vector.x);
    }

    Vector2 SATUtils::SafeNormalize(const Vector2& vector)
    {
        Real length = vector.Length();
        if (length < 1e-9) return Vector2(0, 0);
        return vector / length;
    }
}
