#include "GeometricCalculations.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 면적 계산
    // ========================================

    Real GeometricCalculations::CalculatePolygonArea(const std::vector<Vector2>& vertices)
    {
        if (vertices.size() < 3) return 0;

        Real area = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            area += vertices[i].x * vertices[j].y;
            area -= vertices[j].x * vertices[i].y;
        }
        return std::abs(area) * Real(0.5);
    }

    Real GeometricCalculations::CalculateCircleArea(Real radius)
    {
        return PI * radius * radius;
    }

    Real GeometricCalculations::CalculateEllipseArea(Real a, Real b)
    {
        return PI * a * b;
    }

    Real GeometricCalculations::CalculateTriangleArea(const Vector2& a, const Vector2& b, const Vector2& c)
    {
        return std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) * Real(0.5);
    }

    // ========================================
    // 둘레 계산
    // ========================================

    Real GeometricCalculations::CalculatePolygonPerimeter(const std::vector<Vector2>& vertices)
    {
        if (vertices.size() < 2) return 0;

        Real perimeter = 0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            perimeter += (vertices[j] - vertices[i]).Length();
        }
        return perimeter;
    }

    Real GeometricCalculations::CalculateCirclePerimeter(Real radius)
    {
        return Real(2) * PI * radius;
    }

    Real GeometricCalculations::CalculateEllipsePerimeter(Real a, Real b)
    {
        Real h = ((a - b) * (a - b)) / ((a + b) * (a + b));
        return PI * (a + b) * (1 + (3 * h) / (10 + std::sqrt(4 - 3 * h)));
    }

    // ========================================
    // 중심점 계산
    // ========================================

    Vector2 GeometricCalculations::CalculatePolygonCentroid(const std::vector<Vector2>& vertices)
    {
        if (vertices.empty()) return Vector2(0, 0);
        if (vertices.size() == 1) return vertices[0];

        Vector2 centroid(0, 0);
        Real area = 0;

        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            Real cross = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
            area += cross;
            centroid.x += (vertices[i].x + vertices[j].x) * cross;
            centroid.y += (vertices[i].y + vertices[j].y) * cross;
        }

        area *= Real(0.5);
        if (std::abs(area) > Math::EPSILON) {
            centroid.x /= (6 * area);
            centroid.y /= (6 * area);
        } else {
            // 면적이 0인 경우 단순 평균
            for (const auto& vertex : vertices) {
                centroid += vertex;
            }
            centroid /= static_cast<Real>(vertices.size());
        }

        return centroid;
    }

    Vector2 GeometricCalculations::CalculateTriangleCentroid(const Vector2& a, const Vector2& b, const Vector2& c)
    {
        return (a + b + c) / Real(3);
    }

    // ========================================
    // 볼록 껍질 계산 (Graham Scan)
    // ========================================

    std::vector<Vector2> GeometricCalculations::CalculateConvexHull(const std::vector<Vector2>& points)
    {
        if (points.size() < 3) return points;

        // 최하단 점 찾기
        size_t lowest = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].y < points[lowest].y ||
                (points[i].y == points[lowest].y && points[i].x < points[lowest].x)) {
                lowest = i;
            }
        }

        // 최하단 점을 기준으로 각도 정렬
        std::vector<Vector2> sortedPoints = points;
        std::swap(sortedPoints[0], sortedPoints[lowest]);

        Vector2 pivot = sortedPoints[0];
        std::sort(sortedPoints.begin() + 1, sortedPoints.end(),
            [pivot](const Vector2& a, const Vector2& b) {
                Vector2 va = a - pivot;
                Vector2 vb = b - pivot;
                Real cross = va.x * vb.y - va.y * vb.x;
                if (std::abs(cross) < Math::EPSILON) {
                    return va.LengthSquared() < vb.LengthSquared();
                }
                return cross > 0;
            });

        // Graham Scan
        std::vector<Vector2> hull;
        hull.push_back(sortedPoints[0]);
        hull.push_back(sortedPoints[1]);

        for (size_t i = 2; i < sortedPoints.size(); ++i) {
            while (hull.size() > 1) {
                Vector2 p1 = hull[hull.size() - 2];
                Vector2 p2 = hull[hull.size() - 1];
                Vector2 p3 = sortedPoints[i];

                Vector2 v1 = p2 - p1;
                Vector2 v2 = p3 - p2;
                Real cross = v1.x * v2.y - v1.y * v2.x;

                if (cross > 0) break;
                hull.pop_back();
            }
            hull.push_back(sortedPoints[i]);
        }

        return hull;
    }

    // ========================================
    // 점 포함 확인
    // ========================================

    bool GeometricCalculations::IsPointInsidePolygon(const Vector2& point, const std::vector<Vector2>& polygon)
    {
        if (polygon.size() < 3) return false;

        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
                (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
                 (polygon[j].y - polygon[i].y) + polygon[i].x)) {
                inside = !inside;
            }
        }
        return inside;
    }

    bool GeometricCalculations::IsPointInsideCircle(const Vector2& point, const Vector2& center, Real radius)
    {
        Vector2 toPoint = point - center;
        return toPoint.LengthSquared() <= radius * radius;
    }

    bool GeometricCalculations::IsPointInsideEllipse(const Vector2& point, const Vector2& center, Real a, Real b)
    {
        Vector2 toPoint = point - center;
        Real normalizedX = toPoint.x / a;
        Real normalizedY = toPoint.y / b;
        return normalizedX * normalizedX + normalizedY * normalizedY <= 1;
    }

    // ========================================
    // 거리 계산
    // ========================================

    Real GeometricCalculations::DistancePointToLineSegment(const Vector2& point, const Vector2& lineStart, const Vector2& lineEnd)
    {
        Vector2 line = lineEnd - lineStart;
        Vector2 toPoint = point - lineStart;

        Real t = toPoint.Dot(line) / line.LengthSquared();
        t = std::clamp(t, Real(0), Real(1));

        Vector2 closest = lineStart + line * t;
        return (point - closest).Length();
    }

    Real GeometricCalculations::DistanceLineSegmentToLineSegment(const Vector2& a1, const Vector2& a2,
                                                               const Vector2& b1, const Vector2& b2)
    {
        // 두 선분이 교차하는지 확인
        if (DoLineSegmentsIntersect(a1, a2, b1, b2)) {
            return 0;
        }

        // 각 선분의 끝점에서 다른 선분까지의 거리 중 최솟값
        Real minDist = std::numeric_limits<Real>::max();
        minDist = std::min(minDist, DistancePointToLineSegment(a1, b1, b2));
        minDist = std::min(minDist, DistancePointToLineSegment(a2, b1, b2));
        minDist = std::min(minDist, DistancePointToLineSegment(b1, a1, a2));
        minDist = std::min(minDist, DistancePointToLineSegment(b2, a1, a2));

        return minDist;
    }

    // ========================================
    // 교차 확인
    // ========================================

    bool GeometricCalculations::DoLineSegmentsIntersect(const Vector2& p1, const Vector2& q1,
                                                       const Vector2& p2, const Vector2& q2)
    {
        // 방향성 확인
        auto orientation = [](const Vector2& p, const Vector2& q, const Vector2& r) -> int {
            Real val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
            if (std::abs(val) < Math::EPSILON) return 0; // Collinear
            return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
        };

        auto onSegment = [](const Vector2& p, const Vector2& q, const Vector2& r) -> bool {
            return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
                   q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
        };

        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // 일반적인 경우
        if (o1 != o2 && o3 != o4) return true;

        // 특수한 경우 (Collinear)
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;
    }

    // ========================================
    // 유틸리티 함수
    // ========================================

    int GeometricCalculations::Orientation(const Vector2& p, const Vector2& q, const Vector2& r)
    {
        Real val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (std::abs(val) < Math::EPSILON) return 0; // Collinear
        return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
    }

    Real GeometricCalculations::Cross2D(const Vector2& a, const Vector2& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    Real GeometricCalculations::CalculateAngle(const Vector2& a, const Vector2& b)
    {
        Real dot = a.Dot(b);
        Real det = Cross2D(a, b);
        return std::atan2(det, dot);
    }

    Real GeometricCalculations::CalculateAngleDegrees(const Vector2& a, const Vector2& b)
    {
        return CalculateAngle(a, b) * 180.0 / PI;
    }
}
