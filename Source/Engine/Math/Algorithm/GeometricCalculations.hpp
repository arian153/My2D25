#pragma once

#include <vector>
#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"

namespace Engine2D
{
    // ========================================
    // 기하학적 계산 알고리즘 클래스
    // ========================================

    class GeometricCalculations
    {
    public:
        // ========================================
        // 면적 계산
        // ========================================

        // 다각형 면적 계산 (Shoelace 공식)
        static Real CalculatePolygonArea(const std::vector<Vector2>& vertices);

        // 원 면적 계산
        static Real CalculateCircleArea(Real radius);

        // 타원 면적 계산
        static Real CalculateEllipseArea(Real a, Real b);

        // 삼각형 면적 계산
        static Real CalculateTriangleArea(const Vector2& a, const Vector2& b, const Vector2& c);

        // ========================================
        // 둘레 계산
        // ========================================

        // 다각형 둘레 계산
        static Real CalculatePolygonPerimeter(const std::vector<Vector2>& vertices);

        // 원 둘레 계산
        static Real CalculateCirclePerimeter(Real radius);

        // 타원 둘레 근사 계산 (Ramanujan 근사)
        static Real CalculateEllipsePerimeter(Real a, Real b);

        // ========================================
        // 중심점 계산
        // ========================================

        // 다각형 중심점 계산
        static Vector2 CalculatePolygonCentroid(const std::vector<Vector2>& vertices);

        // 삼각형 중심점 계산
        static Vector2 CalculateTriangleCentroid(const Vector2& a, const Vector2& b, const Vector2& c);

        // ========================================
        // 볼록 껍질 계산 (Graham Scan)
        // ========================================

        static std::vector<Vector2> CalculateConvexHull(const std::vector<Vector2>& points);

        // ========================================
        // 점 포함 확인
        // ========================================

        // 점이 다각형 내부에 있는지 확인 (Ray Casting)
        static bool IsPointInsidePolygon(const Vector2& point, const std::vector<Vector2>& polygon);

        // 점이 원 내부에 있는지 확인
        static bool IsPointInsideCircle(const Vector2& point, const Vector2& center, Real radius);

        // 점이 타원 내부에 있는지 확인
        static bool IsPointInsideEllipse(const Vector2& point, const Vector2& center, Real a, Real b);

        // ========================================
        // 거리 계산
        // ========================================

        // 점에서 선분까지의 최단 거리
        static Real DistancePointToLineSegment(const Vector2& point, const Vector2& lineStart, const Vector2& lineEnd);

        // 두 선분 사이의 최단 거리
        static Real DistanceLineSegmentToLineSegment(const Vector2& a1, const Vector2& a2,
                                                   const Vector2& b1, const Vector2& b2);

        // ========================================
        // 교차 확인
        // ========================================

        // 두 선분이 교차하는지 확인
        static bool DoLineSegmentsIntersect(const Vector2& p1, const Vector2& q1,
                                          const Vector2& p2, const Vector2& q2);

        // ========================================
        // 유틸리티 함수
        // ========================================

        // 세 점의 방향성 확인 (CCW, CW, Collinear)
        static int Orientation(const Vector2& p, const Vector2& q, const Vector2& r);

        // 두 벡터의 외적 (2D에서는 스칼라)
        static Real Cross2D(const Vector2& a, const Vector2& b);

        // 각도 계산 (라디안)
        static Real CalculateAngle(const Vector2& a, const Vector2& b);

        // 각도 계산 (도)
        static Real CalculateAngleDegrees(const Vector2& a, const Vector2& b);
    };
}
