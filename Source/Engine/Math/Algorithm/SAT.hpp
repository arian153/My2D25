#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"

namespace Engine2D
{
    // ========================================
    // SAT: Separating Axis Theorem
    // ========================================

    // SAT 충돌 검출 결과
    struct SATResult
    {
        bool hasCollision;     // 충돌 여부
        Vector2 separationAxis; // 분리축 (충돌이 없을 때)
        Real overlap;          // 겹침 정도 (충돌이 있을 때)
        Vector2 normal;        // 충돌 법선 벡터

        SATResult() : hasCollision(false), separationAxis(0, 0), overlap(0), normal(0, 0) {}
    };

    // 투영 정보
    struct Projection
    {
        Real min;  // 최소값
        Real max;  // 최대값

        Projection() : min(0), max(0) {}
        Projection(Real minVal, Real maxVal) : min(minVal), max(maxVal) {}

        // 두 투영이 겹치는지 확인
        bool Overlaps(const Projection& other) const;

        // 겹침 정도 계산
        Real GetOverlap(const Projection& other) const;
    };

    // ========================================
    // SAT 알고리즘 구현
    // ========================================

    namespace SAT
    {
        // 두 볼록 다각형 간의 충돌 검출
        SATResult DetectCollision(const std::vector<Vector2>& polygonA,
                                 const std::vector<Vector2>& polygonB);

        // 원과 다각형 간의 충돌 검출
        SATResult DetectCirclePolygonCollision(const Vector2& circleCenter, Real radius,
                                              const std::vector<Vector2>& polygon);

        // 두 원 간의 충돌 검출
        SATResult DetectCircleCollision(const Vector2& centerA, Real radiusA,
                                       const Vector2& centerB, Real radiusB);

        // 내부 구현 함수들
        std::vector<Vector2> GetAxes(const std::vector<Vector2>& polygon);

        Projection ProjectPolygon(const std::vector<Vector2>& polygon, const Vector2& axis);

        Projection ProjectCircle(const Vector2& center, Real radius, const Vector2& axis);

        bool TestAxis(const std::vector<Vector2>& polygonA, const std::vector<Vector2>& polygonB,
                      const Vector2& axis, SATResult& result);

        bool TestCirclePolygonAxis(const Vector2& circleCenter, Real radius,
                                  const std::vector<Vector2>& polygon, const Vector2& axis,
                                  SATResult& result);
    }

    // ========================================
    // 고급 SAT 기능
    // ========================================

    namespace SATAdvanced
    {
        // 최소 분리 벡터 (MTV: Minimum Translation Vector) 계산
        Vector2 CalculateMTV(const std::vector<Vector2>& polygonA,
                            const std::vector<Vector2>& polygonB);

        // 충돌 해결을 위한 이동 벡터 계산
        Vector2 CalculateSeparationVector(const std::vector<Vector2>& polygonA,
                                         const std::vector<Vector2>& polygonB);

        // 접촉점 계산
        std::vector<Vector2> CalculateContactPoints(const std::vector<Vector2>& polygonA,
                                                   const std::vector<Vector2>& polygonB);

        // 충돌 깊이 계산
        Real CalculatePenetrationDepth(const std::vector<Vector2>& polygonA,
                                      const std::vector<Vector2>& polygonB);

        // 내부 구현 함수들
        Vector2 FindClosestPointOnPolygon(const Vector2& point, const std::vector<Vector2>& polygon);

        std::vector<Vector2> FindIntersectionPoints(const std::vector<Vector2>& polygonA,
                                                   const std::vector<Vector2>& polygonB);
    }

    // ========================================
    // SAT 유틸리티 함수들
    // ========================================

    namespace SATUtils
    {
        // 다각형이 볼록한지 확인
        bool IsConvex(const std::vector<Vector2>& polygon);

        // 다각형의 법선 벡터들 계산
        std::vector<Vector2> CalculateNormals(const std::vector<Vector2>& polygon);

        // 점이 다각형 내부에 있는지 확인
        bool IsPointInsidePolygon(const Vector2& point, const std::vector<Vector2>& polygon);

        // 두 선분이 교차하는지 확인
        bool DoLinesIntersect(const Vector2& p1, const Vector2& p2,
                             const Vector2& p3, const Vector2& p4);

        // 선분 교차점 계산
        Vector2 CalculateIntersectionPoint(const Vector2& p1, const Vector2& p2,
                                          const Vector2& p3, const Vector2& p4);

        // 벡터의 수직 벡터 계산
        Vector2 GetPerpendicular(const Vector2& vector);

        // 벡터 정규화 (안전한 버전)
        Vector2 SafeNormalize(const Vector2& vector);
    }

    // ========================================
    // 구현 세부사항
    // ========================================

    // Projection 구조체의 멤버 함수들은 cpp 파일에서 구현됨
}
