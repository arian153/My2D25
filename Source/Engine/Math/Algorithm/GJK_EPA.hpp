#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"

namespace Engine2D
{
    // ========================================
    // 데이터 구조 (Data Structures)
    // ========================================

    // 지지점 (Support Point) - Minkowski 차이에서 가장 멀리 있는 점
    struct SupportPoint
    {
        Vector2 point;        // Minkowski 공간의 점
        Vector2 pointA;       // Shape A의 점
        Vector2 pointB;       // Shape B의 점

        SupportPoint() = default;
        SupportPoint(const Vector2& p, const Vector2& a, const Vector2& b)
            : point(p), pointA(a), pointB(b) {}
    };

    // 단체 (Simplex) - GJK에서 사용하는 기하학적 구조
    struct Simplex
    {
        SupportPoint points[4];  // 최대 4개의 점 (2D에서는 3개면 충분하지만 안전을 위해 4개)
        int count;               // 현재 포함된 점의 개수

        Simplex() : count(0) {}

        void AddPoint(const SupportPoint& point);
        void RemovePoint(int index);
        void Clear();

        // 단체의 차원 반환 (0: 점, 1: 선분, 2: 삼각형)
        int GetDimension() const;
    };

    // 다면체 (Polytope) - EPA에서 사용하는 볼록 다면체
    struct Polytope
    {
        std::vector<SupportPoint> vertices;  // 정점들
        std::vector<std::pair<int, int>> edges;  // 모서리들 (정점 인덱스 쌍)

        Polytope() = default;
        explicit Polytope(const Simplex& simplex);

        void AddVertex(const SupportPoint& vertex);
        void AddEdge(int v1, int v2);

        // 가장 가까운 모서리 찾기
        std::pair<int, int> FindClosestEdge() const;

        // 모서리에 새로운 점 삽입
        void InsertPointOnEdge(int edgeIndex, const SupportPoint& point);
    };

    // 충돌 정보 (Collision Information)
    struct CollisionInfo
    {
        bool hasCollision;     // 충돌 여부
        Vector2 normal;        // 충돌 법선 벡터
        Real depth;           // 침투 깊이
        Vector2 contactPoint; // 접촉점

        CollisionInfo() : hasCollision(false), normal(0, 0), depth(0), contactPoint(0, 0) {}
    };

    // ========================================
    // GJK 알고리즘 (Gilbert-Johnson-Keerthi)
    // ========================================

    namespace GJK
    {
        // 두 도형 간의 충돌 검출
        bool DetectCollision(const void* shapeA, const void* shapeB);

        // Minkowski 차이에서 지지점 계산
        SupportPoint GetSupportPoint(const void* shapeA, const void* shapeB, const Vector2& direction);

        // 단체에서 원점으로의 가장 가까운 점 찾기
        Vector2 GetClosestPointToOrigin(const Simplex& simplex);

        // 단체에 새로운 점 추가
        bool UpdateSimplex(Simplex& simplex, const SupportPoint& newPoint);

        // 단체가 원점을 포함하는지 확인
        bool ContainsOrigin(const Simplex& simplex);

        // 다음 검색 방향 계산
        Vector2 GetNextDirection(const Simplex& simplex);
    }

    // ========================================
    // EPA 알고리즘 (Expanding Polytope Algorithm)
    // ========================================

    namespace EPA
    {
        // 충돌 깊이와 정보 계산
        CollisionInfo CalculatePenetration(const Simplex& simplex, const void* shapeA, const void* shapeB);

        // 다면체 확장
        void ExpandPolytope(Polytope& polytope, const void* shapeA, const void* shapeB);

        // 가장 가까운 모서리에 점 추가
        void InsertPointOnClosestEdge(Polytope& polytope, const void* shapeA, const void* shapeB);

        // 충돌 법선과 깊이 계산
        std::pair<Vector2, Real> CalculateNormalAndDepth(const Polytope& polytope);
    }

    // ========================================
    // 통합 충돌 검출 함수
    // ========================================

    // GJK-EPA를 사용한 완전한 충돌 검출
    CollisionInfo DetectCollisionGJK_EPA(const void* shapeA, const void* shapeB);

    // ========================================
    // 구현 세부사항
    // ========================================

    // Simplex, Polytope, GJK, EPA 구조체/클래스의 멤버 함수들은 cpp 파일에서 구현됨

    // ========================================
    // 통합 충돌 검출 함수
    // ========================================

    CollisionInfo DetectCollisionGJK_EPA(const void* shapeA, const void* shapeB);
}
