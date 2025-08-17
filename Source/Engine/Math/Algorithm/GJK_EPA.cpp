#include "GJK_EPA.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Engine2D
{
    // ========================================
    // Simplex 구조체 구현
    // ========================================

    void Simplex::AddPoint(const SupportPoint& point)
    {
        if (count < 4) {
            points[count++] = point;
        }
    }

    void Simplex::RemovePoint(int index)
    {
        if (index >= 0 && index < count) {
            for (int i = index; i < count - 1; ++i) {
                points[i] = points[i + 1];
            }
            --count;
        }
    }

    void Simplex::Clear()
    {
        count = 0;
    }

    int Simplex::GetDimension() const
    {
        return count - 1;
    }

    // ========================================
    // Polytope 구조체 구현
    // ========================================

    Polytope::Polytope(const Simplex& simplex)
    {
        for (int i = 0; i < simplex.count; ++i) {
            vertices.push_back(simplex.points[i]);
        }

        // Simplex의 모서리들 추가
        if (simplex.count >= 2) {
            for (int i = 0; i < simplex.count; ++i) {
                int j = (i + 1) % simplex.count;
                edges.push_back({i, j});
            }
        }
    }

    void Polytope::AddVertex(const SupportPoint& vertex)
    {
        vertices.push_back(vertex);
    }

    void Polytope::AddEdge(int v1, int v2)
    {
        edges.push_back({v1, v2});
    }

    std::pair<int, int> Polytope::FindClosestEdge() const
    {
        if (edges.empty()) return {0, 0};

        Real minDistance = std::numeric_limits<Real>::max();
        std::pair<int, int> closestEdge = edges[0];

        for (const auto& edge : edges) {
            Vector2 v1 = vertices[edge.first].point;
            Vector2 v2 = vertices[edge.second].point;
            Vector2 edgeVector = v2 - v1;
            Vector2 normal = Vector2(-edgeVector.y, edgeVector.x).Normalized();

            Real distance = std::abs(v1.Dot(normal));
            if (distance < minDistance) {
                minDistance = distance;
                closestEdge = edge;
            }
        }

        return closestEdge;
    }

    void Polytope::InsertPointOnEdge(int edgeIndex, const SupportPoint& point)
    {
        if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges.size())) return;

        // 새로운 정점 추가
        int newVertexIndex = static_cast<int>(vertices.size());
        vertices.push_back(point);

        // 기존 모서리 제거
        auto oldEdge = edges[edgeIndex];
        edges.erase(edges.begin() + edgeIndex);

        // 새로운 모서리들 추가
        edges.push_back({oldEdge.first, newVertexIndex});
        edges.push_back({newVertexIndex, oldEdge.second});
    }

    // ========================================
    // GJK 알고리즘 구현
    // ========================================

    bool GJK::DetectCollision(const void* shapeA, const void* shapeB)
    {
        Simplex simplex;

        // 초기 방향 (임의의 방향)
        Vector2 direction(1, 0);

        // 첫 번째 지지점 추가
        SupportPoint support = GetSupportPoint(shapeA, shapeB, direction);
        simplex.AddPoint(support);

        // 다음 방향은 원점에서 첫 번째 점으로의 반대 방향
        direction = -support.point;

        const int maxIterations = 32;
        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            // 새로운 지지점 추가
            support = GetSupportPoint(shapeA, shapeB, direction);
            simplex.AddPoint(support);

            // 원점이 새로운 점의 반대쪽에 있는지 확인
            if (support.point.Dot(direction) < 0) {
                return false; // 분리축이 존재
            }

            // 단체 업데이트
            if (UpdateSimplex(simplex, support)) {
                return true; // 원점이 단체 내부에 있음
            }

            // 다음 검색 방향 계산
            direction = GetNextDirection(simplex);
        }

        return false; // 최대 반복 횟수 초과
    }

    SupportPoint GJK::GetSupportPoint(const void* shapeA, const void* shapeB, const Vector2& direction)
    {
        // 실제 구현에서는 shapeA와 shapeB의 타입에 따라
        // 각각의 지지점을 계산하고 Minkowski 차이를 구해야 합니다.
        // 여기서는 예시로 간단한 구현을 제공합니다.

        Vector2 pointA = Vector2(0, 0); // shapeA에서 direction 방향의 가장 멀리 있는 점
        Vector2 pointB = Vector2(0, 0); // shapeB에서 -direction 방향의 가장 멀리 있는 점

        // Minkowski 차이
        Vector2 minkowskiPoint = pointA - pointB;

        return SupportPoint(minkowskiPoint, pointA, pointB);
    }

    Vector2 GJK::GetClosestPointToOrigin(const Simplex& simplex)
    {
        switch (simplex.count) {
            case 1: // 점
                return simplex.points[0].point;

            case 2: { // 선분
                Vector2 a = simplex.points[0].point;
                Vector2 b = simplex.points[1].point;
                Vector2 ab = b - a;
                Real t = -a.Dot(ab) / ab.Dot(ab);
                t = std::clamp(t, Real(0), Real(1));
                return a + ab * t;
            }

            case 3: { // 삼각형
                Vector2 a = simplex.points[0].point;
                Vector2 b = simplex.points[1].point;
                Vector2 c = simplex.points[2].point;

                // 각 모서리에 대한 가장 가까운 점 계산
                Vector2 closestPoint = a;
                Real minDistance = a.LengthSquared();

                // 모서리 AB
                Vector2 ab = b - a;
                Real t1 = -a.Dot(ab) / ab.Dot(ab);
                if (t1 >= 0 && t1 <= 1) {
                    Vector2 point = a + ab * t1;
                    Real distance = point.LengthSquared();
                    if (distance < minDistance) {
                        minDistance = distance;
                        closestPoint = point;
                    }
                }

                // 모서리 BC
                Vector2 bc = c - b;
                Real t2 = -b.Dot(bc) / bc.Dot(bc);
                if (t2 >= 0 && t2 <= 1) {
                    Vector2 point = b + bc * t2;
                    Real distance = point.LengthSquared();
                    if (distance < minDistance) {
                        minDistance = distance;
                        closestPoint = point;
                    }
                }

                // 모서리 CA
                Vector2 ca = a - c;
                Real t3 = -c.Dot(ca) / ca.Dot(ca);
                if (t3 >= 0 && t3 <= 1) {
                    Vector2 point = c + ca * t3;
                    Real distance = point.LengthSquared();
                    if (distance < minDistance) {
                        minDistance = distance;
                        closestPoint = point;
                    }
                }

                return closestPoint;
            }

            default:
                return Vector2(0, 0);
        }
    }

    bool GJK::UpdateSimplex(Simplex& simplex, const SupportPoint& newPoint)
    {
        switch (simplex.count) {
            case 2: { // 선분에서 삼각형으로
                Vector2 a = simplex.points[0].point;
                Vector2 b = simplex.points[1].point;
                Vector2 c = newPoint.point;

                // 원점이 삼각형 내부에 있는지 확인
                Vector2 ab = b - a;
                Vector2 ac = c - a;
                Vector2 ao = -a;

                Real d1 = ab.Dot(ao);
                Real d2 = ac.Dot(ao);

                if (d1 <= 0 && d2 <= 0) {
                    // 원점이 A의 반대쪽에 있음
                    simplex.RemovePoint(1);
                    simplex.RemovePoint(0);
                    simplex.AddPoint(newPoint);
                    return false;
                }

                // 원점이 삼각형 내부에 있음
                return true;
            }

            case 3: { // 삼각형에서 사각형으로 (2D에서는 불필요하지만 안전을 위해)
                // 원점이 삼각형 내부에 있는지 확인
                return ContainsOrigin(simplex);
            }

            default:
                return false;
        }
    }

    bool GJK::ContainsOrigin(const Simplex& simplex)
    {
        if (simplex.count < 3) return false;

        Vector2 a = simplex.points[0].point;
        Vector2 b = simplex.points[1].point;
        Vector2 c = simplex.points[2].point;

        // 삼각형의 면적 계산
        Real area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);

        // 각 부분 삼각형의 면적 계산
        Real area1 = (b.x - a.x) * (-a.y) - (-a.x) * (b.y - a.y);
        Real area2 = (c.x - b.x) * (-b.y) - (-b.x) * (c.y - b.y);
        Real area3 = (a.x - c.x) * (-c.y) - (-c.x) * (a.y - c.y);

        // 원점이 삼각형 내부에 있는지 확인
        return (area1 * area >= 0) && (area2 * area >= 0) && (area3 * area >= 0);
    }

    Vector2 GJK::GetNextDirection(const Simplex& simplex)
    {
        Vector2 closestPoint = GetClosestPointToOrigin(simplex);
        return -closestPoint; // 원점에서 가장 가까운 점으로의 방향
    }

    // ========================================
    // EPA 알고리즘 구현
    // ========================================

    CollisionInfo EPA::CalculatePenetration(const Simplex& simplex, const void* shapeA, const void* shapeB)
    {
        CollisionInfo info;

        // Simplex를 Polytope으로 변환
        Polytope polytope(simplex);

        const int maxIterations = 32;
        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            // 가장 가까운 모서리 찾기
            auto closestEdge = polytope.FindClosestEdge();

            // 해당 모서리에서 새로운 지지점 계산
            Vector2 edgeVector = polytope.vertices[closestEdge.second].point -
                               polytope.vertices[closestEdge.first].point;
            Vector2 normal = Vector2(-edgeVector.y, edgeVector.x).Normalized();

            SupportPoint support = GJK::GetSupportPoint(shapeA, shapeB, normal);

            // 새로운 점이 이미 충분히 가까운지 확인
            Real distance = support.point.Dot(normal);
            if (std::abs(distance - polytope.vertices[closestEdge.first].point.Dot(normal)) < 1e-6) {
                // 충돌 정보 계산
                auto [collisionNormal, depth] = CalculateNormalAndDepth(polytope);
                info.hasCollision = true;
                info.normal = collisionNormal;
                info.depth = depth;
                info.contactPoint = (polytope.vertices[closestEdge.first].pointA +
                                   polytope.vertices[closestEdge.second].pointA) * Real(0.5);
                return info;
            }

            // 새로운 점을 모서리에 삽입
            polytope.InsertPointOnEdge(closestEdge.first, support);
        }

        return info; // 최대 반복 횟수 초과
    }

    void EPA::ExpandPolytope(Polytope& polytope, const void* shapeA, const void* shapeB)
    {
        // 가장 가까운 모서리에 점 추가
        InsertPointOnClosestEdge(polytope, shapeA, shapeB);
    }

    void EPA::InsertPointOnClosestEdge(Polytope& polytope, const void* shapeA, const void* shapeB)
    {
        auto closestEdge = polytope.FindClosestEdge();
        Vector2 edgeVector = polytope.vertices[closestEdge.second].point -
                           polytope.vertices[closestEdge.first].point;
        Vector2 normal = Vector2(-edgeVector.y, edgeVector.x).Normalized();

        SupportPoint support = GJK::GetSupportPoint(shapeA, shapeB, normal);
        polytope.InsertPointOnEdge(closestEdge.first, support);
    }

    std::pair<Vector2, Real> EPA::CalculateNormalAndDepth(const Polytope& polytope)
    {
        auto closestEdge = polytope.FindClosestEdge();
        Vector2 v1 = polytope.vertices[closestEdge.first].point;
        Vector2 v2 = polytope.vertices[closestEdge.second].point;

        Vector2 edgeVector = v2 - v1;
        Vector2 normal = Vector2(-edgeVector.y, edgeVector.x).Normalized();
        Real depth = std::abs(v1.Dot(normal));

        return {normal, depth};
    }

    // ========================================
    // 통합 충돌 검출 함수
    // ========================================

    CollisionInfo DetectCollisionGJK_EPA(const void* shapeA, const void* shapeB)
    {
        // GJK로 충돌 확인
        if (!GJK::DetectCollision(shapeA, shapeB)) {
            return CollisionInfo(); // 충돌 없음
        }

        // EPA로 침투 정보 계산
        // 실제 구현에서는 GJK의 simplex를 EPA에 전달해야 함
        Simplex simplex; // 임시 simplex 생성
        return EPA::CalculatePenetration(simplex, shapeA, shapeB);
    }
}
