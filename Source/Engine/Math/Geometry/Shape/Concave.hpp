#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../Algorithm/GJK_EPA.hpp"
#include "../Algorithm/SAT.hpp"
#include "IShape.hpp"
#include "../../Structure/Geometry.hpp"

namespace Engine2D
{
    // ========================================
    // 오목 다각형 클래스
    // ========================================

    class Concave : public IShape
    {
    public:
        std::vector<Vector2> vertices;  // 정점들 (시계방향 또는 반시계방향)

        // 생성자
        Concave() = default;
        Concave(const std::vector<Vector2>& vertices) : vertices(vertices) {}
        Concave(std::vector<Vector2>&& vertices) : vertices(std::move(vertices)) {}

        // ========================================
        // Static 함수들 (핵심 알고리즘)
        // ========================================

        // 1. Support Function - GJK/EPA용
        static Vector2 SupportFunction(const std::vector<Vector2>& vertices, const Vector2& direction)
        {
            if (vertices.empty()) {
                return Vector2(0, 0);
            }

            if (direction.LengthSquared() < 1e-12) {
                return vertices[0];
            }

            // 모든 정점 중에서 주어진 방향으로 가장 멀리 있는 점 찾기
            Real maxDot = vertices[0].Dot(direction);
            Vector2 supportPoint = vertices[0];

            for (size_t i = 1; i < vertices.size(); ++i) {
                Real dot = vertices[i].Dot(direction);
                if (dot > maxDot) {
                    maxDot = dot;
                    supportPoint = vertices[i];
                }
            }

            return supportPoint;
        }

        // 2. Area Function - 면적 계산
        static Real AreaFunction(const std::vector<Vector2>& vertices)
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

        // 3. Perimeter Function - 둘레 계산
        static Real PerimeterFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.size() < 2) return 0;

            Real perimeter = 0;
            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                perimeter += (vertices[j] - vertices[i]).Length();
            }
            return perimeter;
        }

        // 4. SDF Function - 부호가 있는 거리 함수
        static SDFResult SDFFunction(const std::vector<Vector2>& vertices, const Vector2& point)
        {
            SDFResult result;

            if (vertices.size() < 3) {
                result.distance = std::numeric_limits<Real>::max();
                result.isInside = false;
                result.closestPoint = Vector2(0, 0);
                result.gradient = Vector2(1, 0);
                return result;
            }

            // 다각형의 모든 변에 대한 거리 계산
            Real minDistance = std::numeric_limits<Real>::max();
            Vector2 closestPoint;
            bool isInside = ContainsPointFunction(vertices, point); // Ray Casting 사용

            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                Vector2 edgeStart = vertices[i];
                Vector2 edgeEnd = vertices[j];

                // 선분에 대한 거리 계산
                Vector2 edge = edgeEnd - edgeStart;
                Vector2 toPoint = point - edgeStart;
                Real edgeLength = edge.Length();

                if (edgeLength < 1e-6) continue;

                Real t = std::clamp(toPoint.Dot(edge) / (edgeLength * edgeLength), Real(0), Real(1));
                Vector2 projection = edgeStart + edge * t;
                Real distance = (point - projection).Length();

                if (distance < minDistance) {
                    minDistance = distance;
                    closestPoint = projection;
                }
            }

            result.distance = isInside ? -minDistance : minDistance;
            result.isInside = isInside;
            result.closestPoint = closestPoint;

            // 그래디언트 계산
            if (std::abs(result.distance) > 1e-6) {
                result.gradient = (point - closestPoint).Normalized();
                if (!isInside) {
                    result.gradient = -result.gradient;
                }
            } else {
                result.gradient = Vector2(1, 0);
            }

            return result;
        }

        // 5. Centroid Function - 무게중심 계산
        static Vector2 CentroidFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.empty()) return Vector2(0, 0);

            Vector2 centroid(0, 0);
            for (const auto& vertex : vertices) {
                centroid += vertex;
            }
            return centroid / static_cast<Real>(vertices.size());
        }

        // 6. BoundingBox Function - 바운딩 박스 계산
        static BoundingBox BoundingBoxFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.empty()) {
                return BoundingBox();
            }

            Vector2 min = vertices[0];
            Vector2 max = vertices[0];

            for (const auto& vertex : vertices) {
                min.x = std::min(min.x, vertex.x);
                min.y = std::min(min.y, vertex.y);
                max.x = std::max(max.x, vertex.x);
                max.y = std::max(max.y, vertex.y);
            }

            return BoundingBox(min, max);
        }

        // 7. Moment of Inertia Function - 관성 모멘트 계산
        static Real MomentOfInertiaFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.size() < 3) return 0;

            Vector2 centroid = CentroidFunction(vertices);
            Real moment = 0;

            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                Vector2 v1 = vertices[i] - centroid;
                Vector2 v2 = vertices[j] - centroid;

                Real cross = v1.x * v2.y - v1.y * v2.x;
                moment += cross * (v1.Dot(v1) + v1.Dot(v2) + v2.Dot(v2));
            }

            return moment / Real(12);
        }

        // 8. ContainsPoint Function - 점 포함 확인 (Ray Casting)
        static bool ContainsPointFunction(const std::vector<Vector2>& vertices, const Vector2& point)
        {
            if (vertices.size() < 3) return false;

            bool inside = false;
            size_t j = vertices.size() - 1;

            for (size_t i = 0; i < vertices.size(); ++i) {
                if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
                    (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) /
                              (vertices[j].y - vertices[i].y) + vertices[i].x)) {
                    inside = !inside;
                }
                j = i;
            }

            return inside;
        }

        // 9. IsConvex Function - 볼록성 확인
        static bool IsConvexFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.size() < 3) return false;

            int sign = 0;
            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                size_t k = (i + 2) % vertices.size();

                Vector2 v1 = vertices[j] - vertices[i];
                Vector2 v2 = vertices[k] - vertices[j];

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

        // 10. IsConcave Function - 오목성 확인
        static bool IsConcaveFunction(const std::vector<Vector2>& vertices)
        {
            return !IsConvexFunction(vertices);
        }

        // 11. HasNoSelfIntersection Function - 자기 교차 확인
        static bool HasNoSelfIntersectionFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.size() < 4) return true;

            // 모든 모서리 쌍에 대해 교차 검사
            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t iNext = (i + 1) % vertices.size();
                Vector2 edge1Start = vertices[i];
                Vector2 edge1End = vertices[iNext];

                for (size_t j = i + 2; j < vertices.size(); ++j) {
                    size_t jNext = (j + 1) % vertices.size();

                    // 인접한 모서리는 제외
                    if (jNext == i) continue;

                    Vector2 edge2Start = vertices[j];
                    Vector2 edge2End = vertices[jNext];

                    if (DoLinesIntersectFunction(edge1Start, edge1End, edge2Start, edge2End)) {
                        return false; // 자기 교차 발견
                    }
                }
            }

            return true;
        }

        // 12. IsStarShaped Function - 별모양 확인
        static bool IsStarShapedFunction(const std::vector<Vector2>& vertices)
        {
            if (vertices.size() < 3) return false;

            // 중심점에서 모든 정점이 보이는지 확인
            Vector2 center = CentroidFunction(vertices);

            for (size_t i = 0; i < vertices.size(); ++i) {
                Vector2 ray = vertices[i] - center;

                // 다른 모든 모서리와의 교차 검사
                for (size_t j = 0; j < vertices.size(); ++j) {
                    size_t jNext = (j + 1) % vertices.size();

                    if (j == i || jNext == i) continue;

                    Vector2 edgeStart = vertices[j];
                    Vector2 edgeEnd = vertices[jNext];

                    if (DoLinesIntersectFunction(center, vertices[i], edgeStart, edgeEnd)) {
                        return false; // 중심점에서 정점이 보이지 않음
                    }
                }
            }

            return true;
        }

        // 13. DoLinesIntersect Function - 선분 교차 확인
        static bool DoLinesIntersectFunction(const Vector2& p1, const Vector2& q1,
                                           const Vector2& p2, const Vector2& q2)
        {
            Real denominator = (q2.y - p2.y) * (q1.x - p1.x) - (q2.x - p2.x) * (q1.y - p1.y);

            if (std::abs(denominator) < 1e-9) return false; // 평행한 선분

            Real ua = ((q2.x - p2.x) * (p1.y - p2.y) - (q2.y - p2.y) * (p1.x - p2.x)) / denominator;
            Real ub = ((q1.x - p1.x) * (p1.y - p2.y) - (q1.y - p1.y) * (p1.x - p2.x)) / denominator;

            return (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1);
        }

        // ========================================
        // IShape 인터페이스 구현
        // ========================================

        ShapeType GetType() const override { return ShapeType::CONCAVE; }
        Vector2 GetCenter() const override { return GetCentroid(); }
        Real GetBoundingRadius() const override { return GetBoundingBox().GetSize().Length() * 0.5f; }

        // SAT를 위한 축 투영
        std::vector<Vector2> GetAxes() const override
        {
            std::vector<Vector2> axes;
            axes.reserve(vertices.size());

            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                Vector2 edge = vertices[j] - vertices[i];
                Vector2 normal(-edge.y, edge.x);
                if (normal.LengthSquared() > 1e-12) {
                    axes.push_back(normal.Normalized());
                }
            }

            return axes;
        }

        std::pair<Real, Real> ProjectOnAxis(const Vector2& axis) const override
        {
            if (axis.LengthSquared() < 1e-12) {
                return {0, 0};
            }

            Real minProj = vertices[0].Dot(axis);
            Real maxProj = minProj;

            for (size_t i = 1; i < vertices.size(); ++i) {
                Real proj = vertices[i].Dot(axis);
                minProj = std::min(minProj, proj);
                maxProj = std::max(maxProj, proj);
            }

            return {minProj, maxProj};
        }

        // SDF를 위한 거리 함수
        SDFResult GetSDF(const Vector2& point) const override
        {
            return SDFFunction(vertices, point);
        }

        // GJK를 위한 지지점 함수
        Vector2 GetSupportPoint(const Vector2& direction) const override
        {
            return SupportFunction(vertices, direction);
        }

        // ========================================
        // 기본 멤버 함수들 (Static 함수 사용)
        // ========================================

        // 면적 계산
        Real GetArea() const
        {
            return AreaFunction(vertices);
        }

        // 둘레 계산
        Real GetPerimeter() const
        {
            return PerimeterFunction(vertices);
        }

        // 무게중심
        Vector2 GetCentroid() const
        {
            return CentroidFunction(vertices);
        }

        // 바운딩 박스
        BoundingBox GetBoundingBox() const
        {
            return BoundingBoxFunction(vertices);
        }

        // 관성 모멘트
        Real GetMomentOfInertia() const
        {
            return MomentOfInertiaFunction(vertices);
        }

        // 점 포함 확인
        bool ContainsPoint(const Vector2& point) const
        {
            return ContainsPointFunction(vertices, point);
        }

        // 볼록성 확인
        bool IsConvex() const
        {
            return IsConvexFunction(vertices);
        }

        // 오목성 확인
        bool IsConcave() const
        {
            return IsConcaveFunction(vertices);
        }

        // 자기 교차 확인
        bool HasNoSelfIntersection() const
        {
            return HasNoSelfIntersectionFunction(vertices);
        }

        // 별모양 확인
        bool IsStarShaped() const
        {
            return IsStarShapedFunction(vertices);
        }

        // ========================================
        // 추가 멤버 함수들
        // ========================================

        // 정점들 가져오기
        const std::vector<Vector2>& GetVertices() const
        {
            return vertices;
        }

        // 모서리들 가져오기
        std::vector<std::pair<Vector2, Vector2>> GetEdges() const
        {
            std::vector<std::pair<Vector2, Vector2>> edges;
            edges.reserve(vertices.size());

            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                edges.push_back({vertices[i], vertices[j]});
            }

            return edges;
        }

        // 오목 다각형과 점 사이의 거리
        Real GetDistanceToPoint(const Vector2& point) const
        {
            SDFResult sdf = GetSDF(point);
            return std::abs(sdf.distance);
        }

        // 오목 다각형의 경계에서 가장 가까운 점
        Vector2 GetClosestPointOnBoundary(const Vector2& point) const
        {
            SDFResult sdf = GetSDF(point);
            return sdf.closestPoint;
        }

        // 오목 다각형의 법선 벡터 (경계상의 점에서)
        Vector2 GetNormalAt(const Vector2& boundaryPoint) const
        {
            SDFResult sdf = GetSDF(boundaryPoint);
            return sdf.gradient;
        }

        // ========================================
        // 변형 함수들
        // ========================================

        // 오목 다각형 이동
        void Translate(const Vector2& offset)
        {
            for (auto& vertex : vertices) {
                vertex += offset;
            }
        }

        // 오목 다각형 크기 조정 (중심점 기준)
        void Scale(Real scale)
        {
            Vector2 centroid = GetCentroid();
            for (auto& vertex : vertices) {
                vertex = centroid + (vertex - centroid) * scale;
            }
        }

        // 오목 다각형 회전 (중심점 기준)
        void Rotate(Real angle)
        {
            Vector2 centroid = GetCentroid();
            Real cosRot = std::cos(angle);
            Real sinRot = std::sin(angle);

            for (auto& vertex : vertices) {
                Vector2 relative = vertex - centroid;
                vertex = centroid + Vector2(
                    relative.x * cosRot - relative.y * sinRot,
                    relative.x * sinRot + relative.y * cosRot
                );
            }
        }

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 두 오목 다각형의 충돌 검출
        bool IntersectsWith(const Concave& other) const
        {
            SATResult result = SAT::DetectCollision(vertices, other.vertices);
            return result.hasCollision;
        }

        // 두 오목 다각형의 충돌 정보
        CollisionResult GetCollisionInfo(const Concave& other) const
        {
            SATResult satResult = SAT::DetectCollision(vertices, other.vertices);

            if (satResult.hasCollision) {
                Vector2 contactPoint = SATAdvanced::CalculateContactPoints(vertices, other.vertices)[0];
                return CollisionResult::Collision(satResult.normal, satResult.overlap, contactPoint);
            } else {
                return CollisionResult::NoCollision();
            }
        }

        // ========================================
        // 기하학적 속성 함수들
        // ========================================

        // 오목 다각형의 변 길이들
        std::vector<Real> GetSideLengths() const
        {
            std::vector<Real> lengths;
            lengths.reserve(vertices.size());

            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t j = (i + 1) % vertices.size();
                lengths.push_back((vertices[j] - vertices[i]).Length());
            }

            return lengths;
        }

        // 오목 다각형의 각도들
        std::vector<Real> GetAngles() const
        {
            if (vertices.size() < 3) return {};

            std::vector<Real> angles;
            angles.reserve(vertices.size());

            for (size_t i = 0; i < vertices.size(); ++i) {
                size_t prev = (i + vertices.size() - 1) % vertices.size();
                size_t next = (i + 1) % vertices.size();

                Vector2 v1 = vertices[prev] - vertices[i];
                Vector2 v2 = vertices[next] - vertices[i];

                Real cosAngle = v1.Dot(v2) / (v1.Length() * v2.Length());
                cosAngle = std::clamp(cosAngle, Real(-1), Real(1));
                angles.push_back(std::acos(cosAngle));
            }

            return angles;
        }

        // 오목 다각형의 대각선들
        std::vector<std::pair<Vector2, Vector2>> GetDiagonals() const
        {
            std::vector<std::pair<Vector2, Vector2>> diagonals;

            for (size_t i = 0; i < vertices.size(); ++i) {
                for (size_t j = i + 2; j < vertices.size(); ++j) {
                    if (j != (i + 1) % vertices.size()) {
                        diagonals.push_back({vertices[i], vertices[j]});
                    }
                }
            }

            return diagonals;
        }

        // 오목 다각형의 면적 밀도 (면적/둘레)
        Real GetAreaDensity() const
        {
            return GetArea() / GetPerimeter();
        }

        // 오목 다각형의 컴팩트니스 (면적/둘레^2)
        Real GetCompactness() const
        {
            Real perimeter = GetPerimeter();
            return GetArea() / (perimeter * perimeter);
        }

        // 오목 다각형의 둘레 밀도 (둘레/면적)
        Real GetPerimeterDensity() const
        {
            return GetPerimeter() / GetArea();
        }

        // 오목 다각형의 둘레 밀도 (둘레/면적^0.5)
        Real GetPerimeterDensitySqrt() const
        {
            return GetPerimeter() / std::sqrt(GetArea());
        }

        // 오목 다각형의 둘레 밀도 (둘레/면적^0.25)
        Real GetPerimeterDensitySqrtSqrt() const
        {
            return GetPerimeter() / std::sqrt(std::sqrt(GetArea()));
        }

        // 오목 다각형의 정점 개수
        size_t GetVertexCount() const
        {
            return vertices.size();
        }

        // 오목 다각형의 모서리 개수
        size_t GetEdgeCount() const
        {
            return vertices.size();
        }

        // 오목 다각형의 대각선 개수
        size_t GetDiagonalCount() const
        {
            if (vertices.size() < 4) return 0;
            return vertices.size() * (vertices.size() - 3) / 2;
        }

        // 오목 다각형의 내각의 합
        Real GetSumOfInteriorAngles() const
        {
            return (vertices.size() - 2) * PI;
        }

        // 오목 다각형의 외각의 합
        Real GetSumOfExteriorAngles() const
        {
            return Real(2) * PI;
        }

        // 오목 다각형의 한 내각
        Real GetInteriorAngle() const
        {
            if (vertices.size() < 3) return 0;
            return GetSumOfInteriorAngles() / vertices.size();
        }

        // 오목 다각형의 한 외각
        Real GetExteriorAngle() const
        {
            if (vertices.size() < 3) return 0;
            return GetSumOfExteriorAngles() / vertices.size();
        }

        // 오목 다각형의 경계 길이 (둘레와 동일)
        Real GetBoundaryLength() const
        {
            return GetPerimeter();
        }

        // 오목 다각형이 유효한지 확인
        bool IsValid() const
        {
            return vertices.size() >= 3;
        }
    };
}
