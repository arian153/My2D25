#include "Triangle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Engine2D
{
    // ========================================
    // 생성자
    // ========================================

    Triangle::Triangle() : vertices{Vector2(0, 0), Vector2(1, 0), Vector2(0, 1)} {}

    Triangle::Triangle(const Vector2& a, const Vector2& b, const Vector2& c) : vertices{a, b, c} {}

    Triangle::Triangle(Real x1, Real y1, Real x2, Real y2, Real x3, Real y3)
        : vertices{Vector2(x1, y1), Vector2(x2, y2), Vector2(x3, y3)} {}

    // ========================================
    // Static 함수들 (핵심 알고리즘)
    // ========================================

    // 1. Support Function - GJK/EPA용
    Vector2 Triangle::SupportFunction(const Vector2& vertices[3], const Vector2& direction)
    {
        if (direction.LengthSquared() < 1e-12) {
            return vertices[0];
        }

        Real maxDot = vertices[0].Dot(direction);
        Vector2 supportPoint = vertices[0];

        for (int i = 1; i < 3; ++i) {
            Real dot = vertices[i].Dot(direction);
            if (dot > maxDot) {
                maxDot = dot;
                supportPoint = vertices[i];
            }
        }

        return supportPoint;
    }

    // 2. Area Function - 면적 계산
    Real Triangle::AreaFunction(const Vector2& vertices[3])
    {
        Vector2 v1 = vertices[1] - vertices[0];
        Vector2 v2 = vertices[2] - vertices[0];
        return std::abs(v1.x * v2.y - v1.y * v2.x) * Real(0.5);
    }

    // 3. Perimeter Function - 둘레 계산
    Real Triangle::PerimeterFunction(const Vector2& vertices[3])
    {
        Real side1 = (vertices[1] - vertices[0]).Length();
        Real side2 = (vertices[2] - vertices[1]).Length();
        Real side3 = (vertices[0] - vertices[2]).Length();
        return side1 + side2 + side3;
    }

    // 4. SDF Function - 부호가 있는 거리 함수
    SDFResult Triangle::SDFFunction(const Vector2& vertices[3], const Vector2& point)
    {
        SDFResult result;

        // 삼각형의 세 변에 대한 거리 계산
        Real minDistance = std::numeric_limits<Real>::max();
        Vector2 closestPoint;
        bool isInside = true;

        for (int i = 0; i < 3; ++i) {
            int j = (i + 1) % 3;
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

            // 내부/외부 판별 (외적 사용)
            Vector2 cross = Vector2(edge.x * toPoint.y - edge.y * toPoint.x, 0);
            if (cross.x < 0) {
                isInside = false;
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
    Vector2 Triangle::CentroidFunction(const Vector2& vertices[3])
    {
        return (vertices[0] + vertices[1] + vertices[2]) / Real(3);
    }

    // 6. Circumcenter Function - 외심 계산
    Vector2 Triangle::CircumcenterFunction(const Vector2& vertices[3])
    {
        Vector2 a = vertices[0];
        Vector2 b = vertices[1];
        Vector2 c = vertices[2];

        Real d = Real(2) * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

        if (std::abs(d) < 1e-12) {
            return CentroidFunction(vertices); // 세 점이 한 직선 위에 있는 경우
        }

        Real ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) +
                  (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                  (c.x * c.x + c.y * c.y) * (a.y - b.y)) / d;

        Real uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) +
                  (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                  (c.x * c.x + c.y * c.y) * (b.x - a.x)) / d;

        return Vector2(ux, uy);
    }

    // 7. Incenter Function - 내심 계산
    Vector2 Triangle::IncenterFunction(const Vector2& vertices[3])
    {
        Real a = (vertices[2] - vertices[1]).Length();
        Real b = (vertices[0] - vertices[2]).Length();
        Real c = (vertices[1] - vertices[0]).Length();

        Real perimeter = a + b + c;
        if (perimeter < 1e-12) {
            return CentroidFunction(vertices);
        }

        Real x = (a * vertices[0].x + b * vertices[1].x + c * vertices[2].x) / perimeter;
        Real y = (a * vertices[0].y + b * vertices[1].y + c * vertices[2].y) / perimeter;

        return Vector2(x, y);
    }

    // 8. Orthocenter Function - 수심 계산
    Vector2 Triangle::OrthocenterFunction(const Vector2& vertices[3])
    {
        Vector2 a = vertices[0];
        Vector2 b = vertices[1];
        Vector2 c = vertices[2];

        Real denominator = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if (std::abs(denominator) < 1e-12) {
            return CentroidFunction(vertices); // 세 점이 한 직선 위에 있는 경우
        }

        Real t = ((c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)) / denominator;
        return Vector2(a.x + t * (b.y - a.y), a.y - t * (b.x - a.x));
    }

    // 9. BoundingBox Function - 바운딩 박스 계산
    BoundingBox Triangle::BoundingBoxFunction(const Vector2& vertices[3])
    {
        Vector2 min = vertices[0];
        Vector2 max = vertices[0];

        for (int i = 1; i < 3; ++i) {
            min.x = std::min(min.x, vertices[i].x);
            min.y = std::min(min.y, vertices[i].y);
            max.x = std::max(max.x, vertices[i].x);
            max.y = std::max(max.y, vertices[i].y);
        }

        return BoundingBox(min, max);
    }

    // 10. Moment of Inertia Function - 관성 모멘트 계산
    Real Triangle::MomentOfInertiaFunction(const Vector2& vertices[3])
    {
        Vector2 centroid = CentroidFunction(vertices);
        Real moment = 0;

        for (int i = 0; i < 3; ++i) {
            Vector2 relative = vertices[i] - centroid;
            moment += relative.LengthSquared();
        }

        return moment / Real(6);
    }

    // 11. IsConvex Function - 볼록성 확인
    bool Triangle::IsConvexFunction(const Vector2& vertices[3])
    {
        Vector2 v1 = vertices[1] - vertices[0];
        Vector2 v2 = vertices[2] - vertices[1];
        Vector2 v3 = vertices[0] - vertices[2];

        Real cross1 = v1.x * v2.y - v1.y * v2.x;
        Real cross2 = v2.x * v3.y - v2.y * v3.x;
        Real cross3 = v3.x * v1.y - v3.y * v1.x;

        return (cross1 >= 0 && cross2 >= 0 && cross3 >= 0) ||
               (cross1 <= 0 && cross2 <= 0 && cross3 <= 0);
    }

    // 12. IsEquilateral Function - 정삼각형 확인
    bool Triangle::IsEquilateralFunction(const Vector2& vertices[3])
    {
        Real side1 = (vertices[1] - vertices[0]).Length();
        Real side2 = (vertices[2] - vertices[1]).Length();
        Real side3 = (vertices[0] - vertices[2]).Length();

        return std::abs(side1 - side2) < 1e-6 &&
               std::abs(side2 - side3) < 1e-6 &&
               std::abs(side3 - side1) < 1e-6;
    }

    // 13. IsIsosceles Function - 이등변삼각형 확인
    bool Triangle::IsIsoscelesFunction(const Vector2& vertices[3])
    {
        Real side1 = (vertices[1] - vertices[0]).Length();
        Real side2 = (vertices[2] - vertices[1]).Length();
        Real side3 = (vertices[0] - vertices[2]).Length();

        return std::abs(side1 - side2) < 1e-6 ||
               std::abs(side2 - side3) < 1e-6 ||
               std::abs(side3 - side1) < 1e-6;
    }

    // 14. IsRight Function - 직각삼각형 확인
    bool Triangle::IsRightFunction(const Vector2& vertices[3])
    {
        Vector2 v1 = vertices[1] - vertices[0];
        Vector2 v2 = vertices[2] - vertices[1];
        Vector2 v3 = vertices[0] - vertices[2];

        Real dot1 = v1.Dot(v2);
        Real dot2 = v2.Dot(v3);
        Real dot3 = v3.Dot(v1);

        return std::abs(dot1) < 1e-6 || std::abs(dot2) < 1e-6 || std::abs(dot3) < 1e-6;
    }

    // ========================================
    // IShape 인터페이스 구현
    // ========================================

    ShapeType Triangle::GetType() const
    {
        return ShapeType::TRIANGLE;
    }

    Vector2 Triangle::GetCenter() const
    {
        return GetCentroid();
    }

    Real Triangle::GetBoundingRadius() const
    {
        return GetBoundingBox().GetSize().Length() * 0.5f;
    }

    // SAT를 위한 축 투영
    std::vector<Vector2> Triangle::GetAxes() const
    {
        std::vector<Vector2> axes;
        axes.reserve(3);

        for (int i = 0; i < 3; ++i) {
            int j = (i + 1) % 3;
            Vector2 edge = vertices[j] - vertices[i];
            Vector2 normal(-edge.y, edge.x);
            if (normal.LengthSquared() > 1e-12) {
                axes.push_back(normal.Normalized());
            }
        }

        return axes;
    }

    std::pair<Real, Real> Triangle::ProjectOnAxis(const Vector2& axis) const
    {
        if (axis.LengthSquared() < 1e-12) {
            return {0, 0};
        }

        Real minProj = vertices[0].Dot(axis);
        Real maxProj = minProj;

        for (int i = 1; i < 3; ++i) {
            Real proj = vertices[i].Dot(axis);
            minProj = std::min(minProj, proj);
            maxProj = std::max(maxProj, proj);
        }

        return {minProj, maxProj};
    }

    // SDF를 위한 거리 함수
    SDFResult Triangle::GetSDF(const Vector2& point) const
    {
        return SDFFunction(vertices, point);
    }

    // GJK를 위한 지지점 함수
    Vector2 Triangle::GetSupportPoint(const Vector2& direction) const
    {
        return SupportFunction(vertices, direction);
    }

    // ========================================
    // 기본 멤버 함수들 (Static 함수 사용)
    // ========================================

    // 면적 계산
    Real Triangle::GetArea() const
    {
        return AreaFunction(vertices);
    }

    // 둘레 계산
    Real Triangle::GetPerimeter() const
    {
        return PerimeterFunction(vertices);
    }

    // 무게중심
    Vector2 Triangle::GetCentroid() const
    {
        return CentroidFunction(vertices);
    }

    // 외심
    Vector2 Triangle::GetCircumcenter() const
    {
        return CircumcenterFunction(vertices);
    }

    // 내심
    Vector2 Triangle::GetIncenter() const
    {
        return IncenterFunction(vertices);
    }

    // 수심
    Vector2 Triangle::GetOrthocenter() const
    {
        return OrthocenterFunction(vertices);
    }

    // 바운딩 박스
    BoundingBox Triangle::GetBoundingBox() const
    {
        return BoundingBoxFunction(vertices);
    }

    // 관성 모멘트
    Real Triangle::GetMomentOfInertia() const
    {
        return MomentOfInertiaFunction(vertices);
    }

    // 볼록성 확인
    bool Triangle::IsConvex() const
    {
        return IsConvexFunction(vertices);
    }

    // 정삼각형 확인
    bool Triangle::IsEquilateral() const
    {
        return IsEquilateralFunction(vertices);
    }

    // 이등변삼각형 확인
    bool Triangle::IsIsosceles() const
    {
        return IsIsoscelesFunction(vertices);
    }

    // 직각삼각형 확인
    bool Triangle::IsRight() const
    {
        return IsRightFunction(vertices);
    }

    // ========================================
    // 추가 멤버 함수들
    // ========================================

    // 정점들 가져오기
    std::vector<Vector2> Triangle::GetVertices() const
    {
        return {vertices[0], vertices[1], vertices[2]};
    }

    // 모서리들 가져오기
    std::vector<std::pair<Vector2, Vector2>> Triangle::GetEdges() const
    {
        return {
            {vertices[0], vertices[1]},
            {vertices[1], vertices[2]},
            {vertices[2], vertices[0]}
        };
    }

    // 점이 삼각형 내부에 있는지 확인 (면적 기반)
    bool Triangle::ContainsPoint(const Vector2& point) const
    {
        // 삼각형의 면적과 세 개의 작은 삼각형 면적의 합을 비교
        Real totalArea = GetArea();
        Real area1 = Triangle(point, vertices[1], vertices[2]).GetArea();
        Real area2 = Triangle(vertices[0], point, vertices[2]).GetArea();
        Real area3 = Triangle(vertices[0], vertices[1], point).GetArea();

        return std::abs(totalArea - (area1 + area2 + area3)) < 1e-6;
    }

    // 삼각형과 점 사이의 거리
    Real Triangle::GetDistanceToPoint(const Vector2& point) const
    {
        SDFResult sdf = GetSDF(point);
        return std::abs(sdf.distance);
    }

    // 삼각형의 경계에서 가장 가까운 점
    Vector2 Triangle::GetClosestPointOnBoundary(const Vector2& point) const
    {
        SDFResult sdf = GetSDF(point);
        return sdf.closestPoint;
    }

    // 삼각형의 법선 벡터 (경계상의 점에서)
    Vector2 Triangle::GetNormalAt(const Vector2& boundaryPoint) const
    {
        SDFResult sdf = GetSDF(boundaryPoint);
        return sdf.gradient;
    }

    // ========================================
    // 변형 함수들
    // ========================================

    // 삼각형 이동
    void Triangle::Translate(const Vector2& offset)
    {
        for (int i = 0; i < 3; ++i) {
            vertices[i] += offset;
        }
    }

    // 삼각형 크기 조정 (중심점 기준)
    void Triangle::Scale(Real scale)
    {
        Vector2 centroid = GetCentroid();
        for (int i = 0; i < 3; ++i) {
            vertices[i] = centroid + (vertices[i] - centroid) * scale;
        }
    }

    // 삼각형 회전 (중심점 기준)
    void Triangle::Rotate(Real angle)
    {
        Vector2 centroid = GetCentroid();
        Real cosRot = std::cos(angle);
        Real sinRot = std::sin(angle);

        for (int i = 0; i < 3; ++i) {
            Vector2 relative = vertices[i] - centroid;
            vertices[i] = centroid + Vector2(
                relative.x * cosRot - relative.y * sinRot,
                relative.x * sinRot + relative.y * cosRot
            );
        }
    }

    // ========================================
    // 충돌 검출 함수들
    // ========================================

    // 두 삼각형의 충돌 검출
    bool Triangle::IntersectsWith(const Triangle& other) const
    {
        std::vector<Vector2> verticesA = GetVertices();
        std::vector<Vector2> verticesB = other.GetVertices();

        SATResult result = SAT::DetectCollision(verticesA, verticesB);
        return result.hasCollision;
    }

    // 두 삼각형의 충돌 정보
    CollisionResult Triangle::GetCollisionInfo(const Triangle& other) const
    {
        std::vector<Vector2> verticesA = GetVertices();
        std::vector<Vector2> verticesB = other.GetVertices();

        SATResult satResult = SAT::DetectCollision(verticesA, verticesB);

        if (satResult.hasCollision) {
            Vector2 contactPoint = SATAdvanced::CalculateContactPoints(verticesA, verticesB)[0];
            return CollisionResult::Collision(satResult.normal, satResult.overlap, contactPoint);
        } else {
            return CollisionResult::NoCollision();
        }
    }

    // ========================================
    // 기하학적 속성 함수들
    // ========================================

    // 삼각형의 각도들 계산
    std::vector<Real> Triangle::GetAngles() const
    {
        Vector2 v1 = vertices[1] - vertices[0];
        Vector2 v2 = vertices[2] - vertices[1];
        Vector2 v3 = vertices[0] - vertices[2];

        Real angle1 = std::acos(-v1.Dot(v3) / (v1.Length() * v3.Length()));
        Real angle2 = std::acos(-v1.Dot(v2) / (v1.Length() * v2.Length()));
        Real angle3 = std::acos(-v2.Dot(v3) / (v2.Length() * v3.Length()));

        return {angle1, angle2, angle3};
    }

    // 삼각형의 변 길이들
    std::vector<Real> Triangle::GetSideLengths() const
    {
        Real side1 = (vertices[1] - vertices[0]).Length();
        Real side2 = (vertices[2] - vertices[1]).Length();
        Real side3 = (vertices[0] - vertices[2]).Length();

        return {side1, side2, side3};
    }

    // 삼각형의 높이들
    std::vector<Real> Triangle::GetHeights() const
    {
        Real area = GetArea();
        std::vector<Real> sides = GetSideLengths();

        return {Real(2) * area / sides[0],
               Real(2) * area / sides[1],
               Real(2) * area / sides[2]};
    }

    // 삼각형의 중선들
    std::vector<Vector2> Triangle::GetMedians() const
    {
        Vector2 centroid = GetCentroid();
        return {centroid - vertices[0],
               centroid - vertices[1],
               centroid - vertices[2]};
    }

    // 삼각형의 각이등분선들
    std::vector<Vector2> Triangle::GetAngleBisectors() const
    {
        std::vector<Real> sides = GetSideLengths();
        Vector2 centroid = GetCentroid();

        // 각이등분선의 방향 벡터들 (근사)
        Vector2 bisector1 = (vertices[1] - vertices[0]).Normalized() +
                           (vertices[2] - vertices[0]).Normalized();
        Vector2 bisector2 = (vertices[0] - vertices[1]).Normalized() +
                           (vertices[2] - vertices[1]).Normalized();
        Vector2 bisector3 = (vertices[0] - vertices[2]).Normalized() +
                           (vertices[1] - vertices[2]).Normalized();

        return {bisector1, bisector2, bisector3};
    }

    // 삼각형의 경계 길이 (둘레와 동일)
    Real Triangle::GetBoundaryLength() const
    {
        return GetPerimeter();
    }
}
