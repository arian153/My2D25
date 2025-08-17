#pragma once

#include <vector>
#include <memory>
#include <functional>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../../Structure/Orientation.hpp"
#include "../../Structure/Geometry.hpp"
#include "../Curve/Curve.hpp"
#include "../Curve/Bezier.hpp"
#include "../Curve/CatmullRom.hpp"
#include "../Curve/NURBS.hpp"
#include "../Curve/Spline.hpp"
#include "../Line/Line.hpp"
#include "../Line/LineSegment.hpp"
#include "../Line/Ray.hpp"
#include "../Line/Edge.hpp"
#include "IShape.hpp"

namespace Engine2D
{
    // ========================================
    // 복잡한 도형 타입 열거형
    // ========================================

    enum class ComplexShapeType
    {
        SWEPT_CIRCLE,      // 원이 곡선을 따라 스윕된 도형
        SWEPT_RECTANGLE,   // 사각형이 곡선을 따라 스윕된 도형
        SWEPT_POLYGON,     // 다각형이 곡선을 따라 스윕된 도형
        MINKOWSKI_SUM,     // 민코스키 합으로 생성된 도형
        SDF_BLEND,         // SDF 블렌딩으로 생성된 도형
        BOOLEAN_OPERATION, // 불린 연산으로 생성된 도형
        CUSTOM_SDF         // 사용자 정의 SDF 도형
    };

    // ========================================
    // 곡선 타입 열거형
    // ========================================

    enum class CurveType
    {
        SPLINE,         // 스플라인
        LINE_SEGMENT,   // 선분
    };

    // ========================================
    // 스윕 프로파일 인터페이스
    // ========================================

    class ISweepProfile
    {
    public:
        virtual ~ISweepProfile() = default;

        // 프로파일의 반지름 (원형 프로파일용)
        virtual Real GetRadius() const = 0;

        // 프로파일의 크기 (사각형 프로파일용)
        virtual Vector2 GetSize() const = 0;

        // 프로파일의 정점들 (다각형 프로파일용)
        virtual std::vector<Vector2> GetVertices() const = 0;

        // 프로파일의 SDF
        virtual SDFResult GetSDF(const Vector2& point) const = 0;

        // 프로파일의 지지점 함수
        virtual Vector2 GetSupportPoint(const Vector2& direction) const = 0;
    };

    // ========================================
    // 기본 프로파일들
    // ========================================

    // 원형 프로파일
    class CircleProfile : public ISweepProfile
    {
    public:
        CircleProfile(Real radius) : radius(radius) {}

        Real GetRadius() const override { return radius; }
        Vector2 GetSize() const override { return Vector2(radius * 2, radius * 2); }
        std::vector<Vector2> GetVertices() const override { return {}; }

        SDFResult GetSDF(const Vector2& point) const override
        {
            SDFResult result;
            Real distance = point.Length();
            result.distance = distance - radius;
            result.isInside = distance < radius;

            if (distance > 1e-6) {
                result.gradient = point / distance;
                result.closestPoint = result.gradient * radius;
            } else {
                result.gradient = Vector2(1, 0);
                result.closestPoint = Vector2(radius, 0);
            }

            return result;
        }

        Vector2 GetSupportPoint(const Vector2& direction) const override
        {
            if (direction.LengthSquared() < 1e-12) {
                return Vector2(radius, 0);
            }
            return direction.Normalized() * radius;
        }

    private:
        Real radius;
    };

    // 사각형 프로파일
    class RectangleProfile : public ISweepProfile
    {
    public:
        RectangleProfile(const Vector2& size) : size(size) {}

        Real GetRadius() const override { return size.Length() * 0.5f; }
        Vector2 GetSize() const override { return size; }
        std::vector<Vector2> GetVertices() const override
        {
            Vector2 halfSize = size * 0.5f;
            return {
                Vector2(-halfSize.x, -halfSize.y),
                Vector2(halfSize.x, -halfSize.y),
                Vector2(halfSize.x, halfSize.y),
                Vector2(-halfSize.x, halfSize.y)
            };
        }

        SDFResult GetSDF(const Vector2& point) const override
        {
            SDFResult result;
            Vector2 halfSize = size * 0.5f;
            Vector2 d = Vector2(std::abs(point.x), std::abs(point.y)) - halfSize;

            Real outsideDistance = Vector2(std::max(d.x, Real(0)), std::max(d.y, Real(0))).Length();
            Real insideDistance = std::min(std::max(d.x, d.y), Real(0));

            result.distance = outsideDistance + insideDistance;
            result.isInside = d.x < 0 && d.y < 0;

            Vector2 closest(
                std::clamp(point.x, -halfSize.x, halfSize.x),
                std::clamp(point.y, -halfSize.y, halfSize.y)
            );
            result.closestPoint = closest;

            if (std::abs(result.distance) > 1e-6) {
                if (d.x > 0 && d.y > 0) {
                    result.gradient = Vector2(d.x > d.y ? 1 : 0, d.x > d.y ? 0 : 1);
                } else if (d.x > 0) {
                    result.gradient = Vector2(1, 0);
                } else if (d.y > 0) {
                    result.gradient = Vector2(0, 1);
                } else {
                    result.gradient = Vector2(0, 0);
                }
            } else {
                result.gradient = Vector2(1, 0);
            }

            return result;
        }

        Vector2 GetSupportPoint(const Vector2& direction) const override
        {
            if (direction.LengthSquared() < 1e-12) {
                return Vector2(size.x * 0.5f, 0);
            }

            Vector2 halfSize = size * 0.5f;
            Vector2 support(
                direction.x > 0 ? halfSize.x : -halfSize.x,
                direction.y > 0 ? halfSize.y : -halfSize.y
            );

            return support;
        }

    private:
        Vector2 size;
    };

    // 다각형 프로파일
    class PolygonProfile : public ISweepProfile
    {
    public:
        PolygonProfile(const std::vector<Vector2>& vertices) : vertices(vertices) {}

        Real GetRadius() const override
        {
            Real maxRadius = 0;
            for (const auto& vertex : vertices) {
                maxRadius = std::max(maxRadius, vertex.Length());
            }
            return maxRadius;
        }

        Vector2 GetSize() const override
        {
            if (vertices.empty()) return Vector2(0, 0);

            Vector2 min = vertices[0], max = vertices[0];
            for (const auto& vertex : vertices) {
                min.x = std::min(min.x, vertex.x);
                min.y = std::min(min.y, vertex.y);
                max.x = std::max(max.x, vertex.x);
                max.y = std::max(max.y, vertex.y);
            }
            return max - min;
        }

        std::vector<Vector2> GetVertices() const override { return vertices; }

        SDFResult GetSDF(const Vector2& point) const override
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
            bool isInside = true;

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

                // 내부/외부 판별 (외적 사용)
                Real cross = edge.x * toPoint.y - edge.y * toPoint.x;
                if (cross < 0) {
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

        Vector2 GetSupportPoint(const Vector2& direction) const override
        {
            if (vertices.empty()) return Vector2(0, 0);

            if (direction.LengthSquared() < 1e-12) {
                return vertices[0];
            }

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

    private:
        std::vector<Vector2> vertices;
    };

    // ========================================
    // 복잡한 도형 클래스
    // ========================================

    class ComplexShape : public IShape
    {
    public:
        ComplexShape();
        virtual ~ComplexShape();

        // 복사 생성자와 대입 연산자 (깊은 복사)
        ComplexShape(const ComplexShape& other);
        ComplexShape& operator=(const ComplexShape& other);

        // ========================================
        // IShape 인터페이스 구현
        // ========================================

        ShapeType GetType() const override { return ShapeType::CONVEX; }
        Vector2 GetCenter() const override;
        Real GetBoundingRadius() const override;

        // SAT를 위한 축 투영
        std::vector<Vector2> GetAxes() const override;
        std::pair<Real, Real> ProjectOnAxis(const Vector2& axis) const override;

        // SDF를 위한 거리 함수
        SDFResult GetSDF(const Vector2& point) const override;

        // GJK를 위한 지지점 함수
        Vector2 GetSupportPoint(const Vector2& direction) const override;

        // ========================================
        // 복잡한 도형 전용 함수들
        // ========================================

        // 도형 타입 반환
        ComplexShapeType GetComplexType() const { return type; }

        // 곡선 기반 스윕 도형의 경우 곡선 정보 반환
        CurveType GetCurveType() const { return curveType; }
        const std::vector<Vector2>& GetControlPoints() const { return controlPoints; }

        // 스윕 프로파일 정보 반환
        const ISweepProfile* GetSweepProfile() const { return profile.get(); }

        // 도형의 경계 상자 반환
        BoundingBox GetBoundingBox() const;

        // 도형의 면적 계산
        Real GetArea() const;

        // 도형의 둘레 계산
        Real GetPerimeter() const;

        // 도형의 무게중심 계산
        Vector2 GetCentroid() const;

        // 도형의 관성 모멘트 계산
        Real GetMomentOfInertia() const;

    protected:
        // 팩토리 메서드들이 사용할 수 있도록 protected로 설정
        ComplexShapeType type;

        // 곡선 기반 스윕 도형용
        CurveType curveType;
        std::vector<Vector2> controlPoints;
        std::unique_ptr<ISweepProfile> profile;

        // 민코스키 합 도형용
        const IShape* shapeA;
        const IShape* shapeB;

        // SDF 블렌딩 도형용
        enum class BlendOperation
        {
            UNION,      // 합집합 (OR)
            INTERSECTION, // 교집합 (AND)
            SUBTRACTION   // 차집합 (A - B)
        };
        BlendOperation blendOp;

        // 사용자 정의 SDF 도형용
        std::function<SDFResult(const Vector2&)> customSDF;

        // 캐싱된 정보들
        mutable BoundingBox cachedBoundingBox;
        mutable bool boundingBoxCached;
        mutable Vector2 cachedCenter;
        mutable bool centerCached;
        mutable Real cachedBoundingRadius;
        mutable bool boundingRadiusCached;

        // ========================================
        // 내부 구현 함수들
        // ========================================

        // 스윕 도형의 SDF 계산
        SDFResult CalculateSweptSDF(const Vector2& point) const;

        // 민코스키 합 도형의 SDF 계산
        SDFResult CalculateMinkowskiSDF(const Vector2& point) const;

        // SDF 블렌딩 계산
        SDFResult CalculateBlendedSDF(const Vector2& point) const;

        // 스윕 도형의 지지점 계산
        Vector2 CalculateSweptSupportPoint(const Vector2& direction) const;

        // 민코스키 합 도형의 지지점 계산
        Vector2 CalculateMinkowskiSupportPoint(const Vector2& direction) const;

        // 경계 상자 계산
        void CalculateBoundingBox() const;

        // 중심점 계산
        void CalculateCenter() const;

        // 바운딩 반지름 계산
        void CalculateBoundingRadius() const;

        // 팩토리 클래스가 접근할 수 있도록 friend 선언
        friend class ComplexShapeFactory;
    };
}
