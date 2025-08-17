#include "CollisionDetection.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Engine2D
{
    // ========================================
    // 전역 충돌 검출 인스턴스
    // ========================================

    CollisionDetection g_CollisionDetection;

    // ========================================
    // 생성자/소멸자
    // ========================================

    CollisionDetection::CollisionDetection()
    {
        ResetStats();
    }

    CollisionDetection::~CollisionDetection()
    {
    }

    // ========================================
    // GJK/EPA 알고리즘
    // ========================================

    bool CollisionDetection::DetectCollisionGJK(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return false;
        return GJK::DetectCollision(shapeA, shapeB);
    }

    CollisionInfo CollisionDetection::CalculatePenetrationEPA(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return CollisionInfo();

        // GJK로 충돌 확인
        if (!GJK::DetectCollision(shapeA, shapeB)) {
            return CollisionInfo(); // 충돌 없음
        }

        // EPA로 침투 정보 계산
        // 실제 구현에서는 GJK의 simplex를 EPA에 전달해야 함
        return EPA::CalculatePenetration(Simplex(), shapeA, shapeB);
    }

    // ========================================
    // SAT 알고리즘
    // ========================================

    bool CollisionDetection::DetectCollisionSAT(const std::vector<Vector2>& polygonA,
                                               const std::vector<Vector2>& polygonB)
    {
        SATResult result = SAT::DetectCollision(polygonA, polygonB);
        return result.hasCollision;
    }

    SATResult CollisionDetection::DetectCollisionSATDetailed(const std::vector<Vector2>& polygonA,
                                                            const std::vector<Vector2>& polygonB)
    {
        return SAT::DetectCollision(polygonA, polygonB);
    }

    bool CollisionDetection::DetectCollisionSATCirclePolygon(const Vector2& circleCenter, Real radius,
                                                            const std::vector<Vector2>& polygon)
    {
        SATResult result = SAT::DetectCirclePolygonCollision(circleCenter, radius, polygon);
        return result.hasCollision;
    }

    bool CollisionDetection::DetectCollisionSATCircles(const Vector2& centerA, Real radiusA,
                                                      const Vector2& centerB, Real radiusB)
    {
        SATResult result = SAT::DetectCircleCollision(centerA, radiusA, centerB, radiusB);
        return result.hasCollision;
    }

    // ========================================
    // SDF 기반 알고리즘
    // ========================================

    Real CollisionDetection::CalculateSDFDistance(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return std::numeric_limits<Real>::max();

        // 두 도형의 중심점 사이의 거리
        Vector2 centerA = shapeA->GetCenter();
        Vector2 centerB = shapeB->GetCenter();
        Real centerDistance = (centerB - centerA).Length();

        // 각 도형의 바운딩 반지름
        Real radiusA = shapeA->GetBoundingRadius();
        Real radiusB = shapeB->GetBoundingRadius();

        // 최소 거리 = 중심 거리 - 반지름 합
        return centerDistance - radiusA - radiusB;
    }

    bool CollisionDetection::DetectCollisionSDF(const IShape* shapeA, const IShape* shapeB)
    {
        Real distance = CalculateSDFDistance(shapeA, shapeB);
        return distance <= 0;
    }

    Vector2 CollisionDetection::FindSDFContactPoint(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return Vector2(0, 0);

        Vector2 centerA = shapeA->GetCenter();
        Vector2 centerB = shapeB->GetCenter();
        Vector2 direction = (centerB - centerA).Normalized();

        // 각 도형에서 접촉점 찾기
        Vector2 pointA = shapeA->GetSupportPoint(direction);
        Vector2 pointB = shapeB->GetSupportPoint(-direction);

        // 두 접촉점의 중점
        return (pointA + pointB) * Real(0.5);
    }

    // ========================================
    // 최적 알고리즘 선택
    // ========================================

    CollisionAlgorithm CollisionDetection::SelectOptimalAlgorithm(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return CollisionAlgorithm::AUTO;

        ShapeType typeA = shapeA->GetType();
        ShapeType typeB = shapeB->GetType();

        // 원과 원: SDF가 가장 효율적
        if ((typeA == ShapeType::CIRCLE || typeA == ShapeType::ELLIPSE) &&
            (typeB == ShapeType::CIRCLE || typeB == ShapeType::ELLIPSE)) {
            return CollisionAlgorithm::SDF;
        }

        // 볼록 다각형들: SAT가 효율적
        if ((typeA == ShapeType::RECTANGLE || typeA == ShapeType::TRIANGLE ||
             typeA == ShapeType::CONVEX) &&
            (typeB == ShapeType::RECTANGLE || typeB == ShapeType::TRIANGLE ||
             typeB == ShapeType::CONVEX)) {
            return CollisionAlgorithm::SAT;
        }

        // 오목 다각형이 포함된 경우: GJK/EPA가 안전
        if (typeA == ShapeType::CONCAVE || typeB == ShapeType::CONCAVE) {
            return CollisionAlgorithm::GJK_EPA;
        }

        // 기본값: GJK/EPA (가장 범용적)
        return CollisionAlgorithm::GJK_EPA;
    }

    // ========================================
    // 통합 충돌 검출
    // ========================================

    CollisionResult CollisionDetection::DetectCollision(const IShape* shapeA, const IShape* shapeB)
    {
        CollisionAlgorithm algorithm = SelectOptimalAlgorithm(shapeA, shapeB);
        return DetectCollision(shapeA, shapeB, algorithm);
    }

    CollisionResult CollisionDetection::DetectCollision(const IShape* shapeA, const IShape* shapeB,
                                                       CollisionAlgorithm algorithm)
    {
        CollisionResult result;
        result.algorithm = algorithm;

        switch (algorithm) {
            case CollisionAlgorithm::GJK_EPA: {
                CollisionInfo info = CalculatePenetrationEPA(shapeA, shapeB);
                result.hasCollision = info.hasCollision;
                result.normal = info.normal;
                result.depth = info.depth;
                result.contactPoint = info.contactPoint;
                break;
            }

            case CollisionAlgorithm::SDF: {
                result.hasCollision = DetectCollisionSDF(shapeA, shapeB);
                if (result.hasCollision) {
                    result.contactPoint = FindSDFContactPoint(shapeA, shapeB);
                    result.distance = 0;
                } else {
                    result.distance = CalculateSDFDistance(shapeA, shapeB);
                }
                break;
            }

            case CollisionAlgorithm::SAT: {
                // SAT는 다각형 정점이 필요하므로 IShape에서 추출해야 함
                // 실제 구현에서는 IShape에서 정점을 가져오는 방법이 필요
                CollisionInfo info = CalculatePenetrationEPA(shapeA, shapeB); // 임시로 GJK/EPA 사용
                result.hasCollision = info.hasCollision;
                result.normal = info.normal;
                result.depth = info.depth;
                result.contactPoint = info.contactPoint;
                break;
            }

            case CollisionAlgorithm::AUTO:
            default:
                return DetectCollision(shapeA, shapeB, SelectOptimalAlgorithm(shapeA, shapeB));
        }

        return result;
    }

    bool CollisionDetection::DetectCollision(const IShape* shapeA, const IShape* shapeB)
    {
        CollisionAlgorithm algorithm = SelectOptimalAlgorithm(shapeA, shapeB);
        return DetectCollision(shapeA, shapeB, algorithm);
    }

    bool CollisionDetection::DetectCollision(const IShape* shapeA, const IShape* shapeB,
                                            CollisionAlgorithm algorithm)
    {
        if (!shapeA || !shapeB) return false;

        switch (algorithm) {
            case CollisionAlgorithm::GJK_EPA:
                return DetectCollisionGJK(shapeA, shapeB);

            case CollisionAlgorithm::SDF:
                return DetectCollisionSDF(shapeA, shapeB);

            case CollisionAlgorithm::SAT:
                // SAT는 다각형 정점이 필요하므로 IShape에서 추출해야 함
                // 실제 구현에서는 IShape에서 정점을 가져오는 방법이 필요
                return DetectCollisionGJK(shapeA, shapeB); // 임시로 GJK 사용

            case CollisionAlgorithm::AUTO:
            default:
                return DetectCollision(shapeA, shapeB, SelectOptimalAlgorithm(shapeA, shapeB));
        }
    }

    // ========================================
    // 거리 계산
    // ========================================

    Real CollisionDetection::CalculateDistance(const IShape* shapeA, const IShape* shapeB)
    {
        return CalculateMinimumDistance(shapeA, shapeB);
    }

    Real CollisionDetection::CalculateMinimumDistance(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return std::numeric_limits<Real>::max();

        // 충돌이 있는 경우 거리는 0
        if (DetectCollision(shapeA, shapeB)) {
            return 0;
        }

        // SDF 기반 거리 계산
        return CalculateSDFDistance(shapeA, shapeB);
    }

    Real CollisionDetection::CalculateDistanceToPoint(const IShape* shape, const Vector2& point)
    {
        if (!shape) return std::numeric_limits<Real>::max();

        SDFResult sdfResult = shape->GetSDF(point);
        return std::abs(sdfResult.distance);
    }

    // ========================================
    // 접촉점 계산
    // ========================================

    std::vector<Vector2> CollisionDetection::CalculateContactPoints(const IShape* shapeA, const IShape* shapeB)
    {
        std::vector<Vector2> contactPoints;

        if (!shapeA || !shapeB) return contactPoints;

        // 충돌이 없는 경우 빈 벡터 반환
        if (!DetectCollision(shapeA, shapeB)) {
            return contactPoints;
        }

        // SDF 기반 접촉점 계산
        Vector2 contactPoint = FindSDFContactPoint(shapeA, shapeB);
        contactPoints.push_back(contactPoint);

        return contactPoints;
    }

    // ========================================
    // 유틸리티 함수
    // ========================================

    bool CollisionDetection::IsConvexShape(const IShape* shape)
    {
        if (!shape) return false;

        ShapeType type = shape->GetType();
        return (type == ShapeType::CIRCLE || type == ShapeType::RECTANGLE ||
                type == ShapeType::TRIANGLE || type == ShapeType::CONVEX);
    }

    bool CollisionDetection::IsConcaveShape(const IShape* shape)
    {
        if (!shape) return false;
        return shape->GetType() == ShapeType::CONCAVE;
    }

    bool CollisionDetection::AreSameShapeType(const IShape* shapeA, const IShape* shapeB)
    {
        if (!shapeA || !shapeB) return false;
        return shapeA->GetType() == shapeB->GetType();
    }

    int CollisionDetection::EvaluateShapeComplexity(const IShape* shape)
    {
        if (!shape) return 0;

        ShapeType type = shape->GetType();
        switch (type) {
            case ShapeType::CIRCLE:
            case ShapeType::ELLIPSE:
                return 1; // 매우 단순

            case ShapeType::RECTANGLE:
            case ShapeType::TRIANGLE:
                return 2; // 단순

            case ShapeType::CONVEX:
                return 3; // 보통

            case ShapeType::CONCAVE:
                return 4; // 복잡

            default:
                return 3; // 기본값
        }
    }

    // ========================================
    // 개별 알고리즘 구현 (private)
    // ========================================

    CollisionResult CollisionDetection::DetectCollisionGJK_EPA(const IShape* shapeA, const IShape* shapeB)
    {
        CollisionResult result;
        CollisionInfo info = CalculatePenetrationEPA(shapeA, shapeB);

        result.hasCollision = info.hasCollision;
        result.normal = info.normal;
        result.depth = info.depth;
        result.contactPoint = info.contactPoint;
        result.algorithm = CollisionAlgorithm::GJK_EPA;

        return result;
    }

    CollisionResult CollisionDetection::DetectCollisionSAT(const IShape* shapeA, const IShape* shapeB)
    {
        CollisionResult result;
        result.algorithm = CollisionAlgorithm::SAT;

        // SAT는 다각형 정점이 필요하므로 IShape에서 추출해야 함
        // 실제 구현에서는 IShape에서 정점을 가져오는 방법이 필요
        // 임시로 GJK/EPA 사용
        return DetectCollisionGJK_EPA(shapeA, shapeB);
    }

    CollisionResult CollisionDetection::DetectCollisionSDF(const IShape* shapeA, const IShape* shapeB)
    {
        CollisionResult result;
        result.algorithm = CollisionAlgorithm::SDF;

        result.hasCollision = DetectCollisionSDF(shapeA, shapeB);
        if (result.hasCollision) {
            result.contactPoint = FindSDFContactPoint(shapeA, shapeB);
            result.distance = 0;
        } else {
            result.distance = CalculateSDFDistance(shapeA, shapeB);
        }

        return result;
    }

    // ========================================
    // 알고리즘별 적합성 판단 (private)
    // ========================================

    bool CollisionDetection::IsGJK_EPASuitable(const IShape* shapeA, const IShape* shapeB)
    {
        return true; // GJK/EPA는 모든 도형에 적용 가능
    }

    bool CollisionDetection::IsSATSuitable(const IShape* shapeA, const IShape* shapeB)
    {
        return IsConvexShape(shapeA) && IsConvexShape(shapeB);
    }

    bool CollisionDetection::IsSDFSuitable(const IShape* shapeA, const IShape* shapeB)
    {
        ShapeType typeA = shapeA->GetType();
        ShapeType typeB = shapeB->GetType();

        return (typeA == ShapeType::CIRCLE || typeA == ShapeType::ELLIPSE) &&
               (typeB == ShapeType::CIRCLE || typeB == ShapeType::ELLIPSE);
    }
}
