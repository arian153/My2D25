#pragma once

#include <vector>
#include "../Algebra/Vector2.hpp"
#include "../Geometry/Line/Ray.hpp"
#include "../Geometry/Line/LineSegment.hpp"

namespace Engine2D
{
    // ========================================
    // 바운딩 박스 구조체
    // ========================================

    struct BoundingBox
    {
        Vector2 min;  // 최소점 (좌하단)
        Vector2 max;  // 최대점 (우상단)

        // 생성자
        BoundingBox() : min(0, 0), max(0, 0) {}
        BoundingBox(const Vector2& minPoint, const Vector2& maxPoint) : min(minPoint), max(maxPoint) {}
        BoundingBox(Real minX, Real minY, Real maxX, Real maxY) : min(minX, minY), max(maxX, maxY) {}

        // ========================================
        // Static 함수들
        // ========================================

        // 점들로부터 바운딩 박스 생성
        static BoundingBox FromPoints(const std::vector<Vector2>& points)
        {
            if (points.empty()) {
                return BoundingBox();
            }

            Vector2 minPoint = points[0];
            Vector2 maxPoint = points[0];

            for (const auto& point : points) {
                minPoint.x = std::min(minPoint.x, point.x);
                minPoint.y = std::min(minPoint.y, point.y);
                maxPoint.x = std::max(maxPoint.x, point.x);
                maxPoint.y = std::max(maxPoint.y, point.y);
            }

            return BoundingBox(minPoint, maxPoint);
        }

        // 두 바운딩 박스의 합집합
        static BoundingBox Union(const BoundingBox& a, const BoundingBox& b)
        {
            Vector2 minPoint(
                std::min(a.min.x, b.min.x),
                std::min(a.min.y, b.min.y)
            );
            Vector2 maxPoint(
                std::max(a.max.x, b.max.x),
                std::max(a.max.y, b.max.y)
            );
            return BoundingBox(minPoint, maxPoint);
        }

        // 두 바운딩 박스의 교집합
        static BoundingBox Intersection(const BoundingBox& a, const BoundingBox& b)
        {
            Vector2 minPoint(
                std::max(a.min.x, b.min.x),
                std::max(a.min.y, b.min.y)
            );
            Vector2 maxPoint(
                std::min(a.max.x, b.max.x),
                std::min(a.max.y, b.max.y)
            );

            // 교집합이 없는 경우
            if (minPoint.x > maxPoint.x || minPoint.y > maxPoint.y) {
                return BoundingBox();
            }

            return BoundingBox(minPoint, maxPoint);
        }

        // ========================================
        // 멤버 함수들
        // ========================================

        // 중심점
        Vector2 GetCenter() const
        {
            return (min + max) * 0.5f;
        }

        // 크기
        Vector2 GetSize() const
        {
            return max - min;
        }

        // 너비
        Real GetWidth() const
        {
            return max.x - min.x;
        }

        // 높이
        Real GetHeight() const
        {
            return max.y - min.y;
        }

        // 면적
        Real GetArea() const
        {
            return GetWidth() * GetHeight();
        }

        // 둘레
        Real GetPerimeter() const
        {
            return 2.0f * (GetWidth() + GetHeight());
        }

        // 점이 박스 내부에 있는지 확인
        bool Contains(const Vector2& point) const
        {
            return point.x >= min.x && point.x <= max.x &&
                   point.y >= min.y && point.y <= max.y;
        }

        // 다른 박스와 겹치는지 확인
        bool Intersects(const BoundingBox& other) const
        {
            return !(max.x < other.min.x || min.x > other.max.x ||
                     max.y < other.min.y || min.y > other.max.y);
        }

        // 박스 확장
        void Expand(const Vector2& point)
        {
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
        }

        // 박스 확장 (여백 추가)
        void Expand(Real margin)
        {
            min.x -= margin;
            min.y -= margin;
            max.x += margin;
            max.y += margin;
        }

        // 박스 이동
        void Translate(const Vector2& offset)
        {
            min += offset;
            max += offset;
        }

        // 박스 스케일
        void Scale(Real scale)
        {
            Vector2 center = GetCenter();
            Vector2 size = GetSize() * scale * 0.5f;
            min = center - size;
            max = center + size;
        }
    };

    // ========================================
    // 충돌 결과 구조체
    // ========================================

    struct CollisionResult
    {
        bool hasCollision;           // 충돌 여부
        Vector2 normal;              // 충돌 법선 벡터
        Real depth;                  // 침투 깊이
        Vector2 contactPoint;        // 접촉점
        std::vector<Vector2> contactPoints; // 접촉점들 (여러 개일 수 있음)

        // 생성자
        CollisionResult() : hasCollision(false), normal(0, 0), depth(0), contactPoint(0, 0) {}
        CollisionResult(bool collision, const Vector2& n, Real d, const Vector2& cp)
            : hasCollision(collision), normal(n), depth(d), contactPoint(cp) {}

        // ========================================
        // Static 함수들
        // ========================================

        // 충돌 없음 결과 생성
        static CollisionResult NoCollision()
        {
            return CollisionResult();
        }

        // 충돌 결과 생성
        static CollisionResult Collision(const Vector2& normal, Real depth, const Vector2& contactPoint)
        {
            return CollisionResult(true, normal, depth, contactPoint);
        }

        // ========================================
        // 멤버 함수들
        // ========================================

        // 접촉점 추가
        void AddContactPoint(const Vector2& point)
        {
            contactPoints.push_back(point);
            if (contactPoints.size() == 1) {
                contactPoint = point;
            }
        }

        // 접촉점들 설정
        void SetContactPoints(const std::vector<Vector2>& points)
        {
            contactPoints = points;
            if (!points.empty()) {
                contactPoint = points[0];
            }
        }

        // 충돌 정보 설정
        void SetCollision(const Vector2& n, Real d, const Vector2& cp)
        {
            hasCollision = true;
            normal = n;
            depth = d;
            contactPoint = cp;
        }

        // 충돌 해제
        void ClearCollision()
        {
            hasCollision = false;
            normal = Vector2(0, 0);
            depth = 0;
            contactPoint = Vector2(0, 0);
            contactPoints.clear();
        }
    };

    // ========================================
    // 레이캐스트 결과 구조체
    // ========================================

    struct RaycastResult
    {
        bool hit;                    // 히트 여부
        Vector2 hitPoint;            // 히트 포인트
        Vector2 normal;              // 히트 표면의 법선 벡터
        Real distance;               // 레이 시작점부터의 거리
        Real fraction;               // 레이 방향으로의 비율 (0~1)
        void* hitObject;             // 히트한 객체 (타입 안전성을 위해 void* 사용)

        // 생성자
        RaycastResult() : hit(false), hitPoint(0, 0), normal(0, 0), distance(0), fraction(0), hitObject(nullptr) {}
        RaycastResult(bool h, const Vector2& hp, const Vector2& n, Real d, Real f, void* obj = nullptr)
            : hit(h), hitPoint(hp), normal(n), distance(d), fraction(f), hitObject(obj) {}

        // ========================================
        // Static 함수들
        // ========================================

        // 히트 없음 결과 생성
        static RaycastResult NoHit()
        {
            return RaycastResult();
        }

        // 히트 결과 생성
        static RaycastResult Hit(const Vector2& hitPoint, const Vector2& normal, Real distance, Real fraction, void* object = nullptr)
        {
            return RaycastResult(true, hitPoint, normal, distance, fraction, object);
        }

        // ========================================
        // 멤버 함수들
        // ========================================

        // 히트 정보 설정
        void SetHit(const Vector2& hp, const Vector2& n, Real d, Real f, void* obj = nullptr)
        {
            hit = true;
            hitPoint = hp;
            normal = n;
            distance = d;
            fraction = f;
            hitObject = obj;
        }

        // 히트 해제
        void ClearHit()
        {
            hit = false;
            hitPoint = Vector2(0, 0);
            normal = Vector2(0, 0);
            distance = 0;
            fraction = 0;
            hitObject = nullptr;
        }

        // 더 가까운 히트로 업데이트
        void UpdateIfCloser(const RaycastResult& other)
        {
            if (other.hit && (!hit || other.distance < distance)) {
                *this = other;
            }
        }
    };


}
