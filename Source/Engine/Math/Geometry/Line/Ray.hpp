#pragma once

#include <cmath>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../../Structure/Geometry.hpp"

namespace Engine2D
{
    // ========================================
    // 레이 클래스
    // ========================================

    class Ray
    {
    public:
        Vector2 origin;      // 시작점
        Vector2 direction;   // 방향 벡터 (정규화됨)
        Real maxDistance;    // 최대 거리

        // 생성자
        Ray() : origin(0, 0), direction(1, 0), maxDistance(INFINITY) {}
        Ray(const Vector2& o, const Vector2& dir) : origin(o), direction(dir.Normalized()), maxDistance(INFINITY) {}
        Ray(const Vector2& o, const Vector2& dir, Real maxDist) : origin(o), direction(dir.Normalized()), maxDistance(maxDist) {}

        // ========================================
        // Static 함수들 (핵심 알고리즘)
        // ========================================

        // 두 점으로부터 레이 생성
        static Ray FromPoints(const Vector2& start, const Vector2& end)
        {
            Vector2 direction = (end - start).Normalized();
            return Ray(start, direction);
        }

        // 점과 방향으로부터 레이 생성
        static Ray FromPointAndDirection(const Vector2& point, const Vector2& direction)
        {
            return Ray(point, direction);
        }

        // 점과 각도로부터 레이 생성
        static Ray FromPointAndAngle(const Vector2& point, Real angle)
        {
            Vector2 direction(std::cos(angle), std::sin(angle));
            return Ray(point, direction);
        }

        // 레이 이동 (Static)
        static Ray Translate(const Ray& ray, const Vector2& offset)
        {
            return Ray(ray.origin + offset, ray.direction, ray.maxDistance);
        }

        // 레이 회전 (Static)
        static Ray Rotate(const Ray& ray, const Vector2& center, Real angle)
        {
            Vector2 rotatedOrigin = Transform::Rotate(ray.origin, center, angle);
            Vector2 rotatedDirection = Transform::Rotate(ray.origin + ray.direction, center, angle) - rotatedOrigin;
            return Ray(rotatedOrigin, rotatedDirection, ray.maxDistance);
        }

        // 레이 스케일 (Static)
        static Ray Scale(const Ray& ray, const Vector2& center, Real scale)
        {
            Vector2 scaledOrigin = Transform::Scale(ray.origin, center, scale);
            Vector2 scaledDirection = ray.direction; // 방향은 스케일링하지 않음
            return Ray(scaledOrigin, scaledDirection, ray.maxDistance * scale);
        }

        // ========================================
        // 멤버 함수들
        // ========================================

        // 레이의 끝점 계산
        Vector2 GetEndPoint() const
        {
            return origin + direction * maxDistance;
        }

        // 특정 거리에서의 점 계산
        Vector2 GetPointAtDistance(Real distance) const
        {
            return origin + direction * distance;
        }

        // 레이의 길이 (최대 거리)
        Real GetLength() const
        {
            return maxDistance;
        }

        // 레이의 방향 벡터
        Vector2 GetDirection() const
        {
            return direction;
        }

        // 레이의 시작점
        Vector2 GetOrigin() const
        {
            return origin;
        }

        // 방향 설정 (자동 정규화)
        void SetDirection(const Vector2& dir)
        {
            direction = dir.Normalized();
        }

        // 시작점 설정
        void SetOrigin(const Vector2& point)
        {
            origin = point;
        }

        // 최대 거리 설정
        void SetMaxDistance(Real distance)
        {
            maxDistance = distance;
        }

        // 레이 이동
        void Translate(const Vector2& offset)
        {
            origin += offset;
        }

        // 레이 회전 (시작점 기준)
        void Rotate(Real angle)
        {
            Real cosAngle = std::cos(angle);
            Real sinAngle = std::sin(angle);

            Vector2 rotated(
                direction.x * cosAngle - direction.y * sinAngle,
                direction.x * sinAngle + direction.y * cosAngle
            );

            direction = rotated;
        }

        // 레이 회전 (중심점 기준)
        void Rotate(const Vector2& center, Real angle)
        {
            origin = Transform::Rotate(origin, center, angle);
            Rotate(angle);
        }

        // 레이 스케일 (시작점 기준)
        void Scale(Real scale)
        {
            maxDistance *= scale;
        }

        // 레이 스케일 (중심점 기준)
        void Scale(const Vector2& center, Real scale)
        {
            origin = Transform::Scale(origin, center, scale);
            maxDistance *= scale;
        }

        // ========================================
        // 기하학적 연산 함수들
        // ========================================

        // 점이 레이 위에 있는지 확인
        bool Contains(const Vector2& point) const
        {
            Vector2 toPoint = point - origin;
            Real distance = toPoint.Dot(direction);

            if (distance < 0 || distance > maxDistance) return false;

            Vector2 projectedPoint = origin + direction * distance;
            return (point - projectedPoint).LengthSquared() < 1e-12;
        }

        // 레이와 점 사이의 거리
        Real GetDistanceToPoint(const Vector2& point) const
        {
            Vector2 toPoint = point - origin;
            Real distance = toPoint.Dot(direction);

            if (distance < 0) {
                return toPoint.Length();
            } else if (distance > maxDistance) {
                return (point - GetEndPoint()).Length();
            } else {
                Vector2 projectedPoint = origin + direction * distance;
                return (point - projectedPoint).Length();
            }
        }

        // 레이 위에서 점에 가장 가까운 점
        Vector2 GetClosestPoint(const Vector2& point) const
        {
            Vector2 toPoint = point - origin;
            Real distance = toPoint.Dot(direction);

            distance = std::clamp(distance, Real(0), maxDistance);
            return origin + direction * distance;
        }

        // 레이와 다른 레이 사이의 거리
        Real GetDistanceToRay(const Ray& other) const
        {
            Vector2 w0 = origin - other.origin;
            Real a = direction.Dot(direction);
            Real b = direction.Dot(other.direction);
            Real c = other.direction.Dot(other.direction);
            Real d = direction.Dot(w0);
            Real e = other.direction.Dot(w0);

            Real denominator = a * c - b * b;

            if (std::abs(denominator) < 1e-12) {
                // 평행한 경우
                return GetDistanceToPoint(other.origin);
            }

            Real s = (b * e - c * d) / denominator;
            Real t = (a * e - b * d) / denominator;

            s = std::clamp(s, Real(0), maxDistance);
            t = std::clamp(t, Real(0), other.maxDistance);

            Vector2 point1 = origin + direction * s;
            Vector2 point2 = other.origin + other.direction * t;

            return (point1 - point2).Length();
        }

        // 레이와 선분 사이의 거리
        Real GetDistanceToLineSegment(const LineSegment& segment) const
        {
            Vector2 segmentDir = segment.GetDirection();
            Vector2 w0 = origin - segment.start;
            Real a = direction.Dot(direction);
            Real b = direction.Dot(segmentDir);
            Real c = segmentDir.Dot(segmentDir);
            Real d = direction.Dot(w0);
            Real e = segmentDir.Dot(w0);

            Real denominator = a * c - b * b;

            if (std::abs(denominator) < 1e-12) {
                // 평행한 경우
                return std::min(GetDistanceToPoint(segment.start), GetDistanceToPoint(segment.end));
            }

            Real s = (b * e - c * d) / denominator;
            Real t = (a * e - b * d) / denominator;

            s = std::clamp(s, Real(0), maxDistance);
            t = std::clamp(t, Real(0), segment.GetLength());

            Vector2 point1 = origin + direction * s;
            Vector2 point2 = segment.start + segmentDir * t;

            return (point1 - point2).Length();
        }

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 레이와 원의 충돌 검출
        RaycastResult IntersectCircle(const Vector2& center, Real radius) const
        {
            Vector2 toCenter = center - origin;
            Real projection = toCenter.Dot(direction);

            if (projection < 0) {
                // 레이 시작점이 원 뒤에 있음
                return RaycastResult::NoHit();
            }

            Vector2 closestPoint = origin + direction * projection;
            Real distanceToCenter = (closestPoint - center).Length();

            if (distanceToCenter > radius) {
                // 레이가 원과 만나지 않음
                return RaycastResult::NoHit();
            }

            Real halfChord = std::sqrt(radius * radius - distanceToCenter * distanceToCenter);
            Real distance1 = projection - halfChord;
            Real distance2 = projection + halfChord;

            // 최대 거리 내에서의 교점 찾기
            if (distance1 >= 0 && distance1 <= maxDistance) {
                Vector2 hitPoint = origin + direction * distance1;
                Vector2 normal = (hitPoint - center).Normalized();
                return RaycastResult::Hit(hitPoint, normal, distance1, distance1 / maxDistance);
            } else if (distance2 >= 0 && distance2 <= maxDistance) {
                Vector2 hitPoint = origin + direction * distance2;
                Vector2 normal = (hitPoint - center).Normalized();
                return RaycastResult::Hit(hitPoint, normal, distance2, distance2 / maxDistance);
            }

            return RaycastResult::NoHit();
        }

        // 레이와 사각형의 충돌 검출
        RaycastResult IntersectRectangle(const Vector2& center, const Vector2& size, Real rotation) const
        {
            // 사각형을 로컬 좌표계로 변환
            Vector2 localOrigin = Transform::Rotate(origin - center, Vector2(0, 0), -rotation);
            Vector2 localDirection = Transform::Rotate(origin + direction - center, Vector2(0, 0), -rotation) - localOrigin;
            localDirection = localDirection.Normalized();

            Vector2 halfSize = size * 0.5f;

            // AABB 충돌 검출
            Real tMin = (-halfSize.x - localOrigin.x) / localDirection.x;
            Real tMax = (halfSize.x - localOrigin.x) / localDirection.x;

            if (tMin > tMax) std::swap(tMin, tMax);

            Real tyMin = (-halfSize.y - localOrigin.y) / localDirection.y;
            Real tyMax = (halfSize.y - localOrigin.y) / localDirection.y;

            if (tyMin > tyMax) std::swap(tyMin, tyMax);

            if (tMin > tyMax || tyMin > tMax) {
                return RaycastResult::NoHit();
            }

            tMin = std::max(tMin, tyMin);
            tMax = std::min(tMax, tyMax);

            if (tMin < 0 || tMin > maxDistance) {
                return RaycastResult::NoHit();
            }

            Vector2 localHitPoint = localOrigin + localDirection * tMin;
            Vector2 worldHitPoint = Transform::Rotate(localHitPoint, Vector2(0, 0), rotation) + center;

            // 법선 벡터 계산
            Vector2 localNormal;
            if (std::abs(localHitPoint.x + halfSize.x) < 1e-6) localNormal = Vector2(-1, 0);
            else if (std::abs(localHitPoint.x - halfSize.x) < 1e-6) localNormal = Vector2(1, 0);
            else if (std::abs(localHitPoint.y + halfSize.y) < 1e-6) localNormal = Vector2(0, -1);
            else localNormal = Vector2(0, 1);

            Vector2 worldNormal = Transform::Rotate(localNormal, Vector2(0, 0), rotation);

            return RaycastResult::Hit(worldHitPoint, worldNormal, tMin, tMin / maxDistance);
        }

        // ========================================
        // 유틸리티 함수들
        // ========================================

        // 레이의 반대 방향
        Ray GetOpposite() const
        {
            return Ray(origin, -direction, maxDistance);
        }

        // 레이의 복사본
        Ray Clone() const
        {
            return Ray(origin, direction, maxDistance);
        }

        // 레이 정보 문자열
        std::string ToString() const
        {
            return "Ray(origin=(" + std::to_string(origin.x) + ", " + std::to_string(origin.y) +
                   "), direction=(" + std::to_string(direction.x) + ", " + std::to_string(direction.y) +
                   "), maxDistance=" + std::to_string(maxDistance) + ")";
        }
    };
}
