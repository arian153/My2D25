#include "LineSegment.hpp"
#include <cmath>
#include <string>

namespace Engine2D
{
    // ========================================
    // 생성자
    // ========================================

    LineSegment::LineSegment() : start(0, 0), end(0, 0) {}

    LineSegment::LineSegment(const Vector2& s, const Vector2& e) : start(s), end(e) {}

    LineSegment::LineSegment(Real x1, Real y1, Real x2, Real y2) : start(x1, y1), end(x2, y2) {}

    // ========================================
    // Static 함수들 (핵심 알고리즘)
    // ========================================

    // 방향 벡터 계산
    Vector2 LineSegment::GetDirection(const Vector2& start, const Vector2& end)
    {
        return (end - start).Normalized();
    }

    // 길이 계산
    Real LineSegment::GetLength(const Vector2& start, const Vector2& end)
    {
        return (end - start).Length();
    }

    // 중심점 계산
    Vector2 LineSegment::GetCenter(const Vector2& start, const Vector2& end)
    {
        return (start + end) * 0.5f;
    }

    // 선분 이동 (Static)
    LineSegment LineSegment::Translate(const LineSegment& segment, const Vector2& offset)
    {
        return LineSegment(segment.start + offset, segment.end + offset);
    }

    // 선분 회전 (Static)
    LineSegment LineSegment::Rotate(const LineSegment& segment, const Vector2& center, Real angle)
    {
        Vector2 rotatedStart = Transform::Rotate(segment.start, center, angle);
        Vector2 rotatedEnd = Transform::Rotate(segment.end, center, angle);
        return LineSegment(rotatedStart, rotatedEnd);
    }

    // 선분 스케일 (Static)
    LineSegment LineSegment::Scale(const LineSegment& segment, const Vector2& center, Real scale)
    {
        Vector2 scaledStart = Transform::Scale(segment.start, center, scale);
        Vector2 scaledEnd = Transform::Scale(segment.end, center, scale);
        return LineSegment(scaledStart, scaledEnd);
    }

    // 두 선분의 교점 계산
    bool LineSegment::GetIntersection(const LineSegment& a, const LineSegment& b, Vector2& intersection)
    {
        Vector2 dirA = a.GetDirection();
        Vector2 dirB = b.GetDirection();
        Vector2 diff = b.start - a.start;

        Real denominator = dirA.x * dirB.y - dirA.y * dirB.x;

        if (std::abs(denominator) < 1e-12) {
            // 평행한 경우
            return false;
        }

        Real tA = (diff.x * dirB.y - diff.y * dirB.x) / denominator;
        Real tB = (diff.x * dirA.y - diff.y * dirA.x) / denominator;

        if (tA >= 0 && tA <= a.GetLength() && tB >= 0 && tB <= b.GetLength()) {
            intersection = a.start + dirA * tA;
            return true;
        }

        return false;
    }

    // ========================================
    // 멤버 함수들
    // ========================================

    // 방향 벡터
    Vector2 LineSegment::GetDirection() const
    {
        return GetDirection(start, end);
    }

    // 길이
    Real LineSegment::GetLength() const
    {
        return GetLength(start, end);
    }

    // 중심점
    Vector2 LineSegment::GetCenter() const
    {
        return GetCenter(start, end);
    }

    // 선분 이동
    void LineSegment::Translate(const Vector2& offset)
    {
        start += offset;
        end += offset;
    }

    // 선분 회전
    void LineSegment::Rotate(const Vector2& center, Real angle)
    {
        start = Transform::Rotate(start, center, angle);
        end = Transform::Rotate(end, center, angle);
    }

    // 선분 스케일
    void LineSegment::Scale(const Vector2& center, Real scale)
    {
        start = Transform::Scale(start, center, scale);
        end = Transform::Scale(end, center, scale);
    }

    // 두 선분의 교점 계산
    bool LineSegment::GetIntersection(const LineSegment& other, Vector2& intersection) const
    {
        return GetIntersection(*this, other, intersection);
    }

    // 점이 선분 위에 있는지 확인
    bool LineSegment::ContainsPoint(const Vector2& point) const
    {
        Vector2 toPoint = point - start;
        Vector2 direction = GetDirection();
        Real projection = toPoint.Dot(direction);
        Real length = GetLength();

        if (projection < 0 || projection > length) {
            return false;
        }

        Vector2 projectedPoint = start + direction * projection;
        Real distance = (point - projectedPoint).Length();

        return distance < 1e-6;
    }

    // 선분과 점 사이의 거리
    Real LineSegment::GetDistanceToPoint(const Vector2& point) const
    {
        Vector2 toPoint = point - start;
        Vector2 direction = GetDirection();
        Real projection = toPoint.Dot(direction);
        Real length = GetLength();

        if (projection <= 0) {
            return (point - start).Length();
        } else if (projection >= length) {
            return (point - end).Length();
        } else {
            Vector2 projectedPoint = start + direction * projection;
            return (point - projectedPoint).Length();
        }
    }

    // 선분에서 가장 가까운 점
    Vector2 LineSegment::GetClosestPoint(const Vector2& point) const
    {
        Vector2 toPoint = point - start;
        Vector2 direction = GetDirection();
        Real projection = toPoint.Dot(direction);
        Real length = GetLength();

        projection = std::clamp(projection, Real(0), length);
        return start + direction * projection;
    }

    // 선분의 법선 벡터
    Vector2 LineSegment::GetNormal() const
    {
        Vector2 direction = GetDirection();
        return Vector2(-direction.y, direction.x);
    }

    // 선분의 접선 벡터
    Vector2 LineSegment::GetTangent() const
    {
        return GetDirection();
    }

    // 선분의 바운딩 박스
    BoundingBox LineSegment::GetBoundingBox() const
    {
        Vector2 min = Vector2(std::min(start.x, end.x), std::min(start.y, end.y));
        Vector2 max = Vector2(std::max(start.x, end.x), std::max(start.y, end.y));
        return BoundingBox(min, max);
    }

    // 선분의 경계 길이 (길이와 동일)
    Real LineSegment::GetBoundaryLength() const
    {
        return GetLength();
    }

    // 선분의 중심점 (무게중심과 동일)
    Vector2 LineSegment::GetCentroid() const
    {
        return GetCenter();
    }

    // 선분의 관성 모멘트
    Real LineSegment::GetMomentOfInertia() const
    {
        Real length = GetLength();
        return length * length / Real(12);
    }

    // 선분의 볼록성 (항상 볼록)
    bool LineSegment::IsConvex() const
    {
        return true;
    }

    // 선분의 정규성 (항상 정규)
    bool LineSegment::IsRegular() const
    {
        return true;
    }

    // 선분의 대칭성 (항상 대칭)
    bool LineSegment::IsSymmetric() const
    {
        return true;
    }

    // 선분의 꼭지점들
    std::vector<Vector2> LineSegment::GetVertices() const
    {
        return {start, end};
    }

    // 선분의 모서리들
    std::vector<std::pair<Vector2, Vector2>> LineSegment::GetEdges() const
    {
        return {{start, end}};
    }

    // 선분의 시작점
    Vector2 LineSegment::GetStart() const
    {
        return start;
    }

    // 선분의 끝점
    Vector2 LineSegment::GetEnd() const
    {
        return end;
    }

    // 선분의 시작점 설정
    void LineSegment::SetStart(const Vector2& newStart)
    {
        start = newStart;
    }

    // 선분의 끝점 설정
    void LineSegment::SetEnd(const Vector2& newEnd)
    {
        end = newEnd;
    }

    // 선분의 길이 제곱
    Real LineSegment::GetLengthSquared() const
    {
        return (end - start).LengthSquared();
    }

    // 선분의 방향 각도
    Real LineSegment::GetAngle() const
    {
        return std::atan2(end.y - start.y, end.x - start.x);
    }

    // 선분의 방향 각도 (도)
    Real LineSegment::GetAngleDegrees() const
    {
        return GetAngle() * 180.0f / PI;
    }

    // 선분이 수평인지 확인
    bool LineSegment::IsHorizontal() const
    {
        return std::abs(end.y - start.y) < 1e-6;
    }

    // 선분이 수직인지 확인
    bool LineSegment::IsVertical() const
    {
        return std::abs(end.x - start.x) < 1e-6;
    }

    // 선분이 대각선인지 확인
    bool LineSegment::IsDiagonal() const
    {
        return !IsHorizontal() && !IsVertical();
    }

    // 선분의 기울기
    Real LineSegment::GetSlope() const
    {
        if (std::abs(end.x - start.x) < 1e-6) {
            return std::numeric_limits<Real>::infinity();
        }
        return (end.y - start.y) / (end.x - start.x);
    }

    // 선분의 y절편
    Real LineSegment::GetYIntercept() const
    {
        if (std::abs(end.x - start.x) < 1e-6) {
            return std::numeric_limits<Real>::infinity();
        }
        Real slope = GetSlope();
        return start.y - slope * start.x;
    }

    // 선분의 x절편
    Real LineSegment::GetXIntercept() const
    {
        if (std::abs(end.y - start.y) < 1e-6) {
            return std::numeric_limits<Real>::infinity();
        }
        Real slope = GetSlope();
        return -start.y / slope + start.x;
    }

    // 선분의 중점
    Vector2 LineSegment::GetMidpoint() const
    {
        return GetCenter();
    }

    // 선분의 중점 설정
    void LineSegment::SetMidpoint(const Vector2& midpoint)
    {
        Vector2 currentMidpoint = GetMidpoint();
        Vector2 offset = midpoint - currentMidpoint;
        Translate(offset);
    }

    // 선분의 길이 설정
    void LineSegment::SetLength(Real newLength)
    {
        Vector2 direction = GetDirection();
        Vector2 center = GetCenter();
        Real halfLength = newLength * 0.5f;

        start = center - direction * halfLength;
        end = center + direction * halfLength;
    }

    // 선분의 방향 설정
    void LineSegment::SetDirection(const Vector2& direction)
    {
        Vector2 center = GetCenter();
        Real halfLength = GetLength() * 0.5f;
        Vector2 normalizedDir = direction.Normalized();

        start = center - normalizedDir * halfLength;
        end = center + normalizedDir * halfLength;
    }

    // 선분의 방향 각도 설정
    void LineSegment::SetAngle(Real angle)
    {
        Vector2 direction(std::cos(angle), std::sin(angle));
        SetDirection(direction);
    }

    // 선분의 방향 각도 설정 (도)
    void LineSegment::SetAngleDegrees(Real angleDegrees)
    {
        SetAngle(angleDegrees * PI / 180.0f);
    }

    // 선분의 문자열 표현
    std::string LineSegment::ToString() const
    {
        return "LineSegment(" + start.ToString() + ", " + end.ToString() + ")";
    }

    // 선분의 해시 값
    size_t LineSegment::GetHash() const
    {
        size_t hash = 0;
        hash ^= std::hash<Real>{}(start.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<Real>{}(start.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<Real>{}(end.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<Real>{}(end.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
}
