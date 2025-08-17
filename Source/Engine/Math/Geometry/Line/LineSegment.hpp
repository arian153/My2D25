#pragma once

#include <cmath>
#include <string>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../../Structure/Geometry.hpp"

namespace Engine2D
{
    // ========================================
    // 선분 클래스
    // ========================================

    class LineSegment
    {
    public:
        Vector2 start;   // 시작점
        Vector2 end;     // 끝점

        // 생성자
        LineSegment();
        LineSegment(const Vector2& s, const Vector2& e);
        LineSegment(Real x1, Real y1, Real x2, Real y2);

        // ========================================
        // Static 함수들 (핵심 알고리즘)
        // ========================================

        // 방향 벡터 계산
        static Vector2 GetDirection(const Vector2& start, const Vector2& end);

        // 길이 계산
        static Real GetLength(const Vector2& start, const Vector2& end);

        // 중심점 계산
        static Vector2 GetCenter(const Vector2& start, const Vector2& end);

        // 선분 이동 (Static)
        static LineSegment Translate(const LineSegment& segment, const Vector2& offset);

        // 선분 회전 (Static)
        static LineSegment Rotate(const LineSegment& segment, const Vector2& center, Real angle);

        // 선분 스케일 (Static)
        static LineSegment Scale(const LineSegment& segment, const Vector2& center, Real scale);

        // 두 선분의 교점 계산
        static bool GetIntersection(const LineSegment& a, const LineSegment& b, Vector2& intersection);

        // ========================================
        // 멤버 함수들
        // ========================================

        // 방향 벡터
        Vector2 GetDirection() const;

        // 길이
        Real GetLength() const;

        // 중심점
        Vector2 GetCenter() const;

        // 선분 이동
        void Translate(const Vector2& offset);

        // 선분 회전
        void Rotate(const Vector2& center, Real angle);

        // 선분 스케일
        void Scale(const Vector2& center, Real scale);

        // 두 선분의 교점 계산
        bool GetIntersection(const LineSegment& other, Vector2& intersection) const;

        // 점이 선분 위에 있는지 확인
        bool ContainsPoint(const Vector2& point) const;

        // 선분과 점 사이의 거리
        Real GetDistanceToPoint(const Vector2& point) const;

        // 선분에서 가장 가까운 점
        Vector2 GetClosestPoint(const Vector2& point) const;

        // 선분의 법선 벡터
        Vector2 GetNormal() const;

        // 선분의 접선 벡터
        Vector2 GetTangent() const;

        // 선분의 바운딩 박스
        BoundingBox GetBoundingBox() const;

        // 선분의 경계 길이 (길이와 동일)
        Real GetBoundaryLength() const;

        // 선분의 중심점 (무게중심과 동일)
        Vector2 GetCentroid() const;

        // 선분의 관성 모멘트
        Real GetMomentOfInertia() const;

        // 선분의 볼록성 (항상 볼록)
        bool IsConvex() const;

        // 선분의 정규성 (항상 정규)
        bool IsRegular() const;

        // 선분의 대칭성 (항상 대칭)
        bool IsSymmetric() const;

        // 선분의 꼭지점들
        std::vector<Vector2> GetVertices() const;

        // 선분의 모서리들
        std::vector<std::pair<Vector2, Vector2>> GetEdges() const;

        // 선분의 시작점
        Vector2 GetStart() const;

        // 선분의 끝점
        Vector2 GetEnd() const;

        // 선분의 시작점 설정
        void SetStart(const Vector2& newStart);

        // 선분의 끝점 설정
        void SetEnd(const Vector2& newEnd);

        // 선분의 길이 제곱
        Real GetLengthSquared() const;

        // 선분의 방향 각도
        Real GetAngle() const;

        // 선분의 방향 각도 (도)
        Real GetAngleDegrees() const;

        // 선분이 수평인지 확인
        bool IsHorizontal() const;

        // 선분이 수직인지 확인
        bool IsVertical() const;

        // 선분이 대각선인지 확인
        bool IsDiagonal() const;

        // 선분의 기울기
        Real GetSlope() const;

        // 선분의 y절편
        Real GetYIntercept() const;

        // 선분의 x절편
        Real GetXIntercept() const;

        // 선분의 중점
        Vector2 GetMidpoint() const;

        // 선분의 중점 설정
        void SetMidpoint(const Vector2& midpoint);

        // 선분의 길이 설정
        void SetLength(Real newLength);

        // 선분의 방향 설정
        void SetDirection(const Vector2& direction);

        // 선분의 방향 각도 설정
        void SetAngle(Real angle);

        // 선분의 방향 각도 설정 (도)
        void SetAngleDegrees(Real angleDegrees);

        // 선분의 문자열 표현
        std::string ToString() const;

        // 선분의 해시 값
        size_t GetHash() const;
    };
}
