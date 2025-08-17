#pragma once

#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include <vector>

namespace Engine2D
{
    // ========================================
    // 곡선 알고리즘 타입
    // ========================================

    enum class CurveAlgorithm
    {
        BEZIER,         // 베지어 곡선
        CATMULL_ROM,    // Catmull-Rom 곡선
        NURBS,          // NURBS 곡선
        HERMITE         // 에르미트 곡선
    };

    // ========================================
    // 곡선 데이터 구조
    // ========================================

    struct Curve
    {
        std::vector<Vector2> controlPoints;  // 제어점들
        std::vector<Real> weights;           // 가중치들 (NURBS용)
        std::vector<Real> knots;             // Knot 벡터 (NURBS용)
        int degree;                          // 곡선의 차수
        bool isClosed;                       // 닫힌 곡선인지 여부

        // 기본 생성자
        Curve();

        // 제어점으로 초기화
        Curve(const std::vector<Vector2>& points);

        // 제어점과 가중치로 초기화
        Curve(const std::vector<Vector2>& points, const std::vector<Real>& w);

        // 제어점, 가중치, knot 벡터로 초기화
        Curve(const std::vector<Vector2>& points, const std::vector<Real>& w,
              const std::vector<Real>& k);

        // 복사 생성자
        Curve(const Curve& other);

        // 대입 연산자
        Curve& operator=(const Curve& other);

        // 소멸자
        ~Curve();

        // ========================================
        // 기본 속성 함수들
        // ========================================

        // 제어점 개수
        int GetControlPointCount() const;

        // 곡선의 차수
        int GetDegree() const;

        // 곡선이 닫혀있는지 확인
        bool IsClosed() const;

        // 곡선이 평면인지 확인
        bool IsPlanar() const;

        // ========================================
        // 기하학적 속성들
        // ========================================

        // 곡선의 경계 상자
        std::pair<Vector2, Vector2> GetBoundingBox() const;

        // 곡선의 중심점
        Vector2 GetCenter() const;

        // 곡선의 시작점
        Vector2 GetStartPoint() const;

        // 곡선의 끝점
        Vector2 GetEndPoint() const;

        // ========================================
        // 편집 함수들
        // ========================================

        // 제어점 추가
        void AddControlPoint(const Vector2& point);

        // 제어점 제거
        void RemoveControlPoint(int index);

        // 제어점 수정
        void SetControlPoint(int index, const Vector2& point);

        // 제어점 가져오기
        Vector2 GetControlPoint(int index) const;

        // 모든 제어점 가져오기
        const std::vector<Vector2>& GetControlPoints() const;

        // 곡선 닫기/열기
        void SetClosed(bool closed);

        // 곡선 차수 설정
        void SetDegree(int degree);

        // ========================================
        // 변환 함수들
        // ========================================

        // 곡선 변환
        void Transform(const Vector2& translation, Real rotation, Real scale);

        // 곡선 반전
        void Reverse();

        // 곡선 정규화
        void Normalize();
    };

    // ========================================
    // 곡선 유틸리티 함수들
    // ========================================

    namespace CurveUtils
    {
        // 곡선 복사
        Curve CopyCurve(const Curve& curve);

        // 곡선 비교
        bool AreEqual(const Curve& curve1, const Curve& curve2, Real tolerance = Math::EPSILON);

        // 곡선의 복잡도 계산
        Real CalculateComplexity(const Curve& curve);

        // 곡선의 부드러움 확인
        bool IsSmooth(const Curve& curve);

        // 곡선의 자기 교차 확인
        bool HasSelfIntersection(const Curve& curve);

        // 곡선의 곡률 변화 확인
        Real GetMaxCurvature(const Curve& curve);
        Real GetMinCurvature(const Curve& curve);

        // 두 곡선의 연속성 확인
        bool CheckG0Continuity(const Curve& curve1, const Curve& curve2);
        bool CheckG1Continuity(const Curve& curve1, const Curve& curve2);
        bool CheckG2Continuity(const Curve& curve1, const Curve& curve2);
    }
}
