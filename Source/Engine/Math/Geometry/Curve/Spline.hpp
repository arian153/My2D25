#pragma once

#include "Curve.hpp"
#include "Algorithm/Bezier.hpp"
#include "Algorithm/CatmullRom.hpp"
#include "Algorithm/NURBS.hpp"
#include "Algorithm/Hermite.hpp"
#include <vector>

namespace Engine2D
{
    // ========================================
    // 연속성 타입 열거형
    // ========================================

    enum class ContinuityType
    {
        G0,  // 위치 연속성 (Position continuity)
        G1,  // 접선 연속성 (Tangent continuity)
        G2   // 곡률 연속성 (Curvature continuity)
    };

    // ========================================
    // 스플라인 클래스 (곡선들의 컨테이너)
    // ========================================

    class Spline
    {
    public:
        // 생성자 (알고리즘 지정)
        Spline(CurveAlgorithm algorithm = CurveAlgorithm::BEZIER);
        virtual ~Spline();

        // ========================================
        // 알고리즘 관리
        // ========================================

        // 알고리즘 설정
        void SetAlgorithm(CurveAlgorithm algorithm);

        // 현재 알고리즘 반환
        CurveAlgorithm GetAlgorithm() const;

        // ========================================
        // 곡선 관리 함수들
        // ========================================

        // 곡선 추가
        void AddCurve(Curve* curve);

        // 곡선 제거
        void RemoveCurve(int index);

        // 곡선 가져오기
        Curve* GetCurve(int index) const;

        // 곡선 개수
        int GetCurveCount() const;

        // 모든 곡선 가져오기
        const std::vector<Curve*>& GetCurves() const;

        // ========================================
        // 스플라인 속성들
        // ========================================

        // 스플라인이 닫혀있는지 확인
        bool IsClosed() const;

        // 스플라인을 닫기/열기
        void SetClosed(bool closed);

        // 스플라인의 전체 길이
        Real GetTotalLength() const;

        // 스플라인의 경계 상자
        std::pair<Vector2, Vector2> GetBoundingBox() const;

        // ========================================
        // 연속성 관리
        // ========================================

        // 연속성 타입 설정
        void SetContinuity(ContinuityType continuity);

        // 현재 연속성 타입 반환
        ContinuityType GetContinuity() const;

        // 연속성 검증
        bool ValidateContinuity() const;

        // 연속성 강제 적용
        void EnforceContinuity();

        // ========================================
        // 곡선 평가 함수들
        // ========================================

        // 스플라인 위의 점 계산 (전체 매개변수 s: 0~totalLength)
        Vector2 Point(Real s) const;

        // 스플라인의 접선 벡터
        Vector2 Tangent(Real s) const;

        // 스플라인의 법선 벡터
        Vector2 Normal(Real s) const;

        // 스플라인의 곡률
        Real Curvature(Real s) const;

        // 스플라인의 곡률 콤
        Vector2 CurvatureComb(Real s) const;

        // ========================================
        // 매개변수 변환 함수들
        // ========================================

        // 전체 매개변수 s를 개별 곡선의 매개변수 (curveIndex, t)로 변환
        std::pair<int, Real> GlobalToLocalParameter(Real s) const;

        // 개별 곡선의 매개변수 (curveIndex, t)를 전체 매개변수 s로 변환
        Real LocalToGlobalParameter(int curveIndex, Real t) const;

        // ========================================
        // 충돌 검출 함수들
        // ========================================

        // 점에서 스플라인까지의 최단 거리
        Real DistanceToPoint(const Vector2& point) const;

        // 스플라인 위의 최단 거리 점
        Vector2 ClosestPoint(const Vector2& point) const;

        // 스플라인 위의 최단 거리 매개변수
        Real ClosestParameter(const Vector2& point) const;

        // ========================================
        // 스플라인 조작 함수들
        // ========================================

        // 스플라인 분할
        std::pair<Spline*, Spline*> Split(Real s) const;

        // 스플라인 연결
        void Connect(const Spline* other);

        // 스플라인 변환
        void Transform(const Vector2& translation, Real rotation, Real scale);

        // 스플라인 반전
        void Reverse();

        // ========================================
        // 스플라인 최적화
        // ========================================

        // 불필요한 곡선 제거 (단순화)
        void Simplify(Real tolerance);

        // 곡선 개수 최적화
        void Optimize(int targetCurveCount);

        // 제어점 수 최적화
        void OptimizeControlPoints(Real tolerance);

        // ========================================
        // 정적 팩토리 함수들
        // ========================================

        // 베지어 곡선들로 스플라인 생성
        static Spline* CreateBezierSpline(const std::vector<std::vector<Vector2>>& controlPointsList);

        // Catmull-Rom 곡선들로 스플라인 생성
        static Spline* CreateCatmullRomSpline(const std::vector<Vector2>& controlPoints);

        // NURBS 곡선들로 스플라인 생성
        static Spline* CreateNURBSSpline(const std::vector<Vector2>& controlPoints,
                                       const std::vector<Real>& knots = {});

        // 복사 생성자
        Spline(const Spline& other);

        // 대입 연산자
        Spline& operator=(const Spline& other);

    private:
        std::vector<Curve*> curves;
        CurveAlgorithm algorithm;  // 스플라인 전체의 알고리즘
        bool isClosed;
        ContinuityType continuity;
        mutable std::vector<Real> curveLengths;
        mutable bool lengthsCalculated;

        // ========================================
        // 내부 구현 함수들
        // ========================================

        // 곡선 길이들 계산
        void CalculateCurveLengths() const;

        // 연속성 강제 적용 (G0)
        void EnforceG0Continuity();

        // 연속성 강제 적용 (G1)
        void EnforceG1Continuity();

        // 연속성 강제 적용 (G2)
        void EnforceG2Continuity();

        // 곡선 간 연속성 확인
        bool CheckCurveContinuity(int index1, int index2, ContinuityType type) const;

        // 곡선 간 연속성 강제 적용
        void EnforceCurveContinuity(int index1, int index2, ContinuityType type);

        // 매개변수 범위 확인
        bool IsValidParameter(Real s) const;

        // 곡선 인덱스 범위 확인
        bool IsValidCurveIndex(int index) const;

        // 알고리즘에 따른 점 계산
        Vector2 CalculatePoint(const Curve* curve, Real t) const;

        // 알고리즘에 따른 접선 계산
        Vector2 CalculateTangent(const Curve* curve, Real t) const;

        // 알고리즘에 따른 법선 계산
        Vector2 CalculateNormal(const Curve* curve, Real t) const;

        // 알고리즘에 따른 곡률 계산
        Real CalculateCurvature(const Curve* curve, Real t) const;
    };

    // ========================================
    // 스플라인 유틸리티 함수들
    // ========================================

    namespace SplineUtils
    {
        // 스플라인 생성 (곡선 벡터로부터)
        Spline* CreateSpline(const std::vector<Curve*>& curves, CurveAlgorithm algorithm = CurveAlgorithm::BEZIER);

        // 스플라인의 부드러움 확인
        bool IsSmooth(const Spline* spline);

        // 스플라인의 자기 교차 확인
        bool HasSelfIntersection(const Spline* spline);

        // 스플라인의 복잡도 계산
        Real CalculateComplexity(const Spline* spline);

        // 스플라인 메모리 해제
        void DeleteSpline(Spline* spline);

        // 두 스플라인의 연속성 확인
        bool CheckSplineContinuity(const Spline* spline1, const Spline* spline2, ContinuityType type);

        // 스플라인 연결 (새로운 스플라인 생성)
        Spline* ConnectSplines(const Spline* spline1, const Spline* spline2);
    }
}
