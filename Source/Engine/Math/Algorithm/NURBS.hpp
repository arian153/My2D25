#pragma once

#include "../Curve.hpp"
#include <cmath>
#include <vector>
#include <functional>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"

namespace Engine2D
{
    // ========================================
    // NURBS 알고리즘 클래스
    // ========================================

    class NURBS
    {
    public:
        // ========================================
        // NURBS 곡선 계산 함수들
        // ========================================

        // NURBS 곡선 위의 점 계산 (t: 0~1)
        static Vector2 Point(const Curve& curve, Real t);

        // NURBS 곡선의 1차 도함수 (접선 벡터)
        static Vector2 Derivative(const Curve& curve, Real t);

        // NURBS 곡선의 2차 도함수
        static Vector2 SecondDerivative(const Curve& curve, Real t);

        // NURBS 곡선의 곡률
        static Real Curvature(const Curve& curve, Real t);

        // NURBS 곡선의 곡률 콤 (Curvature Comb)
        static Vector2 CurvatureComb(const Curve& curve, Real t);

        // NURBS 곡선의 접선 벡터 (정규화된)
        static Vector2 Tangent(const Curve& curve, Real t);

        // NURBS 곡선의 법선 벡터 (정규화된)
        static Vector2 Normal(const Curve& curve, Real t);

        // NURBS 곡선의 종법선 벡터 (2D에서는 항상 (0,0,1))
        static Vector2 Binormal(const Curve& curve, Real t);

        // ========================================
        // 곡선 길이 관련 함수들
        // ========================================

        // NURBS 곡선의 전체 길이
        static Real ArcLength(const Curve& curve);

        // NURBS 곡선의 매개변수 t에서의 곡선 길이
        static Real ArcLength(const Curve& curve, Real t);

        // NURBS 곡선의 길이 s에서의 매개변수 t (역함수)
        static Real InverseArcLength(const Curve& curve, Real s);

        // ========================================
        // 충돌 검출을 위한 함수들
        // ========================================

        // 점에서 NURBS 곡선까지의 최단 거리
        static Real DistanceToPoint(const Curve& curve, const Vector2& point);

        // NURBS 곡선 위의 최단 거리 점
        static Vector2 ClosestPoint(const Curve& curve, const Vector2& point);

        // NURBS 곡선 위의 최단 거리 매개변수
        static Real ClosestParameter(const Curve& curve, const Vector2& point);

        // ========================================
        // NURBS 특화 함수들
        // ========================================

        // NURBS 곡선 분할
        static std::pair<Curve, Curve> Split(const Curve& curve, Real t);

        // NURBS 곡선 승적 (degree elevation)
        static Curve ElevateDegree(const Curve& curve);

        // NURBS 곡선 변환
        static Curve Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale);

        // Knot 삽입
        static Curve InsertKnot(const Curve& curve, Real u, int multiplicity = 1);

        // Knot 제거
        static Curve RemoveKnot(const Curve& curve, Real u, int multiplicity = 1);

        // Knot 정규화
        static Curve NormalizeKnots(const Curve& curve);

        // ========================================
        // NURBS 곡선 생성 함수들
        // ========================================

        // NURBS 곡선 생성 (4개 제어점)
        static Curve CreateNURBSCurve(const Vector2& p0, const Vector2& p1,
                                     const Vector2& p2, const Vector2& p3);

        // NURBS 곡선 생성 (제어점 벡터)
        static Curve CreateNURBSCurve(const std::vector<Vector2>& controlPoints);

        // ========================================
        // NURBS 곡선 계산 함수들 (CurveAlgorithms에서 통합)
        // ========================================

        // NURBS 곡선의 점 계산
        static Vector2 CalculateNURBSPoint(const std::vector<Vector2>& controlPoints,
                                          const std::vector<Real>& weights, Real t);

        // NURBS 곡선 생성
        static std::vector<Vector2> GenerateNURBSCurve(const std::vector<Vector2>& controlPoints,
                                                      const std::vector<Real>& weights, int segments);

        // 균등 매듭 벡터 생성
        static std::vector<Real> GenerateUniformKnots(int numControlPoints, int degree);

        // B-스플라인 기저 함수 계산
        static Real CalculateBSplineBasis(int i, int k, const std::vector<Real>& knots, Real t);

        // B-스플라인 기저 함수의 도함수
        static Real CalculateBSplineBasisDerivative(int i, int k, const std::vector<Real>& knots, Real t);

        // B-스플라인 기저 함수의 2차 도함수
        static Real CalculateBSplineBasisSecondDerivative(int i, int k, const std::vector<Real>& knots, Real t);

        // NURBS 곡선 생성 (제어점과 가중치)
        static Curve CreateNURBSCurve(const std::vector<Vector2>& controlPoints,
                                     const std::vector<Real>& weights);

        // NURBS 곡선 생성 (제어점, 가중치, knot 벡터)
        static Curve CreateNURBSCurve(const std::vector<Vector2>& controlPoints,
                                     const std::vector<Real>& weights,
                                     const std::vector<Real>& knots);

        // ========================================
        // NURBS 곡선 분석 함수들
        // ========================================

        // NURBS 곡선의 부드러움 확인
        static bool IsSmooth(const Curve& curve);

        // NURBS 곡선의 자기 교차 확인
        static bool HasSelfIntersection(const Curve& curve);

        // NURBS 곡선의 최적화
        static Curve Optimize(const Curve& curve, Real tolerance);

        // NURBS 곡선 연결
        static Curve Connect(const Curve& curve1, const Curve& curve2);

        // ========================================
        // NURBS 유틸리티 함수들
        // ========================================

        // 표준 knot 벡터 생성
        static std::vector<Real> CreateStandardKnotVector(int controlPointCount, int degree);

        // 균등 가중치 벡터 생성
        static std::vector<Real> CreateUniformWeights(int controlPointCount);

        // Knot 벡터 유효성 검사
        static bool IsValidKnotVector(const std::vector<Real>& knots, int controlPointCount, int degree);

    private:
        // ========================================
        // 내부 구현 함수들
        // ========================================

        // B-스플라인 기저 함수 계산
        static Real BSplineBasis(int i, int k, Real u, const std::vector<Real>& knots);

        // B-스플라인 기저 함수의 도함수
        static Real BSplineBasisDerivative(int i, int k, Real u, const std::vector<Real>& knots);

        // B-스플라인 기저 함수의 2차 도함수
        static Real BSplineBasisSecondDerivative(int i, int k, Real u, const std::vector<Real>& knots);

        // Knot 벡터 자동 생성
        static std::vector<Real> GenerateKnots(int controlPointCount, int degree);

        // 가중치 자동 생성
        static std::vector<Real> GenerateWeights(int controlPointCount);

        // 매개변수 범위 확인
        static bool IsValidParameter(Real u, const std::vector<Real>& knots);

        // 호 길이 계산 (수치적 적분)
        static Real CalculateArcLength(const Curve& curve, Real t);

        // 이분 탐색으로 매개변수 찾기
        static Real FindParameterByArcLength(const Curve& curve, Real targetLength);

        // 최단 거리 매개변수 찾기 (뉴턴법)
        static Real FindClosestParameterNewton(const Curve& curve, const Vector2& point, Real initialGuess = 0.5);

        // Knot 벡터 정규화
        static std::vector<Real> NormalizeKnotVector(const std::vector<Real>& knots);

        // 가중치 정규화
        static std::vector<Real> NormalizeWeights(const std::vector<Real>& weights);

        // 매개변수 범위 제한 (0~1)
        static Real ClampParameter(Real t);

        // 벡터 정규화
        static Vector2 Normalize(const Vector2& v);

        // 두 벡터의 외적 (2D에서는 스칼라)
        static Real CrossProduct(const Vector2& a, const Vector2& b);

        // 두 벡터의 내적
        static Real DotProduct(const Vector2& a, const Vector2& b);

        // 벡터의 길이
        static Real Length(const Vector2& v);

        // 벡터의 길이 제곱
        static Real LengthSquared(const Vector2& v);
    };
}
