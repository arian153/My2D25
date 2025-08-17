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
    // 베지어 알고리즘 클래스
    // ========================================

    class Bezier
    {
    public:
        // ========================================
        // 베지어 곡선 계산 함수들
        // ========================================

        // 베지어 곡선 위의 점 계산 (t: 0~1)
        static Vector2 Point(const Curve& curve, Real t);

        // 베지어 곡선의 1차 도함수 (접선 벡터)
        static Vector2 Derivative(const Curve& curve, Real t);

        // 베지어 곡선의 2차 도함수
        static Vector2 SecondDerivative(const Curve& curve, Real t);

        // 베지어 곡선의 곡률
        static Real Curvature(const Curve& curve, Real t);

        // 베지어 곡선의 곡률 콤 (Curvature Comb)
        static Vector2 CurvatureComb(const Curve& curve, Real t);

        // 베지어 곡선의 접선 벡터 (정규화된)
        static Vector2 Tangent(const Curve& curve, Real t);

        // 베지어 곡선의 법선 벡터 (정규화된)
        static Vector2 Normal(const Curve& curve, Real t);

        // 베지어 곡선의 종법선 벡터 (2D에서는 항상 (0,0,1))
        static Vector2 Binormal(const Curve& curve, Real t);

        // ========================================
        // 곡선 길이 관련 함수들
        // ========================================

        // 베지어 곡선의 전체 길이
        static Real ArcLength(const Curve& curve);

        // 베지어 곡선의 매개변수 t에서의 곡선 길이
        static Real ArcLength(const Curve& curve, Real t);

        // 베지어 곡선의 길이 s에서의 매개변수 t (역함수)
        static Real InverseArcLength(const Curve& curve, Real s);

        // ========================================
        // 충돌 검출을 위한 함수들
        // ========================================

        // 점에서 베지어 곡선까지의 최단 거리
        static Real DistanceToPoint(const Curve& curve, const Vector2& point);

        // 베지어 곡선 위의 최단 거리 점
        static Vector2 ClosestPoint(const Curve& curve, const Vector2& point);

        // 베지어 곡선 위의 최단 거리 매개변수
        static Real ClosestParameter(const Curve& curve, const Vector2& point);

        // ========================================
        // 베지어 곡선 특화 함수들
        // ========================================

        // 베지어 곡선 분할 (de Casteljau 알고리즘)
        static std::pair<Curve, Curve> Split(const Curve& curve, Real t);

        // 베지어 곡선 승적 (degree elevation)
        static Curve ElevateDegree(const Curve& curve);

        // 베지어 곡선의 볼록 껍질 (Convex Hull)
        static std::vector<Vector2> GetConvexHull(const Curve& curve);

        // 베지어 곡선의 제어 폴리곤
        static std::vector<Vector2> GetControlPolygon(const Curve& curve);

        // 베지어 곡선의 변환
        static Curve Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale);

        // ========================================
        // 베지어 곡선 생성 함수들
        // ========================================

        // 베지어 곡선 생성 (4개 제어점)
        static Curve CreateBezierCurve(const Vector2& p0, const Vector2& p1,
                                      const Vector2& p2, const Vector2& p3);

        // 베지어 곡선 생성 (제어점 벡터)
        static Curve CreateBezierCurve(const std::vector<Vector2>& controlPoints);

        // 베지어 곡선 생성 (차수와 제어점)
        static Curve CreateBezierCurve(int degree, const std::vector<Vector2>& controlPoints);

        // ========================================
        // 베지어 곡선 계산 함수들 (CurveAlgorithms에서 통합)
        // ========================================

        // 베지어 곡선의 점 계산 (de Casteljau 알고리즘)
        static Vector2 CalculateBezierPoint(const std::vector<Vector2>& controlPoints, Real t);

        // 베지어 곡선 생성 (여러 점으로)
        static std::vector<Vector2> GenerateBezierCurve(const std::vector<Vector2>& controlPoints, int segments);

        // 베지어 곡선의 미분 (접선 벡터)
        static Vector2 CalculateBezierDerivative(const std::vector<Vector2>& controlPoints, Real t);

        // 베지어 곡선의 곡률
        static Real CalculateBezierCurvature(const std::vector<Vector2>& controlPoints, Real t);

        // 베지어 곡선의 제어점에서 미분 제어점 계산
        static std::vector<Vector2> GetDerivativePoints(const std::vector<Vector2>& controlPoints);

        // ========================================
        // 베지어 곡선 분석 함수들
        // ========================================

        // 베지어 곡선의 부드러움 확인
        static bool IsSmooth(const Curve& curve);

        // 베지어 곡선의 자기 교차 확인
        static bool HasSelfIntersection(const Curve& curve);

        // 베지어 곡선의 최적화 (제어점 수 줄이기)
        static Curve Optimize(const Curve& curve, Real tolerance);

        // 베지어 곡선의 복잡도 계산
        static Real CalculateComplexity(const Curve& curve);

    private:
        // ========================================
        // 내부 구현 함수들
        // ========================================

        // 베지어 기저 함수 계산
        static Real BernsteinBasis(int i, int n, Real t);

        // 베지어 기저 함수의 도함수
        static Real BernsteinBasisDerivative(int i, int n, Real t);

        // 베지어 기저 함수의 2차 도함수
        static Real BernsteinBasisSecondDerivative(int i, int n, Real t);

        // de Casteljau 알고리즘
        static Vector2 DeCasteljau(const Curve& curve, Real t);

        // 호 길이 계산 (수치적 적분)
        static Real CalculateArcLength(const Curve& curve, Real t);

        // 이분 탐색으로 매개변수 찾기
        static Real FindParameterByArcLength(const Curve& curve, Real targetLength);

        // 최단 거리 매개변수 찾기 (뉴턴법)
        static Real FindClosestParameterNewton(const Curve& curve, const Vector2& point, Real initialGuess = 0.5);

        // 볼록 껍질 계산 (Graham scan)
        static std::vector<Vector2> CalculateConvexHull(const std::vector<Vector2>& points);

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
