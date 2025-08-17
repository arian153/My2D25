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
    // Catmull-Rom 알고리즘 클래스
    // ========================================

    class CatmullRom
    {
    public:
        // ========================================
        // Catmull-Rom 곡선 계산 함수들
        // ========================================

        // Catmull-Rom 곡선 위의 점 계산 (t: 0~1)
        static Vector2 Point(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 1차 도함수 (접선 벡터)
        static Vector2 Derivative(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 2차 도함수
        static Vector2 SecondDerivative(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 곡률
        static Real Curvature(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 곡률 콤 (Curvature Comb)
        static Vector2 CurvatureComb(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 접선 벡터 (정규화된)
        static Vector2 Tangent(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 법선 벡터 (정규화된)
        static Vector2 Normal(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 종법선 벡터 (2D에서는 항상 (0,0,1))
        static Vector2 Binormal(const Curve& curve, Real t);

        // ========================================
        // 곡선 길이 관련 함수들
        // ========================================

        // Catmull-Rom 곡선의 전체 길이
        static Real ArcLength(const Curve& curve);

        // Catmull-Rom 곡선의 매개변수 t에서의 곡선 길이
        static Real ArcLength(const Curve& curve, Real t);

        // Catmull-Rom 곡선의 길이 s에서의 매개변수 t (역함수)
        static Real InverseArcLength(const Curve& curve, Real s);

        // ========================================
        // 충돌 검출을 위한 함수들
        // ========================================

        // 점에서 Catmull-Rom 곡선까지의 최단 거리
        static Real DistanceToPoint(const Curve& curve, const Vector2& point);

        // Catmull-Rom 곡선 위의 최단 거리 점
        static Vector2 ClosestPoint(const Curve& curve, const Vector2& point);

        // Catmull-Rom 곡선 위의 최단 거리 매개변수
        static Real ClosestParameter(const Curve& curve, const Vector2& point);

        // ========================================
        // Catmull-Rom 특화 함수들
        // ========================================

        // 장력 매개변수 설정 (기본값: 0.5)
        static void SetTension(Real tension);

        // 현재 장력 매개변수 반환
        static Real GetTension();

        // 끝점 처리 방식 설정
        enum class EndpointType
        {
            NATURAL,    // 자연 끝점
            CLAMPED,    // 고정 끝점
            CLOSED      // 닫힌 곡선
        };

        static void SetEndpointType(EndpointType type);
        static EndpointType GetEndpointType();

        // Catmull-Rom 곡선 분할
        static std::pair<Curve, Curve> Split(const Curve& curve, Real t);

        // Catmull-Rom 곡선 변환
        static Curve Transform(const Curve& curve, const Vector2& translation, Real rotation, Real scale);

        // ========================================
        // Catmull-Rom 곡선 생성 함수들
        // ========================================

        // Catmull-Rom 곡선 생성 (4개 제어점)
        static Curve CreateCatmullRomCurve(const Vector2& p0, const Vector2& p1,
                                          const Vector2& p2, const Vector2& p3);

        // Catmull-Rom 곡선 생성 (제어점 벡터)
        static Curve CreateCatmullRomCurve(const std::vector<Vector2>& controlPoints);

        // ========================================
        // Catmull-Rom 곡선 계산 함수들 (CurveAlgorithms에서 통합)
        // ========================================

        // Catmull-Rom 스플라인 점 계산
        static Vector2 CalculateCatmullRomPoint(const std::vector<Vector2>& points, Real t);

        // Catmull-Rom 스플라인 생성
        static std::vector<Vector2> GenerateCatmullRomCurve(const std::vector<Vector2>& points, int segments);

        // Catmull-Rom 곡선 생성 (제어점과 장력)
        static Curve CreateCatmullRomCurve(const std::vector<Vector2>& controlPoints, Real tension);

        // ========================================
        // Catmull-Rom 곡선 분석 함수들
        // ========================================

        // Catmull-Rom 곡선의 부드러움 확인
        static bool IsSmooth(const Curve& curve);

        // Catmull-Rom 곡선의 자기 교차 확인
        static bool HasSelfIntersection(const Curve& curve);

        // Catmull-Rom 곡선의 최적화
        static Curve Optimize(const Curve& curve, Real tolerance);

        // Catmull-Rom 곡선 연결
        static Curve Connect(const Curve& curve1, const Curve& curve2);

    private:
        static Real tension;
        static EndpointType endpointType;

        // ========================================
        // 내부 구현 함수들
        // ========================================

        // Catmull-Rom 기저 함수 계산
        static Real CatmullRomBasis(Real t);

        // Catmull-Rom 기저 함수의 도함수
        static Real CatmullRomBasisDerivative(Real t);

        // Catmull-Rom 기저 함수의 2차 도함수
        static Real CatmullRomBasisSecondDerivative(Real t);

        // 끝점 계산
        static Vector2 CalculateEndpoint(const Curve& curve, int index);

        // 호 길이 계산 (수치적 적분)
        static Real CalculateArcLength(const Curve& curve, Real t);

        // 이분 탐색으로 매개변수 찾기
        static Real FindParameterByArcLength(const Curve& curve, Real targetLength);

        // 최단 거리 매개변수 찾기 (뉴턴법)
        static Real FindClosestParameterNewton(const Curve& curve, const Vector2& point, Real initialGuess = 0.5);

        // Catmull-Rom 행렬 계산
        static void CalculateCatmullRomMatrix(Real t, Real& h0, Real& h1, Real& h2, Real& h3);
        static void CalculateCatmullRomMatrixDerivative(Real t, Real& h0, Real& h1, Real& h2, Real& h3);
        static void CalculateCatmullRomMatrixSecondDerivative(Real t, Real& h0, Real& h1, Real& h2, Real& h3);

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
