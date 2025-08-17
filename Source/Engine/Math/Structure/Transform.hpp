#pragma once

#include <cmath>
#include <vector>
#include "../Utility/MathConstants.hpp"
#include "../Algebra/Vector2.hpp"
#include "../Algebra/Matrix22.hpp"
#include "../Algebra/Matrix33.hpp"
#include "Orientation.hpp"

namespace Engine2D
{
    // ========================================
    // 변환 클래스
    // ========================================

    class Transform
    {
    public:
        Vector2 position;    // 위치
        Orientation rotation; // 회전
        Vector2 scale;       // 스케일

        // 생성자
        Transform() : position(0, 0), rotation(), scale(1, 1) {}
        Transform(const Vector2& pos) : position(pos), rotation(), scale(1, 1) {}
        Transform(const Vector2& pos, Real angle) : position(pos), rotation(angle), scale(1, 1) {}
        Transform(const Vector2& pos, const Orientation& rot) : position(pos), rotation(rot), scale(1, 1) {}
        Transform(const Vector2& pos, const Orientation& rot, const Vector2& scl) : position(pos), rotation(rot), scale(scl) {}

        // ========================================
        // Static 변환 함수들
        // ========================================

        // 점 이동
        static Vector2 Translate(const Vector2& point, const Vector2& offset)
        {
            return point + offset;
        }

        // 점 회전 (중심점 기준, 각도 사용)
        static Vector2 Rotate(const Vector2& point, const Vector2& center, Real angle)
        {
            return Rotate(point, center, Orientation(angle));
        }

        // 점 회전 (Orientation 사용)
        static Vector2 Rotate(const Vector2& point, const Vector2& center, const Orientation& orientation)
        {
            Vector2 localPoint = point - center;

            Vector2 rotated(
                localPoint.x * orientation.c - localPoint.y * orientation.s,
                localPoint.x * orientation.s + localPoint.y * orientation.c
            );

            return center + rotated;
        }

        // 점 스케일 (중심점 기준)
        static Vector2 Scale(const Vector2& point, const Vector2& center, const Vector2& scale)
        {
            Vector2 localPoint = point - center;
            return center + Vector2(localPoint.x * scale.x, localPoint.y * scale.y);
        }

        // 점 스케일 (균등 스케일)
        static Vector2 Scale(const Vector2& point, const Vector2& center, Real scale)
        {
            Vector2 localPoint = point - center;
            return center + localPoint * scale;
        }

        // 복합 변환 (이동 + 회전 + 스케일)
        static Vector2 Transform(const Vector2& point, const Vector2& center,
                               const Vector2& offset, const Orientation& rotation, const Vector2& scale)
        {
            Vector2 localPoint = point - center;

            // 스케일
            Vector2 scaled(localPoint.x * scale.x, localPoint.y * scale.y);

            // 회전 (Orientation 클래스 사용)
            Vector2 rotated(
                scaled.x * rotation.c - scaled.y * rotation.s,
                scaled.x * rotation.s + scaled.y * rotation.c
            );

            // 이동
            return center + rotated + offset;
        }

        // ========================================
        // 행렬 기반 변환 함수들
        // ========================================

        // 점 변환 (Matrix33 사용)
        static Vector2 TransformPoint(const Vector2& point, const Matrix33& transform)
        {
            // 동차 좌표로 변환 (z=1)
            Real x = transform.m00 * point.x + transform.m01 * point.y + transform.m02;
            Real y = transform.m10 * point.x + transform.m11 * point.y + transform.m12;
            Real w = transform.m20 * point.x + transform.m21 * point.y + transform.m22;

            // 동차 좌표를 2D로 변환
            if (std::abs(w) > Math::EPSILON) {
                return Vector2(x / w, y / w);
            }
            return Vector2(x, y);
        }

        // 다각형 변환 (Matrix33 사용)
        static std::vector<Vector2> TransformPolygon(const std::vector<Vector2>& polygon,
                                                    const Matrix33& transform)
        {
            std::vector<Vector2> transformed;
            transformed.reserve(polygon.size());

            for (const auto& point : polygon) {
                transformed.push_back(TransformPoint(point, transform));
            }

            return transformed;
        }

        // 점 변환 (Matrix22 사용)
        static Vector2 TransformPoint(const Vector2& point, const Matrix22& transform)
        {
            return Vector2(
                transform.m00 * point.x + transform.m01 * point.y,
                transform.m10 * point.x + transform.m11 * point.y
            );
        }

        // 다각형 변환 (Matrix22 사용)
        static std::vector<Vector2> TransformPolygon(const std::vector<Vector2>& polygon,
                                                    const Matrix22& transform)
        {
            std::vector<Vector2> transformed;
            transformed.reserve(polygon.size());

            for (const auto& point : polygon) {
                transformed.push_back(TransformPoint(point, transform));
            }

            return transformed;
        }

        // ========================================
        // 행렬 생성 함수들
        // ========================================

        // 회전 행렬 생성 (2x2)
        static Matrix22 CreateRotationMatrix(Real angle)
        {
            Real cosAngle = std::cos(angle);
            Real sinAngle = std::sin(angle);

            return Matrix22(
                cosAngle, -sinAngle,
                sinAngle, cosAngle
            );
        }

        // 회전 행렬 생성 (3x3)
        static Matrix33 CreateRotationMatrix3D(Real angle)
        {
            Real cosAngle = std::cos(angle);
            Real sinAngle = std::sin(angle);

            return Matrix33(
                cosAngle, -sinAngle, 0,
                sinAngle, cosAngle, 0,
                0, 0, 1
            );
        }

        // 스케일 행렬 생성 (균등 스케일)
        static Matrix22 CreateScaleMatrix(Real scale)
        {
            return Matrix22(
                scale, 0,
                0, scale
            );
        }

        // 스케일 행렬 생성 (비균등 스케일)
        static Matrix22 CreateScaleMatrix(const Vector2& scale)
        {
            return Matrix22(
                scale.x, 0,
                0, scale.y
            );
        }

        // 스케일 행렬 생성 (3x3)
        static Matrix33 CreateScaleMatrix3D(Real scale)
        {
            return Matrix33(
                scale, 0, 0,
                0, scale, 0,
                0, 0, 1
            );
        }

        // 스케일 행렬 생성 (3x3, 비균등)
        static Matrix33 CreateScaleMatrix3D(const Vector2& scale)
        {
            return Matrix33(
                scale.x, 0, 0,
                0, scale.y, 0,
                0, 0, 1
            );
        }

        // 이동 행렬 생성 (3x3)
        static Matrix33 CreateTranslationMatrix(const Vector2& translation)
        {
            return Matrix33(
                1, 0, translation.x,
                0, 1, translation.y,
                0, 0, 1
            );
        }

        // 변환 행렬 조합 (TRS: Translation, Rotation, Scale)
        static Matrix33 CreateTRSMatrix(const Vector2& translation, Real rotation, Real scale)
        {
            Matrix33 translationMatrix = CreateTranslationMatrix(translation);
            Matrix33 rotationMatrix = CreateRotationMatrix3D(rotation);
            Matrix33 scaleMatrix = CreateScaleMatrix3D(scale);

            return translationMatrix * rotationMatrix * scaleMatrix;
        }

        // 변환 행렬 조합 (TRS, 비균등 스케일)
        static Matrix33 CreateTRSMatrix(const Vector2& translation, Real rotation, const Vector2& scale)
        {
            Matrix33 translationMatrix = CreateTranslationMatrix(translation);
            Matrix33 rotationMatrix = CreateRotationMatrix3D(rotation);
            Matrix33 scaleMatrix = CreateScaleMatrix3D(scale);

            return translationMatrix * rotationMatrix * scaleMatrix;
        }

        // ========================================
        // 멤버 변환 함수들
        // ========================================

        // 점을 이 변환으로 변환
        Vector2 TransformPoint(const Vector2& point) const
        {
            return Transform::Transform(point, Vector2(0, 0), position, rotation, scale);
        }

        // 점을 이 변환의 역으로 변환
        Vector2 InverseTransformPoint(const Vector2& point) const
        {
            Vector2 localPoint = point - position;

            // 역회전
            Vector2 inverseRotated(
                localPoint.x * rotation.c + localPoint.y * rotation.s,
                -localPoint.x * rotation.s + localPoint.y * rotation.c
            );

            // 역스케일
            return Vector2(inverseRotated.x / scale.x, inverseRotated.y / scale.y);
        }

        // 방향 벡터를 이 변환으로 변환 (위치 변화 없음)
        Vector2 TransformDirection(const Vector2& direction) const
        {
            Vector2 rotated(
                direction.x * rotation.c - direction.y * rotation.s,
                direction.x * rotation.s + direction.y * rotation.c
            );

            return Vector2(rotated.x * scale.x, rotated.y * scale.y);
        }

        // 방향 벡터를 이 변환의 역으로 변환
        Vector2 InverseTransformDirection(const Vector2& direction) const
        {
            Vector2 inverseScaled(direction.x / scale.x, direction.y / scale.y);

            Vector2 inverseRotated(
                inverseScaled.x * rotation.c + inverseScaled.y * rotation.s,
                -inverseScaled.x * rotation.s + inverseScaled.y * rotation.c
            );

            return inverseRotated;
        }

        // ========================================
        // 변환 조작 함수들
        // ========================================

        // 이동
        void Translate(const Vector2& offset)
        {
            position += offset;
        }

        // 회전 (현재 회전에 추가)
        void Rotate(Real angle)
        {
            rotation = Orientation::Apply(rotation, Orientation(angle));
        }

        // 회전 설정
        void SetRotation(Real angle)
        {
            rotation.Set(angle);
        }

        // 스케일 (현재 스케일에 곱)
        void Scale(const Vector2& scaleFactor)
        {
            scale.x *= scaleFactor.x;
            scale.y *= scaleFactor.y;
        }

        // 스케일 설정
        void SetScale(const Vector2& newScale)
        {
            scale = newScale;
        }

        // 균등 스케일
        void Scale(Real scaleFactor)
        {
            scale.x *= scaleFactor;
            scale.y *= scaleFactor;
        }

        // ========================================
        // 변환 조합 함수들
        // ========================================

        // 두 변환을 조합
        static Transform Combine(const Transform& a, const Transform& b)
        {
            Transform result;

            // 스케일 조합
            result.scale.x = a.scale.x * b.scale.x;
            result.scale.y = a.scale.y * b.scale.y;

            // 회전 조합
            result.rotation = Orientation::Apply(a.rotation, b.rotation);

            // 위치 조합
            Vector2 bTransformed = a.TransformPoint(b.position);
            result.position = bTransformed;

            return result;
        }

        // 이 변환과 다른 변환을 조합
        Transform Combine(const Transform& other) const
        {
            return Transform::Combine(*this, other);
        }

        // ========================================
        // 유틸리티 함수들
        // ========================================

        // 항등 변환
        static Transform Identity()
        {
            return Transform();
        }

        // 이동 변환
        static Transform Translation(const Vector2& offset)
        {
            return Transform(offset);
        }

        // 회전 변환
        static Transform Rotation(Real angle)
        {
            return Transform(Vector2(0, 0), angle);
        }

        // 스케일 변환
        static Transform Scaling(const Vector2& scale)
        {
            return Transform(Vector2(0, 0), Orientation(), scale);
        }

        // 균등 스케일 변환
        static Transform Scaling(Real scale)
        {
            return Transform(Vector2(0, 0), Orientation(), Vector2(scale, scale));
        }

        // 변환의 역
        Transform Inverse() const
        {
            Transform result;

            // 역스케일
            result.scale.x = 1.0f / scale.x;
            result.scale.y = 1.0f / scale.y;

            // 역회전
            result.rotation.s = -rotation.s;
            result.rotation.c = rotation.c;
            result.rotation.a = -rotation.a;

            // 역위치
            result.position = result.InverseTransformPoint(position);

            return result;
        }

        // 변환이 항등 변환인지 확인
        bool IsIdentity() const
        {
            return position.x == 0 && position.y == 0 &&
                   rotation.s == 0 && rotation.c == 1 &&
                   scale.x == 1 && scale.y == 1;
        }
    };
}
