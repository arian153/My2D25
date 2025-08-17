#pragma once

#include "../Utility/MathConstants.hpp"
#include "../Utility/MathUtility.hpp"

namespace Engine2D
{
    class Vector2;
    class Vector3;

    // row major order matrix
    class Matrix33
    {
    public:
        Real xx = 1.0, xy = 0.0, xz = 0.0;
        Real yx = 0.0, yy = 1.0, yz = 0.0;
        Real zx = 0.0, zy = 0.0, zz = 1.0;

    public:
        Matrix33() = default;
        ~Matrix33() = default;

        Matrix33(const Matrix33& rhs)
            : xx(rhs.xx), xy(rhs.xy), xz(rhs.xz)
            , yx(rhs.yx), yy(rhs.yy), yz(rhs.yz)
            , zx(rhs.zx), zy(rhs.zy), zz(rhs.zz)
        {
        }

        Matrix33(Real m00, Real m01, Real m02,
                 Real m10, Real m11, Real m12,
                 Real m20, Real m21, Real m22)
            : xx(m00), xy(m01), xz(m02)
            , yx(m10), yy(m11), yz(m12)
            , zx(m20), zy(m21), zz(m22)
        {
        }

        explicit Matrix33(Real a[9])
            : xx(a[0]), xy(a[1]), xz(a[2])
            , yx(a[3]), yy(a[4]), yz(a[5])
            , zx(a[6]), zy(a[7]), zz(a[8])
        {
        }

        void Set(Real m00, Real m01, Real m02,
                 Real m10, Real m11, Real m12,
                 Real m20, Real m21, Real m22)
        {
            xx = m00; xy = m01; xz = m02;
            yx = m10; yy = m11; yz = m12;
            zx = m20; zy = m21; zz = m22;
        }

        void SetZero()
        {
            xx = xy = xz = 0.0;
            yx = yy = yz = 0.0;
            zx = zy = zz = 0.0;
        }

        void SetIdentity()
        {
            xx = 1.0; xy = 0.0; xz = 0.0;
            yx = 0.0; yy = 1.0; yz = 0.0;
            zx = 0.0; zy = 0.0; zz = 1.0;
        }

        void SetDiagonal(Real a, Real b, Real c)
        {
            xx = a; xy = 0.0; xz = 0.0;
            yx = 0.0; yy = b; yz = 0.0;
            zx = 0.0; zy = 0.0; zz = c;
        }

        Real operator[](size_t i) const
        {
            return (&xx)[i];
        }

        Real& operator[](size_t i)
        {
            return (&xx)[i];
        }

        Real operator()(size_t i, size_t j) const
        {
            return (&xx)[3 * (i % 3) + (j % 3)];
        }

        Real& operator()(size_t i, size_t j)
        {
            return (&xx)[3 * (i % 3) + (j % 3)];
        }

        Matrix33& operator=(const Matrix33& rhs)
        {
            if (this != &rhs)
            {
                xx = rhs.xx; xy = rhs.xy; xz = rhs.xz;
                yx = rhs.yx; yy = rhs.yy; yz = rhs.yz;
                zx = rhs.zx; zy = rhs.zy; zz = rhs.zz;
            }
            return *this;
        }

        bool operator==(const Matrix33& rhs) const
        {
            return Math::IsEqual(xx, rhs.xx) && Math::IsEqual(xy, rhs.xy) && Math::IsEqual(xz, rhs.xz)
                && Math::IsEqual(yx, rhs.yx) && Math::IsEqual(yy, rhs.yy) && Math::IsEqual(yz, rhs.yz)
                && Math::IsEqual(zx, rhs.zx) && Math::IsEqual(zy, rhs.zy) && Math::IsEqual(zz, rhs.zz);
        }

        bool operator!=(const Matrix33& rhs) const
        {
            return !(*this == rhs);
        }

        Matrix33 operator-() const
        {
            return Matrix33(-xx, -xy, -xz, -yx, -yy, -yz, -zx, -zy, -zz);
        }

        Matrix33& operator+=(const Matrix33& rhs)
        {
            xx += rhs.xx; xy += rhs.xy; xz += rhs.xz;
            yx += rhs.yx; yy += rhs.yy; yz += rhs.yz;
            zx += rhs.zx; zy += rhs.zy; zz += rhs.zz;
            return *this;
        }

        Matrix33& operator-=(const Matrix33& rhs)
        {
            xx -= rhs.xx; xy -= rhs.xy; xz -= rhs.xz;
            yx -= rhs.yx; yy -= rhs.yy; yz -= rhs.yz;
            zx -= rhs.zx; zy -= rhs.zy; zz -= rhs.zz;
            return *this;
        }

        Matrix33& operator*=(Real real)
        {
            xx *= real; xy *= real; xz *= real;
            yx *= real; yy *= real; yz *= real;
            zx *= real; zy *= real; zz *= real;
            return *this;
        }

        Real Determinant() const
        {
            return xx * (yy * zz - yz * zy) - xy * (yx * zz - yz * zx) + xz * (yx * zy - yy * zx);
        }

        Matrix33 Transpose() const
        {
            return Matrix33(xx, yx, zx, xy, yy, zy, xz, yz, zz);
        }

        bool IsValid() const
        {
            return Math::IsValid(xx) && Math::IsValid(xy) && Math::IsValid(xz)
                && Math::IsValid(yx) && Math::IsValid(yy) && Math::IsValid(yz)
                && Math::IsValid(zx) && Math::IsValid(zy) && Math::IsValid(zz);
        }

        bool IsZero() const
        {
            return Math::IsZero(xx) && Math::IsZero(xy) && Math::IsZero(xz)
                && Math::IsZero(yx) && Math::IsZero(yy) && Math::IsZero(yz)
                && Math::IsZero(zx) && Math::IsZero(zy) && Math::IsZero(zz);
        }

        bool IsIdentity() const
        {
            return Math::IsEqual(xx, 1.0) && Math::IsZero(xy) && Math::IsZero(xz)
                && Math::IsZero(yx) && Math::IsEqual(yy, 1.0) && Math::IsZero(yz)
                && Math::IsZero(zx) && Math::IsZero(zy) && Math::IsEqual(zz, 1.0);
        }
    };

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Matrix33 operator+(const Matrix33& a, const Matrix33& b);
    Matrix33 operator-(const Matrix33& a, const Matrix33& b);
    Matrix33 operator*(const Matrix33& a, const Matrix33& b);
    Matrix33 operator*(const Matrix33& matrix, Real real);
    Matrix33 operator*(Real real, const Matrix33& matrix);
}
