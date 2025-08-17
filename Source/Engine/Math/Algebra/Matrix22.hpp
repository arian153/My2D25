#pragma once

#include "../Utility/MathConstants.hpp"
#include "../Utility/MathUtility.hpp"

namespace Engine2D
{
    class Vector2;

    // row major order matrix
    class Matrix22
    {
    public:
        Real xx = 1.0, xy = 0.0;
        Real yx = 0.0, yy = 1.0;

    public:
        Matrix22()  = default;
        ~Matrix22() = default;

        Matrix22(const Matrix22& rhs)
            : xx(rhs.xx), xy(rhs.xy), yx(rhs.yx), yy(rhs.yy)
        {
        }

        Matrix22(Real m00, Real m01, Real m10, Real m11)
            : xx(m00), xy(m01), yx(m10), yy(m11)
        {
        }

        explicit Matrix22(Real a[4])
            : xx(a[0]), xy(a[1]), yx(a[2]), yy(a[3])
        {
        }

        void Set(Real m00, Real m01, Real m10, Real m11)
        {
            xx = m00;
            xy = m01;
            yx = m10;
            yy = m11;
        }

        void SetRows(const Vector2& row1, const Vector2& row2);
        void SetColumns(const Vector2& col1, const Vector2& col2);

        void SetDiagonal(Real a, Real b)
        {
            xx = a;
            yy = b;
        }

        void SetZero()
        {
            xx = 0.0;
            xy = 0.0;
            yx = 0.0;
            yy = 0.0;
        }

        void SetIdentity()
        {
            xx = 1.0;
            xy = 0.0;
            yx = 0.0;
            yy = 1.0;
        }

        void SetInverse()
        {
            Matrix22 copy = *this;
            Real     det  = xx * yy - xy * yx;
            if (Math::IsZero(det))
                return;

            Real inv_det = 1.0 / det;
            xx           = inv_det * copy.yy;
            xy           = -inv_det * copy.xy;
            yx           = -inv_det * copy.yx;
            yy           = inv_det * copy.xx;
        }

        void SetTranspose()
        {
            Real xy_ = xy;

            xy = yx;
            yx = xy_;
        }

        void SetNegate()
        {
            xx = -xx;
            xy = -xy;
            yx = -yx;
            yy = -yy;
        }

        void ClearError()
        {
            xx = ClearError(xx);
            xy = ClearError(xy);
            yx = ClearError(yx);
            yy = ClearError(yy);
        }

        bool IsValid() const
        {
            return Math::IsValid(xx) && Math::IsValid(xy)
                    && Math::IsValid(yx) && Math::IsValid(yy);
        }

        bool IsZero() const
        {
            return Math::IsZero(xx) && Math::IsZero(xy)
                    && Math::IsZero(yx) && Math::IsZero(yy);
        }

        bool IsIdentity() const
        {
            return Math::IsEqual(xx, 1.0) && Math::IsZero(xy)
                    && Math::IsZero(yx) && Math::IsEqual(yy, 1.0);
        }

        void GetRows(Vector2& row1, Vector2& row2) const;
        void GetColumns(Vector2& col1, Vector2& col2) const;

        Vector2 GetRow(size_t i) const;
        Vector2 GetColumn(size_t i) const;

        Real Determinant() const
        {
            return xx * yy - xy * yx;
        }

        Real Trace() const
        {
            return xx + yy;
        }

        Matrix22 Adjoint() const
        {
            Matrix22 result;
            result.xx = yy;
            result.xy = -xy;
            result.yx = -yx;
            result.yy = xx;
            return result;
        }

        Matrix22 Inverse() const
        {
            Matrix22 result;
            Real     det = xx * yy - xy * yx;
            if (Math::IsZero(det))
                return result;

            Real inv_det = 1.0 / det;
            result.xx    = inv_det * yy;
            result.xy    = -inv_det * xy;
            result.yx    = -inv_det * yx;
            result.yy    = inv_det * xx;
            return result;
        }

        Matrix22 Transpose() const
        {
            Matrix22 result;
            result.xx = xx;
            result.xy = yx;
            result.yx = xy;
            result.yy = yy;
            return result;
        }

        Matrix22 Negate() const
        {
            Matrix22 result;
            result.xx = -xx;
            result.xy = -xy;
            result.yx = -yx;
            result.yy = -yy;
            return result;
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
            return (&xx)[2 * (i % 2) + (j % 2)];
        }

        Real& operator()(size_t i, size_t j)
        {
            return (&xx)[2 * (i % 2) + (j % 2)];
        }

        Matrix22& operator =(const Matrix22& rhs)
        {
            if (this != &rhs)
            {
                xx = rhs.xx;
                xy = rhs.xy;
                yx = rhs.yx;
                yy = rhs.yy;
            }
            return *this;
        }

        Matrix22& operator =(Real rhs)
        {
            xx = rhs;
            xy = rhs;
            yx = rhs;
            yy = rhs;
            return *this;
        }

        bool operator ==(const Matrix22& rhs) const
        {
            return Math::IsEqual(xx, rhs.xx) && Math::IsEqual(xy, rhs.xy)
                    && Math::IsEqual(yx, rhs.yx) && Math::IsEqual(yy, rhs.yy);
        }

        bool operator !=(const Matrix22& rhs) const
        {
            return Math::IsNotEqual(xx, rhs.xx) || Math::IsNotEqual(xy, rhs.xy)
                    || Math::IsNotEqual(yx, rhs.yx) || Math::IsNotEqual(yy, rhs.yy);
        }

        Matrix22 operator -() const
        {
            return Matrix22(-xx, -xy, -yx, -yy);
        }

        Matrix22& operator +=(const Matrix22& rhs)
        {
            xx += rhs.xx;
            xy += rhs.xy;
            yx += rhs.yx;
            yy += rhs.yy;
            return *this;
        }

        Matrix22& operator -=(const Matrix22& rhs)
        {
            xx -= rhs.xx;
            xy -= rhs.xy;
            yx -= rhs.yx;
            yy -= rhs.yy;
            return *this;
        }

        Matrix22& operator +=(Real real)
        {
            xx += real;
            xy += real;
            yx += real;
            yy += real;
            return *this;
        }

        Matrix22& operator -=(Real real)
        {
            xx -= real;
            xy -= real;
            yx -= real;
            yy -= real;
            return *this;
        }

        Matrix22& operator *=(Real real)
        {
            xx *= real;
            xy *= real;
            yx *= real;
            yy *= real;
            return *this;
        }

        Matrix22& operator /=(Real real)
        {
            xx /= real;
            xy /= real;
            yx /= real;
            yy /= real;
            return *this;
        }
    };

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Matrix22 operator+(const Matrix22& a, const Matrix22& b);
    Matrix22 operator-(const Matrix22& a, const Matrix22& b);
    Matrix22 operator*(const Matrix22& a, const Matrix22& matrix);
    Matrix22 operator*(const Matrix22& matrix, Real real);
    Matrix22 operator*(Real real, const Matrix22& matrix);

    // ========================================
    // 유틸리티 함수들
    // ========================================

    // hadamard product - multiply its component
    Matrix22 HadamardProduct(const Matrix22& a, const Matrix22& b);
}
