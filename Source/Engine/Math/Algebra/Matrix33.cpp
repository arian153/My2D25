#include "Matrix33.hpp"
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Matrix33 operator+(const Matrix33& a, const Matrix33& b)
    {
        return Matrix33(
            a.xx + b.xx, a.xy + b.xy, a.xz + b.xz,
            a.yx + b.yx, a.yy + b.yy, a.yz + b.yz,
            a.zx + b.zx, a.zy + b.zy, a.zz + b.zz);
    }

    Matrix33 operator-(const Matrix33& a, const Matrix33& b)
    {
        return Matrix33(
            a.xx - b.xx, a.xy - b.xy, a.xz - b.xz,
            a.yx - b.yx, a.yy - b.yy, a.yz - b.yz,
            a.zx - b.zx, a.zy - b.zy, a.zz - b.zz);
    }

    Matrix33 operator*(const Matrix33& a, const Matrix33& b)
    {
        Matrix33 result;
        result.xx = a.xx * b.xx + a.xy * b.yx + a.xz * b.zx;
        result.xy = a.xx * b.xy + a.xy * b.yy + a.xz * b.zy;
        result.xz = a.xx * b.xz + a.xy * b.yz + a.xz * b.zz;
        result.yx = a.yx * b.xx + a.yy * b.yx + a.yz * b.zx;
        result.yy = a.yx * b.xy + a.yy * b.yy + a.yz * b.zy;
        result.yz = a.yx * b.xz + a.yy * b.yz + a.yz * b.zz;
        result.zx = a.zx * b.xx + a.zy * b.yx + a.zz * b.zx;
        result.zy = a.zx * b.xy + a.zy * b.yy + a.zz * b.zy;
        result.zz = a.zx * b.xz + a.zy * b.yz + a.zz * b.zz;
        return result;
    }

    Matrix33 operator*(const Matrix33& matrix, Real real)
    {
        return Matrix33(
            matrix.xx * real, matrix.xy * real, matrix.xz * real,
            matrix.yx * real, matrix.yy * real, matrix.yz * real,
            matrix.zx * real, matrix.zy * real, matrix.zz * real);
    }

    Matrix33 operator*(Real real, const Matrix33& matrix)
    {
        return matrix * real;
    }
}
