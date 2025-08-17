#include "Operator.hpp"
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 유틸리티 함수들
    // ========================================

    // generate tensor from two vector product
    Matrix22 OuterProduct(const Vector2& a, const Vector2& b)
    {
        return Matrix22(
                        a.x * b.x, a.x * b.y,
                        a.y * b.x, a.y * b.y);
    }

    // ========================================
    // Matrix22 멤버 함수들의 구현
    // ========================================

    void Matrix22::SetRows(const Vector2& row1, const Vector2& row2)
    {
        //row 1
        xx = row1.x;
        xy = row1.y;

        //row 2
        yx = row2.x;
        yy = row2.y;
    }

    void Matrix22::SetColumns(const Vector2& col1, const Vector2& col2)
    {
        //column 1
        xx = col1.x;
        yx = col1.y;

        //column 2
        xy = col2.x;
        yy = col2.y;
    }

    void Matrix22::GetRows(Vector2& row1, Vector2& row2) const
    {
        row1.x = xx;
        row1.y = xy;

        row2.x = yx;
        row2.y = yy;
    }

    void Matrix22::GetColumns(Vector2& col1, Vector2& col2) const
    {
        col1.x = xx;
        col1.y = yx;

        col2.x = xy;
        col2.y = yy;
    }

    Vector2 Matrix22::GetRow(size_t i) const
    {
        return Vector2((*this)[2 * i], (*this)[2 * i + 1]);
    }

    Vector2 Matrix22::GetColumn(size_t i) const
    {
        return Vector2((*this)[i], (*this)[i + 2]);
    }

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Vector2 operator*(const Matrix22& matrix, const Vector2& vector)
    {
        Vector2 result;
        result.x = matrix.xx * vector.x + matrix.xy * vector.y;
        result.y = matrix.yx * vector.x + matrix.yy * vector.y;
        return result;
    }

    Vector2 operator*(const Vector2& vector, const Matrix22& matrix)
    {
        Vector2 result;
        result.x = matrix.xx * vector.x + matrix.yx * vector.y;
        result.y = matrix.xy * vector.x + matrix.yy * vector.y;
        return result;
    }

    // ========================================
    // 수학적 연산 함수들
    // ========================================

    Vector2 SolveMatrix22(const Matrix22& a, const Vector2& b)
    {
        //Solve A*x = b; find vector x.

        Real det = a.Determinant();
        det      = Math::IsZero(det) ? 0.0 : 1.0 / det;

        Vector2 x;
        x.x = det * (a.yy * b.x - a.xy * b.y);
        x.y = det * (a.xx * b.y - a.yx * b.x);

        return x;
    }

    Vector2 GetNormal(const Vector2& vector)
    {
        return Vector2(-vector.y, vector.x);
    }
}
