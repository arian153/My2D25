#pragma once

#include <vector>
#include "../../Utility/MathConstants.hpp"
#include "../../Algebra/Vector2.hpp"
#include "../Algorithm/GJK_EPA.hpp"

namespace Engine2D
{
    // ========================================
    // 도형 타입 열거형
    // ========================================

    enum class ShapeType
    {
        CIRCLE,
        RECTANGLE,
        TRIANGLE,
        POLYGON,
        ELLIPSE,
        CONVEX,
        CONCAVE,
        UNKNOWN
    };

    // ========================================
    // 도형 인터페이스
    // ========================================

    class IShape
    {
    public:
        virtual ~IShape() = default;
        virtual ShapeType GetType() const = 0;
        virtual Vector2 GetCenter() const = 0;
        virtual Real GetBoundingRadius() const = 0;

        // SAT를 위한 축 투영
        virtual std::vector<Vector2> GetAxes() const = 0;
        virtual std::pair<Real, Real> ProjectOnAxis(const Vector2& axis) const = 0;

        // SDF를 위한 거리 함수
        virtual SDFResult GetSDF(const Vector2& point) const = 0;

        // GJK를 위한 지지점 함수
        virtual Vector2 GetSupportPoint(const Vector2& direction) const = 0;
    };
}
