#include "ComplexShapeFactory.hpp"
#include <algorithm>
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 스윕 도형 생성
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSweptCircle(
        CurveType curveType, const std::vector<Vector2>& controlPoints, Real radius)
    {
        // 곡선 점들 생성
        std::vector<Vector2> curvePoints = GenerateCurvePoints(curveType, controlPoints);

        // 복잡한 도형 생성 (실제 구현에서는 ComplexShape 클래스 필요)
        auto shape = std::make_unique<ComplexShape>();

        // 스윕된 도형의 정점들 계산
        std::vector<Vector2> vertices;
        for (const auto& point : curvePoints) {
            // 원형 프로파일을 각 점에 적용
            const int numSegments = 16;
            for (int i = 0; i < numSegments; ++i) {
                Real angle = Real(2) * PI * i / numSegments;
                Vector2 offset(std::cos(angle), std::sin(angle));
                vertices.push_back(point + offset * radius);
            }
        }

        // shape->SetVertices(vertices); // 실제 구현에서는 이런 메서드 필요
        return shape;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSweptRectangle(
        CurveType curveType, const std::vector<Vector2>& controlPoints, const Vector2& size)
    {
        // 곡선 점들 생성
        std::vector<Vector2> curvePoints = GenerateCurvePoints(curveType, controlPoints);

        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 스윕된 도형의 정점들 계산
        std::vector<Vector2> vertices;
        for (const auto& point : curvePoints) {
            // 사각형 프로파일을 각 점에 적용
            Vector2 halfSize = size * Real(0.5);
            vertices.push_back(point + Vector2(-halfSize.x, -halfSize.y));
            vertices.push_back(point + Vector2(halfSize.x, -halfSize.y));
            vertices.push_back(point + Vector2(halfSize.x, halfSize.y));
            vertices.push_back(point + Vector2(-halfSize.x, halfSize.y));
        }

        // shape->SetVertices(vertices);
        return shape;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSweptPolygon(
        CurveType curveType, const std::vector<Vector2>& controlPoints, const std::vector<Vector2>& vertices)
    {
        // 곡선 점들 생성
        std::vector<Vector2> curvePoints = GenerateCurvePoints(curveType, controlPoints);

        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 스윕된 도형의 정점들 계산
        std::vector<Vector2> sweptVertices;
        for (const auto& point : curvePoints) {
            // 다각형 프로파일을 각 점에 적용
            for (const auto& vertex : vertices) {
                sweptVertices.push_back(point + vertex);
            }
        }

        // shape->SetVertices(sweptVertices);
        return shape;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSweptShape(
        CurveType curveType, const std::vector<Vector2>& controlPoints,
        std::unique_ptr<ISweepProfile> profile)
    {
        // 곡선 점들 생성
        std::vector<Vector2> curvePoints = GenerateCurvePoints(curveType, controlPoints);

        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 스윕된 도형의 정점들 계산
        std::vector<Vector2> vertices;
        for (const auto& point : curvePoints) {
            // 커스텀 프로파일을 각 점에 적용
            std::vector<Vector2> profileVertices = profile->GetVertices();
            for (const auto& vertex : profileVertices) {
                vertices.push_back(point + vertex);
            }
        }

        // shape->SetVertices(vertices);
        return shape;
    }

    // ========================================
    // 민코스키 합 도형 생성
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateMinkowskiSum(
        const IShape* shapeA, const IShape* shapeB)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 실제 구현에서는 MinkowskiAlgorithms를 사용하여 계산
        // MinkowskiResult result = MinkowskiAlgorithms::CalculateSum(verticesA, verticesB);
        // shape->SetVertices(result.vertices);

        return shape;
    }

    // ========================================
    // SDF 블렌딩 도형 생성
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSDFBlend(
        const IShape* shapeA, const IShape* shapeB, BlendOperation operation)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // SDF 블렌딩 함수 설정
        auto blendFunction = [shapeA, shapeB, operation](const Vector2& point) -> SDFResult {
            SDFResult sdfA = shapeA->GetSDF(point);
            SDFResult sdfB = shapeB->GetSDF(point);
            return BlendSDFs(sdfA, sdfB, operation);
        };

        // shape->SetSDFFunction(blendFunction);
        return shape;
    }

    // ========================================
    // 사용자 정의 SDF 도형 생성
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateCustomSDF(
        std::function<SDFResult(const Vector2&)> sdfFunction,
        const std::pair<Vector2, Vector2>& boundingBox)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // shape->SetSDFFunction(sdfFunction);
        // shape->SetBoundingBox(boundingBox);

        return shape;
    }

    // ========================================
    // 편의 함수들
    // ========================================

    std::unique_ptr<ISweepProfile> ComplexShapeFactory::CreateCircleProfile(Real radius)
    {
        // 원형 프로파일 생성 (실제 구현에서는 ISweepProfile 구현체 필요)
        auto profile = std::make_unique<ISweepProfile>();

        // 원형 정점들 생성
        std::vector<Vector2> vertices;
        const int numSegments = 16;
        for (int i = 0; i < numSegments; ++i) {
            Real angle = Real(2) * PI * i / numSegments;
            vertices.push_back(Vector2(std::cos(angle), std::sin(angle)) * radius);
        }

        // profile->SetVertices(vertices);
        return profile;
    }

    std::unique_ptr<ISweepProfile> ComplexShapeFactory::CreateRectangleProfile(const Vector2& size)
    {
        // 사각형 프로파일 생성
        auto profile = std::make_unique<ISweepProfile>();

        // 사각형 정점들 생성
        std::vector<Vector2> vertices;
        Vector2 halfSize = size * Real(0.5);
        vertices.push_back(Vector2(-halfSize.x, -halfSize.y));
        vertices.push_back(Vector2(halfSize.x, -halfSize.y));
        vertices.push_back(Vector2(halfSize.x, halfSize.y));
        vertices.push_back(Vector2(-halfSize.x, halfSize.y));

        // profile->SetVertices(vertices);
        return profile;
    }

    std::unique_ptr<ISweepProfile> ComplexShapeFactory::CreatePolygonProfile(const std::vector<Vector2>& vertices)
    {
        // 다각형 프로파일 생성
        auto profile = std::make_unique<ISweepProfile>();

        // profile->SetVertices(vertices);
        return profile;
    }

    // ========================================
    // 고급 블렌딩 함수들
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSmoothUnion(
        const IShape* shapeA, const IShape* shapeB, Real smoothness)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 부드러운 합집합 SDF 함수 설정
        auto blendFunction = [shapeA, shapeB, smoothness](const Vector2& point) -> SDFResult {
            SDFResult sdfA = shapeA->GetSDF(point);
            SDFResult sdfB = shapeB->GetSDF(point);
            return SmoothBlendSDFs(sdfA, sdfB, BlendOperation::UNION, smoothness);
        };

        // shape->SetSDFFunction(blendFunction);
        return shape;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSmoothIntersection(
        const IShape* shapeA, const IShape* shapeB, Real smoothness)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 부드러운 교집합 SDF 함수 설정
        auto blendFunction = [shapeA, shapeB, smoothness](const Vector2& point) -> SDFResult {
            SDFResult sdfA = shapeA->GetSDF(point);
            SDFResult sdfB = shapeB->GetSDF(point);
            return SmoothBlendSDFs(sdfA, sdfB, BlendOperation::INTERSECTION, smoothness);
        };

        // shape->SetSDFFunction(blendFunction);
        return shape;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateSmoothSubtraction(
        const IShape* shapeA, const IShape* shapeB, Real smoothness)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 부드러운 차집합 SDF 함수 설정
        auto blendFunction = [shapeA, shapeB, smoothness](const Vector2& point) -> SDFResult {
            SDFResult sdfA = shapeA->GetSDF(point);
            SDFResult sdfB = shapeB->GetSDF(point);
            return SmoothBlendSDFs(sdfA, sdfB, BlendOperation::SUBTRACTION, smoothness);
        };

        // shape->SetSDFFunction(blendFunction);
        return shape;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateRoundBlend(
        const IShape* shapeA, const IShape* shapeB, Real radius)
    {
        // 복잡한 도형 생성
        auto shape = std::make_unique<ComplexShape>();

        // 원형 블렌딩 SDF 함수 설정
        auto blendFunction = [shapeA, shapeB, radius](const Vector2& point) -> SDFResult {
            SDFResult sdfA = shapeA->GetSDF(point);
            SDFResult sdfB = shapeB->GetSDF(point);
            return RoundBlendSDFs(sdfA, sdfB, radius);
        };

        // shape->SetSDFFunction(blendFunction);
        return shape;
    }

    // ========================================
    // 변형 함수들
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateOutline(
        const IShape* shape, Real thickness)
    {
        // 복잡한 도형 생성
        auto shape_outline = std::make_unique<ComplexShape>();

        // 윤곽선 SDF 함수 설정
        auto outlineFunction = [shape, thickness](const Vector2& point) -> SDFResult {
            SDFResult sdf = shape->GetSDF(point);
            sdf.distance = std::abs(sdf.distance) - thickness;
            return sdf;
        };

        // shape_outline->SetSDFFunction(outlineFunction);
        return shape_outline;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateShadow(
        const IShape* shape, const Vector2& lightDirection, Real shadowLength)
    {
        // 복잡한 도형 생성
        auto shadow = std::make_unique<ComplexShape>();

        // 그림자 SDF 함수 설정
        auto shadowFunction = [shape, lightDirection, shadowLength](const Vector2& point) -> SDFResult {
            // 그림자 계산 로직
            SDFResult sdf = shape->GetSDF(point);
            // 실제 구현에서는 더 복잡한 그림자 계산 필요
            return sdf;
        };

        // shadow->SetSDFFunction(shadowFunction);
        return shadow;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateReflection(
        const IShape* shape, const Vector2& mirrorNormal, Real mirrorDistance)
    {
        // 복잡한 도형 생성
        auto reflection = std::make_unique<ComplexShape>();

        // 반사 SDF 함수 설정
        auto reflectionFunction = [shape, mirrorNormal, mirrorDistance](const Vector2& point) -> SDFResult {
            // 반사 계산 로직
            SDFResult sdf = shape->GetSDF(point);
            // 실제 구현에서는 더 복잡한 반사 계산 필요
            return sdf;
        };

        // reflection->SetSDFFunction(reflectionFunction);
        return reflection;
    }

    // ========================================
    // 패턴 생성 함수들
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateGridPattern(
        const IShape* baseShape, Real spacing, Real lineThickness)
    {
        // 복잡한 도형 생성
        auto pattern = std::make_unique<ComplexShape>();

        // 격자 패턴 SDF 함수 설정
        auto patternFunction = [baseShape, spacing, lineThickness](const Vector2& point) -> SDFResult {
            // 격자 패턴 계산 로직
            SDFResult sdf = baseShape->GetSDF(point);
            // 실제 구현에서는 격자 패턴 계산 필요
            return sdf;
        };

        // pattern->SetSDFFunction(patternFunction);
        return pattern;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateDotPattern(
        const IShape* baseShape, Real spacing, Real dotRadius)
    {
        // 복잡한 도형 생성
        auto pattern = std::make_unique<ComplexShape>();

        // 점 패턴 SDF 함수 설정
        auto patternFunction = [baseShape, spacing, dotRadius](const Vector2& point) -> SDFResult {
            // 점 패턴 계산 로직
            SDFResult sdf = baseShape->GetSDF(point);
            // 실제 구현에서는 점 패턴 계산 필요
            return sdf;
        };

        // pattern->SetSDFFunction(patternFunction);
        return pattern;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::CreateStripePattern(
        const IShape* baseShape, Real stripeWidth, Real spacing)
    {
        // 복잡한 도형 생성
        auto pattern = std::make_unique<ComplexShape>();

        // 줄무늬 패턴 SDF 함수 설정
        auto patternFunction = [baseShape, stripeWidth, spacing](const Vector2& point) -> SDFResult {
            // 줄무늬 패턴 계산 로직
            SDFResult sdf = baseShape->GetSDF(point);
            // 실제 구현에서는 줄무늬 패턴 계산 필요
            return sdf;
        };

        // pattern->SetSDFFunction(patternFunction);
        return pattern;
    }

    // ========================================
    // 유틸리티 함수들
    // ========================================

    std::unique_ptr<ComplexShape> ComplexShapeFactory::Clone(const ComplexShape* shape)
    {
        if (!shape) return nullptr;

        // 복잡한 도형 생성
        auto cloned = std::make_unique<ComplexShape>();

        // shape->CopyTo(*cloned); // 실제 구현에서는 복사 메서드 필요

        return cloned;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::Transform(
        const ComplexShape* shape, const Vector2& translation, Real rotation, Real scale)
    {
        if (!shape) return nullptr;

        // 복잡한 도형 생성
        auto transformed = std::make_unique<ComplexShape>();

        // 변형된 SDF 함수 설정
        auto transformFunction = [shape, translation, rotation, scale](const Vector2& point) -> SDFResult {
            // 역변형 적용
            Vector2 transformedPoint = (point - translation) / scale;
            Real cosRot = std::cos(-rotation);
            Real sinRot = std::sin(-rotation);
            Vector2 rotatedPoint(
                transformedPoint.x * cosRot - transformedPoint.y * sinRot,
                transformedPoint.x * sinRot + transformedPoint.y * cosRot
            );

            SDFResult sdf = shape->GetSDF(rotatedPoint);
            sdf.distance *= scale; // 거리도 스케일 조정

            return sdf;
        };

        // transformed->SetSDFFunction(transformFunction);
        return transformed;
    }

    std::unique_ptr<ComplexShape> ComplexShapeFactory::Combine(
        const std::vector<const IShape*>& shapes, BlendOperation operation)
    {
        if (shapes.empty()) return nullptr;
        if (shapes.size() == 1) {
            // 단일 도형인 경우 복사
            return std::make_unique<ComplexShape>();
        }

        // 복잡한 도형 생성
        auto combined = std::make_unique<ComplexShape>();

        // 결합된 SDF 함수 설정
        auto combineFunction = [shapes, operation](const Vector2& point) -> SDFResult {
            SDFResult result = shapes[0]->GetSDF(point);

            for (size_t i = 1; i < shapes.size(); ++i) {
                SDFResult sdf = shapes[i]->GetSDF(point);
                result = BlendSDFs(result, sdf, operation);
            }

            return result;
        };

        // combined->SetSDFFunction(combineFunction);
        return combined;
    }

    // ========================================
    // 내부 헬퍼 함수들
    // ========================================

    std::vector<Vector2> ComplexShapeFactory::GenerateCurvePoints(
        CurveType curveType, const std::vector<Vector2>& controlPoints, int numPoints)
    {
        std::vector<Vector2> points;

        switch (curveType) {
            case CurveType::BEZIER:
                // 베지어 곡선 점들 생성
                for (int i = 0; i < numPoints; ++i) {
                    Real t = static_cast<Real>(i) / (numPoints - 1);
                    // 실제 구현에서는 베지어 곡선 계산 필요
                    points.push_back(controlPoints[0]); // 임시
                }
                break;

            case CurveType::NURBS:
                // NURBS 곡선 점들 생성
                for (int i = 0; i < numPoints; ++i) {
                    Real t = static_cast<Real>(i) / (numPoints - 1);
                    // 실제 구현에서는 NURBS 곡선 계산 필요
                    points.push_back(controlPoints[0]); // 임시
                }
                break;

            case CurveType::CATMULL_ROM:
                // Catmull-Rom 곡선 점들 생성
                for (int i = 0; i < numPoints; ++i) {
                    Real t = static_cast<Real>(i) / (numPoints - 1);
                    // 실제 구현에서는 Catmull-Rom 곡선 계산 필요
                    points.push_back(controlPoints[0]); // 임시
                }
                break;

            case CurveType::HERMITE:
                // Hermite 곡선 점들 생성
                for (int i = 0; i < numPoints; ++i) {
                    Real t = static_cast<Real>(i) / (numPoints - 1);
                    // 실제 구현에서는 Hermite 곡선 계산 필요
                    points.push_back(controlPoints[0]); // 임시
                }
                break;
        }

        return points;
    }

    SDFResult ComplexShapeFactory::BlendSDFs(
        const SDFResult& sdfA, const SDFResult& sdfB, BlendOperation operation)
    {
        SDFResult result;

        switch (operation) {
            case BlendOperation::UNION:
                result.distance = std::min(sdfA.distance, sdfB.distance);
                break;

            case BlendOperation::INTERSECTION:
                result.distance = std::max(sdfA.distance, sdfB.distance);
                break;

            case BlendOperation::SUBTRACTION:
                result.distance = std::max(sdfA.distance, -sdfB.distance);
                break;
        }

        return result;
    }

    SDFResult ComplexShapeFactory::SmoothBlendSDFs(
        const SDFResult& sdfA, const SDFResult& sdfB, BlendOperation operation, Real smoothness)
    {
        SDFResult result;

        switch (operation) {
            case BlendOperation::UNION: {
                Real h = std::clamp(0.5 + 0.5 * (sdfB.distance - sdfA.distance) / smoothness, 0.0, 1.0);
                result.distance = sdfA.distance * (1 - h) + sdfB.distance * h - smoothness * h * (1 - h);
                break;
            }

            case BlendOperation::INTERSECTION: {
                Real h = std::clamp(0.5 - 0.5 * (sdfB.distance - sdfA.distance) / smoothness, 0.0, 1.0);
                result.distance = sdfA.distance * (1 - h) + sdfB.distance * h + smoothness * h * (1 - h);
                break;
            }

            case BlendOperation::SUBTRACTION: {
                Real h = std::clamp(0.5 - 0.5 * (sdfB.distance + sdfA.distance) / smoothness, 0.0, 1.0);
                result.distance = sdfA.distance * (1 - h) - sdfB.distance * h + smoothness * h * (1 - h);
                break;
            }
        }

        return result;
    }

    SDFResult ComplexShapeFactory::RoundBlendSDFs(
        const SDFResult& sdfA, const SDFResult& sdfB, Real radius)
    {
        SDFResult result;

        // 원형 블렌딩 (합집합)
        Real h = std::clamp(0.5 + 0.5 * (sdfB.distance - sdfA.distance) / radius, 0.0, 1.0);
        result.distance = sdfA.distance * (1 - h) + sdfB.distance * h - radius * h * (1 - h);

        return result;
    }
}
