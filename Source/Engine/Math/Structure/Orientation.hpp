#pragma once

#include <cmath>
#include "../Utility/MathConstants.hpp"

namespace Engine2D
{
    class Orientation
    {
    public:
        Orientation();
        ~Orientation();

        explicit Orientation(Real angle);
        Orientation(Real sine, Real cosine);

        void Set(Real angle);
        void Set(Real sine, Real cosine);

        static Orientation Apply(const Orientation& q, const Orientation& r);
        static Orientation ApplyT(const Orientation& q, const Orientation& r);

        static Orientation Identity();

    public:
        Real s = 0;
        Real c = 1;
        Real a = 0;
    };

    Orientation::Orientation()
    {
    }

    Orientation::~Orientation()
    {
    }

    Orientation::Orientation(Real angle)
    {
        s = std::sin(angle);
        c = std::cos(angle);
        a = angle;
    }

    Orientation::Orientation(Real sine,  Real cosine)
    {
        s = sine;
        c = cosine;
        a = std::atan2(sine, cosine);
    }

    void Orientation::Set(Real angle)
    {
        s = std::sin(angle);
        c = std::cos(angle);
        a = angle;
    }

    void Orientation::Set(Real sine, Real cosine)
    {
        s = sine;
        c = cosine;
        a = std::atan2(sine, cosine);
    }

    Orientation Orientation::Apply(const Orientation& q, const Orientation& r)
    {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs

        Orientation qr;
        qr.s = q.s * r.c + q.c * r.s;
        qr.c = q.c * r.c - q.s * r.s;
        qr.a = q.a + r.a;
        return qr;
    }

    Orientation Orientation::ApplyT(const Orientation& q, const Orientation& r)
    {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs

        Orientation qr;
        qr.s = q.c * r.s - q.s * r.c;
        qr.c = q.c * r.c + q.s * r.s;
        qr.a = r.a - q.a;
        return qr;
    }

    Orientation Orientation::Identity()
    {
        return Orientation();
    }

}
