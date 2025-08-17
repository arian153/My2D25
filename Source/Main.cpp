#include <iomanip>
#include <iostream>

import Engine2D.Math.Constants;
import Engine2D.Math.Utility;
import Engine2D.Math.Orientation;

int main()
{
    Engine2D::Real pi_div_2 = Engine2D::Math::PI_DIV_2;

    Engine2D::Orientation rot_a(Engine2D::Math::PI_DIV_2);
    Engine2D::Orientation rot_b(Engine2D::Math::PI_DIV_3);

    Engine2D::Orientation rot_ab = Engine2D::Orientation::Apply(rot_a, rot_b);
    Engine2D::Orientation new_ab(rot_ab.a);

    Engine2D::Orientation rot_iab = Engine2D::Orientation::ApplyT(rot_a, rot_b);
    Engine2D::Orientation new_iab(rot_iab.a);

    return 0;
}