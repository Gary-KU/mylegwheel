#include "leg_conv.h"
#include "leg_model.h"
#include <math.h>
void leg_conv(float F, float Tp, float phi1, float phi4, float T[2])
{
    LegModel m;
    T[0] = 0.0f; T[1] = 0.0f;
    if (!isfinite(F) || !isfinite(Tp) ||
        !leg_model(phi1,phi4,LEG_BRANCH_MATLAB,&m)) return;
    T[0] = m.jacobian[0]*F+m.jacobian[2]*Tp;
    T[1] = m.jacobian[1]*F+m.jacobian[3]*Tp;
}
int leg_conv_3d(float F, float Tp, float phi1, float phi4, float T[2])
{
    LegModel m;
    if (!T) return 0;
    T[0] = 0.0f; T[1] = 0.0f;
    if (!isfinite(F) || !isfinite(Tp) ||
        !leg_model(phi1,phi4,LEG_BRANCH_3D,&m)) return 0;
    T[0] = m.jacobian[1]*F+m.jacobian[3]*Tp;
    T[1] = m.jacobian[0]*F+m.jacobian[2]*Tp;
    return 1;
}
