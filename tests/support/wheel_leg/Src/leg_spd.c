#include "leg_spd.h"
#include "leg_model.h"
#include <math.h>
void leg_spd(float dphi1, float dphi4, float phi1, float phi4, float spd[2])
{
    LegModel m;
    if (!isfinite(dphi1) || !isfinite(dphi4) ||
        !leg_model(phi1,phi4,LEG_BRANCH_MATLAB,&m)) {
        spd[0] = NAN; spd[1] = NAN; return;
    }
    spd[0] = m.jacobian[0]*dphi1+m.jacobian[1]*dphi4;
    spd[1] = m.jacobian[2]*dphi1+m.jacobian[3]*dphi4;
}
