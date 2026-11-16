/* MyProc: 0.05/0.105/0.105/0.05 m links, hip gap 0.06 m. */
#include "leg_pos.h"
#include "leg_model.h"
#include <math.h>
int leg_model(float phi1, float phi4, LegBranch branch, LegModel *out)
{
    float bx, by, dx, dy, vx, vy, d2, dist, h, sign;
    float cx, cy, rx, len2, r1x, r1y, r4x, r4y, det, q1, q4;
    float jx1, jy1, jx4, jy4;
    LegModel result = {0};
    if (!out) return 0;
    *out = result;
    if (!isfinite(phi1) || !isfinite(phi4) ||
        (branch != LEG_BRANCH_MATLAB && branch != LEG_BRANCH_3D && branch != LEG_BRANCH_DOWN)) return 0;
    /* 两髋轴位于 (0,0)、(0.06,0)；B、D 为两条主动短杆的末端，长度单位为 m。 */
    bx = 0.05f*cosf(phi1); by = 0.05f*sinf(phi1);
    dx = 0.06f+0.05f*cosf(phi4); dy = 0.05f*sinf(phi4);
    vx = dx-bx; vy = dy-by; d2 = vx*vx+vy*vy;
    /* 足端是以 B、D 为圆心、长杆长度为半径的交点；重合或相切时不能稳定求导。 */
    if (d2 <= 1e-12f || d2 >= 4.0f*0.105f*0.105f) return 0;
    dist = sqrtf(d2); h = sqrtf(0.105f*0.105f-0.25f*d2);
    /* sign 选择两圆交点所在侧；不同模型的 Y 轴约定对应不同装配分支。 */
    sign = 1.0f;
    if (branch == LEG_BRANCH_3D) sign = vx < 0.0f ? 1.0f : -1.0f;
    if (branch == LEG_BRANCH_DOWN) {
        /* Equal-height intersections do not identify a unique lower branch. */
        if (fabsf(vx) <= 1e-6f) return 0;
        sign = vx < 0.0f ? -1.0f : 1.0f;
    }
    cx = 0.5f*(bx+dx)-sign*h*vy/dist;
    cy = 0.5f*(by+dy)+sign*h*vx/dist;
    /* 虚拟腿从两髋轴中点指向足端；angle 从该坐标系 +X 轴起算。 */
    rx = cx-0.03f; len2 = rx*rx+cy*cy;
    r1x = cx-bx; r1y = cy-by; r4x = cx-dx; r4y = cy-dy;
    det = r1x*r4y-r1y*r4x;
    /* 两长杆近共线或虚拟腿趋于零长时雅可比奇异，拒绝放大噪声的解。 */
    if (fabsf(det) < 1e-9f || len2 <= 1e-12f) return 0;
    q1 = -r1x*by+r1y*bx;
    q4 = -r4x*dy+r4y*(dx-0.06f);
    jx1 = r4y*q1/det; jy1 = -r4x*q1/det;
    jx4 = -r1y*q4/det; jy4 = r1x*q4/det;
    result.length = sqrtf(len2); result.angle = atan2f(cy,rx);
    /* 将足端的笛卡尔导数投影为腿长/摆角导数；乘关节角速度即可得虚拟腿速度。 */
    result.jacobian[0] = (rx*jx1+cy*jy1)/result.length;
    result.jacobian[1] = (rx*jx4+cy*jy4)/result.length;
    result.jacobian[2] = (rx*jy1-cy*jx1)/len2;
    result.jacobian[3] = (rx*jy4-cy*jx4)/len2;
    *out = result;
    return 1;
}
void leg_pos(float phi1, float phi4, float pos[2])
{
    LegModel m;
    if (!leg_model(phi1,phi4,LEG_BRANCH_MATLAB,&m)) {
        pos[0] = NAN; pos[1] = NAN; return;
    }
    pos[0] = m.length; pos[1] = m.angle;
}
