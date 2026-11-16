#ifndef BALANCE_EQUILIBRIUM_H
#define BALANCE_EQUILIBRIUM_H
#include "lqr_model_config.h"
#include "lqr_k.h"
#include <math.h>

/* 小角度静态平衡，与生成LQR的耦合重力/惯量模型使用相同坐标。
 * 非零phi参考不能简单配theta=0和Tp=0，否则停车会靠位置偏差抵消重力。
 * 返回目标世界腿倾角(rad)及每侧髋摆矩前馈(N*m)，轮矩前馈始终为0。
 * 这里只补模型内的静态重力，不能当作真实重心/Kt的实测标定。 */
static inline int BalanceEquilibrium_CalculateWithOffset(
    float length_m,float phi_rad,float offset_m,float *theta_rad,float *tp_nm)
{
    if(!theta_rad || !tp_nm) return 0;
    *theta_rad=*tp_nm=0;
    if(!isfinite(length_m) || length_m<LQR_LEG_MIN_M || length_m>LQR_LEG_MAX_M ||
       !isfinite(phi_rad) || !isfinite(offset_m)) return 0;
    float moment_arm=(LQR_BODY_MASS_PER_SIDE_KG+0.5f*LQR_LEG_MASS_PER_SIDE_KG)*length_m;
    if(!isfinite(moment_arm) || moment_arm<=0) return 0;
    /* Body: 0=m*g*h*phi-m*g*b+Tp; leg: 0=(m+ml/2)*g*L*theta+Tp.
     * The same applied offset b must be used in both equations, including
     * while adaptive phi is slewing. Positive b is forward of the body axis. */
    float gravity_arm=LQR_BODY_COM_HEIGHT_M*phi_rad-offset_m;
    *theta_rad=LQR_BODY_MASS_PER_SIDE_KG*gravity_arm/moment_arm;
    *tp_nm=-LQR_BODY_MASS_PER_SIDE_KG*LQR_GRAVITY_M_S2*gravity_arm;
    return isfinite(*theta_rad) && isfinite(*tp_nm);
}
static inline int BalanceEquilibrium_Calculate(
    float length_m,float phi_rad,float *theta_rad,float *tp_nm)
{
    return BalanceEquilibrium_CalculateWithOffset(length_m,phi_rad,0,theta_rad,tp_nm);
}
#endif
