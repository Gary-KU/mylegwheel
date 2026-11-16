#ifndef BALANCE_BIAS_OBSERVER_H
#define BALANCE_BIAS_OBSERVER_H
#include <stdint.h>
#include <math.h>
#include "lqr_model_config.h"

/* Effective fore/aft gravity offset, not a measured3D centre of mass.
 * Uses actual virtual hip torque and measured accelerations. Positive phi
 * is backward body tilt; a negative learned offset requests forward lean. */
typedef struct {
    float offset_m, phi_reference_rad;
    float previous_velocity[3], acceleration[3], torque_filtered_nm;
    uint32_t initialized, samples;
} BalanceBiasState;
static inline float BalanceBias_Clamp(float value,float limit) {
    return fmaxf(-limit,fminf(limit,value));
}
static inline void BalanceBias_ResetMotion(BalanceBiasState *s) {
    s->initialized=0U;
    for(unsigned i=0;i<3;++i)s->previous_velocity[i]=s->acceleration[i]=0;
    s->torque_filtered_nm=0;
}
static inline uint32_t BalanceBias_Update(BalanceBiasState *s,const float x[6],
        float length,float actual_tp,float dt,uint32_t learn) {
    if(!s || !x || !isfinite(length) || length<.04f || length>.15f ||
       !isfinite(actual_tp) || !isfinite(dt) || dt<=0 || dt>.01f) return 0U;
    for(unsigned i=0;i<6;++i)if(!isfinite(x[i]))return 0U;
    const float velocity[3]={x[3],x[1],x[5]};
    if(!s->initialized) {
        for(unsigned i=0;i<3;++i)s->previous_velocity[i]=velocity[i];
        s->torque_filtered_nm=actual_tp;s->initialized=1U;return 0U;
    }
    float alpha=dt/(.03f+dt);
    for(unsigned i=0;i<3;++i) {
        float a=(velocity[i]-s->previous_velocity[i])/dt;
        s->acceleration[i]+=alpha*(a-s->acceleration[i]);
        s->previous_velocity[i]=velocity[i];
    }
    s->torque_filtered_nm+=alpha*(actual_tp-s->torque_filtered_nm);
    const float h=LQR_BODY_COM_HEIGHT_M,m=LQR_BODY_MASS_PER_SIDE_KG,g=LQR_GRAVITY_M_S2;
    const float inertia=LQR_BODY_PITCH_INERTIA_CG+m*h*h;
    uint32_t learning=learn && fabsf(s->acceleration[0])<.5f &&
        fabsf(s->acceleration[1])<3 && fabsf(s->acceleration[2])<3;
    if(learning) {
        /* Body equation: -m*h*xdd-m*L*h*thetadd+Ihip*phidd
         * =m*g*h*phi-m*g*offset+Tp. All input torque here is measured. */
        float sample=h*x[4]+s->torque_filtered_nm/(m*g)+h/g*s->acceleration[0]+
            length*h/g*s->acceleration[1]-inertia/(m*g)*s->acceleration[2];
        sample=BalanceBias_Clamp(sample,h*.052407779f); /* tan3deg: reject hand/support transients */
        s->offset_m+=dt/(.5f+dt)*(sample-s->offset_m);
        ++s->samples;
    }
    if(learning) {
        /* Freeze the proposal as well as the estimate when learning is
         * disallowed. Actual applied correction is a separate task state. */
        float target=BalanceBias_Clamp(s->offset_m/h,.034906585f);
        s->phi_reference_rad+=BalanceBias_Clamp(target-s->phi_reference_rad,.004363323f*dt); /* 0.25deg/s */
        s->phi_reference_rad=BalanceBias_Clamp(s->phi_reference_rad,.034906585f); /* max 2deg */
    }
    return learning;
}
#endif
