#include "robot_motion_command.h"
#include <math.h>
#include <string.h>

#define MOTION_TIMEOUT_MS 100U /* Same fresh-frame requirement as SBUS receiver. */
#define MOTION_MAX_PERIOD_MS 20U

const RobotMotionConfig g_robot_motion_defaults = {
    .enable=0U, .max_forward_m_s=.40f, .max_turn_rad_s=3.00f,
    .acceleration_m_s2=.80f, .turn_acceleration_rad_s2=4.00f
};

void RobotMotion_Clear(RobotMotionCommand *command)
{
    if (command) memset(command,0,sizeof(*command));
}

static void reject(RobotMotionCommand *command,RobotMotionStatus reason)
{
    RobotMotion_Clear(command);command->status=reason;
}

static float slew(float current,float target,float step)
{
    return current+fmaxf(-step,fminf(step,target-current));
}

/* out 保存上一拍目标以计算斜率；任何失控或权限失效都立即清零，不走减速斜坡。 */
void RobotMotion_Update(const RobotMotionConfig *c,const SbusSticks *s,
    const RobotMotionPermission *p,uint32_t now_ms,uint32_t period_ms,
    RobotMotionCommand *out)
{
    if (!out) return;
    if (!c || c->enable>1U) { reject(out,ROBOT_MOTION_BAD_PARAMETERS);return; }
    if (!c->enable) { reject(out,ROBOT_MOTION_OFF);return; }
    if (!p || p->remote_enabled>1U || p->ch5_on>1U ||
        p->controller_on>1U || p->normal_balance>1U ||
        !period_ms || period_ms>MOTION_MAX_PERIOD_MS ||
        !isfinite(c->max_forward_m_s) || c->max_forward_m_s<0 || c->max_forward_m_s>.40f ||
        !isfinite(c->max_turn_rad_s) || c->max_turn_rad_s<0 || c->max_turn_rad_s>3.00f ||
        !isfinite(c->acceleration_m_s2) || c->acceleration_m_s2<=0 || c->acceleration_m_s2>.80f ||
        !isfinite(c->turn_acceleration_rad_s2) || c->turn_acceleration_rad_s2<=0 || c->turn_acceleration_rad_s2>4.00f) {
        reject(out,ROBOT_MOTION_BAD_PARAMETERS);return;
    }
    if (!s || s->valid!=1U || (uint32_t)(now_ms-s->received_ms)>MOTION_TIMEOUT_MS ||
        !isfinite(s->forward) || !isfinite(s->yaw) ||
        fabsf(s->forward)>1 || fabsf(s->yaw)>1) {
        reject(out,ROBOT_MOTION_REMOTE_INVALID);return;
    }
    if (s->calibrated!=1U) { reject(out,ROBOT_MOTION_UNCALIBRATED);return; }
    if (!p->remote_enabled || !p->ch5_on || !p->controller_on || !p->normal_balance) {
        reject(out,ROBOT_MOTION_NOT_READY);return;
    }
    /* A previous invalid command must not inject NaN or an out-of-range
     * target into a later valid frame. The caller must initialize its state. */
    if (!isfinite(out->forward_m_s) || !isfinite(out->turn_rad_s) ||
        fabsf(out->forward_m_s)>c->max_forward_m_s || fabsf(out->turn_rad_s)>c->max_turn_rad_s)
        RobotMotion_Clear(out);
    out->status=ROBOT_MOTION_TARGET_READY;out->source_frame_count=s->frame_count;
    out->forward_axis=s->forward;out->turn_axis=s->yaw;
    out->requested_forward_m_s=s->forward*c->max_forward_m_s;
    out->requested_turn_rad_s=s->yaw*c->max_turn_rad_s;
    float dt=period_ms*.001f;
    /* 前后反向时先以两倍加速度刹到零；本拍剩余步长不用于反向加速。 */
    float target=out->requested_forward_m_s;
    unsigned reversing=out->forward_m_s*target<0;
    unsigned braking=reversing || fabsf(target)<fabsf(out->forward_m_s);
    out->forward_m_s=slew(out->forward_m_s,reversing ? 0 : target,
        c->acceleration_m_s2*(braking ? 2.0f : 1.0f)*dt);
    out->turn_rad_s=slew(out->turn_rad_s,out->requested_turn_rad_s,c->turn_acceleration_rad_s2*dt);
}
