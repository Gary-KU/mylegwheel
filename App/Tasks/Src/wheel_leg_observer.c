/* 轮腿观测：反馈一致性 -> 五连杆几何 -> 状态/里程与诊断 LQR。
 * 内部历史仅由轮腿任务更新，电机输出与启停状态由控制模块管理。 */
#include "wheel_leg_observer.h"
#include "wheel_leg_config.h"
#include "wheel_leg_runtime.h"
#include "can_task.h"
#include "imu_task.h"
#include "joint_calibration.h"
#include "motor_foc_config.h"
#if MYLEGWHEEL_A1_CURRENT
#include "motor_foc.h"
#endif
#include "lqr_k.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define FEEDBACK_MAX_AGE_MS 10U
#define FEEDBACK_MAX_SKEW_MS 5U
#define DEG_TO_RAD 0.017453292519943295f

static uint32_t last_update;
static uint8_t initialized;
static float position[2], previous_speed[2];
static uint8_t previous_valid[2];
static float previous_leg_length[2];
static uint8_t previous_leg_length_valid[2];
volatile WheelLegPairDebug g_wheel_leg_pair_debug[2];
/* Audit envelopes are static, not added to the control task's stack or ring.
 * The sole observer owner consumes all six inside its normal feedback group. */
static MotorFeedbackHealth feedback_health[6], pending_feedback_health;
static uint32_t pending_feedback_tick, pending_feedback_detail, pending_feedback_age;

void WheelLeg_LatchFeedbackFailure(const WheelLegDebug *out)
{
#if MYLEGWHEEL_A1_CURRENT
    if(out && pending_feedback_health.valid && pending_feedback_tick==out->tick_ms &&
       pending_feedback_health.motor_id==out->feedback_motor_id &&
       pending_feedback_detail==out->feedback_detail && pending_feedback_age==out->feedback_ms)
        MotorFoc_LatchFeedbackFailure(&pending_feedback_health,out->tick_ms,
                                     out->feedback_detail,out->feedback_ms);
#else
    (void)out;
#endif
}

static void preview_control(WheelLegDebug *out)
{
    const float direction[2] = {WHEEL_LEFT_DIRECTION, WHEEL_RIGHT_DIRECTION};
    out->body_pitch_rate_rad_s = BODY_PITCH_DIRECTION * out->gyro_rad_s[BODY_PITCH_AXIS];
    for (unsigned side = 0U; side < 2U; ++side) {
        for (unsigned i = 0U; i < 6U; ++i)
            out->reference[side][i] = g_wheel_leg_reference[side][i];
        if (!out->imu_valid || !out->leg_valid[side] || !out->wheel_valid[side]) {
            previous_valid[side] = 0U;
            continue;
        }
        float *state = out->state[side];
        /* 状态顺序为 theta、dtheta、x、dx、phi、dphi，均用 SI 单位。
         * theta 为相对世界竖直的腿倾角；phi 与本工程“机身前倾为正”的定义相反。 */
        state[0] = out->leg[side].angle - 1.5707963267948966f + out->body_pitch_rad;
        state[1] = out->angle_speed_rad_s[side] + out->body_pitch_rate_rad_s;
        state[3] = WHEEL_RADIUS_M * direction[side] * out->wheel_speed_rad_s[side];
        /* Preview assumes direct drive and a stationary wheel carrier.
         * Restart the displacement origin after any missing feedback/timing gap. */
        if (previous_valid[side])
            position[side] += 0.0005f * (float)out->period_ms * (previous_speed[side] + state[3]);
        else position[side] = 0.0f;
        state[2] = position[side];
        state[4] = -out->body_pitch_rad;
        state[5] = -out->body_pitch_rate_rad_s;
        unsigned finite = 1U;
        for (unsigned i = 0U; i < 6U; ++i) finite &= (unsigned)(isfinite(state[i]) != 0);
        if (!finite) {
            previous_valid[side] = 0U;
            for (unsigned i = 0U; i < 6U; ++i) state[i] = 0.0f;
            continue;
        }
        previous_speed[side] = state[3];
        previous_valid[side] = 1U;
        out->state_valid[side] = 1U;
        out->lqr_valid[side] = (uint32_t)lqr_control(out->leg[side].length, state,
                                                   out->reference[side], out->lqr_torque_nm[side]);
    }
}

static void feedback_error(WheelLegDebug *out, uint32_t detail, uint32_t id, uint32_t ms)
{
    if (out->feedback_detail == WHEEL_LEG_FEEDBACK_OK) {
        out->feedback_detail = detail;
        out->feedback_motor_id = id;
        out->feedback_ms = ms;
    }
}

static int motor_valid(const MotorFeedback *f, const MotorFeedbackHealth *health,
                       uint32_t id, int joint, WheelLegDebug *out)
{
    uint32_t detail = WHEEL_LEG_FEEDBACK_OK;
    if (!f->online) detail = WHEEL_LEG_FEEDBACK_MOTOR;
    else if (f->age_ms > (f->protocol_a1 ? MOTOR_FOC_FAST_TIMEOUT_MS : FEEDBACK_MAX_AGE_MS)) detail = WHEEL_LEG_FEEDBACK_STALE;
    else if (joint && !f->joint_calibrated) detail = WHEEL_LEG_FEEDBACK_CALIBRATION;
    else if ((joint && (!isfinite(f->joint_angle_rad) || !isfinite(f->joint_speed_rad_s))) ||
             (!joint && !isfinite(f->speed_rad_s))) detail = WHEEL_LEG_FEEDBACK_NONFINITE;
    if (detail == WHEEL_LEG_FEEDBACK_OK) return 1;
    if(out->feedback_detail==WHEEL_LEG_FEEDBACK_OK && f->protocol_a1 &&
       health->valid && health->motor_id==id) {
        pending_feedback_health=*health;pending_feedback_tick=out->tick_ms;
        pending_feedback_detail=detail;pending_feedback_age=f->age_ms;
    }
    feedback_error(out, detail, id, f->age_ms);
    return 0;
}

static uint32_t feedback(MotorFeedback joint[4], MotorFeedback wheel[2])
{
    const uint8_t wheel_id[2] = {WHEEL_LEFT_CAN_ID, WHEEL_RIGHT_CAN_ID};
    taskENTER_CRITICAL();
    for (unsigned i = 0U; i < 4U; ++i) {
        joint[i].online = MotorCAN_GetFeedbackWithHealth((uint8_t)g_joint_calibration[i].motor_id,
                                                       &joint[i],&feedback_health[i]);
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        wheel[i].online = MotorCAN_GetFeedbackWithHealth(wheel_id[i], &wheel[i],&feedback_health[4U+i]);
    }
    uint32_t now=HAL_GetTick();
    for(unsigned i=0;i<4U;++i) if(joint[i].protocol_a1)
        joint[i].age_ms=now-joint[i].last_rx_ms;
    for(unsigned i=0;i<2U;++i) if(wheel[i].protocol_a1)
        wheel[i].age_ms=now-wheel[i].last_rx_ms;
    taskEXIT_CRITICAL();
    return now;
}

/* Select received samples only: never extrapolate or change their stamps.
 * The latest two channels must first pass all ordinary health checks. RX
 * history is already scoped to their current transport/unwrap/calibration.
 * Prefer the freshest oldest member, then total age and skew, rather than
 * an older perfectly aligned pair. */
static uint32_t feedback_pair(const MotorFeedback *front,const MotorFeedback *rear,
                             uint32_t now,uint32_t max_skew,MotorFeedback pair[2],unsigned side_index)
{
    volatile WheelLegPairDebug *debug=&g_wheel_leg_pair_debug[side_index];
    debug->last_tick_ms=now;
    debug->last_latest_age_ms[0]=front->age_ms;debug->last_latest_age_ms[1]=rear->age_ms;
    debug->last_history_count[0]=front->fast_history_count;debug->last_history_count[1]=rear->fast_history_count;
    uint32_t reject=0U;
    if(front->feedback_epoch!=rear->feedback_epoch) reject=1U;
    uint32_t counts[2]={front->fast_history_count,rear->fast_history_count};
    if(counts[0]>MOTOR_FAST_HISTORY_SAMPLES || counts[1]>MOTOR_FAST_HISTORY_SAMPLES) reject=2U;
    if(reject) {
        ++debug->reject_count;debug->last_reject_tick_ms=now;debug->last_reject_reason=reject;
        for(unsigned i=0;i<2U;++i) {
            debug->last_reject_latest_age_ms[i]=debug->last_latest_age_ms[i];
            debug->last_reject_history_count[i]=debug->last_history_count[i];
        }
        return 0U;
    }
    const MotorFeedback *latest[2]={front,rear};
    uint32_t found=0U,best_age=UINT32_MAX,best_sum=UINT32_MAX,best_skew=UINT32_MAX;
    unsigned chosen[2]={0U,0U};
    for(unsigned a=0;a<=counts[0];++a) for(unsigned b=0;b<=counts[1];++b) {
        unsigned indices[2]={a,b};uint32_t age[2];
        for(unsigned side=0;side<2U;++side) {
            const MotorFeedback *f=latest[side];
            uint32_t stamp=indices[side] ? f->fast_history[indices[side]-1U].last_rx_ms : f->last_rx_ms;
            age[side]=now-stamp;
        }
        if(age[0]>MOTOR_FOC_FAST_TIMEOUT_MS || age[1]>MOTOR_FOC_FAST_TIMEOUT_MS) continue;
        uint32_t skew=age[0]>age[1]?age[0]-age[1]:age[1]-age[0];
        if(skew>max_skew) continue;
        uint32_t oldest=age[0]>age[1]?age[0]:age[1],sum=age[0]+age[1];
        if(!found || oldest<best_age || (oldest==best_age &&
           (sum<best_sum || (sum==best_sum && skew<best_skew)))) {
            found=1U;best_age=oldest;best_sum=sum;best_skew=skew;chosen[0]=a;chosen[1]=b;
        }
    }
    if(!found) {
        ++debug->reject_count;debug->last_reject_tick_ms=now;debug->last_reject_reason=3U;
        for(unsigned i=0;i<2U;++i) {
            debug->last_reject_latest_age_ms[i]=debug->last_latest_age_ms[i];
            debug->last_reject_history_count[i]=debug->last_history_count[i];
        }
        return 0U;
    }
    uint32_t historical=0U;
    for(unsigned side=0;side<2U;++side) {
        pair[side]=*latest[side];
        if(chosen[side]) {
            const MotorFastSample *s=&latest[side]->fast_history[chosen[side]-1U];
            pair[side].last_rx_ms=s->last_rx_ms;pair[side].rx_count=s->rx_count;
            pair[side].angle_rad=s->angle_rad;pair[side].speed_rad_s=s->speed_rad_s;
            pair[side].angle_deg=s->angle_rad*57.2957795131f;
            pair[side].speed_rpm=s->speed_rad_s*9.549296586f;
            JointCalibration_Apply(&pair[side]);
        }
        pair[side].age_ms=now-pair[side].last_rx_ms;
        pair[side].angle_age_ms=pair[side].age_ms;
        debug->last_selected_age_ms[side]=pair[side].age_ms;
        debug->last_selected_rx_count[side]=pair[side].rx_count;
        if(pair[side].rx_count!=latest[side]->rx_count) historical=1U;
    }
    if(historical) ++debug->history_pair_count;else ++debug->latest_pair_count;
    return 1U;
}

void WheelLeg_Observe(WheelLegDebug *out)
{
    if (!out) return;
    pending_feedback_health.valid=0U;
    *out = (WheelLegDebug){0};
    MotorFeedback joint[4], wheel[2];
    uint8_t joint_valid[4];
    ImuSample imu;
    out->updates = g_wheel_leg.updates + 1U;
    out->stack_free_min_bytes = g_wheel_leg.stack_free_min_bytes;
    out->tick_ms = HAL_GetTick();
    out->period_ms = initialized ? out->tick_ms - last_update : WHEEL_LEG_CONTROL_PERIOD_MS;
    initialized = 1U;
    last_update = out->tick_ms;
    out->status = WHEEL_LEG_WAIT_FEEDBACK;
    if (out->period_ms == 0U || out->period_ms > 2U * WHEEL_LEG_CONTROL_PERIOD_MS) {
        out->status = WHEEL_LEG_BAD_TIMING;
        goto publish;
    }
    if (IMU_GetSample(&imu) && isfinite(imu.attitude_deg[BODY_PITCH_AXIS]) &&
        isfinite(imu.gyro_rad_s[0]) && isfinite(imu.gyro_rad_s[1]) && isfinite(imu.gyro_rad_s[2])) {
        out->imu_valid = 1U;
        out->body_pitch_rad = BODY_PITCH_DIRECTION * imu.attitude_deg[BODY_PITCH_AXIS] * DEG_TO_RAD;
        for (unsigned axis = 0U; axis < 3U; ++axis) out->gyro_rad_s[axis] = imu.gyro_rad_s[axis];
    } else feedback_error(out, WHEEL_LEG_FEEDBACK_IMU, 0U, 0U);

    uint32_t feedback_now=feedback(joint, wheel);
    for (unsigned i = 0U; i < 4U; ++i) {
        joint_valid[i] = (uint8_t)motor_valid(&joint[i], &feedback_health[i],g_joint_calibration[i].motor_id, 1, out);
        out->joint_feedback_valid[i]=joint_valid[i];
        out->joint_feedback_age_ms[i]=joint[i].age_ms;
        out->joint_angle_rad[i]=joint_valid[i] ? joint[i].joint_angle_rad : NAN;
        out->joint_shaft_speed_rad_s[i]=joint_valid[i] ? joint[i].speed_rad_s : NAN;
    }
    for (unsigned side = 0U; side < 2U; ++side) {
        const MotorFeedback *front = &joint[side * 2U], *rear = front + 1;
        MotorFeedback retained[2];
        uint32_t wheel_id = side == 0U ? WHEEL_LEFT_CAN_ID : WHEEL_RIGHT_CAN_ID;
        LegModel leg;
        float length_speed, angle_speed;
        uint32_t skew;
        out->wheel_valid[side] = (uint32_t)motor_valid(&wheel[side], &feedback_health[4U+side],wheel_id, 0, out);
        if (out->wheel_valid[side]) out->wheel_speed_rad_s[side] = wheel[side].speed_rad_s;
        if (!joint_valid[side * 2U] || !joint_valid[side * 2U + 1U]) {
            continue;
        }
        /* 同一条腿的前后关节需在时限内对齐；左右两腿不要求同一反馈时刻。 */
        skew = front->age_ms > rear->age_ms ? front->age_ms - rear->age_ms : rear->age_ms - front->age_ms;
        /* Independent A1 FAST senders run every 5 ms; receive timestamps
         * have 1 ms resolution. Allow one tick of boundary jitter only.
         * Per-joint 10 ms freshness and all fault checks remain enforced. */
        uint32_t max_skew = FEEDBACK_MAX_SKEW_MS +
                            ((front->protocol_a1 && rear->protocol_a1) ? 1U : 0U);
        if (front->protocol_a1 && rear->protocol_a1) {
            if (!feedback_pair(front,rear,feedback_now,max_skew,retained,side)) {
                feedback_error(out, WHEEL_LEG_FEEDBACK_SKEW, 0U, skew);
                continue;
            }
            front=&retained[0];rear=&retained[1];
            for (unsigned j=0U;j<2U;++j) {
                const MotorFeedback *sample=&retained[j];
                unsigned index=2U*side+j;
                out->joint_feedback_age_ms[index]=sample->age_ms;
                out->joint_angle_rad[index]=sample->joint_angle_rad;
                out->joint_shaft_speed_rad_s[index]=sample->speed_rad_s;
            }
        } else if(skew>max_skew) {
            feedback_error(out,WHEEL_LEG_FEEDBACK_SKEW,0U,skew);
            continue;
        }
        LegBranch branch = side == 0U ? LEFT_LEG_BRANCH : RIGHT_LEG_BRANCH;
        if (!leg_model(rear->joint_angle_rad, front->joint_angle_rad, branch, &leg)) {
            out->status = WHEEL_LEG_BAD_GEOMETRY;
            continue;
        }
        length_speed = leg.jacobian[0] * rear->joint_speed_rad_s + leg.jacobian[1] * front->joint_speed_rad_s;
        angle_speed = leg.jacobian[2] * rear->joint_speed_rad_s + leg.jacobian[3] * front->joint_speed_rad_s;
        if (!isfinite(length_speed) || !isfinite(angle_speed)) {
            out->status = WHEEL_LEG_BAD_GEOMETRY;
            continue;
        }
        out->leg[side] = leg;
        out->length_speed_m_s[side] = length_speed;
        out->angle_speed_rad_s[side] = angle_speed;
        out->leg_valid[side] = 1U;
        out->joint_speed_rad_s[side][0] = rear->joint_speed_rad_s;
        out->joint_speed_rad_s[side][1] = front->joint_speed_rad_s;
    }
    if (out->status != WHEEL_LEG_BAD_GEOMETRY) {
        uint32_t valid = out->imu_valid + out->leg_valid[0] + out->leg_valid[1] +
                         out->wheel_valid[0] + out->wheel_valid[1];
        out->status = valid == 5U ? WHEEL_LEG_OBSERVE :
                     valid != 0U ? WHEEL_LEG_OBSERVE_PARTIAL : WHEEL_LEG_WAIT_FEEDBACK;
    }
publish:
    /* 无效样本也更新观测历史；调用方继续执行停机路径。 */
    for (unsigned side=0;side<2;++side) {
        if (out->leg_valid[side] && previous_leg_length_valid[side]) {
            float speed=(out->leg[side].length-previous_leg_length[side])/
                        ((float)out->period_ms*.001f);
            if (isfinite(speed)) {
                out->length_speed_from_position_m_s[side]=speed;
                out->length_speed_position_valid[side]=1U;
            }
        }
        previous_leg_length_valid[side]=(uint8_t)out->leg_valid[side];
        previous_leg_length[side]=out->leg[side].length;
    }
    preview_control(out);
}
