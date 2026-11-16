/* A1 电流后端：每个节点独立完成 STOP 确认、参数读回，再接受轴端力矩请求。 */
#include "motor_foc.h"
#include "bsp_fdcan.h"
#include "bsp_fdcan_deadline.h"
#include "joint_calibration.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define TAU 6.28318530718f
#define READY_FLAGS 0x3eU
#define BAD_DIAGNOSTIC 0x2fU /* bus-off,passive,RX loss,require STOP,uninitialized */
typedef struct {
    uint32_t sent_ms, next_ms, stop_status_count, stop_diag_count;
    uint32_t pending, got_ack, got_parameter, got_calibration, opcode, sequence, parameter;
    uint32_t parameter_value_bits;
    uint32_t stopped_ack, stopped_required, aligning, align_started_ms;
    uint8_t next_sequence;
    float previous_angle;
    uint32_t angle_seeded;
    uint32_t debug_running, debug_until_ms;
    int debug_current_ma;
    uint32_t current_ack_seen, current_ack_status_count, fast_seeded, was_active;
    float previous_fast_angle;
} FocSession;
static FocSession session[8];
static uint32_t last_motion_ms,motion_ever_sent;
volatile MotorFocDebug g_motor_foc;
volatile MotorFocCommandTiming g_motor_foc_command_timing[8];
volatile MotorFocDqDebug g_motor_foc_dq[8];
volatile MotorFocDqTraceDebug g_motor_foc_dq_trace;
volatile MotorFocFailureDebug g_motor_foc_failure;
volatile MotorFocFeedbackFailureDebug g_motor_foc_feedback_failure;
static uint32_t dq_trace_active, dq_trace_tick;
static uint32_t failure_active_mask, failure_run_active, failure_events;

typedef struct {
    MotorFastSample samples[MOTOR_FAST_HISTORY_SAMPLES];
    uint32_t count, generation, epoch, configuration_valid;
    float anchor_rad;
    uint32_t anchor_confirmed, timing_confirmed;
} FastHistory;
static FastHistory fast_history[8];

static void fast_history_clear(unsigned slot) {
    fast_history[slot].count=0U;
    ++fast_history[slot].generation;
}
static uint32_t fast_history_configuration(const FastHistory *h,const MotorFocCalibration *c) {
    return h->configuration_valid && h->anchor_rad==c->angle_anchor_rad &&
        h->anchor_confirmed==c->angle_anchor_confirmed &&
        h->timing_confirmed==c->joint_timing_confirmed;
}

static void capture_failure(uint32_t reason,unsigned slot) {
    taskENTER_CRITICAL();
    ++failure_events;
    if(!g_motor_foc_failure.valid) {
        uint32_t now=HAL_GetTick();MotorFocFailureDebug f={0};
        f.reason=reason;f.node_id=slot<8U?slot+1U:0U;f.tick_ms=now;
        f.active_mask=failure_active_mask;f.transport_epoch=g_motor_foc.transport_epoch;
        f.pending_age_ms=f.ack_age_ms=f.status_age_ms=f.diagnostic_age_ms=f.fast_age_ms=UINT32_MAX;
        f.tx_count=g_motor_foc.tx_count;f.tx_failed=g_motor_foc.tx_failed;
        f.deadline_expired_count=g_fdcan_deadline.expired_count;
        f.deadline_invalid_buffer_count=g_fdcan_deadline.invalid_buffer_count;
        f.communication_blocked=CAN_CommunicationBlocked();
        if(slot<8U) {
            const volatile MotorFocNode *n=&g_motor_foc.node[slot];const FocSession *s=&session[slot];
            f.node_epoch=n->epoch;f.session_ready=n->session_ready;f.inhibit=n->inhibit;
            f.pending=s->pending;f.opcode=s->opcode;f.sequence=s->sequence;f.got_ack=s->got_ack;
            if(s->pending) f.pending_age_ms=now-s->sent_ms;
            f.last_opcode=n->last_opcode;f.last_sequence=n->last_sequence;f.last_result=n->last_result;
            if(n->ack_count) f.ack_age_ms=now-n->ack_ms;
            f.mode=n->mode;f.flags=n->flags;f.diagnostic_flags=n->diagnostic_flags;
            f.faults=n->faults;f.board_faults=n->board_faults;
            if(n->status_count) f.status_age_ms=now-n->status_ms;
            if(n->diagnostic_count) f.diagnostic_age_ms=now-n->diagnostic_ms;
            if(n->fast_count) f.fast_age_ms=now-n->fast_ms;
            f.fast_valid=n->fast_valid;f.fast_speed_clipped=n->fast_speed_clipped;
            f.timeout_count=n->timeout_count;f.rejected_count=n->rejected_count;
        }
        g_motor_foc_failure=f;g_motor_foc_failure.valid=1U; /* Publish last. */
    }
    taskEXIT_CRITICAL();
}

/* Recheck at the actual enqueue boundary: RX can revoke a session after
 * the initial whole-group validation, including a clipped then normal FAST. */
static uint32_t active_send_failure(unsigned slot,uint32_t epoch,uint32_t events) {
    const volatile MotorFocNode *n=&g_motor_foc.node[slot];
    if(g_motor_foc.transport_epoch!=epoch || n->epoch!=epoch)
        return MOTOR_FOC_FAILURE_TRANSPORT_EPOCH;
    if(n->fast_speed_clipped) return MOTOR_FOC_FAILURE_SPEED_CLIPPED;
    if(n->inhibit || !n->session_ready || failure_events!=events)
        return MOTOR_FOC_FAILURE_SESSION;
    return MOTOR_FOC_FAILURE_NONE;
}

static float current_hard_limit(unsigned slot) {
    return (MOTOR_FOC_JOINT_NODE_MASK & (1U<<slot)) ?
        MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A : MOTOR_FOC_CURRENT_HARD_LIMIT_A;
}
static int fixed_iq_limit_ready(unsigned slot,float remote_limit) {
    const float required=MOTOR_FOC_FIXED_IQ_LIMIT_A;
    /* Zero is reserved for historical host fixtures. A production setting
     * must be confirmed by the ordinary STOP parameter exchange. */
    return required==0.0f || (isfinite(required) && required>=.01f &&
        required<=current_hard_limit(slot) && required<=MOTOR_FOC_REMOTE_IQ_LIMIT_MAX_A &&
        isfinite(remote_limit) && remote_limit==required);
}
static int selected(unsigned slot) { return (MOTOR_FOC_NODE_MASK & (1U << slot)) != 0U; }
static int32_t i16(const uint8_t *p) {
    uint32_t u=(uint32_t)p[0]|((uint32_t)p[1]<<8);
    return u<32768U ? (int32_t)u : (int32_t)u-65536;
}
static uint32_t u32(const uint8_t *p) {
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static int fresh(uint32_t now,uint32_t stamp,uint32_t count,uint32_t age) {
    return count!=0U && (uint32_t)(now-stamp)<=age;
}
static void dq_trace_capture(uint32_t now,uint32_t mask) {
    static const unsigned slots[6]={0,1,2,4,5,6};
    if(!mask) {dq_trace_active=0U;return;}
    if(!dq_trace_active) {
        g_motor_foc_dq_trace.next=g_motor_foc_dq_trace.count=0U;
        dq_trace_tick=now-40U;
    }
    dq_trace_active=1U;
    if((uint32_t)(now-dq_trace_tick)<40U) return;
    dq_trace_tick=now;
    MotorFocDqTraceRow row={0};row.tick_ms=now;row.active_mask=mask;
    taskENTER_CRITICAL();
    /* RX can advance while commands are queued. Timestamp the coherent
     * telemetry snapshot here so an intervening RX cannot have negative age. */
    now=HAL_GetTick();row.tick_ms=now;
    for(unsigned i=0;i<6;++i) {
        unsigned slot=slots[i];
        const volatile MotorFocNode *n=&g_motor_foc.node[slot];
        const volatile MotorFocDqDebug *d=&g_motor_foc_dq[slot];
        uint32_t age0=(d->seen&1U)?now-d->page0_ms:UINT32_MAX;
        uint32_t age1=(d->seen&2U)?now-d->page1_ms:UINT32_MAX;
        row.page_age_ms[i][0]=(uint16_t)(age0>65535U?65535U:age0);
        row.page_age_ms[i][1]=(uint16_t)(age1>65535U?65535U:age1);
        /* Retained node commands may belong to an earlier run. A request is
         * this run's only after its first successful IQ enqueue, in this epoch. */
        uint32_t current_epoch=n->epoch==g_motor_foc.transport_epoch;
        row.requested_iq_a[i]=(mask&(1U<<slot)) && current_epoch &&
            session[slot].was_active && g_motor_foc_command_timing[slot].count ? n->requested_iq_a:NAN;
        row.actual_iq_a[i]=current_epoch && n->fast_valid && fresh(now,n->fast_ms,n->fast_count,MOTOR_FOC_FAST_TIMEOUT_MS)?n->fast_iq_a:NAN;
        row.reference_iq_a[i]=age0<=100U?d->iq_reference_a:NAN;
        row.id_a[i]=age0<=100U?d->id_a:NAN;
        row.vd_v[i]=age0<=100U?d->vd_v:NAN;
        row.vq_v[i]=age1<=100U?d->vq_v:NAN;
    }
    unsigned next=g_motor_foc_dq_trace.next;
    g_motor_foc_dq_trace.rows[next]=row;
    g_motor_foc_dq_trace.next=(next+1U)%MOTOR_FOC_DQ_TRACE_ROWS;
    if(g_motor_foc_dq_trace.count<MOTOR_FOC_DQ_TRACE_ROWS) ++g_motor_foc_dq_trace.count;
    ++g_motor_foc_dq_trace.sequence;
    taskEXIT_CRITICAL();
}
static int calibration_opcode(uint32_t opcode) {
    return opcode==0x15U || opcode==0x16U;
}
static int calibration_result_valid(uint32_t result) {
    return result<=13U || (result>=0x80U && result<=0x87U);
}
static void calibration_completed(unsigned slot) {
    const FocSession *s=&session[slot];
    if(s->pending && calibration_opcode(s->opcode) && s->got_ack && s->got_calibration)
        g_motor_foc.stored_calibration[slot].completed=1U;
}
/* 会话失效同时撤销运动和维护请求；恢复通信后必须重新握手，不能重放旧目标。 */
static void invalidate(unsigned slot,uint32_t reason) {
    if(reason && (session[slot].was_active || (failure_active_mask&(1U<<slot))))
        capture_failure(reason,slot);
    volatile MotorFocNode *n=&g_motor_foc.node[slot];
    fast_history_clear(slot);
    n->session_ready=n->limit_verified=n->voltage_verified=n->slew_verified=0U;n->inhibit=1U;
    g_motor_foc_dq[slot].seen=0U;
    session[slot].pending=session[slot].stopped_ack=session[slot].aligning=0U;
    session[slot].stopped_required=1U;
    session[slot].debug_running=0U;
    session[slot].current_ack_seen=0U;
    session[slot].was_active=0U;
    g_motor_foc.debug_request_mask&=~(1U<<slot);
    g_motor_foc.maintenance_opcode[slot]=0U;
}
void MotorFoc_Init(void) {
    static const int direction[8]=MOTOR_FOC_DEFAULT_DIRECTIONS;
    memset((void *)&g_motor_foc,0,sizeof(g_motor_foc));
    memset((void *)&g_motor_foc_failure,0,sizeof(g_motor_foc_failure));
    memset((void *)&g_motor_foc_feedback_failure,0,sizeof(g_motor_foc_feedback_failure));
    failure_active_mask=failure_run_active=failure_events=0U;
    memset((void *)g_motor_foc_dq,0,sizeof(g_motor_foc_dq));
    memset((void *)&g_motor_foc_dq_trace,0,sizeof(g_motor_foc_dq_trace));
    dq_trace_active=dq_trace_tick=0U;
    memset(session,0,sizeof(session));
    memset(fast_history,0,sizeof(fast_history));
    memset((void *)g_motor_foc_command_timing,0,sizeof(g_motor_foc_command_timing));
    last_motion_ms=motion_ever_sent=0U;
    FdcanDeadline_Init();
    for(unsigned i=0;i<8;++i) {
        g_motor_foc.calibration[i].kt_nm_per_a=MOTOR_FOC_DEFAULT_KT_NM_PER_A;
        g_motor_foc.calibration[i].current_limit_a=MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A;
        g_motor_foc.calibration[i].direction=direction[i];
        g_motor_foc.calibration[i].calibration_confirmed=MOTOR_FOC_DEFAULT_CALIBRATION_CONFIRMED;
        session[i].stopped_required=1U;
        g_motor_foc.debug_duration_ms[i]=200U;
        if(MOTOR_FOC_JOINT_NODE_MASK&(1U<<i)) {
            g_motor_foc.calibration[i].kt_nm_per_a=MOTOR_FOC_JOINT_KT_NM_PER_A;
            g_motor_foc.calibration[i].current_limit_a=MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A;
            g_motor_foc.calibration[i].calibration_confirmed=1U;
        } else if(i==2U || i==6U) {
            g_motor_foc.calibration[i].kt_nm_per_a=MOTOR_FOC_WHEEL_ESTIMATED_KT_NM_PER_A;
        }
        if ((MOTOR_FOC_NODE_MASK & MOTOR_FOC_COMMISSIONED_RESPONSE_MASK) & (1U<<i)) {
            g_motor_foc.calibration[i].calibration_confirmed=1U;
            g_motor_foc.calibration[i].minimum_iq_slew_a_per_s=MOTOR_FOC_COMMISSIONED_MIN_IQ_SLEW_A_PER_S;
            g_motor_foc.calibration[i].dynamic_response_confirmed=1U;
        }
    }
    /* Load the commissioned mechanical reference before the first FAST frame.
     * The anchor selects its continuous turn; JointCalibration_Apply remains
     * the sole encoder-to-model angle transform. Table order is not a CAN slot.
     */
    for(unsigned j=0;j<JOINT_CALIBRATION_COUNT;++j) {
        const JointCalibration *joint=&g_joint_calibration[j];
        if(joint->motor_id<1U || joint->motor_id>8U ||
           !isfinite(joint->encoder_reference_rad) || !isfinite(joint->model_reference_rad) ||
           !isfinite(joint->ratio) || joint->ratio<=0 ||
           (joint->direction!=1 && joint->direction!=-1)) continue;
        unsigned slot=joint->motor_id-1U,bit=1U<<slot;
        if(!selected(slot) || !(MOTOR_FOC_JOINT_NODE_MASK&bit)) continue;
        volatile MotorFocCalibration *cal=&g_motor_foc.calibration[slot];
        cal->angle_anchor_rad=joint->encoder_reference_rad;
        cal->angle_anchor_confirmed=(MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK&bit)!=0U;
        cal->joint_timing_confirmed=(MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK&bit)!=0U;
    }
}
void MotorFoc_InvalidateFromISR(void) { ++g_motor_foc.transport_epoch; }

void MotorFoc_OnTransportDown(void) {
    taskENTER_CRITICAL();
    ++g_motor_foc.transport_epoch;
    for(unsigned i=0;i<8;++i) if(selected(i)) invalidate(i,MOTOR_FOC_FAILURE_TRANSPORT_EPOCH);
    g_motor_foc.debug_request_mask=0U;
    taskEXIT_CRITICAL();
}

uint32_t MotorFoc_TransportMotionStopped(void) {
    uint32_t stopped=1U;
    taskENTER_CRITICAL();
    if(g_motor_foc.debug_request_mask ||
       (motion_ever_sent && (uint32_t)(HAL_GetTick()-last_motion_ms)<200U)) stopped=0U;
    for(unsigned i=0;i<8;++i) if(selected(i) && (session[i].was_active || session[i].debug_running ||
       session[i].aligning || g_motor_foc.maintenance_opcode[i]==4U ||
       (session[i].pending && (session[i].opcode==1U || session[i].opcode==4U)))) stopped=0U;
    taskEXIT_CRITICAL();return stopped;
}

uint32_t MotorFoc_OnFrame(uint32_t id,const uint8_t data[8],uint32_t now) {
    unsigned slot=8;uint32_t kind=0;
    /* 各类反馈 ID = 基址 + 电机 ID；内部 slot = ID - 1。 */
    const uint32_t bases[7]={0x180U,0x280U,0x380U,0x480U,0x580U,0x700U,0x640U};
    for(unsigned k=0;k<7;++k) if(id>bases[k] && id<=bases[k]+8U) {slot=id-bases[k]-1U;kind=k;break;}
    if(slot>=8 || !selected(slot)) return 0U;
    if(data[0]!=0xa1U) {
        ++g_motor_foc.malformed;
        if(kind==5U) {
            taskENTER_CRITICAL();
            g_motor_foc.node[slot].fast_seen=1U;g_motor_foc.node[slot].fast_valid=0U;
            fast_history_clear(slot);session[slot].fast_seeded=0U;
            taskEXIT_CRITICAL();
        }
        return 1U;
    }
    taskENTER_CRITICAL();
    volatile MotorFocNode *n=&g_motor_foc.node[slot];FocSession *s=&session[slot];
    /* 总线故障会推进 epoch；新会话不能沿用故障前的计数、解圈状态或就绪标志。 */
    if(n->epoch!=g_motor_foc.transport_epoch) {
        invalidate(slot,MOTOR_FOC_FAILURE_TRANSPORT_EPOCH);s->angle_seeded=s->fast_seeded=0U;n->status_count=n->diagnostic_count=n->angle_count=n->fast_count=0U;n->fast_valid=0U;
        n->epoch=g_motor_foc.transport_epoch;
    }
    if(kind==0U) {
        n->mode=data[1];n->flags=data[2];n->iq_a=(float)i16(data+4)*.001f;
        n->speed_rad_s=(float)i16(data+6)*.001f;n->status_ms=now;++n->status_count;
        if(!(n->flags&8U)) fast_history_clear(slot);
    } else if(kind==1U) {
        uint32_t revoke=data[7]&(1U|8U|32U);
        n->faults=u32(data+2);n->board_faults=data[6];n->diagnostic_flags=data[7];
        n->diagnostic_ms=now;++n->diagnostic_count;
        if(n->faults || n->board_faults || (n->diagnostic_flags&0x2fU)) fast_history_clear(slot);
        if(revoke && (n->session_ready || n->limit_verified || n->voltage_verified || n->slew_verified) &&
           !(s->pending && s->opcode==0U)) invalidate(slot,MOTOR_FOC_FAILURE_DRIVER_HEALTH);
    } else if(kind==2U) {
        ++n->ack_count;n->ack_ms=now;n->last_opcode=data[1];n->last_sequence=data[2];n->last_result=data[3];
        /* 操作码和序号必须同时匹配，迟到的上一条 ACK 不能完成当前命令。 */
        if(s->pending && data[1]==s->opcode && data[2]==s->sequence) {
            if(calibration_opcode(s->opcode)) {
                volatile MotorFocStoredCalibration *stored=&g_motor_foc.stored_calibration[slot];
                ++stored->ack_count;stored->ack_ms=now;stored->ack_result=data[3];
            }
            if(data[3]!=0U || (s->opcode!=0U && u32(data+4)!=0U)) {
                ++n->rejected_count;invalidate(slot,MOTOR_FOC_FAILURE_ACK_REJECTED);
            } else {
                s->got_ack=1U;
                if(s->opcode==1U && !s->current_ack_seen) {
                    s->current_ack_seen=1U;s->current_ack_status_count=n->status_count;
                }
                calibration_completed(slot);
            }
        }
    } else if(kind==3U) {
        if(s->pending && (s->opcode==0x10U || s->opcode==0x11U) && data[1]==s->parameter && data[2]==s->sequence) {
            uint32_t bits=u32(data+4);float limit;memcpy(&limit,&bits,sizeof(limit));
            float voltage_cap=(slot==2U || slot==6U)?MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V:5.0f;
            float cap=s->parameter==3U?MOTOR_FOC_REMOTE_IQ_LIMIT_MAX_A:s->parameter==4U?voltage_cap:10.0f;
            if(data[3]!=0U || !isfinite(limit) || limit<=0 || limit>cap ||
               (s->opcode==0x11U && ((s->parameter!=3U && s->parameter!=5U) ||
                                    bits!=s->parameter_value_bits))) {
                ++n->rejected_count;invalidate(slot,MOTOR_FOC_FAILURE_CALIBRATION_LIMIT);
            } else {
                if(s->parameter==3U) n->remote_iq_limit_a=limit;
                else if(s->parameter==4U) n->remote_voltage_limit_v=limit;
                else n->remote_iq_slew_a_per_s=limit;
                s->got_parameter=1U;
            }
        }
    } else if(kind==5U) {
        n->fast_seen=1U;
        uint32_t raw_angle=(uint32_t)data[2]|((uint32_t)data[3]<<8);
        n->fast_flags=data[1];
        if(raw_angle>6283U || !(data[1]&8U)) {
            n->fast_valid=0U;++g_motor_foc.malformed;
            fast_history_clear(slot);s->fast_seeded=0U;
            taskEXIT_CRITICAL();return 1U;
        }
        float angle=(float)raw_angle*.001f;
        MotorFocCalibration calibration=g_motor_foc.calibration[slot];
        uint32_t configuration_changed=!fast_history_configuration(&fast_history[slot],&calibration);
        /* 首帧或断流后按确认过的整圈锚点重新定位；连续帧把角度增量折回 ±pi 后累加。 */
        if(!s->fast_seeded || configuration_changed ||
           !fresh(now,n->fast_ms,n->fast_count,MOTOR_FOC_FAST_TIMEOUT_MS)) {
            fast_history_clear(slot);
            fast_history[slot].configuration_valid=1U;
            fast_history[slot].anchor_rad=calibration.angle_anchor_rad;
            fast_history[slot].anchor_confirmed=calibration.angle_anchor_confirmed;
            fast_history[slot].timing_confirmed=calibration.joint_timing_confirmed;
            float anchor=g_motor_foc.calibration[slot].angle_anchor_rad;
            n->fast_unwrapped_angle_rad=angle;
            if(g_motor_foc.calibration[slot].angle_anchor_confirmed && isfinite(anchor))
                n->fast_unwrapped_angle_rad+=TAU*roundf((anchor-angle)/TAU);
            s->fast_seeded=1U;
        } else {
            float delta=angle-s->previous_fast_angle;
            if(delta>TAU*.5f) delta-=TAU;else if(delta<-TAU*.5f) delta+=TAU;
            n->fast_unwrapped_angle_rad+=delta;
        }
        s->previous_fast_angle=angle;n->fast_angle_rad=angle;
        int32_t raw_speed=i16(data+4);
        n->fast_speed_rad_s=(float)raw_speed*.001f;n->fast_iq_a=(float)i16(data+6)*.001f;
        n->fast_speed_clipped=raw_speed==INT16_MIN || raw_speed==INT16_MAX;
        n->fast_ms=now;++n->fast_count;n->fast_valid=1U;
        if(n->fast_speed_clipped) fast_history_clear(slot);
        else if(MOTOR_FOC_JOINT_NODE_MASK&(1U<<slot)) {
            FastHistory *history=&fast_history[slot];
            for(unsigned i=MOTOR_FAST_HISTORY_SAMPLES-1U;i>0U;--i)
                history->samples[i]=history->samples[i-1U];
            history->samples[0]=(MotorFastSample){now,n->fast_count,
                n->fast_unwrapped_angle_rad,n->fast_speed_rad_s};
            if(history->count<MOTOR_FAST_HISTORY_SAMPLES) ++history->count;
            history->epoch=n->epoch;
        }
        if(n->fast_speed_clipped && s->was_active) {
            capture_failure(MOTOR_FOC_FAILURE_SPEED_CLIPPED,slot);
            /* A newer FAST frame must not hide a clipping event before the
             * control task sees it. Existing inhibit/STOP handshake clears it. */
            n->inhibit=1U;
        }
    } else if(kind==6U) {
        uint32_t flags=data[4],zero=(uint32_t)data[6]|((uint32_t)data[7]<<8);
        int32_t direction=(int8_t)data[5];
        if(!calibration_opcode(data[1]) || !calibration_result_valid(data[3]) ||
           (flags&~0x1fU) || zero>6283U ||
           ((flags&1U) ? (direction!=1 && direction!=-1) : (direction!=0 || zero!=0U)) ||
           ((flags&4U) && !(flags&1U)) || (data[1]==0x15U && (flags&16U))) {
            ++g_motor_foc.malformed;
        } else if(s->pending && s->opcode==data[1] && s->sequence==data[2]) {
            volatile MotorFocStoredCalibration *stored=&g_motor_foc.stored_calibration[slot];
            ++stored->rx_count;stored->received_ms=now;
            stored->reply_opcode=data[1];stored->reply_sequence=data[2];
            stored->reply_result=data[3];stored->flags=flags;
            stored->sensor_direction=direction;stored->electrical_zero_rad=(float)zero*.001f;
            if(data[3]!=0U || (s->opcode==0x16U && (flags&0x17U)!=0x11U)) {
                ++n->rejected_count;invalidate(slot,MOTOR_FOC_FAILURE_CALIBRATION_LIMIT);
            } else {
                s->got_calibration=1U;calibration_completed(slot);
            }
        }
    } else if(kind==4U && data[1]==2U) {
        float angle=(float)i16(data+2)*.001f;
        if(angle<0 || angle>TAU+.001f) {++g_motor_foc.malformed;taskEXIT_CRITICAL();return 1U;}
        if(!s->angle_seeded || !fresh(now,n->angle_ms,n->angle_count,MOTOR_FOC_ANGLE_TIMEOUT_MS)) {
            float anchor=g_motor_foc.calibration[slot].angle_anchor_rad;
            n->unwrapped_angle_rad=angle;
            if(g_motor_foc.calibration[slot].angle_anchor_confirmed && isfinite(anchor))
                n->unwrapped_angle_rad+=TAU*roundf((anchor-angle)/TAU);
            s->angle_seeded=1U;
        } else {
            float delta=angle-s->previous_angle;
            if(delta>TAU*.5f) delta-=TAU;else if(delta<-TAU*.5f) delta+=TAU;
            n->unwrapped_angle_rad+=delta;
        }
        s->previous_angle=angle;n->angle_rad=angle;n->angle_ms=now;++n->angle_count;
    }
    if(kind==4U) {
        volatile MotorFocDqDebug *d=&g_motor_foc_dq[slot];
        if(data[1]==0U) {
            d->id_a=(float)i16(data+2)*.001f;
            d->iq_reference_a=(float)i16(data+4)*.001f;
            d->vd_v=(float)i16(data+6)*.001f;
            d->page0_ms=now;d->seen|=1U;
        } else if(data[1]==1U) {
            d->vq_v=(float)i16(data+2)*.001f;
            d->page1_ms=now;d->seen|=2U;
        } else if(data[1]==2U) {
            d->configured_bus_v=(float)i16(data+6)*.001f;
            d->page2_ms=now;d->seen|=4U;
        }
    }
    taskEXIT_CRITICAL();return 1U;
}

/* All control kinematics use the same fresh FAST frame. STATUS/DIAGNOSTIC
 * are independent health gates, never a slower substitute for wheel speed. */
uint8_t MotorFoc_GetFeedback(uint8_t id,MotorFeedback *feedback) {
    return MotorFoc_GetFeedbackWithHealth(id,feedback,NULL);
}
uint8_t MotorFoc_GetFeedbackWithHealth(uint8_t id,MotorFeedback *feedback,MotorFeedbackHealth *health) {
    if(health) memset(health,0,sizeof(*health));
    if(!feedback || id<1U || id>8U || !selected(id-1U)) return 0U;
    unsigned slot=id-1U;MotorFocNode n;MotorFocCalibration c;FastHistory history;uint32_t epoch,now;
    taskENTER_CRITICAL();
    now=HAL_GetTick();c=g_motor_foc.calibration[slot];
    if((MOTOR_FOC_JOINT_NODE_MASK&(1U<<slot)) &&
       !fast_history_configuration(&fast_history[slot],&c)) {
        fast_history_clear(slot);fast_history[slot].configuration_valid=0U;
        session[slot].fast_seeded=0U;
    }
    n=g_motor_foc.node[slot];
    history=fast_history[slot];epoch=g_motor_foc.transport_epoch;
    taskEXIT_CRITICAL();
    memset(feedback,0,sizeof(*feedback));feedback->motor_id=id;
    int is_joint=(MOTOR_FOC_JOINT_NODE_MASK&(1U<<slot))!=0U;
    feedback->protocol_a1=1U;feedback->angle_age_ms=now-n.fast_ms;
    feedback->status_age_ms=now-n.status_ms;feedback->diagnostic_age_ms=now-n.diagnostic_ms;
    feedback->angle_rad=n.fast_unwrapped_angle_rad;
    feedback->angle_deg=feedback->angle_rad*57.2957795131f;
    feedback->speed_rad_s=n.fast_speed_rad_s;
    feedback->speed_rpm=feedback->speed_rad_s*9.549296586f;
    float raw_mrad=feedback->angle_rad*1000.0f;
    feedback->raw_angle_mrad=raw_mrad>=2147483520.0f?INT32_MAX:raw_mrad<=-2147483648.0f?INT32_MIN:(int32_t)raw_mrad;
    feedback->raw_speed_rpm_x10=(int32_t)(feedback->speed_rpm*10.0f);
    feedback->last_rx_ms=n.fast_ms;
    feedback->age_ms=now-feedback->last_rx_ms;feedback->rx_count=n.fast_count;
    feedback->online=n.epoch==epoch && fresh(now,n.status_ms,n.status_count,MOTOR_FOC_STATUS_TIMEOUT_MS) &&
        fresh(now,n.diagnostic_ms,n.diagnostic_count,MOTOR_FOC_STATUS_TIMEOUT_MS) && (n.flags&8U) &&
        !n.faults && !n.board_faults && !(n.diagnostic_flags&0x23U) &&
        n.fast_valid && !n.fast_speed_clipped && fresh(now,n.fast_ms,n.fast_count,MOTOR_FOC_FAST_TIMEOUT_MS) &&
        (!is_joint || (c.angle_anchor_confirmed && c.joint_timing_confirmed &&
                      fast_history_configuration(&history,&c)));
    if(health) {
        uint32_t reject=0U;
        health->motor_id=id;health->sample_tick_ms=now;
        health->node_epoch=n.epoch;health->transport_epoch=epoch;
        health->status_ms=n.status_ms;health->status_count=n.status_count;
        health->status_age_ms=now-n.status_ms;
        health->diagnostic_ms=n.diagnostic_ms;health->diagnostic_count=n.diagnostic_count;
        health->diagnostic_age_ms=now-n.diagnostic_ms;
        health->fast_ms=n.fast_ms;health->fast_count=n.fast_count;health->fast_age_ms=now-n.fast_ms;
        health->flags=n.flags;health->diagnostic_flags=n.diagnostic_flags;
        health->faults=n.faults;health->board_faults=n.board_faults;health->mode=n.mode;
        health->fast_flags=n.fast_flags;health->fast_valid=n.fast_valid;
        health->fast_speed_clipped=n.fast_speed_clipped;
        health->anchor_confirmed=c.angle_anchor_confirmed;health->timing_confirmed=c.joint_timing_confirmed;
        health->history_configuration_valid=fast_history_configuration(&history,&c);
        health->history_generation=history.generation;health->history_count=history.count;
        health->history_epoch=history.epoch;
        if(n.epoch!=epoch) reject|=MOTOR_FEEDBACK_REJECT_EPOCH;
        if(!n.status_count) reject|=MOTOR_FEEDBACK_REJECT_STATUS_MISSING;
        else if(health->status_age_ms>MOTOR_FOC_STATUS_TIMEOUT_MS) reject|=MOTOR_FEEDBACK_REJECT_STATUS_STALE;
        if(!n.diagnostic_count) reject|=MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_MISSING;
        else if(health->diagnostic_age_ms>MOTOR_FOC_STATUS_TIMEOUT_MS) reject|=MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_STALE;
        if(!(n.flags&8U)) reject|=MOTOR_FEEDBACK_REJECT_STATUS_SENSOR;
        if(n.faults) reject|=MOTOR_FEEDBACK_REJECT_FAULTS;
        if(n.board_faults) reject|=MOTOR_FEEDBACK_REJECT_BOARD_FAULTS;
        if(n.diagnostic_flags&0x23U) reject|=MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_HEALTH;
        if(!n.fast_valid) reject|=MOTOR_FEEDBACK_REJECT_FAST_INVALID;
        if(n.fast_speed_clipped) reject|=MOTOR_FEEDBACK_REJECT_FAST_CLIPPED;
        if(!n.fast_count) reject|=MOTOR_FEEDBACK_REJECT_FAST_MISSING;
        else if(health->fast_age_ms>MOTOR_FOC_FAST_TIMEOUT_MS) reject|=MOTOR_FEEDBACK_REJECT_FAST_STALE;
        if(is_joint) {
            if(!c.angle_anchor_confirmed) reject|=MOTOR_FEEDBACK_REJECT_ANCHOR;
            if(!c.joint_timing_confirmed) reject|=MOTOR_FEEDBACK_REJECT_TIMING;
            if(!health->history_configuration_valid) reject|=MOTOR_FEEDBACK_REJECT_CONFIGURATION;
        }
        health->reject_mask=reject;health->valid=1U;
    }
    feedback->feedback_epoch=n.epoch;
    feedback->fast_history_generation=history.generation;
    if(is_joint && feedback->online && history.epoch==epoch &&
       history.count<=MOTOR_FAST_HISTORY_SAMPLES) {
        feedback->fast_history_count=history.count;
        for(unsigned i=0;i<history.count;++i) feedback->fast_history[i]=history.samples[i];
    }
    JointCalibration_Apply(feedback);return feedback->online;
}

void MotorFoc_LatchFeedbackFailure(const MotorFeedbackHealth *sample,uint32_t controller_tick_ms,
                                  uint32_t feedback_detail,uint32_t observer_age_ms) {
    if(!sample || !sample->valid || sample->motor_id<1U || sample->motor_id>8U) return;
    taskENTER_CRITICAL();
    if(!g_motor_foc_feedback_failure.valid) {
        g_motor_foc_feedback_failure.schema_version=1U;
        g_motor_foc_feedback_failure.controller_tick_ms=controller_tick_ms;
        g_motor_foc_feedback_failure.feedback_detail=feedback_detail;
        g_motor_foc_feedback_failure.observer_age_ms=observer_age_ms;
        g_motor_foc_feedback_failure.sample=*sample;
        g_motor_foc_feedback_failure.valid=1U; /* Publish only a complete original snapshot. */
    }
    taskEXIT_CRITICAL();
}

/* 可用力矩由本地限流与驱动读回限流中的较小值决定；标定、动态响应或读回未确认则为零。 */
float MotorFoc_TorqueLimitNm(uint8_t id) {
    if(id<1U || id>8U || !selected(id-1U)) return 0;
    unsigned slot=id-1U;
    const volatile MotorFocCalibration *c=&g_motor_foc.calibration[slot];
    const volatile MotorFocNode *n=&g_motor_foc.node[slot];
    float kt, current_limit, minimum_slew, remote_slew, remote_limit;
    int32_t direction;
    uint32_t calibration_confirmed, dynamic_response_confirmed, slew_verified, voltage_verified, limit_verified;
    /* 同一临界区只快照本次校验所需标量，不搬运节点遥测和无关标定字段。 */
    taskENTER_CRITICAL();
    kt=c->kt_nm_per_a;current_limit=c->current_limit_a;direction=c->direction;
    calibration_confirmed=c->calibration_confirmed;dynamic_response_confirmed=c->dynamic_response_confirmed;
    minimum_slew=c->minimum_iq_slew_a_per_s;
    limit_verified=n->limit_verified;voltage_verified=n->voltage_verified;slew_verified=n->slew_verified;
    remote_limit=n->remote_iq_limit_a;remote_slew=n->remote_iq_slew_a_per_s;
    taskEXIT_CRITICAL();
    if(!calibration_confirmed || !dynamic_response_confirmed ||
       !isfinite(minimum_slew) || minimum_slew<=0 ||
       !slew_verified || !voltage_verified || remote_slew<minimum_slew ||
       !isfinite(kt) || kt<=0 ||
       !isfinite(current_limit) || current_limit<=0 || current_limit>current_hard_limit(slot) ||
       (direction!=1 && direction!=-1) || !limit_verified ||
       !fixed_iq_limit_ready(slot,remote_limit)) return 0;
    return kt*fminf(current_limit,remote_limit);
}

float MotorFoc_CurrentForTorque(uint8_t id,float torque) {
    if(id<1U || id>8U || !selected(id-1U) || !isfinite(torque)) return NAN;
    float kt;int32_t direction;
    /* Kt 与方向仍成对原子读取，每次调用重新取值，不跨周期缓存。 */
    taskENTER_CRITICAL();
    kt=g_motor_foc.calibration[id-1U].kt_nm_per_a;direction=g_motor_foc.calibration[id-1U].direction;
    taskEXIT_CRITICAL();
    if(!isfinite(kt) || kt<=0 || (direction!=1 && direction!=-1)) return NAN;
    /* torque是模型坐标下的轴端N*m：Iq = direction * torque / Kt。
     * direction只做模型轴到驱动轴的符号转换，Kt单位N*m/A。
     * Iq是FOC相电流控制量，不能直接与稳压电源显示的直流输入电流比较。
     * 该函数只换算；主控/板端共同限流在Service中取较小值。 */
    return (float)direction*torque/kt;
}

uint32_t MotorFoc_MeasuredTorqueNm(uint8_t id,float *torque) {
    if(!torque)return 0U;
    *torque=0;
    if(id<1U || id>8U || !selected(id-1U))return 0U;
    uint32_t now=HAL_GetTick();
    taskENTER_CRITICAL();
    MotorFocNode n=g_motor_foc.node[id-1U];
    MotorFocCalibration c=g_motor_foc.calibration[id-1U];
    uint32_t epoch=g_motor_foc.transport_epoch;
    taskEXIT_CRITICAL();
    if(n.epoch!=epoch || n.mode!=3U || !(n.fast_flags&1U) || n.faults || n.board_faults || !n.fast_valid ||
       !fresh(now,n.fast_ms,n.fast_count,MOTOR_FOC_FAST_TIMEOUT_MS) || !c.calibration_confirmed ||
       !isfinite(n.fast_iq_a) || !isfinite(c.kt_nm_per_a) || c.kt_nm_per_a<=0 ||
       (c.direction!=1 && c.direction!=-1))return 0U;
    *torque=c.direction*c.kt_nm_per_a*n.fast_iq_a;
    /* 这是基于Kt的力矩估计；电流跟随正常不能独立证明机械力矩、方向或Kt已标定。 */
    return isfinite(*torque)?1U:0U;
}

static int send_command(unsigned slot,uint32_t opcode,int value,uint32_t auxiliary,uint32_t now) {
    FDCAN_TxHeaderTypeDef h={0};uint8_t data[8]={0xa1U,0,0,0,0,0,0,0};
    h.Identifier=0x201U+slot;h.IdType=FDCAN_STANDARD_ID;h.TxFrameType=FDCAN_DATA_FRAME;
    h.DataLength=FDCAN_DLC_BYTES_8;h.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    h.BitRateSwitch=FDCAN_BRS_OFF;h.FDFormat=FDCAN_CLASSIC_CAN;h.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    FocSession *s=&session[slot];
    taskENTER_CRITICAL();
    data[1]=(uint8_t)opcode;data[2]=s->next_sequence++;data[3]=(opcode>=1U && opcode<=4U)?1U:0U;
    if(opcode==0x10U || opcode==0x11U) {
        data[3]=(uint8_t)auxiliary;s->parameter=auxiliary;
        if(opcode==0x11U) {
            s->parameter_value_bits=(uint32_t)value;
            if(auxiliary==3U) g_motor_foc.node[slot].limit_verified=0U;
            if(auxiliary==5U) g_motor_foc.node[slot].slew_verified=0U;
            for(unsigned i=0;i<4;++i)data[4+i]=(uint8_t)((uint32_t)value>>(8U*i));
        }
    }
    else if(opcode==0x16U) {data[4]=0x43U;data[5]=0x41U;data[6]=0x4cU;data[7]=0x31U;}
    else {data[4]=(uint8_t)(uint16_t)value;data[5]=(uint8_t)((uint16_t)value>>8);data[6]=(uint8_t)auxiliary;data[7]=(uint8_t)(auxiliary>>8);}
    s->pending=1U;s->got_ack=s->got_parameter=s->got_calibration=0U;s->sequence=data[2];s->opcode=opcode;s->sent_ms=now;
    if(calibration_opcode(opcode)) {
        volatile MotorFocStoredCalibration *stored=&g_motor_foc.stored_calibration[slot];
        stored->request_opcode=opcode;stored->request_sequence=data[2];
        stored->ack_ms=stored->received_ms=0U;stored->ack_result=stored->reply_result=UINT32_MAX;
        stored->reply_opcode=stored->reply_sequence=stored->flags=0U;
        stored->sensor_direction=0;stored->electrical_zero_rad=0;stored->completed=0U;
    }
    if(opcode==1U || opcode==4U) {last_motion_ms=now;motion_ever_sent=1U;}
    if(opcode==0U) {
        s->stopped_ack=0U;s->stopped_required=0U;s->current_ack_seen=0U;s->was_active=0U;
        volatile MotorFocNode *n=&g_motor_foc.node[slot];
        n->session_ready=n->limit_verified=n->voltage_verified=n->slew_verified=0U;
        ++g_motor_foc.stops;
    }
    taskEXIT_CRITICAL();
    if(Fdcan_SendBounded(&h,data)!=HAL_OK) {
        taskENTER_CRITICAL();++g_motor_foc.tx_failed;capture_failure(MOTOR_FOC_FAILURE_TX_ENQUEUE,slot);
        invalidate(slot,MOTOR_FOC_FAILURE_NONE);taskEXIT_CRITICAL();return 0;
    }
    ++g_motor_foc.tx_count;return 1;
}

/* A short direct-Iq commissioning pulse needs valid rotor feedback but no
 * robot kinematic turn anchor, torque constant or LQR tuning approval. */
static int direct_feedback_ready(unsigned slot,const MotorFocNode *n,uint32_t now) {
    return n->epoch==g_motor_foc.transport_epoch &&
        fresh(now,n->status_ms,n->status_count,MOTOR_FOC_STATUS_TIMEOUT_MS) &&
        fresh(now,n->diagnostic_ms,n->diagnostic_count,MOTOR_FOC_STATUS_TIMEOUT_MS) &&
        (n->mode==0U || n->mode==3U) &&
        (n->mode!=0U || !(n->flags&1U)) && (n->mode!=3U || (n->flags&1U)) &&
        (n->flags&READY_FLAGS)==READY_FLAGS && !(n->flags&64U) &&
        !n->faults && !n->board_faults && !(n->diagnostic_flags&BAD_DIAGNOSTIC) &&
        (!n->fast_seen || (n->fast_valid && !n->fast_speed_clipped)) &&
        (!(MOTOR_FOC_JOINT_NODE_MASK&(1U<<slot)) ||
         (n->fast_valid && fresh(now,n->fast_ms,n->fast_count,MOTOR_FOC_FAST_TIMEOUT_MS)));
}

void MotorFoc_StopAll(void) {
    /* FIFO has 8 slots, no dedicated TX buffers in bsp_fdcan.c. Abort stale
     * queued targets before adding STOP; ACK failure still has 100ms MCU expiry. */
    dq_trace_active=0U; /* Preserve the ring; next accepted run starts a new one. */
    failure_run_active=0U;failure_active_mask=0U;
    FdcanDeadline_AbortAll();
    uint32_t now=HAL_GetTick();
    for(unsigned slot=0;slot<8;++slot) if(selected(slot)) {
        taskENTER_CRITICAL();invalidate(slot,MOTOR_FOC_FAILURE_NONE);session[slot].aligning=0U;g_motor_foc.maintenance_opcode[slot]=0U;taskEXIT_CRITICAL();
        (void)send_command(slot,0U,0,0,now);
    }
}

uint32_t MotorFoc_Service(const float torque[8],uint32_t active_mask) {
    uint32_t now=HAL_GetTick(),good=1U,all_active_mask=active_mask;active_mask&=MOTOR_FOC_NODE_MASK&MOTOR_FOC_OUTPUT_ENABLE_MASK;
    uint32_t events_at_start=failure_events,epoch_at_start=g_motor_foc.transport_epoch;
    failure_active_mask=active_mask;
    if(!active_mask) failure_run_active=0U;
    if(!active_mask) dq_trace_active=0U;
    if(g_motor_foc.stop_request) {
        capture_failure(MOTOR_FOC_FAILURE_STOP_REQUEST,8U);
        g_motor_foc.stop_request=0U;MotorFoc_StopAll();return all_active_mask?0U:1U;
    }
    if(FdcanDeadline_TakeExpired()) {
        capture_failure(MOTOR_FOC_FAILURE_TX_DEADLINE,8U);
        MotorFoc_InvalidateFromISR();MotorFoc_StopAll();
        return all_active_mask?0U:1U;
    }
    /* A control owner appearing during a storage exchange must stop, not
     * wait behind that maintenance request and start after its completion.
     */
    if(all_active_mask) {
        for(unsigned slot=0;slot<8;++slot) if(selected(slot) &&
            session[slot].pending && (calibration_opcode(session[slot].opcode) || session[slot].opcode==0x11U)) {
            capture_failure(MOTOR_FOC_FAILURE_MAINTENANCE,slot);MotorFoc_StopAll();return 0U;
        }
    }
    /* STOP overrides an outstanding current ACK immediately. Old queued IQ
     * is aborted as a batch before STOP is queued, rather than waiting 40ms. */
    for(unsigned slot=0;slot<8;++slot) if(selected(slot) && session[slot].was_active &&
        !session[slot].debug_running && !(active_mask&(1U<<slot))) {
        capture_failure(MOTOR_FOC_FAILURE_ACTIVE_MASK,slot);MotorFoc_StopAll();return all_active_mask?0U:1U;
    }
    /* Validate all selected active motors before queueing either wheel. */
    for(unsigned slot=0;slot<8;++slot) if(selected(slot)) {
        taskENTER_CRITICAL();
        volatile MotorFocNode *n=&g_motor_foc.node[slot];FocSession *s=&session[slot];
        if(n->epoch!=g_motor_foc.transport_epoch) {invalidate(slot,MOTOR_FOC_FAILURE_TRANSPORT_EPOCH);s->angle_seeded=s->fast_seeded=0U;n->fast_valid=0U;n->epoch=g_motor_foc.transport_epoch;}
        uint32_t timeout=calibration_opcode(s->opcode)?MOTOR_FOC_CALIBRATION_TIMEOUT_MS:MOTOR_FOC_ACK_TIMEOUT_MS;
        if(s->pending && (uint32_t)(now-s->sent_ms)>timeout) {++n->timeout_count;invalidate(slot,MOTOR_FOC_FAILURE_ACK_TIMEOUT);}
        /* 读参数既要 ACK 成功，也要收到同序号的数值页，不能只凭命令被接收就放行。 */
        if(s->pending && s->got_ack && ((s->opcode!=0x10U && s->opcode!=0x11U) || s->got_parameter) &&
           (!calibration_opcode(s->opcode) || s->got_calibration)) {
            uint32_t saved=s->opcode==0x16U;
            if(s->opcode==0U) {s->stopped_ack=1U;s->stop_status_count=n->status_count;s->stop_diag_count=n->diagnostic_count;}
            if(s->opcode==0x10U || s->opcode==0x11U) {
                if(s->parameter==3U) n->limit_verified=1U;
                else if(s->parameter==4U) n->voltage_verified=1U;
                else n->slew_verified=1U;
            }
            if(s->opcode>=4U && s->opcode<=7U && s->opcode!=4U) g_motor_foc.maintenance_opcode[slot]=0U;
            if(calibration_opcode(s->opcode) || s->opcode==0x11U) g_motor_foc.maintenance_opcode[slot]=0U;
            s->pending=0U;
            /* SAVE is a terminal remote operation. Never reuse pre-reset
             * STOP/parameter caches or infer reboot success from the ACK.
             */
            if(saved) {invalidate(slot,MOTOR_FOC_FAILURE_NONE);s->next_ms=now;}
        }
        if(s->current_ack_seen && n->status_count>s->current_ack_status_count &&
           (n->mode!=3U || !(n->flags&1U) || (n->flags&64U))) invalidate(slot,MOTOR_FOC_FAILURE_DRIVER_HEALTH);
        /* STOP 的 ACK 之后还要等新的状态和诊断共同确认停机，排除缓存中的旧反馈。 */
        if(s->stopped_ack && n->status_count>s->stop_status_count && n->diagnostic_count>s->stop_diag_count &&
           !(n->flags&1U) && (n->mode==0U || n->mode==5U) && !(n->diagnostic_flags&8U)) n->session_ready=1U;
        if(!(active_mask&(1U<<slot))) n->inhibit=0U;
        if(!n->session_ready && g_motor_foc.maintenance_opcode[slot]) {
            g_motor_foc.maintenance_opcode[slot]=0U;++n->rejected_count;
        }
        if(all_active_mask) {
            g_motor_foc.maintenance_opcode[slot]=0U;
            g_motor_foc.debug_request_mask&=~(1U<<slot);s->debug_running=0U;
        }
        if (!(MOTOR_FOC_OUTPUT_ENABLE_MASK & (1U<<slot))) {
            if (g_motor_foc.debug_request_mask & (1U<<slot)) ++n->rejected_count;
            g_motor_foc.debug_request_mask &= ~(1U<<slot);
            s->debug_running=0U;
            uint32_t op=g_motor_foc.maintenance_opcode[slot];
            if (op>=4U && op<=7U) {
                g_motor_foc.maintenance_opcode[slot]=0U;++n->rejected_count;
            }
        }
        MotorFocNode copy=*n;
        taskEXIT_CRITICAL();
        if(active_mask&(1U<<slot)) {
            MotorFeedback f;
            uint32_t reason=MOTOR_FOC_FAILURE_NONE;
            if(!torque || !isfinite(torque[slot])) reason=MOTOR_FOC_FAILURE_INVALID_TORQUE;
            else if(copy.fast_speed_clipped) reason=MOTOR_FOC_FAILURE_SPEED_CLIPPED;
            else if(!MotorFoc_GetFeedback((uint8_t)(slot+1U),&f)) reason=MOTOR_FOC_FAILURE_FEEDBACK;
            else if(!copy.session_ready || copy.inhibit) reason=MOTOR_FOC_FAILURE_SESSION;
            else if((copy.mode!=0U && copy.mode!=3U) || (copy.mode==0U && (copy.flags&1U)) ||
                    (copy.mode==3U && !(copy.flags&1U)) || (copy.flags&64U) ||
                    (copy.flags&READY_FLAGS)!=READY_FLAGS || (copy.diagnostic_flags&BAD_DIAGNOSTIC))
                reason=MOTOR_FOC_FAILURE_DRIVER_HEALTH;
            else if(MotorFoc_TorqueLimitNm((uint8_t)(slot+1U))<=0) reason=MOTOR_FOC_FAILURE_CALIBRATION_LIMIT;
            if(reason) {capture_failure(reason,slot);good=0U;}
        }
    }
    if(!good) {MotorFoc_StopAll();return 0U;}
    for(unsigned slot=0;slot<8;++slot) if(selected(slot)) {
        MotorFocNode n;FocSession s;MotorFocCalibration c;
        taskENTER_CRITICAL();n=g_motor_foc.node[slot];s=session[slot];c=g_motor_foc.calibration[slot];taskEXIT_CRITICAL();
        int active=(active_mask&(1U<<slot))!=0U;
        if((g_motor_foc.debug_request_mask&(1U<<slot)) && !all_active_mask) {
            g_motor_foc.debug_request_mask&=~(1U<<slot);
            int ma=g_motor_foc.debug_current_ma[slot];
            uint32_t duration=g_motor_foc.debug_duration_ms[slot];
            if(duration>=1U && duration<=1000U && ma>=-300 && ma<=300 &&
               n.session_ready && n.limit_verified && n.voltage_verified && n.slew_verified &&
               fixed_iq_limit_ready(slot,n.remote_iq_limit_a) &&
               isfinite(c.current_limit_a) && c.current_limit_a>0 && c.current_limit_a<=current_hard_limit(slot) &&
               direct_feedback_ready(slot,&n,now) &&
               fabsf((float)ma*.001f)<=fminf(n.remote_iq_limit_a,c.current_limit_a)) {
                session[slot].debug_running=1U;session[slot].debug_until_ms=now+duration;
                session[slot].debug_current_ma=ma;
            } else ++g_motor_foc.node[slot].rejected_count;
        }
        if(session[slot].debug_running) {
            if((int32_t)(now-session[slot].debug_until_ms)>=0 ||
               !direct_feedback_ready(slot,&n,now)) {
                MotorFoc_StopAll();return 1U;
            } else active=1;
        }
        if(active) g_motor_foc.maintenance_opcode[slot]=0U;
        if(active && !session[slot].was_active) {
            /* STOP uses a 100ms retry period; do not inherit that deadline
             * when a newly armed current controller needs its first target. */
            session[slot].was_active=1U;session[slot].next_ms=now;s.next_ms=now;
            g_motor_foc_command_timing[slot]=(MotorFocCommandTiming){0};
        }
        if(!active && s.aligning) {
            if(n.faults || (uint32_t)(now-s.align_started_ms)>MOTOR_FOC_ALIGN_TIMEOUT_MS || (n.flags&32U)) {
                session[slot].aligning=0U;g_motor_foc.maintenance_opcode[slot]=0U;
                if(!send_command(slot,0U,0,0,now)) good=0U;
            } else if(!s.pending && (int32_t)(now-s.next_ms)>=0) {
                if(!send_command(slot,4U,0,0,now)) good=0U;
                session[slot].next_ms=now+50U;
            }
            continue;
        }
        if(!active && (!n.session_ready || (n.flags&1U) || (n.diagnostic_flags&8U) || s.stopped_required)) {
            if(!s.pending && (int32_t)(now-s.next_ms)>=0) {
                if(!send_command(slot,0U,0,0,now)) good=0U;
                session[slot].next_ms=now+MOTOR_FOC_STOP_PERIOD_MS;
            }
            continue;
        }
        if(s.pending) continue;
        if(!active) {
            uint32_t op=g_motor_foc.maintenance_opcode[slot];
            if(op==0x11U && all_active_mask==0U) {
                uint32_t bits=g_motor_foc.maintenance_value[slot];float value;memcpy(&value,&bits,4);
                /* Only the configured fixed IQ_LIMIT and bounded IQ_SLEW
                 * are writable. Voltage, PI gains, bus voltage and IDs stay
                 * excluded; every write is stopped and exactly echoed. */
                uint32_t parameter=g_motor_foc.maintenance_aux[slot];
                unsigned allowed=parameter==5U ? isfinite(value) && value>=.01f && value<=10.0f :
                    parameter==3U && MOTOR_FOC_FIXED_IQ_LIMIT_A>0 &&
                    fixed_iq_limit_ready(slot,value);
                if(!allowed || n.mode!=0U || (n.flags&1U) ||
                   !MotorFoc_TransportMotionStopped()) {
                    g_motor_foc.maintenance_opcode[slot]=0U;++g_motor_foc.node[slot].rejected_count;continue;
                }
                if(!send_command(slot,op,(int)bits,parameter,now)) good=0U;
                continue;
            }
            if(((op>=4U && op<=7U) || calibration_opcode(op)) && all_active_mask==0U) {
                int value=g_motor_foc.maintenance_value[slot];uint32_t aux=g_motor_foc.maintenance_aux[slot];
                if(aux!=0U || (op==6U && value!=1 && value!=-1) ||
                   (calibration_opcode(op) && value!=0)) {
                    g_motor_foc.maintenance_opcode[slot]=0U;++g_motor_foc.malformed;continue;
                }
                if(calibration_opcode(op) && (!MotorFoc_TransportMotionStopped() ||
                   (op==0x16U && (n.mode!=0U || (n.flags&1U) ||
                     (n.flags&READY_FLAGS)!=READY_FLAGS || n.faults || n.board_faults)))) {
                    g_motor_foc.maintenance_opcode[slot]=0U;++g_motor_foc.node[slot].rejected_count;continue;
                }
                if(op!=6U) value=0;
                if(op==4U && (n.flags&32U)) {g_motor_foc.maintenance_opcode[slot]=0U;continue;}
                if(!send_command(slot,op,value,aux,now)) good=0U;
                if(op==4U) {session[slot].aligning=1U;session[slot].align_started_ms=now;session[slot].next_ms=now+50U;}
                continue;
            }
        }
        /* Fault telemetry remains visible. Only an explicit maintenance
         * request can clear it; failed automatic parameter reads must not
         * repeatedly revoke the STOP session and starve that request. */
        if(!active && (n.faults || n.board_faults)) continue;
        /* 按限流、电压上限、电流变化率依次读回；三项均确认前不发送运行电流。 */
        if(!n.limit_verified || !n.voltage_verified || !n.slew_verified) {
            uint32_t parameter=!n.limit_verified?3U:!n.voltage_verified?4U:5U;
            if(!send_command(slot,0x10U,0,parameter,now)) good=0U;
            continue;
        }
        if(!active && all_active_mask==0U && MOTOR_FOC_FIXED_IQ_LIMIT_A>0 &&
           !fixed_iq_limit_ready(slot,n.remote_iq_limit_a) &&
           fixed_iq_limit_ready(slot,MOTOR_FOC_FIXED_IQ_LIMIT_A) &&
           n.mode==0U && !(n.flags&1U) && MotorFoc_TransportMotionStopped()) {
            /* Parameter writes are RAM-only in the supported drivers. Restore
             * the requested current ceiling after their reboot, without Flash
             * writes or energizing output. ACK and exact value must both pass. */
            const float fixed_limit=MOTOR_FOC_FIXED_IQ_LIMIT_A;
            uint32_t bits;memcpy(&bits,&fixed_limit,4);
            if(!send_command(slot,0x11U,(int)bits,3U,now)) good=0U;
            continue;
        }
        if(!active && all_active_mask==0U && MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S>0 &&
           fabsf(n.remote_iq_slew_a_per_s-MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S)>1e-5f &&
           n.mode==0U && !(n.flags&1U) && MotorFoc_TransportMotionStopped()) {
            /* A driver power cycle restores its old default. Reapply the
             * verified fixed execution rate before permitting torque output;
             * ordinary parameter ACK+exact echo validation still applies. */
            const float fixed_slew=MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S;
            uint32_t bits;memcpy(&bits,&fixed_slew,4);
            if(!send_command(slot,0x11U,(int)bits,5U,now)) good=0U;
            continue;
        }
        if(!active) continue;
        /* IQ follows the planned5ms phase, not the previous call's HAL tick.
         * HAL/RTOS millisecond boundaries can make a healthy5ms task appear
         * 4/6ms apart. Allow one early tick so either initial phase works;
         * STOP, maintenance and pending-ACK deadlines above remain strict. */
        if((int32_t)(now-s.next_ms)>=-1) {
            /* 主控配置与板端读回的限流取较小者，避免主控请求越过板端能力。
             * requested_iq为换算请求，applied_iq为限幅后的发送值，fast_iq为反馈。
             * 三者分开保留，才能区分算法请求不足、限幅和电流环跟随误差。 */
            float limit=fminf(c.current_limit_a,n.remote_iq_limit_a);
            float iq=session[slot].debug_running ? (float)session[slot].debug_current_ma*.001f :
                     MotorFoc_CurrentForTorque((uint8_t)(slot+1U),torque[slot]);
            if(!isfinite(iq) || !isfinite(limit) || limit<=0 || limit>current_hard_limit(slot)) {
                capture_failure(MOTOR_FOC_FAILURE_INVALID_CURRENT,slot);MotorFoc_StopAll();return 0U;
            }
            float applied=fminf(limit,fmaxf(-limit,iq));
            g_motor_foc.node[slot].requested_iq_a=iq;g_motor_foc.node[slot].applied_iq_a=applied;
            int milliamp=(int)(applied*1000.0f+(applied>=0?.5f:-.5f));
            /* opcode 0x01：带符号mA目标；next_ms按5ms调度，单节点约200Hz。
             * send_command成功只证明已入发送队列，ACK/FAST新鲜度另行确认。 */
            /* Keep the recheck and queue submission atomic against RX. The
             * bounded HAL submission already uses this same critical lock. */
            taskENTER_CRITICAL();
            uint32_t failure=active_send_failure(slot,epoch_at_start,events_at_start);
            if(failure) {
                capture_failure(failure,slot);taskEXIT_CRITICAL();MotorFoc_StopAll();return 0U;
            }
            if(!send_command(slot,1U,milliamp,0,now)) good=0U;
            else {
                volatile MotorFocCommandTiming *timing=&g_motor_foc_command_timing[slot];
                if(timing->count) {
                    uint32_t gap=now-timing->last_ms;
                    if(gap>timing->max_gap_ms) timing->max_gap_ms=gap;
                } else timing->first_ms=now;
                timing->last_ms=now;++timing->count;
            }
            uint32_t next=s.next_ms+MOTOR_FOC_COMMAND_PERIOD_MS;
            if((int32_t)(now-next)>=0) {
                /* One command per service even after a long delay. Skip
                 * expired slots, retaining the phase and a future deadline. */
                next+=((now-next)/MOTOR_FOC_COMMAND_PERIOD_MS+1U)*MOTOR_FOC_COMMAND_PERIOD_MS;
            }
            session[slot].stopped_required=1U;session[slot].next_ms=next;
            taskEXIT_CRITICAL();
            /* A higher-priority RX task may run at the lock exit. Stop this
             * call before another node receives IQ, rather than one beat later. */
            failure=active_send_failure(slot,epoch_at_start,events_at_start);
            if(failure) {
                capture_failure(failure,slot);MotorFoc_StopAll();return 0U;
            }
        }
    }
    if(!good) MotorFoc_StopAll();
    else {
        taskENTER_CRITICAL();
        /* Failed starts must not erase an earlier failure. RX may have
         * revoked the new session while this service queued commands. */
        if(active_mask && !failure_run_active && failure_events==events_at_start &&
           g_motor_foc.transport_epoch==epoch_at_start) {
            const MotorFocFailureDebug zero={0};g_motor_foc_failure=zero;failure_run_active=1U;
            memset((void *)&g_motor_foc_feedback_failure,0,sizeof(g_motor_foc_feedback_failure));
        }
        taskEXIT_CRITICAL();
        dq_trace_capture(HAL_GetTick(),active_mask);
    }
    if(good && active_mask) for(unsigned slot=0;slot<8;++slot) if(active_mask&(1U<<slot)) {
        uint32_t failure=active_send_failure(slot,epoch_at_start,events_at_start);
        if(failure) {capture_failure(failure,slot);MotorFoc_StopAll();return 0U;}
    }
    return good;
}
