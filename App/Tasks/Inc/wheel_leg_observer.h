#ifndef WHEEL_LEG_OBSERVER_H
#define WHEEL_LEG_OBSERVER_H

#include "wheel_leg_task.h"

/* 在控制任务中每拍调用：采集反馈、检查时效并生成完整观测快照，不发送电机指令。
 * 反馈/周期异常仍返回带有效位和故障详情的快照，由控制层决定停机。 */
void WheelLeg_Observe(WheelLegDebug *out);
/* Called only by the actual controller fault path. Latches this observation's
 * first rejected A1 snapshot; never re-reads a live node after recovery. */
void WheelLeg_LatchFeedbackFailure(const WheelLegDebug *out);

/* Read-only pairing audit, retained across STOP. No ring/Flash layout change.
 * last_reject_reason:1 epoch,2 malformed envelope,3 no fresh coherent pair. */
typedef struct {
    uint32_t latest_pair_count, history_pair_count, reject_count;
    uint32_t last_tick_ms, last_selected_age_ms[2], last_selected_rx_count[2];
    uint32_t last_latest_age_ms[2], last_history_count[2];
    uint32_t last_reject_tick_ms, last_reject_reason;
    uint32_t last_reject_latest_age_ms[2], last_reject_history_count[2];
} WheelLegPairDebug;
extern volatile WheelLegPairDebug g_wheel_leg_pair_debug[2];

#endif
