#ifndef BOARD_FEEDBACK_H
#define BOARD_FEEDBACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PA7: one WS2812B, SPI1/DMA1 Stream0. PB15: passive buzzer, TIM12 CH2. */
typedef enum {
    BOARD_FEEDBACK_IDLE = 0,
    BOARD_FEEDBACK_REMOTE_WAIT, /* Yellow, one pulse. */
    BOARD_FEEDBACK_PREPARING,   /* Blue, two pulses; preparation/ramp still active. */
    BOARD_FEEDBACK_FAULT,       /* Red, three pulses. */
    BOARD_FEEDBACK_BALANCING,   /* Green breathing, silent. */
    BOARD_FEEDBACK_BOOTING      /* Blue flashing during IMU initialization. */
} BoardFeedbackStatus;

typedef struct {
    uint32_t buzzer_enable; /* 0 mutes status beeps; power-on melody is unchanged. */
    BoardFeedbackStatus status;
    uint32_t ready_missing_mask; /* ROBOT_READY_* bits, only during leg preparation. */
    uint32_t flash_count; /* 1/2/3 for the three warning groups; otherwise 0. */
    uint32_t frequency_hz; /* Current requested buzzer frequency; 0 is silent. */
} BoardFeedbackDebug;

extern volatile BoardFeedbackDebug g_board_feedback;

void BoardFeedback_Init(void);
void BoardFeedback_Task_Create(void);

#ifdef __cplusplus
}
#endif
#endif
