#ifndef LEG_MODEL_H
#define LEG_MODEL_H
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    LEG_BRANCH_MATLAB = 0, /* .m files: positive circle normal */
    LEG_BRANCH_3D = 1,     /* Self_Model_3D: smaller y intersection */
    LEG_BRANCH_DOWN = 2    /* Hardware Y-down frame: larger y intersection */
} LegBranch;
typedef struct {
    float length;      /* m */
    float angle;       /* atan2(y, x-gap/2), rad */
    float jacobian[4]; /* 行优先：[dL/dphi1,dL/dphi4,dangle/dphi1,dangle/dphi4]；首行 m/rad，次行 rad/rad。 */
} LegModel;
/* 1=success, 0=invalid/singular, clears result on failure. Inputs: rad. */
int leg_model(float phi1, float phi4, LegBranch branch, LegModel *result);
#ifdef __cplusplus
}
#endif
#endif
