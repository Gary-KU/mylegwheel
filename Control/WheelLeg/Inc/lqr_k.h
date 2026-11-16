#ifndef LQR_K_H
#define LQR_K_H
#ifdef __cplusplus
extern "C" {
#endif
#define LQR_LEG_MIN_M 0.04f
#define LQR_LEG_MAX_M 0.15f
#define LQR_SAMPLE_TIME_S 0.005f
/* Model:1.600kg total,1.250kg body (.625kg per side).
 * Wheel/leg split and body COM height40mm are estimates; see metadata.
 * K[row+2*column]; state=[theta,dtheta,x,dx,phi,dphi].
 * Units: rad,rad/s,m,m/s,rad,rad/s; output=[T,Tp] in N*m.
 * Joint DLQR Q=[800,15,200,40,200,80],R=[100000,10000]; no row scaling.
 * Cubic samples4..15cm; runtime4..15cm. Regenerated2026-11-13.
 * Reproduce: scripts/generate_joint_pose_parking_1217_20261113.py.
 * Eight-state filtered/gain and constrained-slew screens are model checks.
 * Caller enforces runtime domain; real standing/parking needs a new trial.
 */
void lqr_k(float L0, float K[12]);
/* u=-K*(state-reference); 1=success, 0=invalid input (output zeroed).
 * No actuator saturation or motor direction conversion is applied.
 */
int lqr_control(float L0, const float state[6], const float reference[6], float u[2]);
/* 同时输出本次实际使用的增益，供轮力矩分项复用，避免同一控制拍重复计算。
 * gain 必须非空，且不得与 state/reference/u 的缓冲区重叠。
 * 1=成功，gain 与 lqr_k(L0) 一致；0=失败，非空的 u 和 gain 均清零。
 */
int lqr_control_with_gain(float L0, const float state[6], const float reference[6], float u[2], float gain[12]);
#ifdef __cplusplus
}
#endif
#endif
