#ifndef LEG_CONV_H
#define LEG_CONV_H
#ifdef __cplusplus
extern "C" {
#endif
/* MyProc .m branch: J' * [F(N),Tp(N*m)] -> [tau_phi1,tau_phi4](N*m).
 * Invalid/singular inputs output zero. Test compatibility API only.
 */
void leg_conv(float F, float Tp, float phi1, float phi4, float T[2]);
/* 3D wiring: [tau_phi4,tau_phi1], N*m; 0=failure, zeros outputs. */
int leg_conv_3d(float F, float Tp, float phi1, float phi4, float T[2]);
#ifdef __cplusplus
}
#endif
#endif
