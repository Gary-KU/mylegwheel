#ifndef LEG_SPD_H
#define LEG_SPD_H
#ifdef __cplusplus
extern "C" {
#endif
/* MyProc .m branch; input rad/s and rad; output [m/s,rad/s].
 * Invalid/singular inputs return NaN.
 */
void leg_spd(float dphi1, float dphi4, float phi1, float phi4, float spd[2]);
#ifdef __cplusplus
}
#endif
#endif
