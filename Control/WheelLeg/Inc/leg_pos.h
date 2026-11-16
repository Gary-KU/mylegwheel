#ifndef LEG_POS_H
#define LEG_POS_H
#ifdef __cplusplus
extern "C" {
#endif
/* MyProc .m branch. Inputs rad; output [length(m),angle(rad)].
 * Invalid/singular geometry returns NaN. See leg_model.h for checked API.
 */
void leg_pos(float phi1, float phi4, float pos[2]);
#ifdef __cplusplus
}
#endif
#endif
