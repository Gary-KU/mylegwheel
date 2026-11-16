#include "leg_trajectory.h"
#include "leg_model.h"
#include <math.h>
#define TWO_PI 6.2831853071795864769f
static int candidates(float x,float y,float reference,float out[2])
{
    float r=hypotf(x,y),c,offset,direction;
    if(r<=0.0f || r<0.055f-1e-7f || r>0.155f+1e-7f) return 0;
    c=(r*r+0.05f*0.05f-0.105f*0.105f)/(2.0f*r*0.05f);
    if(c>1.0f) c=1.0f;
    if(c< -1.0f) c= -1.0f;
    offset=acosf(c);direction=atan2f(y,x);
    out[0]=reference+remainderf(direction+offset-reference,TWO_PI);
    out[1]=reference+remainderf(direction-offset-reference,TWO_PI);
    return 1;
}
int leg_inverse(float x,float y,float rear,float front,float out[2])
{
    float a[2],b[2],best=INFINITY;
    int found=0;
    if(!out) return 0;
    out[0]=out[1]=0.0f;
    if(!isfinite(x)||!isfinite(y)||!isfinite(rear)||!isfinite(front)||
       !candidates(x,y,rear,a)||!candidates(x-0.06f,y,front,b)) return 0;
    for(int i=0;i<2;i++) for(int j=0;j<2;j++) {
        LegModel m;
        if(leg_model(a[i],b[j],LEG_BRANCH_MATLAB,&m)) {
            float fx=0.03f+m.length*cosf(m.angle),fy=m.length*sinf(m.angle);
            float cost=(a[i]-rear)*(a[i]-rear)+(b[j]-front)*(b[j]-front);
            if(hypotf(fx-x,fy-y)<2e-5f && cost<best) {
                out[0]=a[i];out[1]=b[j];best=cost;found=1;
            }
        }
    }
    return found;
}
int leg_square_sample(float cx,float cy,float side,float phase,float rear,
                      float front,LegTrajectoryPoint *out)
{
    static const float corners[4][2]={{-1,1},{1,1},{1,-1},{-1,-1}};
    LegTrajectoryPoint result={0};
    float p,u,s,angles[2];
    int edge,next;
    if(!out) return 0;
    *out=result;
    if(!isfinite(cx)||!isfinite(cy)||!isfinite(side)||side<=0||!isfinite(phase)) return 0;
    p=fmodf(phase,4.0f);if(p<0) p+=4.0f;
    edge=(int)floorf(p);next=(edge+1)%4;u=p-(float)edge;
    s=u*u*u*(10.0f+u*(-15.0f+6.0f*u));
    result.x_m=cx+0.5f*side*(corners[edge][0]+s*(corners[next][0]-corners[edge][0]));
    result.y_m=cy+0.5f*side*(corners[edge][1]+s*(corners[next][1]-corners[edge][1]));
    result.dx_dphase=0.5f*side*30.0f*u*u*(1.0f-u)*(1.0f-u)*(corners[next][0]-corners[edge][0]);
    result.dy_dphase=0.5f*side*30.0f*u*u*(1.0f-u)*(1.0f-u)*(corners[next][1]-corners[edge][1]);
    if(!leg_inverse(result.x_m,result.y_m,rear,front,angles)) return 0;
    result.rear_rad=angles[0];result.front_rad=angles[1];*out=result;
    return 1;
}
