#include "timevar.h"
TimeVar::TimeVar(double timeStep)
{
    Dt=timeStep;
    DTS[2]=0;
    DTS[1]=0;
    DTS[0]=0;
    i=0;
}
void TimeVar::update(double in)
{
    DTS[2] = DTS[1];
    DTS[1] = DTS[0];
    DTS[0] = in;
    last_d = d;
    d = (DTS[0] - DTS[1]) / Dt;
    dd = (d - last_d) / Dt;
    i += (DTS[0] + DTS[1]) * Dt * 0.5;
}
void TimeVar::clear_i(void)
{
    i=0;
}
