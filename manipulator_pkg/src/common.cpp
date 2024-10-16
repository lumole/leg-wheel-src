#include "manipulator.h"
double anglenormalize(double angle)
{
    if (angle > M_PI)
    {

        angle = angle - 2 * M_PI;
    }

    else if (angle < -M_PI)
    {

        angle = angle + 2 * M_PI;
    }
    return angle;
}

void playset::setFrame(double _t, double *xyz)
{
    frame temp;
    temp.x = *xyz;
    *xyz++;
    temp.y = *xyz;
    *xyz++;
    temp.z = *xyz;
    temp.time = _t;
    key_frame.push_back(temp); // 从前向后
    it = key_frame.begin();
    counter = 0;
    state = true;
}
frame playset::update()
{
    counter++;
    cout << "counter=" << counter << ",time=" << it->time / update_timestep << endl;
    if (counter > it->time / update_timestep)
    {
        if (state == true)
        {
            it++;
            counter = 0;
        }
        else
        {
            delete this;
        }    
    }
    if (it == key_frame.end())
    {
        it--;
        state = false; 
        cout << "play end" << endl;
    }
    return *it;
}