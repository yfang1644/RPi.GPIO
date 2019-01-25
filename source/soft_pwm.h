/*
Copyright (c) 2013-2018 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* Software PWM using threads */

#ifndef _SOFT_PWM_H
#define _SOFT_PWM_H

struct pwm
{
    unsigned int gpio;
    float freq;
    float dutycycle;
    float basetime;
    float slicetime;
    struct timespec req_on, req_off;
    int running;
    struct pwm *next;
};

void pwm_set_duty_cycle(unsigned int gpio, float dutycycle);
void pwm_set_frequency(unsigned int gpio, float freq);
void pwm_start(unsigned int gpio);
void pwm_stop(unsigned int gpio);
int pwm_exists(unsigned int gpio);

#endif    /*  _SOFT_PWM_H */
