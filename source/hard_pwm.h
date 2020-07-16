/*
 * ============================================================================
 *
 *       Filename:  hard_pwm.h
 *
 *    Description:  Hardware pwm for RPi
 *
 *        Version:  1.0
 *        Created:  01/20/2019 02:14:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fang Yuan (yfang@nju.edu.cn)
 *   Organization:  nju
 *
 * ============================================================================
 */

#ifndef _HARD_PWM_H
#define _HARD_PWM_H

/* error code */

#define ERRFREQ     1
#define ERRCOUNT    2
#define ERRDUTY     3
#define ERRMODE     4
#define ERRPIN      5

/* hardware related constants */
#define PWM_BASE_OFFSET     0x20c000
#define CLK_BASE_OFFSET     0x101000

/* PWM registers */
#define PWM_CTL     0
#    define   PWM_MSMODE    (1<<7)
#    define   PWM_USEFIFO   (1<<5)
#    define   PWM_POLARITY  (1<<4)
#    define   PWM_SBIT      (1<<3)
#    define   PWM_REPEAT    (1<<2)
#    define   PWM_SERIAL    (1<<1)
#    define   PWM_ENABLE    (1<<0)

#define PWM_DMAC    2
#define PWM_RNG(x)    (4 + 4*(x))
#define PWM_DAT(x)    (5 + 4*(x))
#define PWM_FIF1    6

#define PWMCLK_CNTL 40
#define PWMCLK_DIV  41

#define BCM_PASSWD  0x5A000000
#define CLK_BUSY    (1<<7)
#define CLK_KILL    (1<<5)
#define CLK_ENABLE  (1<<4)
#define CLK_OSC     (1<<0)

int setFrequency(int gpio, float frequency);

int setDutyCycle(int gpio, float dutycycle);

int setMode(int gpio, int mode);
int setPolarity(int gpio, _Bool pol);
int PWM_enable(int gpio, _Bool enable);

int setup_hard_pwm(int gpio);
void hardwarePWM_init(void);
void hardwarePWM_stop(void);

#endif   /* _HARD_PWM_H */
