#ifndef _HARD_PWM_H
#define _HARD_PWM_H

/* Two PWM modes */
#define PWMMODE     0
#define MSMODE      1

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
#    define   PWM_ENABLE 0
#    define   MODESHIFT  1
#    define   MSSHIFT    7
#    define   POLARITY   4

#define PWM_RNG(x)    (4 + 4*(x))
#define PWM_DAT(x)    (5 + 4*(x))

#define PWMCLK_CNTL 40
#define PWMCLK_DIV  41

int setFrequency(int gpio, float frequency);

int setDutyCycle(int gpio, float dutycycle);

int setMode(int gpio, int mode);
int setPolarity(int gpio, int pol);
int PWM_enable(int gpio, _Bool enable);

int setup_hard_pwm(int gpio);
void hardwarePWM_init(void);
void hardwarePWM_stop(void);

#endif   /* _HARD_PWM_H */
