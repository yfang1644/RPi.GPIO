#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "c_gpio.h"
#include "hard_pwm.h"

extern volatile unsigned int *clk_map, *pwm_map, *gpio_map;

static int divisor = 8;
/* 
* ===  FUNCTION  =============================================================
*         Name:  PWM_enable(int pin, bool enable)
*  Description:  enable or disable PWM pin
* ============================================================================
*/
int PWM_enable(int gpio, _Bool enable)
{
    int reg = 0;
    int shift = 0;
    int pin = gpio - 18;

    if (pin < 0 || pin > 2)
        return ERRPIN;

    shift = PWM_ENABLE + pin*8;
    reg = *(pwm_map + PWM_CTL);
    reg &= ~(1 << shift);
    reg |= (enable << shift);

    *(pwm_map + PWM_CTL) = reg;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setMode(int pin, int mode)
*  Description:  set pin to MSMODE(normal) or PWMMODE(distributed)
*g ============================================================================
*/
int setMode(int gpio, int mode)
{
    int reg = 0;
    int shift = 0;
    int pin = gpio - 18;

    if (pin < 0 || pin > 2)
        return ERRPIN;

    if((mode != PWMMODE) && (mode != MSMODE))
        return ERRMODE;

    shift = MSSHIFT + pin*8;
    reg = *(pwm_map + PWM_CTL);
    reg &= ~(1 << shift);
    reg |= (mode << shift);

    *(pwm_map + PWM_CTL) = reg;
    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setPolarity(int pin, int pol)
*  Description:  set pin to specified polarity (0, normal, 1, inverted)
* ============================================================================
*/
int setPolarity(int gpio, int pol)
{
    int reg = 0;
    int shift = 0;
    int pin = gpio - 18;

    if(pin < 0 || pin > 2)
        return ERRPIN;

    shift = POLARITY + pin*8;
    reg = *(pwm_map + PWM_CTL);
    reg &= ~(1 << shift);
    reg |= (pol << shift);

    *(pwm_map + PWM_CTL) = reg;
    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  PWMCLK_reset()
*  Description:  set PWM clock.
* ============================================================================
*/
#define BCM_PASSWD  0x5A000000
#define CLK_OSC     0     
#define CLK_ENABLE  4
#define CLK_KILL    5
#define CLK_BUSY    7
int PWMCLK_reset(int div0)
{
    /* put the PWM peripheral registers in their original state */
    *(pwm_map + PWM_CTL) = 0;
    *(pwm_map + PWM_RNG(0)) = 0x20;
    *(pwm_map + PWM_DAT(0)) = 0;
    *(pwm_map + PWM_RNG(1)) = 0x20;
    *(pwm_map + PWM_DAT(1)) = 0;

    /* stop clock and waiting for busy flag doesn't work,
     * so kill clock  first. '5A' is CLK password */
    *(clk_map + PWMCLK_CNTL) = BCM_PASSWD | (1 << CLK_KILL);
    usleep(2);  

    // wait until busy flag is set 
    while ((*(clk_map + PWMCLK_CNTL)) & (1<<CLK_BUSY)) { }   

    /* set divisor */
    *(clk_map + PWMCLK_DIV) = BCM_PASSWD | (div0 << 12);

    /* source=osc and enable clock */
    *(clk_map + PWMCLK_CNTL) = BCM_PASSWD | (1<<CLK_ENABLE) | (1<<CLK_OSC);

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setFrequency(int pin, float frequency)
*  Description:  set PWM pin frequency between 0.1Hz and 19.2MHz
* ============================================================================
*/
int setFrequency(int gpio, float frequency)
{
    int counts, bits;
	float f;
    int pin = gpio - 18;
    if (pin < 0 || pin > 2)
        return ERRPIN;

    /* make sure the frequency is valid */
    if (frequency < 0 || frequency > 19200000.0f)
        return ERRFREQ;

	counts = *(pwm_map + PWM_RNG(pin));
    bits = *(pwm_map + PWM_DAT(pin));

    if (counts)
        f = (float)bits/counts;
    else
        f = 0;

    counts = 19200000.0/divisor/frequency;
    bits = counts * f;
    *(pwm_map + PWM_RNG(pin)) = counts;
    *(pwm_map + PWM_DAT(pin)) = bits;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setDutyCycle(int pin, float dutycycle)
*  Description:  set duty cycle from 0% to 100%
* ============================================================================
*/
int setDutyCycle(int gpio, float dutycycle)
{
    int counts, bits;
    int pin = gpio - 18;
    if (pin < 0 || pin > 2)
        return ERRPIN;

    if(dutycycle < 0 || dutycycle > 100.0)
        return ERRDUTY;
    counts = *(pwm_map + PWM_RNG(pin));

    bits = counts * dutycycle/100.0;
    *(pwm_map + PWM_DAT(pin)) = bits;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  PWM_stop()
*  Description:  Puts all Peripheral registers in their original (reset state)
* ============================================================================
*/
void PWM_stop()
{
	PWMCLK_reset(0);
    if(munmap((void*)pwm_map, BLOCK_SIZE) < 0){
        perror("munmap (pwm)");
        exit(1);
    }

    if(munmap((void*)clk_map, BLOCK_SIZE) < 0){
        perror("munmap (clk)");
        exit(1);
    }

    /* reset GPIO18 and GPIO19 to GPIO INPUT */
    *(gpio_map+1) &= ~(7 << 24);
    *(gpio_map+1) &= ~(7 << 27);
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  PWM_init()
*  Description:  map registers, set initial divisor
*                without output signal
* ============================================================================
*/
void PWM_init()
{
    PWMCLK_reset(divisor);
}
