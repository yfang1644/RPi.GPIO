#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "c_gpio.h"
#include "hard_pwm.h"

extern volatile uint32_t *gpio_map;
volatile uint32_t *clk_map=NULL, *pwm_map=NULL;

static int divisor = 2;

/* 
* ===  FUNCTION  =============================================================
*         Name:  PWM_enable(int gpio, bool enable)
*  Description:  enable or disable PWM pin
* ============================================================================
*/
int PWM_enable(int gpio, _Bool enable)
{
    int reg = 0;
    int enable_bit = 0;
    int pwm = gpio - 18;

    if (pwm < 0 || pwm > 1)
        return ERRPIN;

    enable_bit = PWM_ENABLE << (pwm*8);
    reg = *(pwm_map + PWM_CTL);
    reg &= ~enable_bit;
    if (enable)
        reg |= enable_bit;

    *(pwm_map + PWM_CTL) = reg;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setup_hard_pwm
*  Description:  
* ============================================================================
*/
int setup_hard_pwm(int gpio)
{
    int offset = (gpio/10);
    int shift = (gpio%10)*3;
    int reg = 0;

    reg = *(gpio_map+offset);
    reg &= ~(7 << shift);

    if (gpio == 18 || gpio==19) {
        reg |= (ALT5 <<shift);
    }
    *(gpio_map + offset) = reg;
    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setMode(int gpio, int mode)
*  Description:  set gpio pin to PWM mode
* ============================================================================
*/
int setMode(int gpio, int mode)
{
    int reg = 0;
    int shift = 0;
    int pwm = gpio - 18;

    if (pwm < 0 || pwm > 1)
        return ERRPIN;

    shift = mode << (pwm*8);
    reg = *(pwm_map + PWM_CTL);
    reg &= ~(0xff << (pwm*8));
    reg |= shift;

    *(pwm_map + PWM_CTL) = reg;
    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setPolarity(int gpio, int pol)
*  Description:  set pin to specified polarity (0, normal, 1, inverted)
* ============================================================================
*/
int setPolarity(int gpio, _Bool pol)
{
    int reg = 0;
    int polarity_bit = 0;
    int pwm = gpio - 18;

    if(pwm < 0 || pwm > 1)
        return ERRPIN;

    polarity_bit = PWM_POLARITY << (pwm*8);
    reg = *(pwm_map + PWM_CTL);
    reg &= ~polarity_bit;
    if (pol)
        reg |= polarity_bit;

    *(pwm_map + PWM_CTL) = reg;
    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  PWMCLK_reset()
*  Description:  set PWM clock.
* ============================================================================
*/
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
    *(clk_map + PWMCLK_CNTL) = BCM_PASSWD | CLK_KILL;
    usleep(2);  

    // wait until busy flag is set 
    while ((*(clk_map + PWMCLK_CNTL)) & CLK_BUSY)   ;

    /* set divisor */
    *(clk_map + PWMCLK_DIV) = BCM_PASSWD | (div0 << 12);

    /* source=osc and enable clock */
    *(clk_map + PWMCLK_CNTL) = BCM_PASSWD | CLK_ENABLE | CLK_OSC;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setFrequency(int gpio, float frequency)
*  Description:  set PWM pin frequency between 0.1Hz and 19.2MHz
* ============================================================================
*/
int setFrequency(int gpio, float frequency)
{
    int counts, bits;
    float f;
    int pwm = gpio - 18;
    if (pwm < 0 || pwm > 1)
        return ERRPIN;

    /* make sure the frequency is valid */
    if (frequency < 0 || frequency > 19200000.0f)
        return ERRFREQ;

    counts = *(pwm_map + PWM_RNG(pwm));
    bits = *(pwm_map + PWM_DAT(pwm));

    if (counts)
        f = (float)bits/counts;
    else
        f = 0;

    counts = 19200000.0/divisor/frequency;
    bits = counts * f;
    *(pwm_map + PWM_RNG(pwm)) = counts;
    *(pwm_map + PWM_DAT(pwm)) = bits;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  setDutyCycle(int gpio, float dutycycle)
*  Description:  set duty cycle from 0% to 100%
* ============================================================================
*/
int setDutyCycle(int gpio, float dutycycle)
{
    int counts, bits;
    int pwm = gpio - 18;
    if (pwm < 0 || pwm > 1)
        return ERRPIN;

    if(dutycycle < 0 || dutycycle > 100.0)
        return ERRDUTY;
    counts = *(pwm_map + PWM_RNG(pwm));

    bits = counts * dutycycle/100.0;
    *(pwm_map + PWM_DAT(pwm)) = bits;

    return 0;
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  PWM_stop()
*  Description:  Puts all Peripheral registers in their original (reset state)
* ============================================================================
*/
void hardwarePWM_stop()
{
    if (pwm_map != NULL) {
        PWMCLK_reset(0);
        munmap((void*)pwm_map, BLOCK_SIZE);
        pwm_map = NULL;
    }

    if(clk_map != NULL) {
        munmap((void*)clk_map, BLOCK_SIZE);
        clk_map = NULL;
    }
    /* reset GPIO18 and GPIO19 to GPIO INPUT */
    setup_gpio(18, INPUT, PUD_OFF);
    setup_gpio(19, INPUT, PUD_OFF);
}

/* 
* ===  FUNCTION  =============================================================
*         Name:  hardwarePWM_init()
*  Description:  map registers, set initial divisor
*                without output signal
* ============================================================================
*/
void hardwarePWM_init()
{
    if (pwm_map == NULL)
        pwm_map = mapRegAddr(peri_base + PWM_BASE_OFFSET);
    if (clk_map == NULL) {
        clk_map = mapRegAddr(peri_base + CLK_BASE_OFFSET);
        PWMCLK_reset(divisor);
    }
}
