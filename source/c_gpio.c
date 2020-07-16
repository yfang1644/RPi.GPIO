/*
Copyright (c) 2012-2019 Ben Croston

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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include "c_gpio.h"

volatile uint32_t *gpio_map=NULL;
uint32_t peri_base = 0;

void short_wait(int cycle)
{
    int i;

    for (i=0; i<cycle; i++) {    // wait 150 cycles
        asm volatile("nop");
    }
}

void *mapRegAddr(unsigned int baseAddr)
{
    uint32_t pagemask = ~0UL ^ (getpagesize() - 1);
    uint32_t offsetmask = getpagesize() - 1;
    int mem_fd;
    void *regAddrMap = MAP_FAILED;

    // mmap the GPIO memory registers
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0)
        return MAP_FAILED;

    regAddrMap = mmap(NULL,
                      BLOCK_SIZE,
                      PROT_READ|PROT_WRITE,
                      MAP_SHARED,
                      mem_fd,
                      baseAddr & pagemask);

    close(mem_fd);

    return (char *)regAddrMap + (baseAddr & offsetmask);
}

int setup(void)
{
    unsigned char buf[4];
    FILE *fp;
    char buffer[1024];
    char hardware[1024];
    int found = 0;

    // determine peri_base
    if ((fp = fopen("/proc/device-tree/soc/ranges", "rb")) != NULL) {
        // get peri base from device tree
        fseek(fp, 4, SEEK_SET);
        if (fread(buf, 1, sizeof buf, fp) == sizeof buf) {
            peri_base = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0;
        }
        fclose(fp);
    } else {
        // guess peri base based on /proc/cpuinfo hardware field
        if ((fp = fopen("/proc/cpuinfo", "r")) == NULL)
            return SETUP_CPUINFO_FAIL;

        while(!feof(fp) && !found && fgets(buffer, sizeof(buffer), fp)) {
            sscanf(buffer, "Hardware : %s", hardware);
            if (strcmp(hardware, "BCM2708") == 0 || strcmp(hardware, "BCM2835") == 0) {
                // pi 1 hardware
                peri_base = BCM2708_PERI_BASE_DEFAULT;
                found = 1;
            } else if (strcmp(hardware, "BCM2709") == 0 || strcmp(hardware, "BCM2836") == 0) {
                // pi 2 hardware
                peri_base = BCM2709_PERI_BASE_DEFAULT;
                found = 1;
            }
        }
        fclose(fp);
        if (!found)
            return SETUP_NOT_RPI_FAIL;
    }

    if (!peri_base)
        return SETUP_NOT_RPI_FAIL;

    if (gpio_map == NULL)
        gpio_map = mapRegAddr(peri_base + GPIO_BASE_OFFSET);

    return SETUP_OK;
}

int eventdetected(int gpio)
{
    int offset, value, bit;

    offset = EVENT_DETECT_OFFSET + (gpio/32);
    bit = (1 << (gpio%32));
    value = *(gpio_map+offset) & bit;
    if (value)
        *(gpio_map+offset) |= bit;      /* clear bit */
    return value;
}

void set_rising_event(int gpio, int enable)
{
    int offset = RISING_ED_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable) {
        *(gpio_map+offset) |= 1 << shift;
    } else {
        *(gpio_map+offset) &= ~(1 << shift);
    }
}

void set_falling_event(int gpio, int enable)
{
    int offset = FALLING_ED_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable) {
        *(gpio_map+offset) |= (1 << shift);
    } else {
        *(gpio_map+offset) &= ~(1 << shift);
    }
}

void set_high_event(int gpio, int enable)
{
    int offset = HIGH_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable) {
        *(gpio_map+offset) |= (1 << shift);
    } else {
        *(gpio_map+offset) &= ~(1 << shift);
    }
}

void set_low_event(int gpio, int enable)
{
    int offset = LOW_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

    if (enable) {
        *(gpio_map+offset) |= 1 << shift;
    } else {
        *(gpio_map+offset) &= ~(1 << shift);
    }
}

void set_pullupdn(int gpio, int pud)
{
    int clk_offset = PULLUPDNCLK_OFFSET + (gpio/32);
    int shift = (gpio%32);
    int reg = 0;
    reg = *(gpio_map + PULLUPDN_OFFSET);
    reg &= ~3;
    reg |= pud;
    *(gpio_map+PULLUPDN_OFFSET) = reg;

    short_wait(150);
    *(gpio_map+clk_offset) = 1 << shift;
    short_wait(150);
    *(gpio_map+PULLUPDN_OFFSET) &= ~3;
    *(gpio_map+clk_offset) = 0;
}

void setup_gpio(int gpio, int direction, int pud)
{
    int offset = FSEL_OFFSET + (gpio/10);
    int shift = (gpio%10)*3;
    int reg = 0;

    set_pullupdn(gpio, pud);

    reg = *(gpio_map+offset);
    reg &= ~(7 << shift);

    if ((direction == INPUT) || (direction == OUTPUT))
        reg |= (direction <<shift);

    *(gpio_map + offset) = reg;
}

/* return gpio function */
int gpio_function(int gpio)
{
    int offset = FSEL_OFFSET + (gpio/10);
    int shift = (gpio%10)*3;
    int value = *(gpio_map+offset);
    value >>= shift;
    value &= 7;
    return value; // 0=input, 1=output, 4=alt0, 2=alt5
}

void output_gpio(int gpio, int value)
{
    int offset, shift;

    if (value) // value == HIGH
        offset = SET_OFFSET + (gpio/32);
    else       // value == LOW
       offset = CLR_OFFSET + (gpio/32);

    shift = (gpio%32);

    *(gpio_map+offset) = 1 << shift;
}

int input_gpio(int gpio)
{
   int offset, value, mask;

   offset = PINLEVEL_OFFSET + (gpio/32);
   mask = (1 << gpio%32);
   value = *(gpio_map+offset) & mask;
   return value;
}

void hardwarePWM_stop(void);

void cleanup(void)
{
    hardwarePWM_stop();
    if(gpio_map != NULL) {
        munmap((void *)gpio_map, BLOCK_SIZE);
        gpio_map = NULL;
    }
}
