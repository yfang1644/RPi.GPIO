/*
Copyright (c) 2013-2015 Ben Croston

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

#define NO_EDGE      0
#define RISING_EDGE  1
#define FALLING_EDGE 2
#define BOTH_EDGE    3

struct gpios
{
    int gpio;
    int value_fd;
    int exported;
    int edge;
    int initial_thread;
    int initial_wait;
    int thread_added;
    int bouncetime;
    unsigned long long lastcall;
    struct gpios *next;
};

// event callbacks
struct callback
{
    int gpio;
    void (*func)(int gpio);
    struct callback *next;
};

int add_edge_detect(int gpio, int edge, int bouncetime);
void remove_edge_detect(int gpio);
int add_edge_callback(int gpio, void (*func)(int gpio));
int event_detected(int gpio);
int gpio_event_added(int gpio);
void event_cleanup(int gpio);
void event_cleanup_all(void);
int blocking_wait_for_edge(int gpio, int edge, int bouncetime, int timeout);
