#ifdef __VARS__
#define __VARS__

int FADESTEP = 500;
int vcc = 0;
int led_blink_state = 1;
int pulse_width = 0;

struct device  *uart;

#endif