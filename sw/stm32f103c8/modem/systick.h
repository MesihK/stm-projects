#include <libopencm3/cm3/systick.h>
#include <stdint.h>

extern volatile uint32_t system_millis;
void msleep(uint32_t delay);
void systick_setup(void);
