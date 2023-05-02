#include "toggler.h"
#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>

//TODO: Change this to use proper device tree as samuel did, not DK buttons and leds.

void togglePinSetup(void) {
}
void toggleClientPin(void) {
  static int toggle = 0;
  dk_set_led(DK_LED2, toggle);
  toggle = !toggle;
}
void toggleServerPin(void) {
  static int toggle = 0;
  dk_set_led(DK_LED4, toggle);
  toggle = !toggle;
}
