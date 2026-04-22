#include "button.h"

void button_init() {
    static uint8_t initialized = 0;
    if (initialized) return;

    // 1) Gate control when enabled to run a GPIO enables a clock and access to module
    //Used to save power
    SYSCTL_RCGCGPIO_R |= 0x10;
    //this is timer
    //Indicates if ready to be accessed
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {}

    // 2) PE3-PE0 inputs  Direction determines input or output
    GPIO_PORTE_DIR_R &= ~0x0F;

    // 3) digital enable PE3-PE0
    //Activated by default when deactive does not allow voltage
    GPIO_PORTE_DEN_R |= 0x0F;

    initialized = 1;
}


uint8_t button_getButton() {
    uint32_t data = GPIO_PORTE_DATA_R & 0x0F;

    // Active-low: pressed == 0 on that bit
    // PE3->Button1 (leftmost), PE2->Button2, PE1->Button3, PE0->Button4 (rightmost)

    if ((data & 0x01) == 0) return 1; // PE0 pressed
    if ((data & 0x02) == 0) return 2; // PE1 pressed
    if ((data & 0x04) == 0) return 3; // PE2 pressed
    if ((data & 0x08) == 0) return 4; // PE3 pressed
    return 0;
}
