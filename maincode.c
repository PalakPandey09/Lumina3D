#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/multicore.h"
#include "hardware/structs/systick.h"

#define IR_SENSOR 28
#define MOTOR 29

#define PANEL_WIDTH 32
#define PANEL_HEIGHT 64

volatile bool active_frame = 0;
uint32_t framebuffer[2][PANEL_WIDTH][PANEL_HEIGHT];

volatile uint32_t period = 0;
volatile bool idle = 1;
int mode = 0;

#define SIG_START 1
#define MOTOR_TIMEOUT_MS 250
#define IDLE_TIMEOUT_MS 1000

#define SYSTICK_RVR 0x00FFFFFF

void sleep_cycles_break(uint32_t cycles){
    systick_hw->cvr = systick_hw->rvr;
    uint32_t until = systick_hw->rvr - cycles;
    while (systick_hw->cvr > until) {
        if ( multicore_fifo_get_status() &(1<<0) ) return;
    }
}

void draw_slice(uint32_t slice){
    // For simplicity, let's just turn on all pixels in this example
    for(int y = 0; y < PANEL_HEIGHT; y++) {
        framebuffer[active_frame][slice][y] = 0xFFFFFFFF; // Set all pixels to white
    }
}

void core1_entry(void){
    systick_hw->csr = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS;
    systick_hw->rvr = SYSTICK_RVR;

    while (1) {
        start:

        while (multicore_fifo_pop_blocking() != SIG_START);

        for (int i = 0; i < PANEL_WIDTH; i++) {
            draw_slice(i);

            if (multicore_fifo_get_status() & (1 << 0)) goto start;
        }

        // delay but break early on signal
        for (int i = 0; i < MOTOR_TIMEOUT_MS; i++) {
            if (multicore_fifo_get_status() & (1 << 0)) goto start;
            sleep_cycles_break(125000);
        }

        // motor off
        pwm_set_gpio_level(MOTOR, 0);

        for (int i = 0; i < IDLE_TIMEOUT_MS; i++) {
            if (multicore_fifo_get_status() & (1 << 0)) goto start;
            sleep_cycles_break(125000);
        }

        idle = 1;
    }
}

void load_frame(const uint32_t* data){
    int k = 0;
    bool n = !active_frame;
    for (int i = 0; i < PANEL_WIDTH; i++) {
        for (int j = 0; j < PANEL_HEIGHT; j++) {
            framebuffer[n][i][j] = data[k++];
        }
    }
    active_frame = !active_frame;
}

void clr(){
    for (int i = 0; i < PANEL_WIDTH; i++) {
        for (int j = 0; j < PANEL_HEIGHT; j++) {
            framebuffer[active_frame][i][j] = 0x00000000; // Clear all pixels
        }
    }
}

void set_voxel(uint32_t x, uint32_t y, uint32_t color){
    // Set a specific pixel with a specific color
    framebuffer[active_frame][x][y] = color;
}

int main(){
    gpio_init_mask((1 << MOTOR) | (1 << IR_SENSOR));

    gpio_set_function(MOTOR, GPIO_FUNC_PWM); // Channel 6B
    uint slice_num = pwm_gpio_to_slice_num(MOTOR);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    pwm_init(slice_num, &config, true);

    multicore_launch_core1(core1_entry);

    systick_hw->csr = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS;
    systick_hw->rvr = SYSTICK_RVR;

    load_frame(&framebuffer[0][0][0]);

    while (1) {
        while (gpio_get(IR_SENSOR) == 1);
        uint32_t t0 = systick_hw->cvr;
        multicore_fifo_push_blocking(SIG_START);
        systick_hw->cvr = SYSTICK_RVR;
        period = SYSTICK_RVR - t0;

        if (idle) {
            // Additional functionality like checking battery can be added here
            clr(); // Clear the framebuffer
        }
        idle = 0;

        if (period > 6250000) pwm_set_gpio_level(MOTOR, 0.9 * 65535);
        else pwm_set_gpio_level(MOTOR, 0.6 * 65535);

        while (gpio_get(IR_SENSOR) == 0) sleep_us(1);
    }
}
