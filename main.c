/*
 * MIT License
 *
 * Copyright (c) 2023 tinyVision.ai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "pico/stdio.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "ice_usb.h"
#include "ice_fpga.h"
#include "ice_led.h"
#include "hardware/i2c.h"
#include "math.h"
//#include "boards/pico_ice.h"

#define I2C_HW i2c1

// Sensor has an address of 0x44
#define OPT_ADDR _u(0x44)

#define REG_CH0       _u(0x0) // 2 bytes each
#define REG_CH1       _u(0x2)
#define REG_CH2       _u(0x4)
#define REG_CH3       _u(0x6)
#define REG_THRESHOLD _u(0x8) // 2 bytes
#define REG_CONFIG    _u(0xA) // 2 bytes
#define REG_STATUS    _u(0xC)
#define REG_DID       _u(0x11)


#define ICE_PMOD3B_I2C_SDA_PIN 19
#define ICE_PMOD3B_I2C_SCL_PIN 18

int init_opt4048(void) {
    uint8_t tx_data[3];
    int ret;
    tx_data[0] = REG_CONFIG;
    tx_data[1] = 0x31; // 6.5ms conv time, @TBD: change this some day so there are no magic numbers
    tx_data[2] = 0x38; // Enable continuous conversions
    ret = i2c_write_blocking(I2C_HW, OPT_ADDR, tx_data, 3, false);
    return ret;
}

int get_adc_code_opt4048(uint8_t channel, uint32_t *adc_code) {
    uint8_t rx_data[4];
    uint8_t reg = channel *2; // Shortcut to the register address
    int ret = i2c_write_blocking(I2C_HW, OPT_ADDR, &reg, 1, false);
    ret = i2c_read_blocking(I2C_HW, OPT_ADDR, rx_data, 4, false);

    // Interpret the received data
    uint32_t mantissa = rx_data[2];
    mantissa += rx_data[1] << 8;
    mantissa += (rx_data[0]&0xf)<<16;

    uint8_t exp = rx_data[0]>>4;
    *adc_code = (mantissa << exp);

    uint8_t count = rx_data[3]>>4;

    //printf("count: %2u, Exp: %u, Mantissa: %u, Code: %u\n",count, exp, mantissa, *adc_code);

    return ret;
}

int main(void) {
    stdio_init_all(); // uses CDC0, next available is CDC1
    tusb_init();

    // Let the FPGA start and give it a clock
    ice_fpga_init(48);
    ice_fpga_start();

    // Enable the UART
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // Bind USB CDC1 callback for piping input data to UART0
    tud_cdc_rx_cb_table[0] = &ice_usb_cdc_to_uart0;
    tud_cdc_rx_cb_table[1] = &ice_usb_cdc_to_uart0;

    // Bind UART0 interrupt for piping to USB CDC1
    irq_set_exclusive_handler(UART0_IRQ, ice_usb_uart0_to_cdc1);

    ice_led_init();

    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(I2C_HW, 100 * 1000);
    gpio_init(ICE_PMOD3B_I2C_SDA_PIN);
    gpio_init(ICE_PMOD3B_I2C_SCL_PIN);
    gpio_set_dir(ICE_PMOD3B_I2C_SDA_PIN, GPIO_OUT);
    gpio_set_dir(ICE_PMOD3B_I2C_SCL_PIN, GPIO_OUT);

    gpio_set_function(ICE_PMOD3B_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ICE_PMOD3B_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ICE_PMOD3B_I2C_SDA_PIN);
    gpio_pull_up(ICE_PMOD3B_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    init_opt4048();

    uint16_t cntr = 0;
    uint32_t code[4];
    int ret, i;
    double x, y, z, cie_x, cie_y, lux, num, den, n, temp;
    while(true) {
        if (cntr == 500) { // Every ~500ms
            
            // Get the raw RGBW values
            for (i=0; i<4; i++) get_adc_code_opt4048(i, (code+i));
            
            lux = code[1]*2.15e-3; // Green channel is used for lux

            // Convert from RGB to XYZ
            x = 2.34892992e-4  * code[0] + 4.07467441e-5 * code[1] + 9.28619404e-5 * code[2];
            y = -1.89652390e-5 * code[0] + 1.98958202e-4 * code[1] - 1.69739553e-5 * code[2];
            z = 1.20811684e-5  * code[0] - 1.58848115e-5 * code[1] + 6.74021520e-4 * code[2];

            // Then to CIE coordinates
            cie_x = x/(x+y+z);
            cie_y = y/(x+y+z);

            // Get the approx. color temp from the CIE coordinates
            num = (cie_x - 0.332);
            den = (0.1858 - cie_y);
            n = num/den;
            temp = 437*n*n*n + 3601*n*n + 6861*n + 5517;

            printf("Lux: %u, temp: %u\n", (int)lux, (int)temp);
            cntr = 0;
        }

        tud_task();
        sleep_ms(1);
        cntr++;
        }

}

