#include "serial.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define RX_BUF_SIZE 512

#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

void serialInit(void){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM_0,      /* Identificación de la UART*/
        RX_BUF_SIZE,     /* Tamaño del buffer de recepción */ 
        0,               /* Tamño del buffer de transmisión (no se usa)*/
        0,               /* Tamño de la cola */
        NULL,            /* Cola a utilizar */
        0                /* Parámetros asociados a la interrución (no utilizado)*/
    ));

    ESP_ERROR_CHECK(uart_param_config(
        UART_NUM_0,  /* Identificación de la UART */ 
        &uart_config /* Estructura con la configuración que deseamos aplicar */
    ));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM_0, /*  Identificación de la UART a usar */
        TXD_PIN,    /* Pin utilizado para la transmición */
        RXD_PIN,    /* Pin utilizado para la recepsión   */
        UART_PIN_NO_CHANGE, /* Pines utilizados para señalización, rts*/
        UART_PIN_NO_CHANGE /* cts. No se utiliza señalización por hardware. */
        ));

}

void serialWrite(uint8_t val){
  uart_write_bytes(UART_NUM_0, &val, 1);
}

uint8_t serialRead(uint8_t *val){
        return uart_read_bytes(UART_NUM_0, val, 1, 10 / portTICK_PERIOD_MS);
}