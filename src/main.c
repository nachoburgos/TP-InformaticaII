#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include <freertos/queue.h>
#include "serial.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>

#define pkg_h 0x0C
#define ledPin GPIO_NUM_2

void hardware_init(void);
void ProgramaPwm(double high, double low);

typedef enum {
    ST_Header=0,
    ST_NivelAlto,
    ST_NivelBajo,
    ST_SumaComprobacion
} ValorEstado;


typedef union {
  unsigned short val;
  uint8_t p[2];
} shortVal;

typedef struct {
  shortVal Valor;
  uint8_t inx;
} shortParts;

typedef struct
{
    ValorEstado ActividadEstado;
    shortParts ValorAlto;
    shortParts ValorBajo;
    unsigned int sum;
}Estado;

typedef bool (*fcnState)(Estado *, unsigned char);
bool EstadoHeader(Estado *s, unsigned char ValorNuevo);
bool EstadoAlto(Estado *s, unsigned char ValorNuevo);
bool EstadoBajo(Estado *s, unsigned char ValorNuevo);
bool EstadoSumaComprobacion(Estado *s, unsigned char ValorNuevo);


void hardware_init(void)
{
  gpio_config_t gpioLed =
      {
          .pin_bit_mask = 1ULL << ledPin,
          .mode = GPIO_MODE_DEF_OUTPUT,
          .pull_up_en = GPIO_PULLUP_DISABLE,
          .pull_down_en = GPIO_PULLDOWN_DISABLE,
          .intr_type = GPIO_INTR_DISABLE};
  ESP_ERROR_CHECK(gpio_config(&gpioLed));
}

void PwmProgramer(double high, double low){

        gpio_set_level(ledPin, 1);
        vTaskDelay(high / portTICK_PERIOD_MS);
        gpio_set_level(ledPin, 0);
        vTaskDelay(low / portTICK_PERIOD_MS);
    
}
QueueHandle_t cola;


// Tarea del Estado GPIO 2 (Manejo Led)

void led_management_task(void *param) {
    Estado ledParams, TEM;
   
    uint8_t vect[6];
    ledParams.ValorAlto.Valor.val=50;
    ledParams.ValorBajo.Valor.val=50;
    while (1) {
        while (xQueueReceive(cola, &TEM, 0) ) {
          ledParams = TEM;
          vect[5]=pkg_h+ (TEM.ValorAlto.Valor.val % 256)+
           (TEM.ValorAlto.Valor.val>>8) +
           (TEM.ValorBajo.Valor.val % 256 )+ (TEM.ValorBajo.Valor.val >>8);
           vect[0]=pkg_h;
           vect[1]=TEM.ValorAlto.Valor.val % 256;
           vect[2]=TEM.ValorAlto.Valor.val >> 8;
           vect[3]=TEM.ValorBajo.Valor.val % 256;
           vect[4]=TEM.ValorBajo.Valor.val >> 8;
           for(int i =0;i<6;i++){
           serialWrite(vect[i]);
           }
        }
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay((ledParams.ValorAlto.Valor.val)/portTICK_PERIOD_MS);
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay((ledParams.ValorBajo.Valor.val)/portTICK_PERIOD_MS);

    }
    }


//Comunicacion Puerto Serie - UART


void serial_comm_task( void *param){
   uint8_t val;
   static fcnState listEstado[4] = {
      EstadoHeader,
      EstadoAlto,
      EstadoBajo,
      EstadoSumaComprobacion};
  static Estado vEstado = {ST_Header, {{0},0}, {{0},0}, 0};
  while (1)
  {
   if(serialRead(&val)){ 
    if(listEstado[vEstado.ActividadEstado](&vEstado, val)){

        xQueueSendToBack(cola, &vEstado, portMAX_DELAY); 
      } 
    } 
  }
}



void app_main() {

  // INICIALIZACION

   serialInit();
   hardware_init();

   cola = xQueueCreate(10, sizeof(Estado));

   // CREANDO TAREAS

   xTaskCreate(led_management_task, "Led Task", configMINIMAL_STACK_SIZE + 4096, NULL, 1, NULL); 
   xTaskCreate(serial_comm_task, "Serial Task", configMINIMAL_STACK_SIZE + 4096, NULL, 1, NULL); 

}


//Maquina de Estado

bool EstadoHeader(Estado *s, unsigned char ValorNuevo)
{
    if(ValorNuevo == 0x0B)
    {
        s->ActividadEstado = ST_NivelAlto;
        s->ValorAlto.inx = 0;
        s->ValorBajo.inx = 0;
        s->sum = 0x0B;
    }
    else
        s->ActividadEstado = ST_Header;
    return false;
}

bool EstadoAlto(Estado *s, unsigned char ValorNuevo)
{

    s->ValorAlto.Valor.p[s->ValorAlto.inx] = ValorNuevo;
    s->sum += ValorNuevo;
    s->ValorAlto.inx++;
    if(s->ValorAlto.inx == sizeof(short))
      s->ActividadEstado = ST_NivelBajo;
    return false;
}

bool EstadoBajo(Estado *s, unsigned char ValorNuevo)
{
    s->ValorBajo.Valor.p[s->ValorBajo.inx] = ValorNuevo;
    s->sum += ValorNuevo;
    s->ValorBajo.inx++;
    if(s->ValorBajo.inx == sizeof(short))
      s->ActividadEstado = ST_SumaComprobacion;
    return false;
}

bool EstadoSumaComprobacion(Estado *s, unsigned char ValorNuevo)
{
    bool ret = false;

    if(ValorNuevo == (s->sum%256))
        ret = true;
    s->ActividadEstado = ST_Header;
    return ret;
}