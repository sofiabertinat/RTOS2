/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"   //Motor del OS
#include "task.h"		//Api de control de tareas y temporizaciÃ³n
#include "semphr.h"		//Api de sincronizaciÃ³n (sem y mutex)
#include "queue.h"      //Api de colas

// sAPI header
#include "sapi.h"

#include "FreeRTOSConfig.h"
#include "board.h"


/*==================[definiciones y macros]==================================*/
#define UP      1
#define FALLING 2
#define DOWN	3
#define RISING  4

#define CANT_TECLAS 4
#define CANT_LEDS 4
#define ANTIREBOTE_MS 20

//indices de teclas para el vector de estructuras
enum Teclas_t {Tecla1, Tecla2, Tecla3, Tecla4};

//estructura de control de datos capturados por la interrupcion
struct Button_Control {
	TickType_t Tiempo_inicial;
	uint8_t Flanco;
	uint8_t Tecla;
};

//estructura de control de la mÃ¡quina de estados de cada boton
struct Buttons_SM_t{
	uint8_t Estado;
	TickType_t Tiempo_inicial;
};

struct Lectura_t{
	uint8_t Tecla;
	TickType_t Tiempo_medido;
};

/*==================[definiciones de datos internos]=========================*/
SemaphoreHandle_t Mutex_uart; //Mutex que protege la UART de concurrencia

xQueueHandle Cola_Lecturas;
xQueueHandle Cola_Control_Teclas;

//Definicion de vector de estructuras de control
struct Buttons_SM_t Buttons_SM[CANT_TECLAS];

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
//FunciÃ³n de inicializaciÃ³n de interrupciones
void My_IRQ_Init (void);

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void Tecla_task( void* taskParmPtr );
void Led_task( void* taskParmPtr );
void Uart_task( void* taskParmPtr );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   uint8_t Error_state = 0;
	// ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   //Iniciamos las interrupciones
   My_IRQ_Init();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );
   printf( "Antirrebote con IRQ freeRTOS y sAPI\r\n" );

   /* Creamos colas de capturas de teclas */
	if (NULL == (Cola_Control_Teclas = xQueueCreate(10,sizeof(struct Button_Control)))){
	   Error_state =1;
	}

   	/* Creamos cola de lecturas completadas */
   	if (NULL == (Cola_Lecturas = xQueueCreate(10,sizeof(struct Lectura_t)))){
   	   	   Error_state =1;
   	   }

   if (NULL == (Mutex_uart = xSemaphoreCreateMutex())){
   	   Error_state =1;
   }

   // Crear tarea de Teclas en freeRTOS
   xTaskCreate(Tecla_task, (const char *)"Tec", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0 );

   // Crear tarea LED en freeRTOS,Lectura con antirrebote de 2 teclas y medición de tiempo de pulsación
   // Al medirse el tiempo de pulsación, enviar el mensaje "TECx Tyyyy" a la "cola_1"
   //donde x es el índice de tecla e yyyy la cantidad  de ms que fué pulsada.
   xTaskCreate( Led_task, (const char *)"Led", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+2, 0  );

   // Crear tarea Uart en freeRTOS, Deberá obtener mensajes de texto, y enviarlos por la uart
   xTaskCreate( Uart_task, (const char *)"uart", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+2, 0  );

   // Iniciar scheduler
   if (0 == Error_state){
  	  vTaskStartScheduler();
   } else{
	  printf("Error al iniciar el sistema !!!!!!!!!!!!!!");
   }

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}


#define MY_ASSERT(CONDICION) my_assert_debug(CONDICION)

/*==================[definiciones de funciones internas]=====================*/

//FunciÃ³n de inicializacion de IRQs
void My_IRQ_Init (void){
		//Inicializamos las interrupciones (LPCopen)
		Chip_PININT_Init(LPC_GPIO_PIN_INT);

		//Inicializamos de cada evento de interrupcion (LPCopen)

		// TEC1 FALL
		Chip_SCU_GPIOIntPinSel(0, 0, 4); 	//(Canal 0 a 7, Puerto GPIO, Pin GPIO)
		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);//Se configura el canal para que se active por flanco
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);//Se configura para que el flanco sea el de bajada

		// TEC1 RISE
		Chip_SCU_GPIOIntPinSel(1, 0, 4);	//(Canal 0 a 7, Puerto GPIO, Pin GPIO)
		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);//Se configura el canal para que se active por flanco
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH1);//En este caso el flanco es de subida

		// TEC2 FALL
		Chip_SCU_GPIOIntPinSel(2, 0, 8);
		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH2);
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH2);

		// TEC2 RISE
		Chip_SCU_GPIOIntPinSel(3, 0, 8);
		Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH3);
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH3);


		//Una vez que se han configurado los eventos para cada canal de interrupcion
		//Se activan las interrupciones para que comiencen a llamar al handler
		NVIC_SetPriority(PIN_INT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(PIN_INT0_IRQn);
		NVIC_SetPriority(PIN_INT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(PIN_INT1_IRQn);
		NVIC_SetPriority(PIN_INT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(PIN_INT2_IRQn);
		NVIC_SetPriority(PIN_INT3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(PIN_INT3_IRQn);

}

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea genÃ©rica Tecla
void Tecla_task( void* taskParmPtr ){

	//Definicion de vector de estructuras de control
	static struct Buttons_SM_t Tecla_SM[CANT_TECLAS];

	struct Lectura_t Lectura;

	struct Button_Control Control;

	TickType_t Last_Snapshot = 0;

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {

	   if (xQueueReceive(Cola_Control_Teclas, &Control, portMAX_DELAY)){

			   switch (Tecla_SM[Control.Tecla].Estado){

					 case UP:
						  if(Control.Flanco == FALLING){

							  if (pdFALSE == (xQueueReceive(Cola_Control_Teclas, &Control, (ANTIREBOTE_MS / portTICK_RATE_MS)))){
								  Tecla_SM[Control.Tecla].Estado = DOWN;
								  Tecla_SM[Control.Tecla].Tiempo_inicial = Control.Tiempo_inicial;

							  }
						  }
					 break;

					 case DOWN:
						 if(Control.Flanco == RISING){

							  if (pdFALSE == (xQueueReceive(Cola_Control_Teclas, &Control, (ANTIREBOTE_MS / portTICK_RATE_MS)))){
								  Tecla_SM[Control.Tecla].Estado = UP;
								  Lectura.Tecla = Control.Tecla;
								  Lectura.Tiempo_medido = xTaskGetTickCount() - Tecla_SM[Control.Tecla].Tiempo_inicial;
								  xQueueSend(Cola_Lecturas, &Lectura, portMAX_DELAY);
							  }
						  }
					 break;

					 default:
						 Tecla_SM[Control.Tecla].Estado = UP;
					 break;

			  }
	   	  }
	}
}

// Implementacion de funcion de la tarea Led
void Uart_task( void* taskParmPtr ){

	struct Lectura_t Lectura;

	while (TRUE){
		//Espero evento de Lectura completada
		if (xQueueReceive(Cola_Lecturas, &Lectura, portMAX_DELAY)){

			if (pdTRUE == xSemaphoreTake( Mutex_uart, portMAX_DELAY)){

				if(Lectura.Tiempo_medido == 0){

					printf("LED ON\r\n");
				}else{

				  printf("TEC%d, T %dms\r\n",Lectura.Tecla+1,Lectura.Tiempo_medido*portTICK_RATE_MS);
				}

				  xSemaphoreGive( Mutex_uart );

			}


		}
	}
}



// Implementacion de funcion de la tarea Led
void Led_task( void* taskParmPtr ){

	struct Lectura_t Lectura;

	while (TRUE){


			vTaskDelay( 1000 );

			gpioWrite(LED1,ON);

			Lectura.Tiempo_medido = 0;
			xQueueSend(Cola_Lecturas, &Lectura, portMAX_DELAY);

			//Espero tiempo de encendido
			vTaskDelay( 1000 );

			gpioWrite(LED1,OFF);


	}
}


void GPIO0_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable


	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0){ //Verificamos que la interrupciÃ³n es la esperada
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0); //Borramos el flag de interrupciÃ³n

		struct Button_Control Snapshot;
		Snapshot.Flanco = FALLING;
		Snapshot.Tiempo_inicial = xTaskGetTickCountFromISR();
		Snapshot.Tecla =Tecla1;

		xQueueSendFromISR( Cola_Control_Teclas, &Snapshot, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO1_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if (Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH1){
			Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);

			struct Button_Control Snapshot;
			Snapshot.Flanco = RISING;
			Snapshot.Tiempo_inicial = xTaskGetTickCountFromISR();
			Snapshot.Tecla =Tecla1;

			xQueueSendFromISR(Cola_Control_Teclas, &Snapshot, &xHigherPriorityTaskWoken );
		}
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO2_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable


	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH2){ //Verificamos que la interrupciÃ³n es la esperada
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2); //Borramos el flag de interrupciÃ³n

		struct Button_Control Snapshot;
		Snapshot.Flanco = FALLING;
		Snapshot.Tiempo_inicial = xTaskGetTickCountFromISR();
		Snapshot.Tecla =Tecla2;

		xQueueSendFromISR( Cola_Control_Teclas, &Snapshot, &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO3_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if (Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH3){
			Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH3);

			struct Button_Control Snapshot;
			Snapshot.Flanco = RISING;
			Snapshot.Tiempo_inicial = xTaskGetTickCountFromISR();
			Snapshot.Tecla =Tecla2;

			xQueueSendFromISR( Cola_Control_Teclas, &Snapshot, &xHigherPriorityTaskWoken );
		}
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



/*==================[fin del archivo]========================================*/
