/**
  ******************************************************************************
  * @file    main.c
  * @author  Nirgal
  * @date    03-July-2019
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f1xx_hal.h"
#include "stm32f1_uart.h"
#include "stm32f1_sys.h"
#include "stm32f1_gpio.h"
#include "macro_types.h"
#include "systick.h"
#include "stm32f1_timer.h"
#include "stm32f1xx.h"

uint16_t position = 0;
uint16_t butt_test = 0;
#define MAX_DURATION 200 // durée maximale d'une ....
#define MAX_POSITION 200
#define MIN_POSITION 0
int target_position = 100; // Position cible
static uint16_t current_position;
int adc_value = 0;

static void SUNBED_state_machine(void);

void writeLED(bool_e b)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b);
}

bool_e readButton(void)
{
	return !HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN);
}

static volatile uint32_t t = 0;
void process_ms(void)
{
	if(t)
		t--;
}

void SERVO_set_position(uint16_t position) {
	if (position > 100)
		position = 100; // Ã©cretage si l'utilisateur demande plus de 100%
	// TODO : mise Ã  jour du rapport cyclique.
	// duty doit Ãªtre exprimÃ© ici de 100 Ã  200 (sur 1000) (pour un rapport cyclique de 10% Ã  20%, câest-Ã -dire une durÃ©e de pulse de 1 Ã  2ms dans la pÃ©riode de 10ms)
	// Donc on additionne 100 Ã  position.
	current_position = position;
	position += 100;
	TIMER_set_duty(TIMER1_ID, TIM_CHANNEL_1, position);

}

int main(void)
{
	//Initialisation de la couche logicielle HAL (Hardware Abstraction Layer)
	//Cette ligne doit rester la premiÃ¨re Ã©tape de la fonction main().
	HAL_Init();
	ADC_init();
	//HAL_ADC_Init(ADC_CHANNEL_8);
	//Initialisation de l'UART2 Ã  la vitesse de 115200 bauds/secondes (92kbits/s) PA2 : Tx  | PA3 : Rx.
		//Attention, les pins PA2 et PA3 ne sont pas reliÃ©es jusqu'au connecteur de la Nucleo.
		//Ces broches sont redirigÃ©es vers la sonde de dÃ©bogage, la liaison UART Ã©tant ensuite encapsulÃ©e sur l'USB vers le PC de dÃ©veloppement.
	UART_init(UART2_ID,115200);

	//"Indique que les printf sortent vers le pÃ©riphÃ©rique UART2."
	SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

	//Initialisation du port de la led Verte (carte Nucleo)
	BSP_GPIO_PinCfg(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);

	//Initialisation du port du bouton bleu (carte Nucleo)
	BSP_GPIO_PinCfg(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);

	//On ajoute la fonction process_ms Ã  la liste des fonctions appelÃ©es automatiquement chaque ms par la routine d'interruption du pÃ©riphÃ©rique SYSTICK
	Systick_add_callback_function(&process_ms);
	//initialisation et lancement du timer1 Ã  une pÃ©riode de 10 ms

	//----------------HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);

	TIMER_run_us(TIMER1_ID, 10000, FALSE); //10000us = 10ms

	//activation du signal PWM sur le canal 1 du timer 1 (broche PA8)
	TIMER_enable_PWM(TIMER1_ID, TIM_CHANNEL_1, 150, FALSE, FALSE);

	// Mise Ã  jour du rapport cyclique.
	TIMER_set_duty(TIMER1_ID, TIM_CHANNEL_1, 100);

	// My code :


	while(1)	//boucle de tÃ¢che de fond
	{
		/*if (position < 100)
		{
		    position = 100; // Ã©cretage si l'utilisateur demande plus de 100%
		    current_position = position;
		    position += 100;
		    TIMER_set_duty(TIMER1_ID, TIM_CHANNEL_1, position);
		}*/


		// Tourner dans un sens
		for (position=0; position <= 100; position=position+10) {
			SERVO_set_position(position);
			adc_value = ADC_getValue(ADC_CHANNEL_8);
		}

		/*ADC_Start(ADC_CHANNEL_8);
		adc_value = ADC_getValue(ADC_CHANNEL_8);
		printf ("adc = ",adc_value);
		ADC_Stop(ADC_CHANNEL_8);*/

		//testtttttttt
		/*if(!t)
		{
			t = 200;
			HAL_GPIO_TogglePin(LED_GREEN_GPIO, LED_GREEN_PIN);
		}*/
		SUNBED_state_machine();
	}
}

//#define ...

//static volatile uint32_t t = 0;

void SUNBED_process_ms(void){
	if(t)
		t--;
}

typedef enum{
	MESSAGE_WELCOME,
	MESSAGE_PARAMETERS,
	MESSAGE_THANKS
}message_id_e;

void SUNBED_display(message_id_e message_id)
{
	switch(message_id)
	{
		case MESSAGE_WELCOME:
			printf("Hello world");
			break;
		case MESSAGE_PARAMETERS:
			//TODO
			break;
		case MESSAGE_THANKS:
			break;
		default:
			printf("You must define the content for the case %d\n", message_id);
			break;
	}
}

static void SUNBED_state_machine(void)
{
	typedef enum
	{
		INIT,
		WAIT_DOOR_CLOSED,
		SET_DURATION_AND_WAIT_LAUNCH,
		RUNNING,
		DISPLAY_THANKS,
		ERROR
	}state_e;

	static state_e state = INIT;
	static state_e previous_state = INIT;
	bool_e entrance;
	entrance = (state != previous_state) ? TRUE : FALSE;

	static uint16_t duration = 0;

	//Lecture des boutons
	//bool_e button_up = FALSE;
	//bool_e button_up = FALSE;
	//bool_e button_run = FALSE;
	//BUTTONS_get_events(&button_up, &button_up, &button_run);

	switch(state)
	{
		case INIT:

			//on ajoute la fonction SUNBED_process_ms à la liste des fonction appelées auto chaque ms
			Systick_add_callback_function(&SUNBED_process_ms);

			state = WAIT_DOOR_CLOSED;
			break;
		case WAIT_DOOR_CLOSED:
			if (entrance)
				SUNBED_display(MESSAGE_WELCOME);
			state = SET_DURATION_AND_WAIT_LAUNCH;
			break;
		case SET_DURATION_AND_WAIT_LAUNCH:
			if(entrance)
				duration = 60; //durée par défaut : 60 secondes
			if (butt_test)
			{
				if (duration < MAX_DURATION)
					duration += 10;
			}
			if (butt_test)
			{
				if (duration > 10)
					duration -= 10;
			}
			if (butt_test)
				state = RUNNING;
			break;
		case RUNNING:
				if (entrance)
				{
					t = ((uint32_t)(duration))*1000; // en ms
					// TODO allumer le TFT
				}
				if (!t || butt_test)
				{
					// TODO éteindre
					state = DISPLAY_THANKS;
				}

				//Tâche critique, fonction __disable_irq() évite l'enteruption de celle-ci
				__disable_irq();
					if (butt_test)
						t += 10*1000;
				__disable_irq();
			break;
		case DISPLAY_THANKS:
			if (entrance)
			{
				t = 2000; // en ms (2s)
				SUNBED_display(MESSAGE_THANKS);
			}
			if (!t)
				state = WAIT_DOOR_CLOSED;

			state = ERROR;
			break;
		case ERROR:
			break;
		default:
			break;
	}
}
