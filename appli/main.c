/**
  ******************************************************************************
  * @file    main.c
  * @author  Nirgal
  * @date    03-July-2019
  * @brief   Default main function.
  ******************************************************************************

/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |                                                                                  \_|

*/

#include "stm32f1xx_hal.h"
#include "stm32f1_uart.h"
#include "stm32f1_sys.h"
#include "stm32f1_gpio.h"
#include "macro_types.h"
#include "systick.h"
#include "stm32f1_timer.h"
#include "stm32f1xx.h"
#include "tft_ili9341/stm32f1_ili9341.h"


// Pin definitions
#define SERVO_HORIZONTAL TIM_CHANNEL_2 // A9
#define SERVO_VERTICAL TIM_CHANNEL_3 // A10
#define PHOTOCELL_0 ADC_CHANNEL_0 // A0
#define PHOTOCELL_1 ADC_CHANNEL_1 // A1
#define PHOTOCELL_2 ADC_CHANNEL_8 // B0
#define PHOTOCELL_3 ADC_CHANNEL_9 // B1

// Servo angle limits
#define HORIZONTAL_MIN_ANGLE 25
#define HORIZONTAL_MAX_ANGLE 220
#define VERTICAL_MIN_ANGLE 130
#define VERTICAL_MAX_ANGLE 250

// States definitions
	// Track
	typedef enum{
		INIT,
		CALIBRATE,
		TRACK_SUN,
		DARK_MODE
	} State;

	// Affichage
	typedef enum{
		MESSAGE_WELCOME,
		MESSAGE_PARAMETERS,
		MESSAGE_THANKS
	} message_id_e;

/**
 *  Par exemple, si vous avez un capteur de lumière qui renvoie des valeurs analogiques de 0 à 1023,
 *  mais vous voulez utiliser ces valeurs pour contrôler un servomoteur qui fonctionne dans une plage d'angles de 0 à 180 degrés,
 *  vous devrez mapper les valeurs du capteur aux valeurs des angles du servomoteur.
 */

/*
  int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

int constrain(int value, int min, int max) {
  if (value <= min) {
    return min;
  } else if (value >= max) {
    return max;
  } else {
    return value;
  }
}
*/

// Function prototypes
void initialize();
void ihm_tft();
void calibrate();
void trackSun();
void sweepArea(int startAngle, int endAngle, int stepDelay);
static void SUNBED_state_machine(void);
static void SUNBED_display(message_id_e message_id);
void adjustServos(int horizontalAngle, int verticalAngle);

// Global variables
TIM_HandleTypeDef htim_servo;
ADC_HandleTypeDef hadc;

State currentState = INIT;
uint16_t posHorizontal = 115; //min = 25, centre = 115, max = 220 // Initial horizontal angle
uint16_t posVertical = 190;   //min= 130, centre = 190, max = 250 // Initial vertical angle


int main(void)
{
	initialize();


	//SUNBED_display(MESSAGE_WELCOME);

	while (1)
	{
		//TIMER_set_duty(TIMER1_ID, SERVO_HORIZONTAL, horizontalAngle);
		//sweepArea(HORIZONTAL_MIN_ANGLE, HORIZONTAL_MAX_ANGLE, 100);
		SUNBED_state_machine();
//		ILI9341_demo();
//		ILI9341_DrawLine(20,20,20,100,ILI9341_COLOR_RED);
//		ILI9341_DrawCircle(50, 50, 80, ILI9341_COLOR_RED);
//		ILI9341_Puts(20,20,'Hi',&Font_11x18,ILI9341_COLOR_BLUE,ILI9341_COLOR_GREEN);
//		ILI9341_Putc(20,20,'Hiii',&Font_11x18,ILI9341_COLOR_BLUE,ILI9341_COLOR_GREEN);
	}
}

void initialize()
{
	//Initialisation de la couche logicielle HAL (Hardware Abstraction Layer)
	//Cette ligne doit rester la premiÃ¨re Ã©tape de la fonction main().
	HAL_Init();

	ADC_init();
	//Initialisation de l'UART2 Ã  la vitesse de 115200 bauds/secondes (92kbits/s) PA2 : Tx  | PA3 : Rx.
	//Attention, les pins PA2 et PA3 ne sont pas reliÃ©es jusqu'au connecteur de la Nucleo.
	//Ces broches sont redirigÃ©es vers la sonde de dÃ©bogage, la liaison UART Ã©tant ensuite encapsulÃ©e sur l'USB vers le PC de dÃ©veloppement.
	UART_init(UART1_ID,115200);

	//"Indique que les printf sortent vers le pÃ©riphÃ©rique UART2."
	SYS_set_std_usart(UART1_ID, UART1_ID, UART1_ID);

	//Initialisation du port de la led Verte (carte Nucleo)
	BSP_GPIO_PinCfg(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);

	//Initialisation du port du bouton bleu (carte Nucleo)
	BSP_GPIO_PinCfg(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);

	TIMER_run_us(TIMER1_ID, 10000, FALSE); //10000us = 10ms

	//activation du signal PWM sur le canal 2 et 3 du timer 1 (A9, A10)
	TIMER_enable_PWM(TIMER1_ID, SERVO_HORIZONTAL, posHorizontal, FALSE, FALSE);
	TIMER_enable_PWM(TIMER1_ID, SERVO_VERTICAL, posVertical, FALSE, FALSE);

	// Mise Ã  jour du rapport cyclique.
	//TIMER_set_duty(TIMER1_ID, SERVO_HORIZONTAL, horizontalAngle);
	//TIMER_set_duty(TIMER1_ID, SERVO_VERTICAL, verticalAngle);

	init_screen();

	// Start calibration
	currentState = CALIBRATE;
}

void calibrate() {
	// Perform calibration routine
	// Adjust servos to their starting positions
	adjustServos(posHorizontal, posVertical);
	currentState = TRACK_SUN;
}

// Fonction pour ajuster les servos
void adjustServos(int deltaX, int deltaY) {
    int sensibilite = 500;

    // Ajustement du servo horizontal
    if (deltaX > sensibilite)
    {
        posHorizontal++; // Augmente la position horizontale
        if (posHorizontal > HORIZONTAL_MAX_ANGLE)
        {
            posHorizontal = HORIZONTAL_MAX_ANGLE;
        }
        TIMER_set_duty(TIMER1_ID, SERVO_HORIZONTAL, posHorizontal);
    }
    else if (deltaX < -sensibilite)
    {
        posHorizontal--; // Diminue la position horizontale
        if (posHorizontal < HORIZONTAL_MIN_ANGLE)
        {
            posHorizontal = HORIZONTAL_MIN_ANGLE;
        }
        TIMER_set_duty(TIMER1_ID, SERVO_HORIZONTAL, posHorizontal);
    }
    else
    {
        // Rien à faire
    }

    // Ajustement du servo vertical
    if (deltaY > sensibilite)
    {
        posVertical++; // Augmente la position verticale
        if (posVertical > VERTICAL_MAX_ANGLE)
        {
            posVertical = VERTICAL_MAX_ANGLE;
        }
        TIMER_set_duty(TIMER1_ID, SERVO_VERTICAL, posVertical);
    }
    else if (deltaY < -sensibilite)
    {
        posVertical--; // Diminue la position verticale
        if (posVertical < VERTICAL_MIN_ANGLE)
        {
            posVertical = VERTICAL_MIN_ANGLE;
        }
        TIMER_set_duty(TIMER1_ID, SERVO_VERTICAL, posVertical);
    }
    else
    {
        // Rien à faire
    }
}

void trackSun() {
	// Read photo cell values
	int photocell0 = ADC_getValue(PHOTOCELL_0);
	int photocell1 = ADC_getValue(PHOTOCELL_1);
	int photocell2 = ADC_getValue(PHOTOCELL_2);
	int photocell3 = ADC_getValue(PHOTOCELL_3);
	// Determine servo adjustments based on photo cell readings
	// Calculate the difference between pairs of photocells
	int deltX = photocell0 - photocell2; //x
	int deltY = photocell1 - photocell3; //y

	int average = (photocell0 + photocell1 + photocell2 + photocell3)/4;

	snprintf("pc0:      %11.5lf,    pc2:   %11.5lf\n", photocell0, photocell2);
	snprintf("pc1:   %11.5lf,    pc3:  %11.5lf\n", photocell1, photocell3);
	// Adjust servos
	adjustServos(deltX, deltY);
}

// Fonction principale de la machine à état
static void SUNBED_state_machine(void)
{
	switch(currentState)
	{
	  case INIT:
		break;
	  case CALIBRATE:
		//calibrate();
		currentState = TRACK_SUN;
		break;
	  case TRACK_SUN:
		trackSun();
		break;
	  case DARK_MODE:

	  	break;
	}
}

void ihm_tft()
{
	//draw_full_circle();
	//draw_cross();

	// Affiche les valeurs des photorésistances à des positions spécifiques
	//draw_value(50, 50, photoresistor_value_1);
	//draw_value(190, 50, photoresistor_value_2);
	//draw_value(50, 210, photoresistor_value_3);
	//draw_value(190, 210, photoresistor_value_4);
}

// Initialisation de l'écran TFT
void init_screen()
{
	ILI9341_Init();
	//ILI9341_DisplayOn();
	ILI9341_Fill(ILI9341_COLOR_YELLOW);
}

//ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
//draw_value(int x, int y, int value);
//ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);


void SUNBED_display(message_id_e message_id)
{
	switch(message_id)
	{
		case MESSAGE_WELCOME:
			printf("Hello world");

			ILI9341_DrawLine(20,20,20,100,ILI9341_COLOR_RED);
			ILI9341_Puts(20,20,'Hi',&Font_11x18,ILI9341_COLOR_BLUE,ILI9341_COLOR_GREEN);
			break;
		case MESSAGE_PARAMETERS:
			// TODO

			break;
		case MESSAGE_THANKS:
			printf("Bye bye ...");

			ILI9341_DrawLine(20,20,20,100,ILI9341_COLOR_RED);
			ILI9341_Putc(130,11,'Bye',&Font_11x18,ILI9341_COLOR_BLUE,ILI9341_COLOR_WHITE);
			break;
		default:
			printf("You must define the content for the case %d\n", message_id);
			ILI9341_DrawLine(20,20,20,100,ILI9341_COLOR_RED);
			ILI9341_Putc(130,11,'case ?',&Font_11x18,ILI9341_COLOR_BLUE,ILI9341_COLOR_WHITE);
			break;
	}
}
