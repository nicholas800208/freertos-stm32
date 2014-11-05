/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "draw_graph.h"
#include "move_car.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f429i_discovery_l3gd20.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xQueueHandle t_queue; /* Traffic light queue. */
xQueueHandle t_mutex; /* Traffic light mutex. */

static int traffic_index = 0; 
static int button_change_traffic = 0;
static int states[] = {TRAFFIC_RED, TRAFFIC_YELLOW, TRAFFIC_GREEN, 
							TRAFFIC_YELLOW};
//l3gd20
static float axes[3] = {0};
static float time = 0.0f, frametime = 0.0f;
static float delta;
static int count = 0;
static float initune = 10 ;
static float tune = 10;
static float scale;
static uint8_t SHOW=0xFF;
static uint8_t HIDE=0x00;
static uint32_t fps = 0;
static uint8_t fps_str[8] = "? FPS";
static uint8_t info_str[16];
static float count1=90; //紀錄layer1所畫的點
static float count2=90; //紀錄layer2所畫的點
static char *itoa(int value, char* result, int base);
/*uint8_t SHOW=0xFF;
uint8_t HIDE=0x00;*/
void
prvInit()
{
	//LCD init
	LCD_Init();
	IOE_Config();
	LTDC_Cmd( ENABLE );

	LCD_LayerInit();
	LCD_SetLayer( LCD_FOREGROUND_LAYER );
	LCD_Clear( LCD_COLOR_BLACK );
	LCD_SetTextColor( LCD_COLOR_WHITE );
	
	LCD_SetLayer( LCD_BACKGROUND_LAYER );
	LCD_Clear( LCD_COLOR_BLACK );
	LCD_SetTextColor( LCD_COLOR_WHITE );

	//Button
	STM_EVAL_PBInit( BUTTON_USER, BUTTON_MODE_GPIO );
	
	//l3gd20
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	L3GD20_FilterStructure.HighPassFilter_Mode_Selection = 		  L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure);
	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

static void GetTrafficState(int change_state, int *v_state, int *h_state)
{

	switch (change_state) {
	case TRAFFIC_RED:
		*v_state = TRAFFIC_RED;
		*h_state = TRAFFIC_GREEN;
		break;
	case TRAFFIC_YELLOW:
		if (*v_state == TRAFFIC_GREEN)
			*v_state = TRAFFIC_YELLOW;
		else
			*h_state = TRAFFIC_YELLOW;
		break;
	case TRAFFIC_GREEN:
		*v_state = TRAFFIC_GREEN;
		*h_state = TRAFFIC_RED;
		break;
	default:
		ReportError("out of range");
		break;
	}
}

static void DrawGraphTask( void *pvParameters)
{
	const portTickType ticks = 100 / portTICK_RATE_MS;
	int value;
	int traffic_v_state = TRAFFIC_GREEN;
	int traffic_h_state = TRAFFIC_RED;

	portBASE_TYPE status;

	//DrawBackground();

	while ( 1 ) {
		/*
		 * Check if the traffic changed event is sent to
		 * the queue. If so, we need to change the traffic
		 * light.
		 */
		status = xQueueReceive(t_queue, &value, ticks);

		if (status == pdPASS) {
			GetTrafficState(value, &traffic_v_state, 
						&traffic_h_state);
		}

		//MoveCar(traffic_v_state, traffic_h_state);
	}
}

static void ChgTrafficLightTask(void *pvParameters)
{
	int num_ticks;
	int states_num = sizeof(states) / sizeof(states[0]);
	portBASE_TYPE status;
	portTickType ticks = TRAFFIC_GREEN_TICK;

	while ( 1 ) {
		ticks = (states[traffic_index] == TRAFFIC_YELLOW ? 
			TRAFFIC_YELLOW_TICK : TRAFFIC_GREEN_TICK);

		num_ticks = ticks / TRAFFIC_TICK_SLICE;

		status = xQueueSendToBack(t_queue, &states[traffic_index++], 0);
	
		if (status != pdPASS)
			ReportError("Cannot send to the queue!");

		if (traffic_index >= states_num)
			traffic_index = 0;

		while (num_ticks--) { 
			xSemaphoreTake(t_mutex, portMAX_DELAY);
			
			if (button_change_traffic) {
				button_change_traffic = 0;
				xSemaphoreGive(t_mutex);
				break;
			}

			xSemaphoreGive(t_mutex);

			vTaskDelay(TRAFFIC_TICK_SLICE);
		}
	}
}

static void ButtonEventTask(void *pvParameters)
{
	while (1) {
		if( STM_EVAL_PBGetState( BUTTON_USER ) ){

			while( STM_EVAL_PBGetState( BUTTON_USER ) );

			xSemaphoreTake(t_mutex, portMAX_DELAY);
			button_change_traffic = 1;
			xSemaphoreGive(t_mutex);
		}
	}
}

static void update(void){
//l3gd20_start
	LCD_SetTextColor( LCD_COLOR_BLACK );
	uint8_t tmp[6] = {0};
	uint8_t tmpreg = 0;
	int16_t a[3] = {0};
	L3GD20_Read(&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);
	L3GD20_Read(tmp, L3GD20_OUT_X_L_ADDR, 6);
	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
	if (!(tmpreg & 0x40)) {
		for (int i = 0; i < 3; i++)
			a[i] = (int16_t)(((uint16_t)tmp[2 * i + 1] << 8) | (uint16_t)tmp[2 * i]);
	} else {
		for (int i = 0; i < 3; i++)
			a[i] = (int16_t)(((uint16_t)tmp[2 * i] << 8) | (uint16_t)tmp[2 * i + 1]);
	}
	
	delta = frametime - time;
	frametime = time; // make frametime a consistent time value during the frames
	for (int i = 0; i < 3; i++){
		axes[i] = a[i] / 114.285f ;
		//if (axes[i] < 0) axes[i] = 0;
		//if (axes[i] > 180) axes[i] = 180;
	}	
	fps++;
	tune += axes[1] * 0.05;
	//l3gd20_end
	
}
static void render(void){
	char str[16] = "X: ";
	/*LCD_DisplayStringLine(LCD_LINE_1, info_str);
	LCD_DisplayStringLine(LCD_LINE_3, fps_str);

	char str[16] = "X: ";
	itoa(axes[0], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_5, str);
	str[0] = 'Y';
	itoa(axes[1], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_6, str);
	str[0] = 'Z';
	itoa(axes[2], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_7, str);
	
	str[0] = 'D';
	itoa(delta, str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_8, str);
	
	str[0] = 'F';
	itoa(count1, str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_9, str);
	str[0] = '1';
	itoa(count1, str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_10, str);
	str[0] = '2';
	itoa(count2, str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_11, str);

	str[0] = 'O';
	itoa(tune, str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_12, str);*/
	
}

void ClearLine(){
	/*LCD_SetTextColor( LCD_COLOR_BLACK );
	LCD_ClearLine(LCD_LINE_1);
	LCD_ClearLine(LCD_LINE_3);
	LCD_ClearLine(LCD_LINE_5);
	LCD_ClearLine(LCD_LINE_6);
	LCD_ClearLine(LCD_LINE_7);
	LCD_ClearLine(LCD_LINE_8);
	LCD_ClearLine(LCD_LINE_9);
	LCD_ClearLine(LCD_LINE_10);
	LCD_ClearLine(LCD_LINE_11);
	LCD_ClearLine(LCD_LINE_12);*/
	
}

static void KeyboardTask( void *pvParameters)
{	
	static int layer=0;
	static float s=0.025;
	static float s1=0.075;
	static float s2=0.1;
	static float count1=90;
	static float count2=90;
	static float rect1_count1=10;
	static float rect1_count2=10;
	static float rect2_count1=50;
	static float rect2_count2=50;
	while(1){
		LCD_SetLayer( LCD_FOREGROUND_LAYER );
		LCD_SetTransparency (0xFF);
		LCD_SetLayer( LCD_BACKGROUND_LAYER );
		LCD_SetTransparency (0x00);
		//drawblack layer2
		LCD_SetTextColor( LCD_COLOR_BLACK );
		LCD_DrawFullRect ( count2 ,50 ,20 ,20 );
		LCD_SetTextColor( LCD_COLOR_BLACK );
		LCD_DrawFullRect ( 20 ,rect1_count2 ,30 ,20 );
		LCD_SetTextColor( LCD_COLOR_BLACK );
		LCD_DrawFullRect ( 180 ,rect2_count2 ,30 ,20 );
		//ClearLine();
		//update & draw layer2
		if( tune < initune-200)
		    scale=-s;
		else if (tune > initune+200)
		    scale=s;
		else 
		    scale=0;
		if( count2+scale >=0 )
		    count2 = count2+scale;
		if( count2+scale <=100 )
		    count2 = count2+scale;
		rect1_count2+=s1;
		if (rect1_count2 >=320)
			rect1_count2=0;
		rect2_count2+=s2;
		if (rect2_count2 >=320)
			rect2_count2=0;
		LCD_SetTextColor( LCD_COLOR_BLUE );
		LCD_DrawFullRect ( count2 ,50 ,20 ,20 );
		LCD_SetTextColor( LCD_COLOR_YELLOW );
		LCD_DrawFullRect ( 20 ,rect1_count2 ,30 ,20 );
		LCD_SetTextColor( LCD_COLOR_YELLOW );
		LCD_DrawFullRect ( 180 ,rect2_count2 ,30 ,20 );
		update();
		//render();
		//cancel layer1 & display layer2
		LCD_SetLayer( LCD_BACKGROUND_LAYER );
		LCD_SetTransparency (0xFF);
		update();
		//render();
		LCD_SetLayer( LCD_FOREGROUND_LAYER );
		LCD_SetTransparency (0x00);

		//drawback layer1 
		LCD_SetLayer( LCD_FOREGROUND_LAYER );
		LCD_SetTextColor( LCD_COLOR_BLACK );
		LCD_DrawFullRect ( count1 ,50 ,20 ,20 );
		LCD_SetTextColor( LCD_COLOR_BLACK );
		LCD_DrawFullRect ( 20 ,rect1_count1 ,30 ,20 );
		LCD_SetTextColor( LCD_COLOR_BLACK );
		LCD_DrawFullRect ( 180 ,rect2_count1 ,30 ,20 );
		//ClearLine();
		//update & draw layer 1 
		if( tune < initune-200 )
		    scale=-s;
		else if (tune > initune+200)
		    scale=s;
		else 
		    scale=0;
		if( count1+scale >=0 )
		    count1 = count1+scale;
		if( count1+scale <=100 )
		    count1 = count1+scale;
		rect1_count1+=s1;
		if (rect1_count1 >=320)
			rect1_count1=0;
		rect2_count1+=s2;
		if (rect2_count1 >=320)
			rect2_count1=0;
		LCD_SetLayer( LCD_FOREGROUND_LAYER );
		LCD_SetTextColor( LCD_COLOR_BLUE );
		LCD_DrawFullRect ( count1 ,50 ,20 ,20 );
		LCD_SetTextColor( LCD_COLOR_YELLOW );
		LCD_DrawFullRect ( 20 ,rect1_count1 ,30 ,20 );
		LCD_SetTextColor( LCD_COLOR_YELLOW );
		LCD_DrawFullRect ( 180 ,rect2_count1 ,30 ,20 );
		update();
		//render();
		/*if (scale == 0){
		count1=count2;
		}*/
	}
	
	/*LCD_SetTextColor( LCD_COLOR_BLACK );
        while(1){
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
        char t = USART_ReceiveData(USART1);
        uint8_t *a=t;
        LCD_DisplayStringLine(LCD_LINE_1, &a);
        
        if ((t == '\r')) {
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, t);
            t = '\n';make
        }
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, t);
        }*/
}

uint32_t fps_countdown = 100;
void OnSysTick(void)
{
	if (--fps_countdown == 0) {
		itoa(fps, fps_str, 10);
		strcat(fps_str, " FPS");

		fps = 0;
		fps_countdown = 100;
	}
	time += 0.0001f;
	count += 1;
		if (count >=300)
		count = 0 ;
	tune = tune +1.5f;
}

uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
	//return 0;
	while(1){}
}

static char* itoa(int value, char* result, int base)
{
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}
	char *ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while (value);

	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}


/////////////////////////////////////////////////////////////////////////
void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}
 
/**************************************************************************************/
 
void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 9600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}
/////////////////////////////////////////////////////////////////////////


//Main Function
int main(void)
{
	SysTick_Config(SystemCoreClock / 100); // SysTick event each 10ms
	RCC_Configuration();
        GPIO_Configuration();
        USART1_Configuration();
        
	t_queue = xQueueCreate(1, sizeof(int));
	if (!t_queue) {
		ReportError("Failed to create t_queue");
		while(1);
	}

	t_mutex = xSemaphoreCreateMutex();
	if (!t_mutex) {
		ReportError("Failed to create t_mutex");
		while(1);
	}

	prvInit();

	xTaskCreate(ChgTrafficLightTask, "Traffic Light Task", 256, 
			( void * ) NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(ButtonEventTask, (char *) "Button Event Task", 256,
		   	NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(DrawGraphTask, (char *) "Draw Graph Task", 256,
		   	NULL, tskIDLE_PRIORITY + 1, NULL);
		   	
	xTaskCreate(KeyboardTask, (char *) "Keyboard Task", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);


	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
        RNG_Cmd(ENABLE);

	//Call Scheduler
	vTaskStartScheduler();
}

