/*
 * main.c
 *
 *  Created on: Dec 18, 2024
 *      Author: ariat
 */
/*
 * PA11 (CAN_Rx) and PA12 are CAN Pins
 *
 *
 */

#include "main.h"
#include"string.h"

void UART2_Init(void);
void Error_handler(void);
void GPIO_Init(void);
void CAN_Tx(void);
void CAN1_Init(void);
void SystemClock_Config_HSE(uint8_t clkFreq);
void CAN_Rx(void);
void CAN_Filter_Config(void);
void TIMER6_Init(void);
void CAN_TransmitLED(void);
void GPIO_Init(void);

TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;
CAN_HandleTypeDef hcan1;
uint32_t mailbox;
uint32_t FLatency=0;
GPIO_InitTypeDef ledgpio2;
uint8_t counter = 0;
CAN_TxHeaderTypeDef canh;




int main(void){



	HAL_Init();
	srand(1);
	//clock config should be second always
	SystemClock_Config_HSE(SYS_CLK_FREQ_50_MHZ);
	UART2_Init();
	CAN1_Init();
	TIMER6_Init();
	GPIO_Init();




	CAN_Filter_Config();

	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY |CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF);

	if(HAL_CAN_Start(&hcan1) != HAL_OK){
			Error_handler();
		}


	while(1);

	return 0;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//send LED_NUM
	CAN_TxHeaderTypeDef canh1;
	uint32_t mailbox1;
	if(counter == 4){

		       uint8_t our_msg;

		       canh1.DLC = 2;
		       canh1.StdId = 0x65D;
		       canh1.IDE = CAN_ID_STD;
		       canh1.RTR = CAN_RTR_REMOTE;


				if(HAL_CAN_AddTxMessage(&hcan1, &canh1, &our_msg, &mailbox1)!= HAL_OK){
					Error_handler();
				}

				counter=0;


	}else{
		counter++;
		CAN_TransmitLED();
	}



}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

		CAN_RxHeaderTypeDef canRx;
		uint8_t rcvd_msg[2];
				char msg[50];


		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &canRx, rcvd_msg) != HAL_OK){
					Error_handler();
				}

		if(canRx.StdId == 0x65a && canRx.RTR == 1){

			sprintf(msg,"He says: %s\r\n", rcvd_msg);
			HAL_UART_Transmit(&huart2,(uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);


		}







}

void GPIO_Init(void){

	__HAL_RCC_GPIOC_CLK_ENABLE();
	//__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_13;
	ledgpio.Pull = GPIO_NOPULL;
	ledgpio.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(GPIOC, &ledgpio);

	ledgpio2.Pin = GPIO_PIN_5;
	ledgpio2.Pull = GPIO_NOPULL;
	ledgpio2.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &ledgpio2);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void TIMER6_Init(void){

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 4999;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 10000-1 ;
	if(HAL_TIM_Base_Init(&htim6) != HAL_OK){

		Error_handler();
	}




}
void CAN_TransmitLED(void){




		uint8_t our_msg;

		canh.DLC = 1;
		canh.StdId = 0x65d;
		canh.IDE = CAN_ID_STD;
		canh.RTR = CAN_RTR_DATA;

		uint8_t random_number = (rand() % 4) + 1;
		our_msg = random_number;

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		if(HAL_CAN_AddTxMessage(&hcan1, &canh, &our_msg, &mailbox)!= HAL_OK){
			Error_handler();


}
}

void SystemClock_Config_HSE(uint8_t clkFreq){

	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_BYPASS;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clkFreq)
	{
	case SYS_CLK_FREQ_180_MHZ: {



				//enable clock for pwr controller

				__HAL_RCC_PWR_CLK_ENABLE();

				//set voltage scalar so we run on max clock
				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

				//turn on overdrive
				__HAL_PWR_OVERDRIVE_ENABLE();

		        osc_init.PLL.PLLM = 8;
				osc_init.PLL.PLLP = 1;
				osc_init.PLL.PLLN = 360;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
				FLatency = FLASH_ACR_LATENCY_5WS;



		break;
	}
	case SYS_CLK_FREQ_50_MHZ:{
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLR = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		FLatency = FLASH_ACR_LATENCY_1WS;

		break;
	}

	case SYS_CLK_FREQ_84_MHZ:{

				osc_init.PLL.PLLM = 8;
				osc_init.PLL.PLLP = 2;
				osc_init.PLL.PLLN = 168;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;


				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
				FLatency = FLASH_ACR_LATENCY_2WS;

				break;

	}

	case SYS_CLK_FREQ_120_MHZ:{

		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLN = 240;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLR = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		FLatency = FLASH_ACR_LATENCY_3WS;
				break;

	}
	default:
		return;

	}

	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK){

		Error_handler();
	}
	if(HAL_RCC_ClockConfig(&clk_init, FLatency)!= HAL_OK){
		Error_handler();
	}

	//SYSTICK config
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


}

void CAN_Filter_Config(void){

	CAN_FilterTypeDef can_filter;

	can_filter.FilterActivation = ENABLE;
	can_filter.FilterBank = 0;
	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterIdHigh = 0x0;
	can_filter.FilterIdLow = 0x0;
	can_filter.FilterMaskIdHigh = 0x0;
	can_filter.FilterMaskIdLow = 0x0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

	if ( HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK){
		Error_handler();
	}

}
void CAN1_Init(void){

	hcan1.Instance = CAN1;
	hcan1.Init.AutoBusOff =  DISABLE;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	hcan1.Init.Prescaler = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

	if(HAL_CAN_Init(&hcan1)!= HAL_OK){
		Error_handler();
	}




}

void UART2_Init(){

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength =  UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if(HAL_UART_Init(&huart2)!= HAL_OK ){

		Error_handler();
	}
}

void Error_handler(void){

}



