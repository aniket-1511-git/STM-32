/*
 * STM32Lxx_gpio_driver.c
 *
 *  Created on: Sep 11, 2025
 *      Author: HP
 */
/**********************************************************************************************************************
 * Include files
**********************************************************************************************************************/
#include"STM32Lxx.H"
#include"STM32Lxx_gpio_header.h"

/*********************************************** end of Include files ************************************************/

/**********************************************************************************************************************
 * PRAGMAS
**********************************************************************************************************************/

/******************************************** end of PRAGMAs *********************************************************/

/**********************************************************************************************************************
 * Global Variables
**********************************************************************************************************************/
/* Flags used to indicate updated data is received*/


/*************************************** end of Global Variables ****************************************************/

/*********************************************************************************************************************
 * Static Function Prototypes
*********************************************************************************************************************/

/***************************************** end of Static Function Prototypes ****************************************/

/*********************************************************************************************************************
 * Function Definitions
*********************************************************************************************************************/

/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *GPIOx_p, uint8_t state_u8)
{
	if(ENABLE == state_u8)
	{
		if(GPIOA == GPIOx_p)
		{
			GPIOA_PCLK_EN();
		}
		else if(GPIOB == GPIOx_p)
		{
			GPIOB_PCLK_EN();
		}
		else if(GPIOC == GPIOx_p)
		{
			GPIOC_PCLK_EN();
		}
		else if(GPIOD == GPIOx_p)
		{
			GPIOD_PCLK_EN();
		}
		else if(GPIOE  == GPIOx_p)
		{
			GPIOE_PCLK_EN();
		}
		else if(GPIOF == GPIOx_p)
		{
			GPIOF_PCLK_EN();
		}
		else if(GPIOG == GPIOx_p)
		{
			GPIOG_PCLK_EN();
		}
		else if(GPIOH == GPIOx_p)
		{
			GPIOH_PCLK_EN();
		}

		else
		{

		}

	}
	else if(DISABLE == state_u8)
	{
		if(GPIOA == GPIOx_p)
		{
			GPIOA_PCLK_DI();
		}
		else if(GPIOB == GPIOx_p)
		{
			GPIOB_PCLK_DI();
		}
		else if(GPIOC == GPIOx_p)
		{
			GPIOC_PCLK_DI();
		}
		else if(GPIOD == GPIOx_p)
		{
			GPIOD_PCLK_DI();
		}
		else if(GPIOE  == GPIOx_p)
		{
			GPIOE_PCLK_DI();
		}
		else if(GPIOF == GPIOx_p)
		{
		GPIOF_PCLK_DI();
		}
		else if(GPIOG == GPIOx_p)
		{
			GPIOG_PCLK_DI();
		}
		else if(GPIOH == GPIOx_p)
		{
			GPIOH_PCLK_DI();
		}

		else
		{

		}
	}
	else
	{

	}
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp =0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp=( pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0X03 << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle ->pGPIOx->MODER |= temp;
		temp=0;
	}
	else
	{
	//for interrupts
	}

	temp=0;
	temp=(pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle ->pGPIOx->OSPEEDR &= ~(0X03 << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	temp=(pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle ->pGPIOx->PUPDR &= ~(0X03 << (2* pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	temp=(pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType<<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle ->pGPIOx->OTYPER &= ~(0X01 << ( pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1 ,temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0X0f<< (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *GPIOx_p)
{
	if(GPIOA == GPIOx_p)
	{
		GPIOA_REG_RESET();
	}
	else if(GPIOB == GPIOx_p)
	{
		GPIOB_REG_RESET();
	}
	else if(GPIOC == GPIOx_p)
	 {
		 GPIOC_REG_RESET();
	 }
	else if(GPIOD == GPIOx_p)
	 {
		 GPIOD_REG_RESET();
	 }
	else if(GPIOE  == GPIOx_p)
	 {
		 GPIOE_REG_RESET();
	 }
	else if(GPIOF == GPIOx_p)
	 {
		 GPIOF_REG_RESET();
	 }
	else if(GPIOG == GPIOx_p)
	 {
		 GPIOG_REG_RESET();
	 }
	else if(GPIOH == GPIOx_p)
	 {
		 GPIOH_REG_RESET();
	 }

	else
	{

	}
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber_u8)
{
	uint8_t value;
	value= (uint8_t) (( pGPIOx->IDR >> pinNumber_u8 ) & 0X01);
	return value;
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value= (uint16_t) ( pGPIOx->IDR );
	return value;
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber_u8,uint8_t state_u8)
{
	if( GPIO_PIN_SET == state_u8 )
	{
		pGPIOx->ODR |= (1<<pinNumber_u8);
	}
	else if( GPIO_PIN_RESET == state_u8 )
	{
		pGPIOx->ODR &= ~(1<<pinNumber_u8);
	}
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t state_u16)
{
	pGPIOx->ODR = state_u16;
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber_u8)
{
	pGPIOx->ODR ^= (1<< pinNumber_u8);
}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/

void GPIO_IRQConfig(uint8_t IRQNumber_u8,uint8_t IRQPriority, uint8_t state_u8)
{

}
/*********************************************************************************************************************
* Function name      :
* Description        :
*

*********************************************************************************************************************/

void GPIO_IRQHandler(uint8_t pinNumber_u8)
{

}

/************************************************ end of STM32Lxx_gpio_driver.c ***************************************************/


