/*
 * STM32Lxx_LCD_Driver.c
 *
 *  Created on: Sep 19, 2025
 *      Author: HP
 */
/**********************************************************************************************************************
 * Include files
**********************************************************************************************************************/
#include"STM32Lxx.h"
#include "STM32Lxx_LCD_header.h"

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
static void LCD_write4Bits(uint8_t val);
static void LCD_enable(void);
static void LCD_mdelay(uint32_t );
static void LCD_udelay(uint32_t );
/***************************************** end of Static Function Prototypes ****************************************/

/*********************************************************************************************************************
 * Function Definitions
*********************************************************************************************************************/

/*********************************************************************************************************************
* Function name      :
* Description        :
*
* Passing Arguments  : None
* Function Called By :
* Return value       : None
*********************************************************************************************************************/


/************************************************ end of Function Definitions ****************************************/
void LCD_init(void)
{
	GPIO_Handle_t lcd_signal;


		lcd_signal.pGPIOx = LCD_GPIO_PORT;
		lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS; GPIO_Init(&lcd_signal);
		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW; GPIO_Init(&lcd_signal);
		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN; GPIO_Init(&lcd_signal);


		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4; GPIO_Init(&lcd_signal);
		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5; GPIO_Init(&lcd_signal);
		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6; GPIO_Init(&lcd_signal);
		lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7; GPIO_Init(&lcd_signal);


		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);


		LCD_mdelay(40);

		LCD_write4Bits(0x3); LCD_mdelay(5);
		LCD_write4Bits(0x3); LCD_udelay(150);
		LCD_write4Bits(0x3);
		LCD_write4Bits(0x2);


		LCD_sendCommand(LCD_CMD_4DL_2N_5X8F);
		LCD_sendCommand(LCD_CMD_DON_CURON);
		LCD_displayClear();
		LCD_sendCommand(LCD_CMD_INCADD);
}
void LCD_sendCommand(uint8_t cmd)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	LCD_write4Bits(cmd >> 4);
	LCD_write4Bits(cmd & 0x0F);
}
void LCD_sendData(uint8_t data)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	LCD_write4Bits(data >> 4);
	LCD_write4Bits(data & 0x0F);
}
void LCD_displayClear(void)
{
	LCD_sendCommand(LCD_CMD_DIS_CLEAR);
	LCD_mdelay(2);
}
void LCD_displayReturnHome(void)
{
	LCD_sendCommand(LCD_CMD_DIS_RETURN_HOME);
	LCD_mdelay(2);
}
void LCD_sendString(char * msg)
{
	  do
	    {
		  LCD_sendData((uint8_t)*msg++);
	    }
	    while (*msg != '\0');
}
void LCD_setCursur(uint8_t row, uint8_t col)
{
	col--;
	  switch (row)
	  {
	    case 1:
	    	LCD_sendCommand((col |= 0x80));
	      break;
	    case 2:
	    	LCD_sendCommand((col |= 0xC0));
	      break;
	    default:
	      break;
	  }
}
static void LCD_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	LCD_udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	LCD_udelay(100);
}


static void LCD_mdelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1000); i++);
}

static void LCD_udelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < cnt; i++);
}
static void LCD_write4Bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, (value >> 0) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, (value >> 1) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, (value >> 2) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, (value >> 3) & 0x1);

	LCD_enable();
}


/************************************************ end of STM32Lxx_LCD_Driver.c ***************************************************/


