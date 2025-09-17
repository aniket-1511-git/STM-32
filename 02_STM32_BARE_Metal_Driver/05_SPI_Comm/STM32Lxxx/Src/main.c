#include"main.h"

void delay(void);                     // Function prototype for delay
void SPI2_GPIOInits(void);            // Function prototype for SPI2 GPIO setup
extern  void initialise_monitor_handles(void);

// Function to configure GPIO pins for SPI2
void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;                              // Handle for GPIO config
    SPIPins.pGPIOx = GPIOB;                             // Use GPIOB port
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;     // Alternate function mode
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;             // AF5 = SPI2
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // Push-pull output
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No pull-up/pull-down
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;    // High speed output
    GPIO_PeriClockControl(GPIOB, ENABLE);                      // Enable GPIOB clock

    // Configure NSS pin (PB12)
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);

    // Configure SCK pin (PB13)
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // Configure MISO pin (PB14)
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // Configure MOSI pin (PB15)
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);
}


// Function to configure SPI2 peripheral
void SPI2_Inits_master(void)
{
    SPI_Handle_t SPI2Handle;                         // Handle for SPI config
    SPI2Handle.pSPIx = SPI2;                         // Use SPI2 peripheral
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER; // Set as master
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;       // Full-duplex
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;    // Clock speed = PCLK/32
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;                 // 8-bit data frame
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                 // Clock polarity = 0
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;                 // Clock phase = 0
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                    // Hardware NSS management

    SPI_Init(&SPI2Handle);  // Initialize SPI2 with this configuration
}
void SPI2_Inits_Slave(void)
{
    SPI_Handle_t SPI2Handle;                         // Handle for SPI config
    SPI2Handle.pSPIx = SPI2;                         // Use SPI2 peripheral
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE; // Set as master
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;       // Full-duplex
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;    // Clock speed = PCLK/32
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;                 // 8-bit data frame
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                 // Clock polarity = 0
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;                 // Clock phase = 0
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                    // Hardware NSS management

    SPI_Init(&SPI2Handle);
}

int main(void)
{
	 GPIO_Handle_t GPIOBtn;
	memset(&GPIOBtn,0,sizeof(GPIOBtn));
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
    uint8_t data=15,rData;

    SPI2_GPIOInits();
    SPI2_Inits_master();

    SPI_SSOEConfig(SPI2, ENABLE);
    SPI_PeripheralControl(SPI2, ENABLE);
    initialise_monitor_handles();
    printf("SPI DRIVER \n");
	SPI_SendData(SPI2, &data, 1);
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	delay();
	SPI_PeripheralControl(SPI2, DISABLE);

	SPI2_Inits_Slave();
	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_ReceiveData(SPI2, &rData, 1);
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	printf("Received Data : %d\n",rData);
	while(1)
	   {

	   }
}


// Simple delay function
void delay()
{
    for(volatile int i = 0; i < 500000/2; i++);
}
