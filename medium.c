/*                                                                                                                                          *
 * Loads bootlaoader over UART for STM32                                                                                                    */

#include "main.h"                  

#define BOOT_FLAG_ADDRESS           0x08004000U //Boot flag address
#define APP_ADDRESS                 0x08008000U //Application address
#define TIMEOUT_VALUE               SystemCoreClock/4 //timeout

#define ACK     0x06U //ack bit address
#define NACK    0x16U //nack bit address

static UART_HandleTypeDef huart; //We call huart from STM32 HAL library UART_HandleTypeDef structure
static uint8_t RX_Buffer[32]; //We allocate 32 bytes of space in the buffer for the messages received from RX

//Enum structure is written to hold commands
typedef enum
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;

//functions are called

static void Jump2App(void);
static void Boot_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);

int main(void)
{
    Clk_Update(); //clock update?
    Boot_Init();  //Boot function is started
    
    /*Initially, the ACK is transmitted. If there is no response to the ACK during the timeout, the NACK is transmitted. 
    Switch to Jump2App function. */

    Transmit_ACK(&huart);
    if(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT) //STM32 UART interface with HAL. Data is transmitted using blocking mode
    {
        Transmit_NACK(&huart);
        Jump2App();
    }

    /*Checking if Buffer size is not equal to 1 or 0th index of Buffer is not equal to ACK. As a result, NACK is transmitted 
    and sent to the Jum2App function.*/

    if(Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK)
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
    
    //If ACK is transmitted without a timeout, cases will occur. But if timeout occurs, NACK is transmitted.
	for(;;) //endless loop
	{
        while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
        
        if(Check_Checksum(RX_Buffer, 2) != 1)
        {
            Transmit_NACK(&huart);
        }
        else
        {
            switch(RX_Buffer[0])
            {
                case ERASE:
                    Transmit_ACK(&huart);
                    Erase();
                    break;
                case WRITE:
                    Transmit_ACK(&huart);
                    Write();
                    break;
                case CHECK:
                    Transmit_ACK(&huart);
                    Check();
                    break;
                case JUMP:
                    Transmit_ACK(&huart);
                    Jump2App();
                    break;
                default: 
                    Transmit_NACK(&huart);
                    break;
            }
        }
	}
    
    for(;;);
	return 0;
}

/*It is a jump to software function. The application address is checked. 
If there is no problem, it enters the function. First of all, interrupts are disabled. Main Stack Pointer is set.*/

static void Jump2App(void)
{
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000) 
    {
        __disable_irq();
        uint32_t jump_address = *(__IO uint32_t *)(APP_ADDRESS + 4);
        __set_MSP(*(__IO uint32_t *)APP_ADDRESS);
        void (*pmain_app)(void) = (void (*)(void))(jump_address);
        pmain_app();
    }
    
}

//Boot start function. It is done using UART. For this reason, UART pins are set.

static void Boot_Init(void)
{
    GPIO_InitTypeDef gpio_uart;
    
    gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_uart.Mode = GPIO_MODE_AF_PP;
    gpio_uart.Pull = GPIO_PULL_NONE;
    gpio_uart.Speed = GPIO_SPEED_LOW;
    gpio_uart.Alternate = GPIO_AF7_USART2;
    
    HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &gpio_uart);
    
    huart.Init.BaudRate = 115200;
    huart.Init.Mode = HAL_UART_MODE_TX_RX;
    huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16;
    huart.Init.Parity = HAL_UART_PARITY_NONE;
    huart.Init.StopBits = HAL_UART_STOP_1;
    huart.Init.WordLength = HAL_UART_WORD8;
    huart.Instance = USART2;
    
    HAL_RCC_USART2_CLK_ENABLE();
    HAL_UART_Init(&huart);
}

//ACK is transmitted
static void Transmit_ACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {ACK, ACK};

    HAL_UART_Tx(handle, msg, 2);
}

//NACK is transmitted
static void Transmit_NACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {NACK, NACK};
    
    HAL_UART_Tx(handle, msg, 2);
}

//Checks the message
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)
{
    uint8_t initial = 0xFF;
    uint8_t result = 0x7F; 
    
    result = initial ^ *pBuffer++;
    len--;
    while(len--)
    {
        result ^= *pBuffer++;
    }
    
    result ^= 0xFF;
    
    if(result == 0x00)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//Erase function via flash memory
static void Erase(void)
{
    Flash_EraseInitTypeDef flashEraseConfig;
    uint32_t sectorError;
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 3) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    
    if(RX_Buffer[0] == 0xFF)
    {
        Transmit_NACK(&huart);
    }
    else
    {
       
        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;
        flashEraseConfig.NbSectors = RX_Buffer[0];
        flashEraseConfig.Sector = RX_Buffer[1];

        HAL_Flash_Unlock(); 
        HAL_Flash_Erase(&flashEraseConfig, &sectorError);
        HAL_Flash_Lock();
        
        Transmit_ACK(&huart);
    }
}

//Flash memory write function
static void Write(void)
{
    uint8_t numBytes;
    uint32_t startingAddress = 0;
    uint8_t i;

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }

    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    numBytes = RX_Buffer[0];
    
    while(HAL_UART_Rx(&huart, RX_Buffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }

    i = 0;
    HAL_Flash_Unlock();
    while(numBytes--)
    {
        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, RX_Buffer[i]);
        startingAddress++;
        i++; 
    }
    HAL_Flash_Lock();
    Transmit_ACK(&huart);
}

//Flash memory control function
static void Check(void)
{
    uint32_t startingAddress = 0;
    uint32_t endingAddress = 0;
    uint32_t address;
    uint32_t *data;
    uint32_t crcResult;
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    HAL_RCC_CRC_CLK_ENABLE();
    data = (uint32_t *)((__IO uint32_t*) startingAddress);
    for(address = startingAddress; address < endingAddress; address += 4)
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(data, 1);
    }
    
    HAL_RCC_CRC_CLK_DISABLE();
    if(crcResult == 0x00)
    {
        Transmit_ACK(&huart);
    }
    else
    {
        Transmit_NACK(&huart);
    }
    
    Jump2App();
}