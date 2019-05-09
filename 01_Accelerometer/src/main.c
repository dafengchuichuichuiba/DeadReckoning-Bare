/****************************************************************************\
 File:          main.c
 Date:
 
 Description:
 
 Known bugs/missing features:
 
\****************************************************************************/

#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "globals.h"
#include "main.h"

/*********************** defines                    *************************/

/*********************** global variables           *************************/

/*********************** ISR definitions            *************************/

/*********************** function definitions       *************************/

/*  Function:       init_accelerometer()
    Description:    configure accelerometer to communicate via i2c
    Parameters:     SDA and SCL pins
    Returns:        void
*/
void init_accelerometer() {

}

/*  Function:       init_gpio_pins()
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void init_gpio_pins() {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable i2c 1 on board
    // GPIO Mode, Type, Speed, and Alternate Function for pins 6 and 7
    GPIOB->CRL  |= GPIO_CRL_MODE1_0; // sets gpio high to output mode speed 10MHz
    GPIOB->CRL  &= ~GPIO_CRL_MODE1_1; // sets gpio high to output mode speed 10MHz
    GPIOB->CRH  |= GPIO_CRH_MODE9_0; // does the same but for high mode
    GPIOB->CRH  &= ~GPIO_CRH_MODE9_1; // does the same but for high mode

    // Set According to Ref 9.2.2
    GPIOB->CRL  |= GPIO_CRL_CNF1_1; // sets alternate function open drain mode
    GPIOB->CRL  |= ~GPIO_CRL_CNF1_0; // sets alternate function open drain mode 
    GPIOB->CRH  |= ~GPIO_CRH_CNF9_0; // does the same but for high mode
    GPIOB->CRH  |= GPIO_CRH_CNF9_1; // does the same but for high mode

    // Set alternate gpio scl sda behavior to default
    AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP_Msk;
}

/*  Function:       init_i2c()
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void init_i2c() {
    // Note: Assumes 4.7 kohm pullup and 400 kHz scl
    // enable I2C #1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 &= ~I2C_CR1_PE; // disable peripherals to configure Trise and CCR
    // Programs CR2 to generate correct timings
    I2C1->CR2 |= (001010 << 0); // sets peripheral clock frequency to 10 MHz
    // Configure CCR
    I2C1->CCR |= I2C_CCR_FS; // set to fast mode
    I2C1->CCR &= ~I2C_CCR_DUTY; // set duty to zero
    I2C1->CCR = (uint32_t)0x801E; // set clock control register
    // Configure Rise Time Registers
    I2C1->TRISE = (00100 << 0); // Sets Trise to 4 as per Data Sheet
    // Set i2c sensor settings
    I2C1->OAR1 |= I2C_OAR1_ADDMODE; // slave address uses 7 bits
    I2C1->OAR1 |= (1101000 << 1); // 7 bit slave address
    // Program CR1 to enable peripherals
    I2C1->CR1 |= I2C_CR1_PE; // enable peripherals
    // Set Start bit in CR1 to generate Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while(I2C1->CR1 & I2C_CR1_START); // confirm start condition is met

    // Assign Byte to be transimitted
    I2C1->DR = 0x6B; // PWR_MGT_1 address on mpu-9250

    while(!(I2C1->SR1 && I2C_SR1_TXE)); // wait til transmit buffer is empty

    // Check Stop Condition, automatically applies NACK to SDA line, asserts to slave that no further data will be transmitted or received
    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP); // confirm end condition is met
}

/*  Function:       init_clock()
    Description:    configure SysClock to run at 72MHz
    Parameters:     void
    Returns:        void 
*/
void init_clock(void) {
    // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE
    RCC->CR         |= RCC_CR_PLLON;        // enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // set clock source to pll

    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // wait for PLL as source
    
    SystemCoreClockUpdate();                // calculate the SYSCLOCK value
}

void init_hardware() {
    init_clock();
    init_gpio_pins();
    init_i2c();
}

/*
    Function:      main()
    Description:   program entry point
    Parameters:    void
    Returns:       void
*/
int main(void) 
{
    init_hardware();

    while (1)
    {
        // main loop
    }
    
    return 0;
}

