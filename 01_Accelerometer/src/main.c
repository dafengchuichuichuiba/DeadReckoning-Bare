#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "globals.h"
#include "main.h"

/*********************** defines                    *************************/

/** 
  * @brief  Communication start
  * 
  * After sending the START condition the master 
  * has to wait for this event. It means that the Start condition has been correctly 
  * released on the I2C bus (the bus is free, no other devices is communicating).
  * 
  */
#define  I2C_EVENT_MASTER_MODE_SELECT                      ((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

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

    // Set alternate gpio scl sda behavior to default
    AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;
    // GPIO Mode, Type, Speed, and Alternate Function for pins 6 and 7
    GPIOB->CRL  |= GPIO_CRL_MODE1_0; // sets gpio high to output mode max speed 50MHz
    GPIOB->CRL  |= GPIO_CRL_MODE1_1; // sets gpio high to output mode max speed 50MHz
    GPIOB->CRH  |= GPIO_CRH_MODE9_0; // does the same but for high mode
    GPIOB->CRH  |= GPIO_CRH_MODE9_1; // does the same but for high mode

    // Set According to Ref 9.2.2
    GPIOB->CRL  |= GPIO_CRL_CNF1_1; // sets alternate function open drain mode
    GPIOB->CRL  |= GPIO_CRL_CNF1_0; // sets alternate function open drain mode 
    GPIOB->CRH  |= GPIO_CRH_CNF9_0; // does the same but for high mode
    GPIOB->CRH  |= GPIO_CRH_CNF9_1; // does the same but for high mode
}

/**
  * @brief  Returns the last I2Cx Event.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  *     
  * @note: For detailed description of Events, please refer to section 
  *    I2C_Events in stm32f10x_i2c.h file.
  *    
  * @retval The last event
  */
uint32_t i2c_get_last_event()
{
  uint32_t lastevent = 0;
  uint32_t flag1 = 0, flag2 = 0;

  /* Read the I2Cx status register */
  flag1 = I2C1->SR1;
  flag2 = I2C1->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_Mask;

  /* Return status */
  return lastevent;
}

void i2c_start(uint8_t num_bytes) {
    // Set Start bit in CR1 to generate Start condition
    I2C1->CR1 |= I2C_CR1_START;
}

/*  Function: i2c_start_write
    Description: Start writing a set number of bits to i2c
    Parameters: Number of Bytes To Write
    Returns: void
*/
void i2c_generate_start(uint8_t num_bytes) {
    I2C1->OAR1 &= I2C_OAR1_ADD0; // sets address to write
    i2c_start(num_bytes);
}

void i2c_send_7bit_address(uint32_t slave_address) {
    slave_address &= ~I2C_OAR1_ADD0;
    I2C1->DR = slave_address;
}

void i2c_stop(int num_bytes) {
    // Check Stop Condition, automatically applies NACK to SDA line, asserts to slave that no further data will be transmitted or received
    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP); // confirm end condition is met
}

/*  Function: i2c_start_read
    Description: Start writing a set number of bits to i2c
    Parameters: Number of Bytes To Write
    Returns: void
*/
void i2c_start_read(uint8_t num_bytes) {
    I2C1->OAR1 |= ~I2C_OAR1_ADD0; // sets address to read
    i2c_start(num_bytes);
}

void i2c_send_data(uint8_t data) {
    I2C1->DR = data;
}

/*  Function:       init_i2c()
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void init_i2c() {
    // Note: Assumes 4.7 kohm pullup and 20 kHz scl
    // enable I2C #1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable i2c1 clock
    I2C1->CR1 &= ~I2C_CR1_PE; // disable peripherals to configure Trise and CCR
    // Programs CR2 to generate correct timings
    I2C1->CR2 |= (100100 << 0); // sets peripheral clock frequency f_pclk to 36 MHz
    // Configure CCR
    I2C1->CCR |= I2C_CCR_FS; // set to fast mode
    I2C1->CCR &= ~I2C_CCR_DUTY; // set duty to zero
    I2C1->CCR = (uint32_t)0x0384; // set clock control register according to specs
    // Configure Rise Time Registers
    I2C1->TRISE = (010101 << 0); // Sets Trise to 21 as per Data Sheet
    
    /*---------------------------- I2Cx CR1 Configuration ------------------------*/
    I2C1->CR1 |= I2C_CR1_PE; // enable peripherals
    /* Clear ACK, SMBTYPE and  SMBUS bits */
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 &= ~I2C_CR1_SMBTYPE;
    I2C1->CR1 &= ~I2C_CR1_SMBUS;
    /* Configure I2Cx: mode and acknowledgement */
    /* Already Set SMBTYPE and SMBUS bits according to I2C_Mode value */
    /* Set ACK bit according to I2C_Ack value */
    I2C1->CR1 |= I2C_CR1_ACK;
    
    /*---------------------------- I2Cx OAR1 Configuration -----------------------*/
    /* Set I2Cx Own Address1 and acknowledged address */
    I2C1->OAR1 &= ~I2C_OAR1_ADDMODE; // slave address uses 7 bits
    I2C1->OAR1 |= (0x0410 << 1); // 7 bit STM32 Blue ID with LSB reset

    i2c_generate_start(1); 
    uint32_t lastevent = i2c_get_last_event;
    while((lastevent & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT){
        lastevent = i2c_get_last_event;
    } // confirm start condition is met

    i2c_send_7bit_address(1101000); // send slave mpu address 1101000
    lastevent = i2c_get_last_event;
    while(((lastevent & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        lastevent = i2c_get_last_event;
    }

    // Assign Byte to be transimitted
    i2c_send_data(0x6B); // PWR_MGT_1 address on mpu-9250
    while(!(I2C1->SR1 & I2C1->SR2) & I2C_SR1_TXE); // wait til transmit buffer is empty adn Byte Transfer completed # Potential Error
    i2c_start_read(1);

    while(!(I2C1->SR1 & I2C_SR1_RXNE)); // wait until data is received
    uint8_t received_data = I2C1->DR;

    // Check Stop Condition, automatically applies NACK to SDA line, asserts to slave that no further data will be transmitted or received
    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP); // confirm end condition is met

    i2c_stop(1);
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
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // multiply by 9
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

