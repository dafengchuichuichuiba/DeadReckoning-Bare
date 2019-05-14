#include <stdint.h>
#include <stdlib.h>
#include "stm32f1xx.h"
// #include "stm32f103xb.h"
#include "globals.h"
#include "main.h"

/*********************** defines                    *************************/
#define MPU_ADDRESS                     0x68
#define BLUE_PILL                       0x0410
#define MPU_DELAY   150000UL

/*********************** global variables           *************************/

/*********************** ISR definitions            *************************/

/*********************** function definitions       *************************/

/*  Function:       init_gpio_pins()
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void init_gpio_pins();

void i2c_start(uint8_t num_bytes) {
    // Set Start bit in CR1 to generate Start condition
    I2C1->CR1 |= I2C_CR1_START;
}

void i2c_send_7bit_address(uint32_t slave_address) {
    I2C1->SR1 |= I2C_SR1_SB; // Set start bit 
    I2C1->DR = slave_address; // send slave address
}

void i2c_stop() {
    // Check Stop Condition, automatically applies NACK to SDA line, asserts to slave that no further data will be transmitted or received
    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP); // confirm end condition is met
}

uint8_t i2c_receive_data() {
    return (uint8_t) I2C1->DR;
}

void i2c_send_data(uint8_t data) {
    I2C1->DR = data;
}

/*  Function:       I2C_Low_Level_Init
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void I2C_Low_Level_Init(int ClockSpeed, int OwnAddress);

/* 
    Handles everything needed to write to address
*/
void I2C_Write(const uint8_t *buf, uint32_t nbyte, uint8_t SlaveAddress);

/* 
    Handles everything needed to write to address
*/
void I2C_Read(uint8_t *buf, uint32_t nbyte, uint8_t SlaveAddress);

/*  Function:       init_clock()
    Description:    configure SysClock to run at 72MHz
    Parameters:     void
    Returns:        void 
*/
void init_clock(void);

void init_MPU9250();

void readAccelerometer(double* acc);

void init_hardware();

/*
    Function:      main()
    Description:   program entry point
    Parameters:    void
    Returns:       void
*/
int main(void) {
    init_hardware();
    init_MPU9250();

    // Initialize Acceleration Array
    double* acc = (double *)malloc(sizeof(double)*3);
    acc[0] = 11;
    acc[1] = 12;

    uint32_t       mpu_delay_count = 0;
    while (1)
    {
        // main loop
        mpu_delay_count = ( (mpu_delay_count + 1) % MPU_DELAY );
        if (mpu_delay_count == 0) {
            readAccelerometer(acc);
        }
    }
    
    return 0;
}

void init_MPU9250() {
    // MPU Initialization
    uint8_t buffer[2] = { 0x6B, 0 };
    I2C_Write(buffer, 2, MPU_ADDRESS);

    // Gyroscope Initialization
    buffer[0] = 0x1B;
    buffer[1] = 0b00011000;
    I2C_Write(buffer, 2, MPU_ADDRESS);

    // Accelerometer Initialization
    buffer[0] = 0x1C;
    buffer[1] = 0b00011000;
    I2C_Write(buffer, 2, MPU_ADDRESS);
}

void readAccelerometer(double* acc) {
    uint8_t RawData[6] = { 2, 2, 2, 2, 2, 2 };
    uint8_t buf[6] = {0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40};

    for (int i = 0; i < 6; i++) {
        I2C_Write(&buf[i], 1, MPU_ADDRESS);
        I2C_Read(&RawData[i], 1, MPU_ADDRESS);
    }
    // I2C_Write(buf, 6, MPU_ADDRESS);
    // I2C_Read(RawData, 6, MPU_ADDRESS);

    double RealData[3];
    RealData[0] = RawData[0]<<8|RawData[1];
    RealData[1] = RawData[2]<<8|RawData[3];
    RealData[2] = RawData[4]<<8|RawData[5];

    acc[0] = RealData[0]/2048.0;
    acc[1] = RealData[1]/2048.0;
    acc[2] = RealData[2]/2048.0;
}

void init_hardware() {
    init_clock();
    I2C_Low_Level_Init(0b100100, BLUE_PILL);
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

/*  Function:       init_i2c()
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void I2C_Low_Level_Init(int ClockSpeed, int OwnAddress) {

    // Enable GPIOB clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* Enable I2C1 Clock */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; 

    /* Enable AFIO Pins for I2C1 */
    init_gpio_pins();

    /* I2C1 SDA and SCL configuration */
    I2C1->CCR &= ~I2C_CCR_FS; // set to slow mode
    I2C1->CCR &= ~I2C_CCR_DUTY; // set duty cycle to 2
    I2C1->CR1 |= I2C_CR1_ACK; // Enable acknowledge bit

    //TODO: FIGURE OUT HOW TF I2C_Init Works in I2C.c
    /*---------------------------- I2Cx CR2 Configuration ------------------------*/
    I2C1->CR2 |= ClockSpeed; // Ex: 100100 sets to f_pclk to 36 MHz and is equal to 36

    /*---------------------------- I2Cx CCR Configuration ------------------------*/
    I2C1->CR1 &= ~I2C_CR1_PE; // disable peripherals to configure Trise and CCR
    I2C1->CCR = (uint32_t)0x28; // set clock control register according pg 782 of datasheet
    // Configure Rise Time Registers
    I2C1->TRISE = (001001 << 0); // Sets Trise to 9 as per Data Sheet thus PClk_1 = 125 ns

    /*---------------------------- I2Cx CR1 Configuration ------------------------*/
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
    I2C1->OAR1 |= (OwnAddress << 1); // set self address STM32 Blue pill is 0x0410
    I2C1->OAR1 &= ~I2C_OAR1_ADDMODE; // slave address uses 7 bits

    /* Complete Initialization Sequence By Re-enabling Peripherals */
    I2C1->CR1 |= I2C_CR1_PE;
}

/*  Function:       init_gpio_pins()
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void init_gpio_pins() {
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

    /* I2C1 Reset */
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
}

/* 
    Handles everything needed to write to slave address
*/
void I2C_Write(const uint8_t *buf, uint32_t nbyte, uint8_t SlaveAddress) {
    if (nbyte) {
        while(I2C1->SR2 & ~I2C_SR2_BUSY);

        /* Initiate Start Sequence */
        i2c_start(1);
        while(I2C1->SR1 & I2C_SR1_SB);

        /* Send 7 bit slave Address */
        i2c_send_7bit_address(SlaveAddress); // send slave mpu address 1101000
        /* Wait for ADDR bit to be set and then clear by reading SR1 and SR2 */
        while(I2C1->SR1 & I2C_SR1_ADDR);
        while(I2C1->SR2 & ~I2C_SR1_SB);
        while(I2C1->SR1 & ~I2C_SR1_ADDR);

        /* Send Address to Be Transmitted */
        i2c_send_data(*buf++);

        while (--nbyte) {
            // wait on Byte Transfer Finish
            while(I2C1->SR1 & I2C_SR1_BTF);
            i2c_send_data(*buf++);
        }
        while(I2C1->SR1 & I2C_SR1_BTF);

        /* Generate Stop Condition */
        i2c_stop();
        while(I2C1->CR1 & ~I2C_CR1_STOP);
    }
}

/* 
    Handles everything needed to write to address
*/
void I2C_Read(uint8_t *buf, uint32_t nbyte, uint8_t SlaveAddress) {
    if (!nbyte) return;

    /* Wait for Idle I2C Interface */
    while(I2C1->SR2 & ~I2C_SR2_BUSY);

    /* Initiate Start Sequence */
    i2c_start(1);
    while(I2C1->SR1 & I2C_SR1_SB); // wait for start bit to be set
    // Send Address
    i2c_send_7bit_address(SlaveAddress);
    while(I2C1->SR1 & I2C_SR1_ADDR); // Wait For Addresses to Match

    if (nbyte == 1) {
        // Clear Ack bit
        I2C1->CR1 &= ~I2C_CR1_ACK;

        // EV6_1 -- must be atomic -- Clear ADDR, generate STOP
        // (void) I2C1->SR2;
        __disable_irq();
        (void) I2C1->SR1;
        (void) I2C1->SR2;
        while(I2C1->SR1 & ~I2C_SR1_ADDR);
        /* Generate a STOP condition */
        I2C1->CR1 |= I2C_CR1_STOP;
        __enable_irq();

        // Receive data
        while(I2C1->SR1 & I2C_SR1_RXNE); // wait until data is received
        *buf++ = i2c_receive_data();

        /* Wait until stop is cleared by hardware and prepare ack bit for another reception*/
        while(I2C1->CR1 & ~I2C_CR1_STOP);
        I2C1->CR1 |= I2C_CR1_ACK;
    } else if (nbyte == 2) {
        // Set POS and ACK flag
        I2C1->CR1 |= I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_POS;

        /* Clears ADDR and ACK bit */
        __disable_irq();
        (void) I2C1->SR1;
        (void) I2C1->SR2;
        while(I2C1->SR1 & ~I2C_SR1_ADDR);
        I2C1->CR1 &= ~I2C_CR1_ACK;
        __enable_irq();

        while (I2C1->SR1 & I2C_SR1_BTF); // wait btf to be set by hardware

        /* Disable Interrupts As Per Spec Sheet */
        __disable_irq();
        i2c_stop();
        *buf++ = i2c_receive_data();
        __enable_irq();

        *buf++ = i2c_receive_data();
        /* Wait until stop is cleared by hardware */
        while(I2C1->CR1 & ~I2C_CR1_STOP);

        /* reset pos and set ack for another reception */
        I2C1->CR1 &= ~I2C_CR1_POS;
        I2C1->CR1 |= I2C_CR1_ACK;
    } else {
        /* Clear ACK bit */
        // I2C1->CR1 &= ~I2C_CR1_ACK;
        while (nbyte-- > 3) {
            while (I2C1->SR1 & I2C_SR1_BTF); // wait btf to be set by hardware
            /* Read in next data bit */
            *buf++ = i2c_receive_data();
        }

        while (I2C1->SR1 & I2C_SR1_BTF); // wait btf to be set by hardware
        /* Clear ACK bit */
        I2C1->CR1 &= ~I2C_CR1_ACK;

        __disable_irq();
        i2c_stop();

        /* Read in Data N-1 from register */
        *buf++ = i2c_receive_data();
        __enable_irq();

        /* Wait for received bit register to have bits again */
        while(I2C1->SR1 & I2C_SR1_RXNE);
        
        *buf++ = i2c_receive_data();
        while(I2C1->CR1 & ~I2C_CR1_STOP); // wait until stop generation cleared by hardware

         /* Set ACK bit for another reception */
        I2C1->CR1 |= I2C_CR1_ACK;
    }
}
