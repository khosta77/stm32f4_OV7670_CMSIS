#include "../system/include/cmsis/stm32f4xx.h"

void I2C2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11;
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
    GPIOB->AFR[1] |= GPIO_AFRH_AFRH10_2 | GPIO_AFRH_AFRH11_2;

    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN ;
    I2C2->CR2 = 0x2A;
    I2C2->CCR = 0xD2;
    I2C2->TRISE = 0x2B;
    I2C2->CR1 |= I2C_CR1_PE;
}

void I2C2_write_bytes(uint8_t addr, uint8_t *data, uint8_t len) {
	while (I2C2->SR2 & I2C_SR2_BUSY);
	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));
	I2C2->DR = (addr << 1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR2;
	for (int i = 0; i < len; i++, *data++){
		I2C2->DR = *data;
		while (!(I2C2->SR1 & I2C_SR1_BTF));
	}
	I2C2->CR1 |= I2C_CR1_STOP;
}

void I2C2_write_code(uint8_t address, uint8_t code, uint8_t data) {
	while (I2C2->SR2 & I2C_SR2_BUSY);
	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));
	I2C2->DR = (address << 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR2;
	I2C2->DR = code;
	while (!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->DR = data;
	while (!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->CR1 |= I2C_CR1_STOP;
}

void I2C2_write_byte(uint8_t address, uint8_t data) {
	while (I2C2->SR2 & I2C_SR2_BUSY);
	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));
	I2C2->DR = (address << 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR2;
	I2C2->DR = data;
	while (!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->CR1 |= I2C_CR1_STOP;
}

void I2C2_read_bytes(uint8_t address, uint8_t *data, uint8_t size) {
    while (I2C2->SR2 & I2C_SR2_BUSY);
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    I2C2->CR1 |= I2C_CR1_ACK;
    I2C2->DR = ((address << 1) | 0x01);
    while (!(I2C2->SR1 & I2C_SR1_ADDR))
    (void) I2C2->SR2;
    while (size--) {
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        *data++ = I2C2->DR;
    }
    I2C2->CR1 |= I2C_CR1_STOP;
    I2C2->CR1 &=~ I2C_CR1_ACK;
}

uint8_t I2C2_read_byte(uint8_t address, uint8_t code) {
    while (I2C2->SR2 & I2C_SR2_BUSY);
	uint8_t data; // Возращаемые данные 
    I2C2_write_byte(address, code);
	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));
    I2C2->CR1 |= I2C_CR1_ACK;
	I2C2->DR = ((address << 1) | 0x1);
	while (!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR2;
	while (!(I2C2->SR1 & I2C_SR1_RXNE));
	data = I2C2->DR;
	I2C2->CR1 |= I2C_CR1_STOP;
    I2C2->CR1 &= ~I2C_CR1_ACK;
	return data;
}

void DMCI_init() {
    //// GPIO
    // GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (GPIO_MODER_MODER4_1 |
                     GPIO_MODER_MODER6_1 |
                     GPIO_MODER_MODER9_1 |
                     GPIO_MODER_MODER10_1);
    GPIOA->OSPEED |= ((GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR4_0) | 
                      (GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0) |
                      (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR9_0) |
                      (GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR10_0));
    GPIOA->AFR[0] |= ((0xD << 16) | (0xD << 24));
    GPIOA->AFR[1] |= ((0xD << 4) | (0xD << 8));

    // GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 |
                     GPIO_MODER_MODER7_1 |
                     GPIO_MODER_MODER8_1 |
                     GPIO_MODER_MODER9_1);
    GPIOB->OSPEED |= ((GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0) | 
                      (GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0) |
                      (GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR8_0) |
                      (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR9_0));
    GPIOB->AFR[0] |= ((0xD << 24) | (0xD << 28));
    GPIOB->AFR[1] |= ((0xD << 0) | (0xD << 4));

    // GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (GPIO_MODER_MODER8_1 |
                     GPIO_MODER_MODER9_1 |
                     GPIO_MODER_MODER11_1);
    GPIOC->OSPEED |= ((GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR8_0) |
                      (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR9_0) |
                      (GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR11_0));
    GPIOC->AFR[1] |= ((0xD << 0) | (0xD << 4) | (0xD << 12));

    
}

int main(void) {
    I2C2_init();


	while(1) {

	}
}


