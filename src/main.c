#include "../system/include/cmsis/stm32f4xx.h"
#include "../system/include/stm32f4-hal/stm32f4xx_hal.h"
#include "../system/include/stm32f4-hal/stm32f4xx_hal_dcmi.h"
#include "../system/include/stm32f4-hal/stm32f4xx_hal_dcmi_ex.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//===========================================================================================================
#if 0
// SCCB write address
#define SCCB_REG_ADDR 		0x01

// OV7670 camera settings
#define OV7670_REG_NUM 		121
#define OV7670_WRITE_ADDR 	0x42

// Image settings
#define IMG_ROWS            144
#define IMG_COLUMNS   		174

uint8_t temp_buffer[IMG_ROWS * IMG_COLUMNS];
//int frame_flag = 0;
volatile uint16_t frame_buffer[IMG_ROWS * IMG_COLUMNS];
I2C_HandleTypeDef I2C_InitStructure;

static volatile bool frame_flag = false;
static volatile bool send_sync_frame = false;

const uint8_t OV7670_reg[OV7670_REG_NUM][2] = { 
    { 0x12, 0x80 },
// Image format
	{ 0x12, 0x8  },	 // 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV, 0xc = QCIF (RGB)
	{ 0xc,  0x8  },  //
	{ 0x11, 0x40 },  //
	{ 0xb0, 0x84 },	 // Color mode (Not documented??)
// Hardware window
	{ 0x11, 0x01 },  // PCLK settings, 15fps
	{ 0x32, 0x80 },	 // HREF
	{ 0x17, 0x17 },	 // HSTART
	{ 0x18, 0x05 },	 // HSTOP
	{ 0x03, 0x0a },	 // VREF
	{ 0x19, 0x02 },	 // VSTART
	{ 0x1a, 0x7a },	 // VSTOP
// Scalling numbers
	{ 0x70, 0x3a },	 // X_SCALING
	{ 0x71, 0x35 },	 // Y_SCALING
	{ 0x72, 0x11 },	 // DCW_SCALING
	{ 0x73, 0xf0 },	 // PCLK_DIV_SCALING
	{ 0xa2, 0x02 },	 // PCLK_DELAY_SCALING
// Matrix coefficients
	{ 0x4f, 0x80 },  //
	{ 0x50, 0x80 },  //
	{ 0x51, 0x00 },  //
	{ 0x52, 0x22 },  //
	{ 0x53, 0x5e },  //
	{ 0x54, 0x80 },  //
	{ 0x58, 0x9e },
// Gamma curve values
	{ 0x7a, 0x20 },  //
	{ 0x7b, 0x10 },  //
	{ 0x7c, 0x1e },  //
	{ 0x7d, 0x35 },  //
	{ 0x7e, 0x5a },  //
	{ 0x7f, 0x69 },  //
	{ 0x80, 0x76 },  //
	{ 0x81, 0x80 },  //
	{ 0x82, 0x88 },  //
	{ 0x83, 0x8f },  //
	{ 0x84, 0x96 },  //
	{ 0x85, 0xa3 },  //
	{ 0x86, 0xaf },  //
	{ 0x87, 0xc4 },  //
	{ 0x88, 0xd7 },  //
	{ 0x89, 0xe8 },
// AGC and AEC parameters
	{ 0xa5, 0x05 },  //
	{ 0xab, 0x07 },  //
	{ 0x24, 0x95 },  //
	{ 0x25, 0x33 },  //
	{ 0x26, 0xe3 },  //
	{ 0x9f, 0x78 },  //
	{ 0xa0, 0x68 },  //
	{ 0xa1, 0x03 },  //
	{ 0xa6, 0xd8 },  //
	{ 0xa7, 0xd8 },  //
	{ 0xa8, 0xf0 },  //
	{ 0xa9, 0x90 },  //
	{ 0xaa, 0x94 },  //
	{ 0x10, 0x00 },
// AWB parameters
	{ 0x43, 0x0a },  //
	{ 0x44, 0xf0 },  //
	{ 0x45, 0x34 },  //
	{ 0x46, 0x58 },  //
	{ 0x47, 0x28 },  //
	{ 0x48, 0x3a },  //
	{ 0x59, 0x88 },  //
	{ 0x5a, 0x88 },  //
	{ 0x5b, 0x44 },  //
	{ 0x5c, 0x67 },  //
	{ 0x5d, 0x49 },  //
	{ 0x5e, 0x0e },  //
	{ 0x6c, 0x0a },  //
	{ 0x6d, 0x55 },  //
	{ 0x6e, 0x11 },  //
	{ 0x6f, 0x9f },  //
	{ 0x6a, 0x40 },  //
	{ 0x01, 0x40 },  //
	{ 0x02, 0x60 },  //
	{ 0x13, 0xe7 },
// Additional parameters
	{ 0x34, 0x11 },  //
	{ 0x3f, 0x00 },  //
	{ 0x75, 0x05 },  //
	{ 0x76, 0xe1 },  //
	{ 0x4c, 0x00 },  //
	{ 0x77, 0x01 },  //
	{ 0xb8, 0x0a },  //
	{ 0x41, 0x18 },  //
	{ 0x3b, 0x12 },  //
	{ 0xa4, 0x88 },  //
	{ 0x96, 0x00 },  //
	{ 0x97, 0x30 },  //
	{ 0x98, 0x20 },  //
	{ 0x99, 0x30 },  //
	{ 0x9a, 0x84 },  //
	{ 0x9b, 0x29 },  //
	{ 0x9c, 0x03 },  //
	{ 0x9d, 0x4c },  //
	{ 0x9e, 0x3f },  //
	{ 0x78, 0x04 },  //
	{ 0x0e, 0x61 },  //
	{ 0x0f, 0x4b },  //
	{ 0x16, 0x02 },  //
	{ 0x1e, 0x00 },  //
	{ 0x21, 0x02 },  //
	{ 0x22, 0x91 },  //
	{ 0x29, 0x07 },  //
	{ 0x33, 0x0b },  //
	{ 0x35, 0x0b },  //
	{ 0x37, 0x1d },  //
	{ 0x38, 0x71 },  //
	{ 0x39, 0x2a },  //
	{ 0x3c, 0x78 },  //
	{ 0x4d, 0x40 },  //
	{ 0x4e, 0x20 },  //
	{ 0x69, 0x00 },  //
	{ 0x6b, 0x3a },  //
	{ 0x74, 0x10 },  //
	{ 0x8d, 0x4f },  //
	{ 0x8e, 0x00 },  //
	{ 0x8f, 0x00 },  //
	{ 0x90, 0x00 },  //
	{ 0x91, 0x00 },  //
    { 0x96, 0x00 },  //
	{ 0x9a, 0x00 },  //
	{ 0xb1, 0x0c },  //
    { 0xb2, 0x0e },  //
//	{ 0xb3, 0x82 },  //
//	{ 0x4b, 0x01 } 
};

void Delay(uint32_t nCount) {
	while (nCount--) {
    }
}
//===========================================================================================================
void I2C2_init() {
#if 0
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11;
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;  // SCL | SDA
    GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
    GPIOB->AFR[1] |= ((0x4 << 8) | (0x4 << 12));//GPIO_AFRH_AFRH10_2 | GPIO_AFRH_AFRH11_2;

    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN ;
    I2C2->CR2 = 0x2A;
    I2C2->CCR = 0xD2;
    I2C2->TRISE = 0x2B;
    I2C2->CR1 |= I2C_CR1_PE;
#else
    GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

	// GPIO config
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_I2C2);

	// I2C config
	//I2C_DeInit(I2C2);
    I2C_InitStructure.Instance = I2C2;
    I2C_InitStructure.Init.ClockSpeed = 100000;
    I2C_InitStructure.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2C_InitStructure.Init.OwnAddress1 = 0x0;
    I2C_InitStructure.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_InitStructure.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_InitStructure.Init.OwnAddress2 = 0;
    I2C_InitStructure.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_InitStructure.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&I2C_InitStructure);

	//I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	//I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	//I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	//I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	//I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	//I2C_InitStructure.I2C_ClockSpeed = 100000;
	//I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
	//I2C_Init(I2C2, &I2C_InitStructure);
	//I2C_Cmd(I2C2, ENABLE);
#endif
}

#define SUCCESS 0
#define ERROR -1
// Ошибка где то здесь

#define I2C2_CMSIS 1
#if I2C2_CMSIS
int SCCB_write_reg(uint8_t reg_addr, uint8_t data) {
#else
bool SCCB_write_reg(uint8_t reg_addr, uint8_t* data) {
#endif
#if I2C2_CMSIS
	uint32_t timeout = 0x7F;



	while (I2C2->SR2 & I2C_SR2_BUSY) ; // Тайм-аут занятости
	//	if ((timeout--) == 0) return ERROR;


	// Send start bit
    I2C2->CR1 |= I2C_CR1_START;
 //   GPIOD->ODR |= GPIO_ODR_OD13;

	while (!(I2C2->SR1 & I2C_SR1_SB))  // Тайм-аут начального бита
		if ((timeout--) == 0) return ERROR;

	// Send slave address (camera write address)
	I2C2->DR = (OV7670_WRITE_ADDR << 1);
	while (!(I2C2->SR1 & I2C_SR1_ADDR))  // Тайм-аут подчиненного адреса
        if ((timeout--) == 0) return ERROR;
    (void) I2C2->SR2;

	// Send register address
    I2C2->DR = reg_addr;
	while (!(I2C2->SR1 & I2C_SR1_TXE))  // Время ожидания регистрации
		if ((timeout--) == 0) return ERROR;
	
	// Send new register value
    I2C2->DR = data;
	while (!(I2C2->SR1 & I2C_SR1_TXE))  // Значение тайм-аута
		if ((timeout--) == 0) return ERROR;
    
	// Send stop bit
	I2C2->CR1 |= I2C_CR1_STOP;
    return SUCCESS;
#else
    GPIOD->ODR |= GPIO_ODR_OD13;
    HAL_I2C_Master_Transmit(&I2C_InitStructure, reg_addr, data, 1, HAL_TIMEOUT);
    GPIOD->ODR |= GPIO_ODR_OD14;
/*
    uint32_t timeout = 0x7FFFFF;

	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			//Serial_log("Busy Timeout\r\n");
			return true;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C2, ENABLE);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			//Serial_log("Start bit Timeout\r\n");
			return true;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C2, OV7670_WRITE_ADDR, I2C_Direction_Transmitter);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			//Serial_log("Slave address timeout\r\n");
			return true;
		}
	}

	// Send register address
	I2C_SendData(I2C2, reg_addr);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			//Serial_log("Register timeout\r\n");
			return true;
		}
	}

	// Send new register value
	I2C_SendData(I2C2, *data);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			//Serial_log("Value timeout\r\n");
			return true;
		}
	}

	// Send stop bit
	I2C_GenerateSTOP(I2C2, ENABLE);
	return false;
*/
    return true;
#endif
}

#define OV7670_INIT_CMSIS 1

#if OV7670_INIT_CMSIS
int OV7670_init() {
	int err = SUCCESS;

	// Configure camera registers
	for (uint8_t i = 0; i < OV7670_REG_NUM; i++) {
          //  GPIOD->ODR |= GPIO_ODR_OD13;

		err = SCCB_write_reg(OV7670_reg[i][0], OV7670_reg[i][1]);
		
        if (err == ERROR) break;
		Delay(0xFFFF);
	}

	return err;
#else
bool OV7670_init() {
   // bool OV7670_init(void) {
	uint8_t data, i = 0;
	bool err;

	// Configure camera registers
	for (i = 0; i < OV7670_REG_NUM; i++) {
		data = OV7670_reg[i][1];
		err = SCCB_write_reg(OV7670_reg[i][0], &data);
		//Serial_log("Writing register: ");
		//Serial_logi(i);
		//Serial_log("\r\n");

		if (err == true) {
			//Serial_log("Failed to update register\r\n");
			break;
		}

		Delay(0xFFFF);
	}

	return err;
#endif
}
//===========================================================================================================

#if 0
void DMA2_Stream1_IRQHandler(void) {
	// DMA complete
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) { // Transfer complete
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		frame_flag = true;
	} else if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TEIF1) != RESET) { // Transfer error
		// Not used, just for debug
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TEIF1);
	}
}

void DCMI_IRQHandler(void) {
	if (DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET) { // Frame received
		DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
		// After receiving a full frame we disable capture and the DMA transfer. This is probably a very inefficient way of capturing and sending frames
		// but it's the only way I've gotten to reliably work.
		DMA_Cmd(DMA2_Stream1, DISABLE);
		DCMI_Cmd(DISABLE);
		DCMI_CaptureCmd(DISABLE);
	}
	if (DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET) { // Overflow
		// Not used, just for debug
		DCMI_ClearFlag(DCMI_FLAG_OVFRI);
	}
	if (DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET) { // Error
		// Not used, just for debug
		DCMI_ClearFlag(DCMI_FLAG_ERRRI);
	}
}
#endif

#if 1
void DMA2_Stream1_IRQHandler(void) {
	if((DMA2->LISR & DMA_LISR_TCIF1) == DMA_LISR_TCIF1) {
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
	} else if ((DMA2->LISR & DMA_LISR_TEIF1) == DMA_LISR_TEIF1) {
        DMA2->LIFCR |= DMA_LIFCR_CTEIF1;
    }
}

void DCMI_IRQHandler(void) {
    if ((DCMI->RISR & DCMI_RIS_FRAME_RIS) == DCMI_RIS_FRAME_RIS) {
        //DCMI->ICR |= DCMI_ICR_FRAME_ISC;
        // disable DMA
    }
    if ((DCMI->RISR & DCMI_RIS_OVR_RIS) == DCMI_RIS_OVR_RIS) {
        //
    }
    if ((DCMI->RISR & DCMI_RIS_ERR_RIS) == DCMI_RIS_ERR_RIS) {

    }
}
#endif

void DCMI_init() {
#if 1
    /* GPIO */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // VSY | VSYNC
    GPIOB->MODER |= GPIO_MODER_MODER7_1;
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;
    GPIOB->AFR[0] |= (0xD << 28);

    // HRE | HSYNC
    GPIOA->MODER |= GPIO_MODER_MODER4_1;
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR4_0);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0;
    GPIOA->AFR[0] |= (0xD << 16);
    
    // PCLK
    GPIOA->MODER |= GPIO_MODER_MODER6_1;
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0);
    GPIOA->AFR[0] |= (0xD << 24);

    // D0
    GPIOA->MODER |= GPIO_MODER_MODER9_1;
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR9_0);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0;
    GPIOA->AFR[1] |= (0xD << 4);

    // D1
    GPIOA->MODER |= GPIO_MODER_MODER10_1;
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR10_0);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
    GPIOA->AFR[1] |= (0xD << 8);

    // D2
    GPIOC->MODER |= GPIO_MODER_MODER8_1;
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR8_0);
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR8_0;
    GPIOC->AFR[1] |= (0xD << 0);

    // D3
    GPIOC->MODER |= GPIO_MODER_MODER9_1;
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR9_0);
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0;
    GPIOC->AFR[1] |= (0xD << 4);

    // D4
    GPIOC->MODER |= GPIO_MODER_MODER11_1;
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR11_0);
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR11_0;
    GPIOC->AFR[1] |= (0xD << 12);

    // D5
    GPIOB->MODER |= GPIO_MODER_MODER6_1;
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;
    GPIOB->AFR[0] |= (0xD << 24);

    // D6
    GPIOB->MODER |= GPIO_MODER_MODER8_1;
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR8_0);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0;
    GPIOB->AFR[1] |= (0xD << 0);

    // D7
    GPIOB->MODER |= GPIO_MODER_MODER9_1;
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR9_0);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR9_0;
    GPIOB->AFR[1] |= (0xD << 4);

    /* DMA */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream1->CR |= (0x1 << 25);
    
	DMA2_Stream1->PAR |= (uint32_t) (&DCMI->DR);
	DMA2_Stream1->M0AR |= (uint32_t)&temp_buffer;
	DMA2_Stream1->NDTR = (IMG_ROWS * IMG_COLUMNS) / 2;

    // 3. Настройка
    // 3.1 DMA_SxCR_CIRC - Включаем круговой режим работы
    // 3.2 DMA_SxCR_MINC - Режим увеличения объема памяти
    // 3.3 DMA_SxCR_PSIZE_1 - Длина 32-bit
    // 3.4 DMA_SxCR_MSIZE_1 - Длина 32-bit
    // 3.5 DMA_SxCR_PL_1 - Высокий уровень приоритета, не обезательная скорее всего настройка, 
    //                     добавить если несколько будеь
    // 3.6 DMA_SxCR_TCIE - Прерывания
    DMA2_Stream1->CR = (/*DMA_SxCR_CIRC |*/ DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_TCIE | DMA_SxCR_TEIE);
    DMA2_Stream1->FCR |= (0x3 << 0 );
    // 4. Прерывания
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    NVIC_SetPriority(DMA2_Stream1_IRQn, 0);

    // 5. Запуск
	//DMA2_Stream1->CR |= DMA_SxCR_EN;

    /* DCMI */
 //   DCMI->IER |= (DCMI_IER_FRAME_IE | DCMI_IER_OVF_IE | DCMI_IER_ERR_IE);
   // NVIC_EnableIRQ(DCMI_IRQn);
  //  NVIC_SetPriority(DCMI_IRQn, 1);
    DCMI->CR |= (DCMI_CR_CM | DCMI_CR_VSPOL | DCMI_CR_HSPOL | DCMI_CR_PCKPOL | DCMI_CR_ENABLE);
#else
    GPIO_InitTypeDef GPIO_InitStructure;
	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// GPIO config

	// PA4 - HREF (HSYNC), PA6 - PCLK (PIXCLK)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;	//PA4 - HREF (HSYNC)
															//PA6 - PCLK (PIXCLK)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PB6 - D5, PB7 - VSYNC, PB8 - D6, PB9 - D7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PC6 - D0, PC7 - D1, PC8 - D2, PC9 - D3, PC11 - D4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI);

	// DCMI config
	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot; //DCMI_CaptureMode_SnapShot
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame; //DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_Init(&DCMI_InitStructure);
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
	DCMI_ITConfig(DCMI_IT_ERR, ENABLE);

	// DMA config
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) frame_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = IMG_ROWS * IMG_COLUMNS / 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TE, ENABLE);

	/* DMA2 IRQ channel Configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
#endif
}
//===========================================================================================================
#define GPIO_AF_MCO    0x00UL

void MCO1_init() {
#if 1
    //enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
    //PA8 -> MCO(AF) -> PLLCLK
    GPIOA->MODER |= GPIO_MODER_MODER8_1; //AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8; //PP
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; //PU
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; //High speed

    //AF0->MCO
    GPIOA->AFR[1] |= (GPIO_AF_MCO << 0);
    RCC->CFGR &= ~RCC_CFGR_MCO1; //HSI
#else
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		//PA8 - XCLK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	// MCO clock source
//	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4); // Using the fast PLL clock results in garbage output, using HSI (at 16Mhz works fine)
	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);
#endif
}

void TIM4_IRQHandler(void) {
	TIM4->SR &= ~TIM_SR_UIF;
	TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void PWM_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (0x2 << (2 * 12));
    GPIOD->AFR[1] |= (0x2 << 16);
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 2;
    TIM4->ARR = 2;
    TIM4->CCMR1 |= 0x60;
    TIM4->CCR1 = 2;
    TIM4->CCER |= 0x1;
    TIM4->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, 2);
    TIM4->CR1 |= TIM_CR1_CEN;
}
//===========================================================================================================
void dumpFrame() {
#if 0
	// Enable capture and DMA after we have sent the photo. This is a workaround for the timing issues I've been having where
	// the DMA transfer is not in sync with the frames being sent
	DMA2_Stream1->NDTR = (IMG_ROWS * IMG_COLUMNS) / 2;
	DMA2_Stream1->CR |= DMA_SxCR_EN;
#else

	uint8_t *buffer = (uint8_t *) frame_buffer;
	int length = IMG_ROWS * IMG_COLUMNS * 2;
	// Copy every other byte from the main frame buffer to our temporary buffer (this converts the image to grey scale)
	int i;
	for (i = 1; i < length; i += 2) {
		temp_buffer[i / 2] = buffer[i];
	}
	// We only send the sync frame if it has been requested
	if (send_sync_frame) {
		for (i = 0x7f; i > 0; i--) {
			uint8_t val = i;
			//Serial_sendb(&val);
		}
		send_sync_frame = false;
	}

	for (i = 0; i < (length / 2); i++) {
		if (i > 100) {
		//	Serial_sendb(&temp_buffer[i]);
		} else {
			uint8_t val = 0xff;
		//	Serial_sendb(&val); // Change first 100 pixels to white to provide a reference for where the frame starts
		}
	}
	// Enable capture and DMA after we have sent the photo. This is a workaround for the timing issues I've been having where
	// the DMA transfer is not in sync with the frames being sent
	//DMA_Cmd(DMA2_Stream1, ENABLE);
	//DCMI_Cmd(ENABLE);
	//DCMI_CaptureCmd(ENABLE);
#endif
}

void my_GPIO_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
    GPIOD->ODR &= ~GPIO_ODR_OD13;
    GPIOD->ODR &= ~GPIO_ODR_OD14;
    GPIOD->ODR &= ~GPIO_ODR_OD15;
}

#if 0
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

//#if defined(DATA_IN_ExtSRAM) || defined(DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl(); 
//#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */
         
  /* Configure the System clock source, PLL Multiplier and Divider factors, 
     AHB/APBx prescalers and Flash settings ----------------------------------*/
//  SetSysClock();

  /* Configure the Vector Table location add offset address ------------------*/
//#ifdef VECT_TAB_SRAM
  //SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
//#else
  //SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
//#endif
}
#endif

int main(void) {
#if 
    GPIO_init();
   // GPIOD->ODR |= GPIO_ODR_OD13;
    I2C2_init();
   // GPIOD->ODR |= GPIO_ODR_OD14;
  //  PWM_init();
  //  GPIOD->ODR |= GPIO_ODR_OD15;
    //MCO1_init();
//	OV7670_init();
	DCMI_init();

   // GPIOD->ODR |= GPIO_ODR_OD13;
    int err;
    err = OV7670_init();
    //GPIOD->ODR |= GPIO_ODR_OD14;
    if (err == ERROR) {
      //  GPIOD->ODR |= GPIO_ODR_OD15;
        while (1) {}
    }

   // GPIOD->ODR |= GPIO_ODR_OD15;
	while(1) {
        if (frame_flag == SUCCESS) {
            //Delay(0xFFF);
			frame_flag = ERROR;
			dumpFrame();
		}
	}
#else
    bool err;

	//SystemInit();
        my_GPIO_init();
	MCO1_init();

	I2C2_init();

    DCMI_init();

    //Serial_init();
      //  GPIOD->ODR |= GPIO_ODR_OD13;

	// Initialize camera over SCCB
	err = OV7670_init();
		GPIOD->ODR |= GPIO_ODR_OD14;

	if (err == true) {
		GPIOD->ODR |= GPIO_ODR_OD14;
        //Serial_log("Failed to initialize\r\n");
		while (1) {
		}
	}

    GPIOD->ODR |= GPIO_ODR_OD15;
	// Infinite program loop
	while (1) {
		if (frame_flag == true) {
			frame_flag = false;
			dumpFrame();
		}
	}
#endif
    
}
#else
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_12
#define I2C2_SDA_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

#define FRAME_BUFFER 0xC0000000
/* USER CODE END Private defines */


#define OV7670_MIDH 0x1c /* Manufacturer ID Byte - High */
#define OV7670_MIDL 0x1D /* Manufacturer ID Byte - LOW */
#define OV7670_PID 0x0A /* Product ID Number MSB */
#define OV7670_VER 0x0B /* Product ID Number LSB */
#define OV7670_REG_NUM 122
#define DevAddress_write 0x42
#define DevAddress_read 0x43


void OV7670_Init(void);
void OV7670_RESET(void);

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QVGA 10//320*240

/* HandleTypedef */

I2C_HandleTypeDef i2c;
DCMI_HandleTypeDef OV7670_hdcmi_eval;
//void NMI_Handler(void);
//void HardFault_Handler(void);
//void MemManage_Handler(void);
//void BusFault_Handler(void);
//void UsageFault_Handler(void);
//void SVC_Handler(void);
//void DebugMon_Handler(void);
//void PendSV_Handler(void);
//void SysTick_Handler(void);
//void DMA1_Stream5_IRQHandler(void);
//void DMA1_Stream6_IRQHandler(void);
//void DMA2_Stream1_IRQHandler(void);
//void DCMI_IRQHandler(void);


/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dcmi;

extern DMA_HandleTypeDef hdma_usart2_tx;

extern DMA_HandleTypeDef hdma_usart2_rx;

/**
* @brief DCMI MSP Initialization
* This function configures the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DCMI_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**DCMI GPIO Configuration
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCLK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PC8     ------> DCMI_D2
    PC9     ------> DCMI_D3
    PC11     ------> DCMI_D4
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PB8     ------> DCMI_D6
    PB9     ------> DCMI_D7
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* DCMI DMA Init */
    /* DCMI Init */
    hdma_dcmi.Instance = DMA2_Stream1;
    hdma_dcmi.Init.Channel = DMA_CHANNEL_1;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_CIRCULAR;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_dcmi.Init.MemBurst = DMA_MBURST_INC4;
    hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    {
     // Error_Handler();
    }

    __HAL_LINKDMA(hdcmi,DMA_Handle,hdma_dcmi);

    /* DCMI interrupt Init */
    HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
  /* USER CODE BEGIN DCMI_MspInit 1 */

  /* USER CODE END DCMI_MspInit 1 */
  }

}

/**
* @brief DCMI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_CLK_DISABLE();

    /**DCMI GPIO Configuration
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCLK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PC8     ------> DCMI_D2
    PC9     ------> DCMI_D3
    PC11     ------> DCMI_D4
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PB8     ------> DCMI_D6
    PB9     ------> DCMI_D7
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);

    /* DCMI DMA DeInit */
    HAL_DMA_DeInit(hdcmi->DMA_Handle);

    /* DCMI interrupt DeInit */
    HAL_NVIC_DisableIRQ(DCMI_IRQn);
  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }

}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PC12     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = I2C2_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PC12     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin);

    HAL_GPIO_DeInit(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }

}

const uint8_t OV7670_reg[OV7670_REG_NUM][2] =
{
	/*
	 * Reset register
	 *
	 * */
    { 0x12, 0x80 }, /* COM7:
                           Bit[7]: SCCB register reset
                               0: No change
                               1: Reset all registers to default values
                           Bit[6]: Reserved
                           Bit[5]: Output format - CIF selection
                           Bit[4]: Output format - QVGA selection
                           Bit[3]: Output format - QCIF selection
                           Bit[2]: Output format - RGB selection(see below)
                           Bit[1]: Color bar
                               0: Disable
                               1: Enable
                           Bit[0]: Output format - Raw RGB(see below)

                           COM7[2] COM7[0]
                     YUV        0       0
                     RGB        1       0
               Bayer RAW        0       1
     Processed Bayer RAW        1       1 */

	/*
	 * Image format
	 *
	 * */
	{ 0x12, 0x14}, /* 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV, 0xc = QCIF (RGB) */
	{ 0x40, 0xd0},		/* COM15:
	                           Bit[7:6]: Data formatRGB565*/
	{ 0xb0, 0x84},		//Color mode


	// Hardware window
		{0x11, 0x01},		//PCLK settings, 15fps
		/* CLKRC: Internal clock
			                            Bit[7]: Reserved
			                            Bit[6]: Use external clock directly
			                            Bit[5:0]: Internal clock pre-scaler */
		{0x32, 0x80},		//HREF
		{0x17, 0x17},		//HSTART
		{0x18, 0x05},		//HSTOP
		{0x03, 0x0a},		//VREF
		{0x19, 0x02},		//VSTART
		{0x1a, 0x7a},		//VSTOP
	// Scalling numbers
		{0x70, 0x3a},		//X_SCALING
		{0x71, 0x35},		//Y_SCALING
		{0x72, 0x11},		//DCW_SCALING
		{0x73, 0xf0},		//PCLK_DIV_SCALING
		{0xa2, 0x02},		//PCLK_DELAY_SCALING

		// Matrix coefficients
		{0x4f, 0x80},		{0x50, 0x80},
		{0x51, 0x00},		{0x52, 0x22},
		{0x53, 0x5e},		{0x54, 0x80},
		{0x58, 0x9e},

		// Gamma curve values
		{0x7a, 0x20},		{0x7b, 0x10},
		{0x7c, 0x1e},		{0x7d, 0x35},
		{0x7e, 0x5a},		{0x7f, 0x69},
		{0x80, 0x76},		{0x81, 0x80},
		{0x82, 0x88},		{0x83, 0x8f},
		{0x84, 0x96},		{0x85, 0xa3},
		{0x86, 0xaf},		{0x87, 0xc4},
		{0x88, 0xd7},		{0x89, 0xe8},

		// AGC and AEC parameters
		{0xa5, 0x05},		{0xab, 0x07},
		{0x24, 0x95},		{0x25, 0x33},
		{0x26, 0xe3},		{0x9f, 0x78},
		{0xa0, 0x68},		{0xa1, 0x03},
		{0xa6, 0xd8},		{0xa7, 0xd8},
		{0xa8, 0xf0},		{0xa9, 0x90},
		{0xaa, 0x94},		{0x10, 0x00},

		// AWB parameters
		{0x43, 0x0a},		{0x44, 0xf0},
		{0x45, 0x34},		{0x46, 0x58},
		{0x47, 0x28},		{0x48, 0x3a},
		{0x59, 0x88},		{0x5a, 0x88},
		{0x5b, 0x44},		{0x5c, 0x67},
		{0x5d, 0x49},		{0x5e, 0x0e},
		{0x6c, 0x0a},		{0x6d, 0x55},
		{0x6e, 0x11},		{0x6f, 0x9f},
		{0x6a, 0x40},		{0x01, 0x40},
		{0x02, 0x60},		{0x13, 0xe7},

		// Additional parameters
		{0x34, 0x11},		{0x3f, 0x00},
		{0x75, 0x05},		{0x76, 0xe1},
		{0x4c, 0x00},		{0x77, 0x01},
		{0xb8, 0x0a},		{0x41, 0x18},
		{0x3b, 0x12},		{0xa4, 0x88},
		{0x96, 0x00},		{0x97, 0x30},
		{0x98, 0x20},		{0x99, 0x30},
		{0x9a, 0x84},		{0x9b, 0x29},
		{0x9c, 0x03},		{0x9d, 0x4c},
		{0x9e, 0x3f},		{0x78, 0x04},
		{0x0e, 0x61},		{0x0f, 0x4b},
		{0x16, 0x02},		{0x1e, 0x00},
		{0x21, 0x02},		{0x22, 0x91},
		{0x29, 0x07},		{0x33, 0x0b},
		{0x35, 0x0b},		{0x37, 0x1d},
		{0x38, 0x71},		{0x39, 0x2a},
		{0x3c, 0x78},		{0x4d, 0x40},
		{0x4e, 0x20},		{0x69, 0x00},
		{0x6b, 0x3a},		{0x74, 0x10},
		{0x8d, 0x4f},		{0x8e, 0x00},
		{0x8f, 0x00},		{0x90, 0x00},
		{0x91, 0x00},		{0x96, 0x00},
		{0x9a, 0x00},		{0xb1, 0x0c},
		{0xb2, 0x0e},		{0xb3, 0x82},
		{0x4b, 0x01}

};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* Image size */
static uint32_t frameBuffer[QVGA];
uint32_t picSize = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint16_t DevAddress_write = 0x42;
//	uint16_t DevAddress_read = 0x43;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
 // SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  /* USER CODE BEGIN 2 */
  OV7670_Init();
  HAL_Delay(1000);  //Delay for the camera to output correct data
 // Im_size =  0x9600;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* I2C code (SCCB) */
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET){
	//	__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
	//	memset((void *)frameBuffer, 0, sizeof(frameBuffer));// reset buffer
	//	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) FRAME_BUFFER, QVGA);// Start DCMI, will start DMA interrupt inside this function.
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1/*PWR_REGULATOR_VOLTAGE_SCALE3*/);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  //RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
   /// Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
  //  Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */
  __HAL_RCC_DCMI_CLK_ENABLE();

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
 // hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
 // hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
 // hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
 // hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
  //  Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    //Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void OV7670_Init(void)
{
	OV7670_RESET();
	HAL_Delay(1000);
    uint8_t *data;
    HAL_StatusTypeDef result;
    data = malloc(sizeof(uint8_t));
    if (!data)
    	return;
    for (int i = 0; i < OV7670_REG_NUM; i++){
    	*data = OV7670_reg[i][1];
    	result = HAL_I2C_Master_Transmit(&i2c, DevAddress_write, data, 2, HAL_MAX_DELAY); /* Write regiter address */
    	if (result == HAL_ERROR)
    		return;
    }
    free(data);

}

void OV7670_RESET(void)
{

}
#endif
