/*
                        ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ
                               STM8F103
                              12.07.2013
                                    */

#ifndef _peripheral_
#define _peripheral_

#include "stm8s.h"

//#define IS_TRANSMIT
#define TIM4_PERIOD             124
#define MIDDLEADC               127

//***************************************************************************//
//---------------------------     BUTTON START     --------------------------//
//***************************************************************************//
#define BUTTON_GPIO_PORT        GPIOB
#define BUTTON_GPIO_PINS        GPIO_PIN_5

//***************************************************************************//
//---------------------     defines for motor START     ---------------------//
//***************************************************************************//
#define PERIOD_PWM              533
#define MAX_TRANSMITTER_ADC     127
#define RIGHT_DIRECTION         GPIO_PIN_4
#define LEFT_DIRECTION          GPIO_PIN_5
#define DIRECTION_PORT          GPIOD

#define motorLeftForward()      GPIO_WriteHigh(DIRECTION_PORT, (GPIO_Pin_TypeDef)LEFT_DIRECTION)
#define motorLeftBack()         GPIO_WriteLow(DIRECTION_PORT, (GPIO_Pin_TypeDef)LEFT_DIRECTION)
#define motorRightForward()     GPIO_WriteHigh(DIRECTION_PORT, (GPIO_Pin_TypeDef)RIGHT_DIRECTION)
#define motorRightBack()        GPIO_WriteLow(DIRECTION_PORT, (GPIO_Pin_TypeDef)RIGHT_DIRECTION)

#define setMotorLeftPwm(x)    TIM2_SetCompare2(PERIOD_PWM - (int32_t)x)
#define setMotorRightPwm(x)   TIM2_SetCompare3(PERIOD_PWM - (int32_t)x)

//***************************************************************************//
//---------------------     defines for RFM70 START     ---------------------//
//***************************************************************************//
#define PIN_NSS                 GPIO_PIN_3      // from RFM70 side - connected to CSN
#define PORT_NSS                GPIOA           // NSS port
#define PIN_CE                  GPIO_PIN_3
#define PIN_IRQ                 GPIO_PIN_4
#define PIN_SCK                 GPIO_PIN_5
#define PIN_MOSI                GPIO_PIN_6
#define PIN_MISO                GPIO_PIN_7
#define PORT_SPI_IRQ_CE         GPIOC           // SPI port

//***************************************************************************//
//-------------------------     function defines     ------------------------//
//***************************************************************************//
void buttonInit(void);

bool isButtonPressed(void);

void ADC_Init(void);

uint16_t getADC3(void);

uint16_t getADC4(void);

void TIM4_Config(void);

void waitUs(void);

void waitMs(uint8_t);

void SPI_Init_RFM73(void);

void TIM2_PWM_Init(void);

void L293D_GpioInit(void);

void motorHeandler(uint8_t, uint8_t, uint8_t);

#endif