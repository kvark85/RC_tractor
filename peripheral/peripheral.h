/*
                        ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ
                               STM8F103
                              12.07.2013
                                    */

#ifndef _peripheral_
#define _peripheral_

#include "stm8s.h"   

#define TIM4_PERIOD             124
#define MIDDLEADC               127

//***************************************************************************//
//---------------------------     BUTTON START     --------------------------//
//***************************************************************************//
#define BUTTON_GPIO_PORT        GPIOD
#define BUTTON_GPIO_PINS        GPIO_PIN_1

//***************************************************************************//
//-----------------------------     LED START     ---------------------------//
//***************************************************************************//
#define LED_GPIO_PORT           GPIOD
#define LED_GPIO_PINS           GPIO_PIN_4
#define INVERT_LED()            GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS)

//***************************************************************************//
//---------------------     defines for motor START     ---------------------//
//***************************************************************************//
#define MAX_PWM                 533
#define RIGHT_DIRECTION         GPIO_PIN_4
#define LEFT_DIRECTION          GPIO_PIN_5
#define DIRECTION_PORT          GPIOD

#define motorLeftForward()      GPIO_WriteHigh(DIRECTION_PORT, (GPIO_Pin_TypeDef)LEFT_DIRECTION)
#define motorLeftBack()         GPIO_WriteLow(DIRECTION_PORT, (GPIO_Pin_TypeDef)LEFT_DIRECTION)
#define motorRightForward()     GPIO_WriteHigh(DIRECTION_PORT, (GPIO_Pin_TypeDef)RIGHT_DIRECTION)
#define motorRightBack()        GPIO_WriteLow(DIRECTION_PORT, (GPIO_Pin_TypeDef)RIGHT_DIRECTION)

#define setMotorLeftPwm( x )    TIM2_SetCompare2(MAX_PWM - (int32_t)x)
#define setMotorRightPwm( x )   TIM2_SetCompare3(MAX_PWM - (int32_t)x)

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
void LED_Init(void);
void ADC_Init(void);
uint16_t getADC3(void);
uint16_t getADC4(void);
void TIM4_Config(void);
void waitMs(uint32_t);
void SPI_Init_RFM70(void);
void TIM2_PWM_Init(void);
void L293D_GpioInit(void);
void motorHeandler1(uint8_t motor1, uint8_t motor2);
void motorHeandler2(uint8_t motor1, uint8_t motor2);

#endif