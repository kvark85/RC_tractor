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

//*****************************  пины LED START  *****************************//
#define LED_GPIO_PORT           (GPIOD)
#define LED_GPIO_PINS           (GPIO_PIN_4)
#define INVERT_LED()            GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS)
//******************************  пины LED END  ******************************//


//************************  defines from motor START  ************************//
#define maxPWM                  533
#define rightDirection          GPIO_PIN_4      // пины управления направлением двигателя
#define leftDirection           GPIO_PIN_5
#define portL293D               GPIOD           // порт управления направлением двигателя

#define motorLeftForward()      GPIO_WriteHigh(portL293D, (GPIO_Pin_TypeDef)leftDirection)
#define motorLeftBack()         GPIO_WriteLow(portL293D, (GPIO_Pin_TypeDef)leftDirection)
#define motorRightForward()     GPIO_WriteHigh(portL293D, (GPIO_Pin_TypeDef)rightDirection)
#define motorRightBack()        GPIO_WriteLow(portL293D, (GPIO_Pin_TypeDef)rightDirection)

#define setMotorLeftPwm( x )    TIM2_SetCompare2(maxPWM - (int32_t)x)
#define setMotorRightPwm( x )   TIM2_SetCompare3(maxPWM - (int32_t)x)
//*************************  defines from motor END  *************************//


//***************  пины STM8 которые подключены к RFM70 START  ***************//
#define pin_NSS                 GPIO_PIN_3      // со стороны RFM70 подключено к CSN
#define port_NSS                GPIOA           // порт NSS
#define pin_CE                  GPIO_PIN_3
#define pin_IRQ                 GPIO_PIN_4
#define pin_SCK                 GPIO_PIN_5
#define pin_MOSI                GPIO_PIN_6
#define pin_MISO                GPIO_PIN_7
#define port_SPI_IRQ_CE         GPIOC           // SPI порт
//****************  пины STM8 которые подключены к RFM70 END  ****************//

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