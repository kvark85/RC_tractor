/*
                        ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ
                               STM8F103				
                              12.07.2013
                                    */

#include "peripheral.h"

uint8_t msVar = 0;
float transformationValue = PERIOD_PWM/MAX_TRANSMITTER_ADC;

//******************************************************************************
//------------------------     button initialization     -----------------------
//******************************************************************************
void buttonInit(void) {
  GPIO_Init(BUTTON_GPIO_PORT, (GPIO_Pin_TypeDef)BUTTON_GPIO_PINS, GPIO_MODE_IN_FL_NO_IT);
}

//******************************************************************************
//------------------------     press button handler     ------------------------
//******************************************************************************
bool isButtonPressed(void) {
  if(GPIO_ReadInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PINS) == RESET) {
    for(uint8_t i = 0; i < 250; i++) {
      if(GPIO_ReadInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PINS) == SET) {
      return false;
      }
    }
    while (GPIO_ReadInputPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PINS) == RESET);
    return true;
  }
  return false;
}

//******************************************************************************
//-------------------------     ADC initialization     -------------------------
//******************************************************************************
void ADC_Init(void){
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
  
  ADC1_DeInit();
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_4, ADC1_PRESSEL_FCPU_D2, \
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL4, \
            DISABLE);
  ADC1_ScanModeCmd(ENABLE); 
  ADC1_StartConversion();
}

//******************************************************************************
//---------------------------     get ADC3 value     ---------------------------
//******************************************************************************
uint16_t getADC3(void) {
    uint16_t temph_ADC = 0;
    uint8_t templ_ADC = 0;
    
    if ((ADC1->CR2 & ADC1_CR2_ALIGN) != 0) {
        templ_ADC = ADC1->DB3RL;
        temph_ADC = ADC1->DB3RH;
        temph_ADC = (uint16_t)(templ_ADC | (uint16_t)(temph_ADC << (uint8_t)8));
    }else {
        temph_ADC = ADC1->DB3RL;
        templ_ADC = ADC1->DB3RH;
        temph_ADC = (uint16_t)((uint16_t)((uint16_t)templ_ADC << 6) | (uint16_t)((uint16_t)temph_ADC << 8));

    }
    return (uint16_t)temph_ADC;
}

//******************************************************************************
//---------------------------     get ADC4 value     ---------------------------
//******************************************************************************
uint16_t getADC4(void) {
    uint16_t temph_ADC = 0;
    uint8_t templ_ADC = 0;
    
    if ((ADC1->CR2 & ADC1_CR2_ALIGN) != 0) {
        templ_ADC = ADC1->DB4RL;
        temph_ADC = ADC1->DB4RH;
        temph_ADC = (uint16_t)(templ_ADC | (uint16_t)(temph_ADC << (uint8_t)8));
    }else {
        temph_ADC = ADC1->DB4RL;
        templ_ADC = ADC1->DB4RH;
        temph_ADC = (uint16_t)((uint16_t)((uint16_t)templ_ADC << 6) | (uint16_t)((uint16_t)temph_ADC << 8));

    }
    return (uint16_t)temph_ADC;
}

//******************************************************************************
//-------------------------     TIM4 configuration     -------------------------
//******************************************************************************
void TIM4_Config(void)
{ /*
  - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
  - We need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

//******************************************************************************
//------------------------------     us delay     ------------------------------
//******************************************************************************
void waitUs(void) {}

//******************************************************************************
//------------------------------     ms delay     ------------------------------
//******************************************************************************
void waitMs(uint8_t val) {
  msVar = val;
  while(msVar);
}

//******************************************************************************
//---------------------     SPI and RFM70 initialization    --------------------
//******************************************************************************
void SPI_Init_RFM73(void) {
    GPIO_Init (PORT_NSS,        PIN_NSS,  GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем CS (SS)
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_CE,   GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем CE
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_SCK,  GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем CLK
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_MOSI, GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем MOSI
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_MISO, GPIO_MODE_IN_FL_NO_IT);               // конфигурируем MISO
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_IRQ, GPIO_MODE_IN_FL_NO_IT);                // вывод прерывания радиомодуля
}

//******************************************************************************
//--------------------     PWM initialization (on TIM2)     --------------------
//******************************************************************************
void TIM2_PWM_Init(void){
    TIM2_DeInit();
    /* Time base configuration */
    TIM2_TimeBaseInit(TIM2_PRESCALER_1, PERIOD_PWM);

      /* PWM1 Mode configuration: Channel2 */ 
    TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, PERIOD_PWM, TIM2_OCPOLARITY_HIGH);
    TIM2_OC2PreloadConfig(ENABLE);

    /* PWM1 Mode configuration: Channel3 */  
    TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, PERIOD_PWM, TIM2_OCPOLARITY_HIGH);
    TIM2_OC3PreloadConfig(ENABLE);
    //TIM2_CCxCmd(TIM2_CHANNEL_3, ENABLE);
    
    TIM2_ARRPreloadConfig(ENABLE);

    /* TIM2 enable counter */
    TIM2_Cmd(ENABLE);
}

//******************************************************************************
//------------------------     L293D initialization     ------------------------
//******************************************************************************
void L293D_GpioInit(void) {
  GPIO_Init(DIRECTION_PORT, (GPIO_Pin_TypeDef)RIGHT_DIRECTION, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(DIRECTION_PORT, (GPIO_Pin_TypeDef)LEFT_DIRECTION, GPIO_MODE_OUT_PP_LOW_FAST);
}

/**
 * Motor control.
 * Speed parameter from transmitter:
 * 0 => speed = -max;
 * 127  => speed = 0;
 * 255 => speed = max.
 * Direction parameter from remote control:
 * 0 => direction = left;
 * 127  => direction = center;
 * 255 => direction = right.
 * @param {uint8_t} adcSpeed
 * @param {uint8_t} adcDirection
 * @param {uint8_t} mode - Tractor (0), simple car1 (1).
 */

void motorHeandler(uint8_t adcSpeed, uint8_t adcDirection, uint8_t mode) {
  float centerSpeed, direction, motorLeftSpeed, motorRightSpeed;

  centerSpeed = adcSpeed - MAX_TRANSMITTER_ADC;
  direction = adcDirection - MAX_TRANSMITTER_ADC;

  centerSpeed *= transformationValue;

  if(mode == 0) {
  // tractor
    direction *= transformationValue;

    motorLeftSpeed = centerSpeed + direction;
    motorRightSpeed = centerSpeed - direction;
  } else {
    // simple car
    motorLeftSpeed = centerSpeed;
    motorRightSpeed = centerSpeed;
    
    if (direction > 0) {
      float attenuationCoefficient = 1 - (direction / MAX_TRANSMITTER_ADC);
      motorRightSpeed *= attenuationCoefficient;
    } else {
      float attenuationCoefficient = 1 + (direction / MAX_TRANSMITTER_ADC);
      motorLeftSpeed *= attenuationCoefficient;
    }
  }
    
  if(motorLeftSpeed > 0) {
    motorLeftForward();
  } else {
    motorLeftBack();
    motorLeftSpeed *= -1;
  }
   if(motorRightSpeed > 0) {
    motorRightForward();
  } else {
    motorRightBack();
    motorRightSpeed *= -1;
  }
  
  if (motorLeftSpeed > PERIOD_PWM) motorLeftSpeed = PERIOD_PWM;
  if (motorRightSpeed > PERIOD_PWM) motorRightSpeed = PERIOD_PWM;

  setMotorLeftPwm(motorLeftSpeed);
  setMotorRightPwm(motorRightSpeed);
}