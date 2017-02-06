/*
                        ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ
                               STM8F103				
                              12.07.2013
                                    */

#include "peripheral.h"

uint32_t msVar = 0;
double a = 2 * MAX_PWM;
double b = 255 * MAX_PWM;

//******************************************************************************
//-------------------------     led initialization     -------------------------
//******************************************************************************
void LED_Init(void) {
  GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
}

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
//------------------------------     ms delay     ------------------------------
//******************************************************************************
void waitMs(uint32_t val) {
  msVar = val;
  while(msVar);
}

//******************************************************************************
//---------------------     SPI and RFM70 initialization    --------------------
//******************************************************************************
void SPI_Init_RFM70(void) {
    GPIO_Init (PORT_NSS,        PIN_NSS,  GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем CS (SS)
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_CE,   GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем CE
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_SCK,  GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем CLK
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_MOSI, GPIO_MODE_OUT_PP_HIGH_FAST);          // конфигурируем MOSI
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_MISO, GPIO_MODE_IN_FL_NO_IT);               // конфигурируем MISO
    GPIO_Init (PORT_SPI_IRQ_CE, PIN_IRQ, GPIO_MODE_IN_FL_NO_IT);                // вывод прерывания радиомодуля
   

    GPIO_WriteHigh(PORT_SPI_IRQ_CE, PIN_CE);
    GPIO_WriteHigh(PORT_SPI_IRQ_CE, PIN_NSS);   
  
    SPI_DeInit();
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, 
            SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE, 
            SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
    SPI_Cmd(ENABLE);
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

//******************************************************************************
// ---------------------------     motor control     ---------------------------
//******************************************************************************
void motorHeandler(uint8_t adcSpeed, uint8_t adcDirection, uint8_t mode) {
  float centerSpeed, direction;
  float motorLeftSpeed, motorRightSpeed;
  //float transformationValue = 6;
  float transformationValue = 10.66;
  uint32_t maxValuePwm = (uint32_t)(MAX_PWM * transformationValue);

  centerSpeed = ((a * (float)adcSpeed) - b) / 255; // cpnvertion to (-MAX_PWM)-MAX_PWM ((-50)-50)
  direction = ((a * (float)adcDirection) - b) / 255;

  centerSpeed *= transformationValue;

  if(mode == 1) {
    direction *= transformationValue;
   
    motorLeftSpeed = centerSpeed + direction;
    motorRightSpeed = centerSpeed - direction;
  } else {
    motorLeftSpeed = centerSpeed;
    motorRightSpeed = centerSpeed;
    
    if (direction > 0) {
      motorRightSpeed *= (MAX_PWM - direction) * 0.02;
    } else {
      motorLeftSpeed *= (MAX_PWM - ((-1) * direction)) * 0.02;
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
  
  if (motorLeftSpeed > maxValuePwm) motorLeftSpeed = maxValuePwm;
  if (motorRightSpeed > maxValuePwm) motorRightSpeed = maxValuePwm;

  setMotorLeftPwm(motorLeftSpeed);
  setMotorRightPwm(motorRightSpeed);
}