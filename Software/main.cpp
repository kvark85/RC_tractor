/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "peripheral.h"
#include "rfm70.h"
#include "rfm70-config.h"

extern int32_t msVar; // ms counter
uint8_t stopTimer = 0;
bool isTransmit = false;
uint8_t rfm70buf[3];
uint8_t statusReg;
uint8_t controlMode = 1;

//uint32_t clockFreq; // for determinating clock frequency

int main()
{
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); // select Clock = 16 MHz
  CLK_HSICmd(ENABLE);
  //clockFreq = CLK_GetClockFreq(); // getting clock frequency
  
  asm("rim"); 
  TIM4_Config(); // this timer for ms delay

  waitMs(10);
  SPI_Init_RFM70();
  rfm70_init();
  rfm70_channel(0);
  rfm70_retransmit_delay_attempts(0,15);
  
  if(isTransmit) {
    LED_Init();
    buttonInit();
    ADC_Init();
    rfm70_mode_transmit();
    
    while(1) {
      if(isButtonPressed()) {  
        controlMode ^= 1;
      }

      INVERT_LED();
      rfm70buf[0] = getADC3()/4; // speed;
      rfm70buf[1] = getADC4()/4; // direction;
      rfm70buf[2] = controlMode;
  
      rfm70_transmit_message(rfm70buf, 3);
      while (1){
        statusReg = rfm70_register_read( RFM70_REG_STATUS );
        if (statusReg & 0x30) { // 0b0011 0000
          rfm70_register_write( RFM70_REG_STATUS ,statusReg );
          break;
        } 
      }   
      waitMs(5);
    }
  } else {
    L293D_GpioInit();
    TIM2_PWM_Init();
    rfm70_mode_receive();

    while(1) {
      //rfm70_receive( &pipe, rfm70buf, &length );
      if(!GPIO_ReadInputPin(PORT_SPI_IRQ_CE, PIN_IRQ)) {
        rfm70_buffer_read( RFM70_CMD_R_RX_PAYLOAD, rfm70buf, 3 );
        rfm70_register_write( RFM70_REG_STATUS ,  0x42); // 0b0100 0010 reset interrupt pin
        motorHeandler(rfm70buf[0], rfm70buf[1], rfm70buf[2]);
        stopTimer = 0; // обнуляем счетчик остановки
      } else {
        stopTimer++;
        waitMs(1);
        if (stopTimer > 250) {
          uint8_t readBuf[32];
          rfm70_buffer_read(RFM70_CMD_R_RX_PAYLOAD, readBuf, 32);
          rfm70_register_write(RFM70_REG_STATUS,  0x42); // 0b0100 0010 reset interrupt pin
          stopTimer = 0; // обнуляем счетчик остановки
          rfm70buf[0] = MIDDLEADC;
          rfm70buf[1] = MIDDLEADC;
          setMotorLeftPwm(0);
          setMotorRightPwm(0) ;
        }
      }
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) { while (1) {}; };
#endif
