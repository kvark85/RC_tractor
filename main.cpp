/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "peripheral.h"
#include "rfm70.h"
#include "rfm70-config.h"

extern int32_t msVar; // мС счетчик
int32_t stopTimer = 0;
bool isTransmit = false;
uint8_t rfm70buf[32];
uint8_t statusReg;

uint8_t a1, a2, a3;

uint32_t clockFreq; // для определения тактовой частоты

int main()
{
  //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2); // select Clock = 8 MHz
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); // select Clock = 16 MHz
  CLK_HSICmd(ENABLE);
  //clockFreq = CLK_GetClockFreq(); // определение тактовой частоты
  
  asm("rim"); 
  TIM4_Config(); // на этом таймере таймер в мС
  waitMs(10);
  SPI_Init_RFM70();
  rfm70_init();
  rfm70_channel(0);
  rfm70_retransmit_delay_attempts(0,15);
  
  if(isTransmit) {
    LED_Init();
    ADC_Init();
    rfm70_mode_transmit();
  } else {
    rfm70_mode_receive();
    TIM2_PWM_Init();
    L293D_GpioInit();
  }
  
  while(1) {
    if(isTransmit) {
      INVERT_LED();
      rfm70buf[0] = getADC3()/4; // speed;
      rfm70buf[1] = getADC4()/4; // direction;
      rfm70buf[2] += 1;
  
      rfm70_transmit_message(rfm70buf, 3);
      while (1){
        statusReg = rfm70_register_read( RFM70_REG_STATUS );
        if (statusReg & 0x30) { // 0b0011 0000
          rfm70_register_write( RFM70_REG_STATUS ,statusReg );
          break;
        } 
      }   
      waitMs(5);
    } else {
      //rfm70_receive( &pipe, rfm70buf, &length );
      if(!GPIO_ReadInputPin(port_SPI_IRQ_CE, pin_IRQ)) {
        rfm70_buffer_read( RFM70_CMD_R_RX_PAYLOAD, rfm70buf, 3 );
        rfm70_register_write( RFM70_REG_STATUS ,  0x42);
        a1 = rfm70buf[0];
        a2 = rfm70buf[1];
        a3 = rfm70buf[2];
        motorHeandler2(rfm70buf[0], rfm70buf[1]);
        stopTimer = 0; // обнуляем счетчик остановки
      } else {
        stopTimer++;
        waitMs(1);
        if (stopTimer > 1000) {
          stopTimer = 0; // обнуляем счетчик остановки
          rfm70buf[0] = MIDDLEADC;
          rfm70buf[1] = MIDDLEADC;
          motorHeandler2(rfm70buf[0], rfm70buf[1]);
        }
      }
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) { while (1) {}; };
#endif
