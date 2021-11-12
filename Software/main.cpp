/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "peripheral.h"
#include "rfm73.h"
#include "rfm73-config.h"

int main() {
  uint8_t rfm73buf[3];

  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); // select Clock = 16 MHz
  CLK_HSICmd(ENABLE);

  asm("rim");
  TIM4_Config(); // this timer for ms delay

  waitMs(10);
  SPI_Init_RFM73();
  rfm73_init();
  rfm73_channel(0);

#ifdef IS_TRANSMIT
  uint8_t statusReg;
  uint8_t controlMode = 0;

  buttonInit();
  ADC_Init();
  rfm73_mode_transmit();

  while(1) {
   if(isButtonPressed()) {
     controlMode ^= 1;
   }

   rfm73buf[0] = getADC3()/4; // speed;
   rfm73buf[1] = getADC4()/4; // direction;
   rfm73buf[2] = controlMode;

   rfm73_transmit_message(rfm73buf, 3);
   while (1){
     statusReg = rfm73_register_read( RFM73_REG_STATUS );
     if (statusReg & 0x30) { // 0b0011 0000
       rfm73_register_write( RFM73_REG_STATUS ,statusReg );
       break;
     }
   }
   waitMs(10);
  }
#else
  uint8_t stopTimer = 0;

  L293D_GpioInit();
  TIM2_PWM_Init();
  rfm73_mode_receive();

  while (1) {
    //rfm70_receive( &pipe, rfm70buf, &length );
    if (!GPIO_ReadInputPin(PORT_SPI_IRQ_CE, PIN_IRQ)) {
      rfm73_buffer_read(RFM73_CMD_R_RX_PAYLOAD, rfm73buf, 3);
      rfm73_register_write(RFM73_REG_STATUS, 0x42); // 0b0100 0010 reset interrupt pin
      motorHeandler(rfm73buf[0], rfm73buf[1], rfm73buf[2]);
      stopTimer = 0; // обнуляем счетчик остановки
    } else {
      stopTimer++;
      waitMs(1);
      if (stopTimer > 250) {
        for (uint8_t i = 0; i < 32; i++) {
          rfm73_buffer_read(RFM73_CMD_R_RX_PAYLOAD, rfm73buf, 1);
        }
        rfm73_register_write(RFM73_REG_STATUS, 0x42); // 0b0100 0010 reset interrupt pin
        stopTimer = 0; // обнуляем счетчик остановки
        setMotorLeftPwm(0);
        setMotorRightPwm(0);
      }
    }
  }
#endif
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) { while (1) {}; };
#endif
