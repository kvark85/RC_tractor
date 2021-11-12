## Proportional remote control of a tractor (transmitter and receiver)

Were used STM8 microcontroller and RFM73 or RFM70 transceiver.

The joystick can be used with [USB receiver](https://github.com/kvark85/joystick1) for computer games.

| Receiver PCB | Transmitter PCB |
|--------------|-----------------|
|<img src="https://github.com/kvark85/RC_tractor/raw/master/Foto/IMG_8858.JPG" width="250" >|<img src="https://github.com/kvark85/RC_tractor/raw/master/Foto/IMG_8859.JPG" width="250"> | 

### For correct receiver work, remapping bit AFR1 should be set to 1

It is necessary to remap TIM2 PWM channel3 to PD2. Otherwise, PWM will go to PA3 (CSN of the transceiver).
Bit AFR can be configured in **ST Visual Programmer**.

<img src="https://github.com/kvark85/RC_tractor/raw/master/Foto/ST%20Visual%20Programmer.png" >