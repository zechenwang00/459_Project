/** 
 * ---------------------------------------------------------------+ 
 * @desc        OLED SSD1306 example
 * ---------------------------------------------------------------+ 
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       06.10.2020
 * @update      19.07.2021
 * @file        main.c
 * @tested      AVR Atmega328p
 *
 * @depend      ssd1306.h
 * ---------------------------------------------------------------+
 * @descr       Version 1.0 -> applicable for 1 display
 *              Version 2.0 -> rebuild to 'cacheMemLcd' array
 * ---------------------------------------------------------------+
 */

// include libraries
#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lib/ssd1306.h"

/**
 * @desc    Main function
 *
 * @param   void
 *
 * @return  int
 */
int main(void)
{
  uint8_t addr = SSD1306_ADDRESS;

  // init ssd1306
  SSD1306_Init (addr);

  // clear screen
  SSD1306_ClearScreen ();

  // init ports

  // PD2 = sound, PD3 = motion, PD7 & PB0 = tilt
  DDRD &= ~(1 << DDD2 || 1 << DDD3 || 1 << DDD7);
  DDRB &= ~(1 << DDB0);
  PORTD &= ~(1 << PD2 || 1 << PD3 || 1 << PD7);
  PORTB &= ~(1 << PB0);

  // vars
  bool sound_signal = false;
  bool motion_signal = false;
  bool tilt_sig_1 = false;
  bool tilt_sig_2 = false;

  // init readings
  bool tilt_sig_1_default = PIND & (1 << PD7);
  bool tilt_sig_2_default = PINB & (1 << PB0);

  while(1) {
      // SSD1306_ClearScreen ();
      // read sensors

      if ((PIND & (1 << PD2)) != 0) {       // PD2 = sound
          sound_signal = true;
      } else {
          sound_signal = false;
      }

      if ((PIND & (1 << PD3)) != 0) {       // PD3 = motion
            motion_signal = true;
      } else {
            motion_signal = false;
      }

      if ((PIND & (1 << PD7)) != 0) {       // PD7 = tilt 1
          tilt_sig_1 = true;
      } else {
          tilt_sig_1 = false;
      }

      if ((PINB & (1 << PB0)) != 0) {       // PB0 = tilt 2
          tilt_sig_2 = true;
      } else {
          tilt_sig_2 = false;
      }



          // update LCD
      if (sound_signal) {
          SSD1306_SetPosition(0,1);
          SSD1306_DrawString("sound ON ");
          SSD1306_UpdateScreen (addr);
      } else {
          SSD1306_SetPosition(0,1);
          SSD1306_DrawString("sound OFF");
          SSD1306_UpdateScreen (addr);
      }

      if (motion_signal) {
          SSD1306_SetPosition(0,2);
          SSD1306_DrawString("motion ON ");
          SSD1306_UpdateScreen (addr);
      } else {
          SSD1306_SetPosition(0,2);
          SSD1306_DrawString("motion OFF");
          SSD1306_UpdateScreen (addr);
      }

      if ((tilt_sig_1 == tilt_sig_1_default) && (tilt_sig_2 == tilt_sig_2_default)) {
          SSD1306_SetPosition(0,4);
          SSD1306_DrawString("tilt off");
          SSD1306_UpdateScreen (addr);
      } else {
          SSD1306_SetPosition(0,4);
          SSD1306_DrawString("tilt on ");
          SSD1306_UpdateScreen (addr);
      }

      SSD1306_SetPosition(0,6);
      if (tilt_sig_1) {
          SSD1306_DrawString("tilt b1: 1");
      } else {
          SSD1306_DrawString("tilt b1: 2");
      }

      SSD1306_SetPosition(0,7);
      if (tilt_sig_2) {
          SSD1306_DrawString("tilt b2: 1");
      } else {
          SSD1306_DrawString("tilt b2: 2");
      }


      _delay_ms(100);   // read signal every 200ms
  }




//  // draw line
//  SSD1306_DrawLine (0, MAX_X, 4, 4);
//  // set position
//  SSD1306_SetPosition (7, 1);
//  // draw string
//  SSD1306_DrawString ("SSD1306 OLED DRIVER");
//  // draw line
//  SSD1306_DrawLine (0, MAX_X, 18, 18);
//  // set position
//  SSD1306_SetPosition (40, 3);
//  // draw string
//  SSD1306_DrawString ("MATIASUS");
//  // set position
//  SSD1306_SetPosition (53, 5);
//  // draw string
//  SSD1306_DrawString ("2021");
//  // update
//  SSD1306_UpdateScreen (addr);

  // return value
  return 0;
}
