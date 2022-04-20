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

void switch_ADC_mux_and_convert(short num){
    // only supports 1 and 2
    switch(num) {
        case 1:
            ADMUX &= ~(1 << MUX3 | 1 << MUX2 | 1 << MUX1);
            ADMUX |=  (1 << MUX0);
            break;
        case 2:
            ADMUX &= ~(1 << MUX3 | 1 << MUX2 | 1 << MUX0);
            ADMUX |=  (1 << MUX1);
            break;
        default:
            break;
    }
    // start conversion
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC)) != 0);

    return;
}


int main(void)
{
    uint8_t addr = SSD1306_ADDRESS;

    // init ssd1306
    SSD1306_Init (addr);
    SSD1306_ClearScreen ();

    // init ports
    // PD2 = sound, PD3 = motion, PD7 & PB0 = tilt
    DDRD &= ~(1 << DDD2 | 1 << DDD3 | 1 << DDD4 | 1 << DDD7);
    DDRB &= ~(1 << DDB0);
    DDRC &= ~(1 << PC1 | 1 << PC2);

    // init ADC
    // ADMUX: ref=AVCC, result=10bit, input=ADC1(potentiometer)
    ADMUX &= ~(1 << REFS1 | 1 << ADLAR | 1 << MUX3 | 1 << MUX2 | 1 << MUX1);
    ADMUX |=  (1 << REFS0 | 1 << MUX0);
    // ADCSRA: prescalar=128
    ADCSRA |= (1 << ADEN | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0);


    // init vars
    bool sound_signal = false;
    bool motion_signal = false;
    bool button_signal = false;
    bool tilt_sig_1 = false;
    bool tilt_sig_2 = false;
    bool tilt_sig_1_default = PIND & (1 << PD7);
    bool tilt_sig_2_default = PINB & (1 << PB0);
    unsigned short pot; // potentiometer ADC1
    unsigned short pressure; // pressure ADC2
    char pressure_str[4];
    char pot_str[4];



    while(1) {
        // ---- Main Loop ----
        // TODO: Implement State Machine
        // TODO: Implement Interrupts for tilt sensor
        // TODO: Implement different intervals for reading different sensors
        // TODO: Implement rotary encoder switching commands on lcd

         SSD1306_ClearScreen ();
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

        if ((PIND & (1 << PD4)) != 0) {       // PD4 = button
            button_signal = true;
        } else {
            button_signal = false;
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

        // read ADC
        // TODO: use interrupts
            // potentiometer = ADC1
            switch_ADC_mux_and_convert(1);
            pot = ADC;
            sprintf(pot_str, "%d   ", pot);

            // pressure = ADC2
            switch_ADC_mux_and_convert(2);
            pressure = ADC;
            sprintf(pressure_str, "%d   ", pressure);



        // update LCD
        if (sound_signal) {
            SSD1306_SetPosition(0,1);
            SSD1306_DrawString("sound ON ");
        } else {
            SSD1306_SetPosition(0,1);
            SSD1306_DrawString("sound OFF");
        }

        if (motion_signal) {
            SSD1306_SetPosition(0,2);
            SSD1306_DrawString("motion ON ");
        } else {
            SSD1306_SetPosition(0,2);
            SSD1306_DrawString("motion OFF");
        }

        if (button_signal) {
            SSD1306_SetPosition(0,3);
            SSD1306_DrawString("button ON ");
        } else {
            SSD1306_SetPosition(0,3);
            SSD1306_DrawString("button OFF");
        }

//        if ((tilt_sig_1 == tilt_sig_1_default) && (tilt_sig_2 == tilt_sig_2_default)) {
//            SSD1306_SetPosition(0,3);
//            SSD1306_DrawString("tilt off");
//        } else {
//            SSD1306_SetPosition(0,3);
//            SSD1306_DrawString("tilt on ");
//        }

        SSD1306_SetPosition(0,5);
        SSD1306_DrawString("poten: ");
        SSD1306_DrawString(pot_str);

        SSD1306_SetPosition(0,6);
        SSD1306_DrawString("pressure: ");
        SSD1306_DrawString(pressure_str);


//        SSD1306_SetPosition(0,6);
//        if (tilt_sig_1) {
//            SSD1306_DrawString("tilt b1: 0");
//        } else {
//            SSD1306_DrawString("tilt b1: 1");
//        }
//
//        SSD1306_SetPosition(0,7);
//        if (tilt_sig_2) {
//            SSD1306_DrawString("tilt b2: 0");
//        } else {
//            SSD1306_DrawString("tilt b2: 1");
//        }

        SSD1306_UpdTxtPosition();
        SSD1306_UpdateScreen (addr);
        _delay_ms(50);   // read signal every 50ms
    }

    // return value
    return 0;
}
