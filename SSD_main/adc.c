#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lib/ssd1306.h"

int main(void){
    uint8_t addr = SSD1306_ADDRESS;

    // init ssd1306
    SSD1306_Init (addr);
    SSD1306_ClearScreen ();

    // init ports
    DDRC &= ~(1 << PC1);

    // init ADC
    // ADMUX: ref=AVCC, result=10bit, input=ADC1
    ADMUX &= ~(1 << REFS1 | 1 << ADLAR | 1 << MUX3 | 1 << MUX2 | 1 << MUX1);
    ADMUX |=  (1 << REFS0 | 1 << MUX0);
    // ADCSRA: prescalar=128
    ADCSRA |= (1 << ADEN | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0);

    // init vars
    unsigned short pressure = 0;
    char pressure_str[4];

    while(1) {
        // read from ADC
        ADCSRA |= (1 << ADSC);
        while ((ADCSRA & (1 << ADSC)) != 0);
        pressure = ADC;
        sprintf(pressure_str, "%d   ", pressure);

        SSD1306_SetPosition(0,2);
        SSD1306_DrawString(pressure_str);
        SSD1306_UpdateScreen (addr);

        _delay_ms(500);

    }



}