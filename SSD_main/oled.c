#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lib/ssd1306.h"

/* 
func showMenu
input unit_t addr: address of ssd
input short recyclable: selection from rotary encoder
output short: selected recyclable type
0 - Plastic Bottles
1 - Glass Bottles 
2 - Card Boxes
3 - Paper
*/
void showMenu(uint8_t addr, short recyclable){
    SSD1306_ClearScreen ();
    SSD1306_SetPosition(0,1);
    SSD1306_DrawString("---Select Recyclable---");
    switch (recyclable)
    {
    case 0:
        SSD1306_SetPosition(0,2);
        SSD1306_DrawString("0. Plastic Bottles <-");
        SSD1306_SetPosition(0,3);
        SSD1306_DrawString("1. Glass Bottles   ");
        SSD1306_SetPosition(0,4);
        SSD1306_DrawString("2. Card Boxes   ");
        SSD1306_SetPosition(0,5);
        SSD1306_DrawString("3. Paper   ");
        break;
    case 1:
        SSD1306_SetPosition(0,2);
        SSD1306_DrawString("0. Plastic Bottles   ");
        SSD1306_SetPosition(0,3);
        SSD1306_DrawString("1. Glass Bottles <-");
        SSD1306_SetPosition(0,4);
        SSD1306_DrawString("2. Card Boxes   ");
        SSD1306_SetPosition(0,5);
        SSD1306_DrawString("3. Paper   ");
        break;
    case 2:
        SSD1306_SetPosition(0,2);
        SSD1306_DrawString("0. Plastic Bottles   ");
        SSD1306_SetPosition(0,3);
        SSD1306_DrawString("1. Glass Bottles   ");
        SSD1306_SetPosition(0,4);
        SSD1306_DrawString("2. Card Boxes <-");
        SSD1306_SetPosition(0,5);
        SSD1306_DrawString("3. Paper   ");
        break;
    case 3:
        SSD1306_SetPosition(0,2);
        SSD1306_DrawString("0. Plastic Bottles   ");
        SSD1306_SetPosition(0,3);
        SSD1306_DrawString("1. Glass Bottles   ");
        SSD1306_SetPosition(0,4);
        SSD1306_DrawString("2. Card Boxes   ");
        SSD1306_SetPosition(0,5);
        SSD1306_DrawString("3. Paper <-");
        break;
    default:
        SSD1306_SetPosition(0,2);
        SSD1306_DrawString("0. Plastic Bottles <-");
        SSD1306_SetPosition(0,3);
        SSD1306_DrawString("1. Glass Bottles   ");
        SSD1306_SetPosition(0,4);
        SSD1306_DrawString("2. Card Boxes   ");
        SSD1306_SetPosition(0,5);
        SSD1306_DrawString("3. Paper   ");
        break;
    }
}

int main(void){
    uint8_t addr = SSD1306_ADDRESS;

    // init ssd1306
    SSD1306_Init (addr);
    SSD1306_ClearScreen ();

    // init ports
    DDRC &= ~(1 << PC1);

    // init ADC
    // ADMUX: ref=AVCC, result=10bit, input=ADC1(potentiometer)
    ADMUX &= ~(1 << REFS1 | 1 << ADLAR | 1 << MUX3 | 1 << MUX2 | 1 << MUX1);
    ADMUX |=  (1 << REFS0 | 1 << MUX0);
    // ADCSRA: prescalar=128
    ADCSRA |= (1 << ADEN | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0);

    // init vars
    unsigned short pot;
    char rec_str[1];
    // char pressure_str[4];

    while(1) {
        // read from ADC
        ADCSRA |= (1 << ADSC);
        while ((ADCSRA & (1 << ADSC)) != 0);
        pot = ADC;
        // sprintf(pressure_str, "%d", pressure);

        short rec = 0;
        //range of pot: 336 - 1023
        if (pot < 500 ){
            rec = 0;
        }
        else if (pot < 650){
            rec = 1;
        }
        else if (pot < 800){
            rec = 2;
        }
        else if (pot < 950){
            rec = 3;
        }
        else{
            rec = 3;
        }
        showMenu(addr, rec);

        // print debug info
        sprintf(rec_str, "%d", rec);
        SSD1306_SetPosition(0,7);
        SSD1306_DrawString(rec_str);

        // update screen and sleep
        SSD1306_UpdateScreen(addr);
        _delay_ms(50);
    }



}