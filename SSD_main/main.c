#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "lib/ssd1306.h"
#include "lib/i2c.h"

// definitions
#define WELCOME_SCREEN  0
#define MENU_SCREEN     1
#define SENSOR_SCREEN   2
#define CARBON_SCREEN   3

#define MATERIAL_PLASTIC    0
#define MATERIAL_GLASS      1
#define MATERIAL_ALUM       2
#define MATERIAL_PAPER      3

#define FOSC 7372800
#define BDIV ( FOSC / 100000 - 16) / 2 + 1

// global vars
unsigned short pot_choice = 0;
unsigned short curr_screen = WELCOME_SCREEN;
unsigned short material = MATERIAL_PLASTIC;
uint8_t addr = SSD1306_ADDRESS;
bool button_signal = false;

// functions
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

void update_pot(short pot){
    // potentiometer range: 336 -1023
    // divided into 8 ranges
    if (pot < 421 ){
        pot_choice = 7;
    }
    else if (pot < 506){
        pot_choice = 6;
    }
    else if (pot < 591){
        pot_choice = 5;
    }
    else if (pot < 676){
        pot_choice = 4;
    }
    else if (pot < 761){
        pot_choice = 3;
    }
    else if (pot < 846){
        pot_choice = 2;
    }
    else if (pot < 931){
        pot_choice = 1;
    }
    else {
        pot_choice = 0;
    }
}

void showWelcome(void){
    // draw welcome screen
    SSD1306_DrawLine (0, MAX_X, 4, 4);
    SSD1306_SetPosition (1, 1);
    SSD1306_DrawString ("EE459 CAPSTONE");
    SSD1306_SetPosition (1, 2);
    SSD1306_DrawString ("Smart Recycling Bin");
    SSD1306_DrawLine (0, MAX_X, 26, 26);
    SSD1306_SetPosition (40, 4);
    SSD1306_DrawString ("GROUP 16");
    SSD1306_SetPosition (53, 5);
    SSD1306_DrawString ("2022");

//    unsigned char i2c_rbuf[9];
//
//    i2c_init(BDIV);
//    i2c_io(0x08, NULL, 0, NULL, 0, i2c_rbuf, 9);
//    SSD1306_SetPosition (0, 7);
//    SSD1306_DrawString( (char *) i2c_rbuf);
}

void selectMenu (short choice, char* symbol) {
    switch (choice) {
        case MATERIAL_PLASTIC:
            SSD1306_SetPosition(0, 2);
            SSD1306_DrawString(symbol);
            if (button_signal)  material = MATERIAL_PLASTIC;
            break;
        case MATERIAL_GLASS:
            SSD1306_SetPosition(0, 3);
            SSD1306_DrawString(symbol);
            if (button_signal)  material = MATERIAL_GLASS;
            break;
        case MATERIAL_ALUM:
            SSD1306_SetPosition(0, 4);
            SSD1306_DrawString(symbol);
            if (button_signal)  material = MATERIAL_ALUM;
            break;
        case MATERIAL_PAPER:
            SSD1306_SetPosition(0, 5);
            SSD1306_DrawString(symbol);
            if (button_signal)  material = MATERIAL_PAPER;
            break;
        case 4:
            SSD1306_SetPosition(0, 7);
            SSD1306_DrawString(symbol);
            break;
        default:
            SSD1306_SetPosition(0, 2);
            SSD1306_DrawString(symbol);
            break;

    }
}

void showMenu(uint8_t addr){
    // draws title and menu
    SSD1306_SetPosition(0,0);
    SSD1306_DrawString("  Select Recyclable  ");
    SSD1306_DrawLine (0, MAX_X, 12, 12);
    SSD1306_SetPosition(0,2);
    SSD1306_DrawString("  0. Plastic");
    SSD1306_SetPosition(0,3);
    SSD1306_DrawString("  1. Glass");
    SSD1306_SetPosition(0,4);
    SSD1306_DrawString("  2. Aluminium");
    SSD1306_SetPosition(0,5);
    SSD1306_DrawString("  3. Paper");
    SSD1306_DrawLine (0, MAX_X, 52, 52);
    SSD1306_SetPosition(0,7);
    SSD1306_DrawString("  SENSOR INFO");

    short choice = pot_choice % 5;
    selectMenu(choice, "->");
}

unsigned short calcWeight(unsigned short reading){
//    double result;

//    result = exp((10 * (double)reading * 5 / 1024 - 18) / 7) * 100;

    return reading;
}

void showCarbon(unsigned short reading) {
    unsigned short weight = calcWeight(reading);
    unsigned short emissions = 0;
    char emissions_str[10];
    char weight_str[10];

    SSD1306_SetPosition(0, 0);
    switch (material) {
        case MATERIAL_PLASTIC:
            SSD1306_DrawString("Material: PLASTIC");
            emissions = weight * 6;
            break;
        case MATERIAL_GLASS:
            SSD1306_DrawString("Material: GLASS");
            emissions = weight / 2;
            break;
        case MATERIAL_ALUM:
            SSD1306_DrawString("Material: ALUMINIUM");
            emissions = weight * 11;
            break;
        case MATERIAL_PAPER:
            SSD1306_DrawString("Material: PAPER");
            emissions = weight;
            break;
        default:
            SSD1306_DrawString("Material: PLASTIC");
            emissions = weight * 6;
            break;
    }

    snprintf(emissions_str, 5, "%d", emissions);
    snprintf(weight_str, 4, "%d", weight);

    SSD1306_SetPosition(0, 2);
    SSD1306_DrawString("Current Weight: ");
    SSD1306_SetPosition(0, 3);
    SSD1306_DrawString("    ");
    SSD1306_DrawString(weight_str);
    SSD1306_DrawString(" g");
    SSD1306_SetPosition(0, 4);
    SSD1306_DrawString("Equivalent CO2 Saved: ");
    SSD1306_SetPosition(0, 5);
    SSD1306_DrawString("    ");
    SSD1306_DrawString(emissions_str);
    SSD1306_DrawString(" g");
    SSD1306_DrawLine (0, MAX_X, 12, 12);
    SSD1306_SetPosition(0, 7);
    SSD1306_DrawString("->  Back  <-");
}

void detectButton(bool button){
    if (button) {
        if (curr_screen == WELCOME_SCREEN){
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        } else if ( (curr_screen == MENU_SCREEN) & ((pot_choice % 5) != 4) ) {
            curr_screen = CARBON_SCREEN;
            _delay_ms(100);
        } else if ( (curr_screen == MENU_SCREEN) & ((pot_choice % 5) == 4) ) {
            curr_screen = SENSOR_SCREEN;
            _delay_ms(100);
        } else if (curr_screen == SENSOR_SCREEN) {
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        } else if (curr_screen == CARBON_SCREEN) {
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        }
    }
}


int main(void)
{
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

        // read ADC
        // TODO: use interrupts
            // potentiometer = ADC1
            switch_ADC_mux_and_convert(1);
            pot = ADC;
            sprintf(pot_str, "%d   ", pot);
            update_pot(pot);

            // pressure = ADC2
            switch_ADC_mux_and_convert(2);
            pressure = ADC;
            sprintf(pressure_str, "%d   ", pressure);



        // update LCD
        if (curr_screen == WELCOME_SCREEN){
            showWelcome();
        }

        if (curr_screen == MENU_SCREEN) {
            // display menu
            showMenu(addr);

        }

        if (curr_screen == SENSOR_SCREEN) {
            // debug info
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

            SSD1306_SetPosition(0,4);
            SSD1306_DrawString("poten: ");
            SSD1306_DrawString(pot_str);

            SSD1306_SetPosition(0,5);
            SSD1306_DrawString("pressure: ");
            SSD1306_DrawString(pressure_str);

            SSD1306_SetPosition(0,7);
            SSD1306_DrawString("->  Back  <-");
        }

        if (curr_screen == CARBON_SCREEN) {
            showCarbon(pressure);

        }

        detectButton(button_signal);

        SSD1306_UpdTxtPosition();
        SSD1306_UpdateScreen (addr);
        _delay_ms(50);   // read signal every 50ms
    }

    // return value
    return 0;
}
