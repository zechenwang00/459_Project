#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>
#include "lib/ssd1306.h"
#include "lib/i2c.h"
#include "lib/rtc.h"

// definitions
#define WELCOME_SCREEN  0
#define MENU_SCREEN     1
#define SENSOR_SCREEN   2
#define CARBON_SCREEN   3
#define DEBUG_SCREEN    4

#define MATERIAL_PLASTIC    0
#define MATERIAL_GLASS      1
#define MATERIAL_ALUM       2
#define MATERIAL_PAPER      3

#define FOSC 7372800
#define BDIV ( FOSC / 100000 - 16) / 2 + 1

#define RTC_I2C_ADDR 0xD0

// global vars
volatile unsigned short pot_choice = 0;
volatile unsigned short curr_screen = WELCOME_SCREEN;
volatile unsigned short material = MATERIAL_PLASTIC;
uint8_t addr = SSD1306_ADDRESS;
volatile bool button_signal = false;

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

char* intToDay(int day){
    switch (day) {
        case 1:
            return "Mon";
            break;
        case 2:
            return "Tue";
            break;
        case 3:
            return "Wed";
            break;
        case 4:
            return "Thu";
            break;
        case 5:
            return "Fri";
            break;
        case 6:
            return "Sat";
            break;
        case 7:
            return "Sun";
            break;
        default:
            return "Mon";
            break;
    }
}

void showTime(struct rtc_time timeData, int line) {
    rtc_get_time(&timeData);
    char yrs_str[4], month_str[4], date_str[4], hrs_str[4], min_str[4], sec_str[4];


    SSD1306_SetPosition(0,line);
    itoa(timeData.year, yrs_str, 10);
    itoa(timeData.month, month_str, 10);
    itoa(timeData.date, date_str, 10);
    itoa(timeData.hours, hrs_str, 10);
    itoa(timeData.minutes, min_str, 10);
    itoa(timeData.seconds, sec_str, 10);

    SSD1306_DrawString(hrs_str);
    SSD1306_DrawChar(':');
    SSD1306_DrawString(min_str);
    SSD1306_DrawChar(':');
    SSD1306_DrawString(sec_str);
    SSD1306_DrawChar(' ');
    SSD1306_DrawString(intToDay(timeData.day));
    SSD1306_DrawChar(' ');
    SSD1306_DrawString(month_str);
    SSD1306_DrawChar('/');
    SSD1306_DrawString(date_str);
    SSD1306_DrawChar('/');
    SSD1306_DrawString(yrs_str);
}

void showWelcome(struct rtc_time timeData){
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
    SSD1306_DrawLine (0, MAX_X, 52, 52);

    showTime(timeData, 7);

//    SSD1306_SetPosition (0, 7);
//    char message_str[4];
//    itoa((int)message, message_str, 10);
//    SSD1306_DrawString(message_str);

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

void showMenu(void){
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
//    SSD1306_SetPosition(0,6);
//    SSD1306_DrawString("  DEBUG");
//    SSD1306_DrawLine (0, MAX_X, 52, 52);
    SSD1306_SetPosition(0,7);
    SSD1306_DrawString("  SENSOR INFO");

    short choice = pot_choice % 5;
    selectMenu(choice, "->");
}

unsigned short calcWeight(unsigned short reading){
    double result;
    result = exp((10 * (double)reading * 5 / 1024 - 18) / 7) * 100 - 7;
    if (result < 0) result = 0;

    return result;
}

void showCarbon(unsigned short reading) {
    unsigned short weight = calcWeight(reading);
    unsigned short emissions;
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

    snprintf(emissions_str, 5, "%u", emissions);
    snprintf(weight_str, 4, "%u", weight);

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

void playSound(void){
//    uint8_t pwm = 0x80;
//    bool up = true;
//    for(;;) {
//        OCR0A = pwm;
//
//        pwm += up ? 1 : -1;
//        if (pwm == 0xff)
//            up = false;
//        else if (pwm == 0x00);
//            up = true;
//
//        _delay_ms(10);
//    }

    unsigned long period;
    unsigned int freq = 440;
    period = 1000000 / freq;

    while(freq--) {
        PORTD |=  (1 << PD7);
        _delay_us(period/2);
        PORTD &= ~(1 << PD7);
        _delay_us(period/2);
    }
}

void servoClose() {
    //  3% -  600us, 19400us
    // 13% - 2600us, 17400us
    unsigned short time = 50;                 // 1s

    while(time--) {
        PORTD |=  (1 << PD6);
        _delay_us(1000);
        PORTD &= ~(1 << PD6);
        _delay_us(19000);
    }
}

void servoOpen() {
    //  3% -  600us, 19400us
    // 13% - 2600us, 17400us
    unsigned short time = 50;                 // 1s

    while(time--) {
        PORTD |=  (1 << PD6);
        _delay_us(1600);
        PORTD &= ~(1 << PD6);
        _delay_us(18400);
    }
}

void showDebug(void) {
    SSD1306_SetPosition(0,0);
    SSD1306_DrawString("DEBUG");
    SSD1306_DrawLine (0, MAX_X, 12, 12);
    SSD1306_SetPosition(0,2);
    SSD1306_DrawString("  BUZZER");
    SSD1306_SetPosition(0,3);
    SSD1306_DrawString("  SERVO");
    SSD1306_SetPosition(0,7);
    SSD1306_DrawString("  Back");

    short choice = pot_choice % 3;
    switch (choice) {
        case 0:
            SSD1306_SetPosition(0, 2);
            SSD1306_DrawString("->");
            if (button_signal) playSound();
            break;
        case 1:
            SSD1306_SetPosition(0, 3);
            SSD1306_DrawString("->");
            if (button_signal) {
                SSD1306_SetPosition(0, 4);
                servoOpen();
                _delay_ms(200);
                servoClose();
                _delay_ms(200);
            }
            break;
        case 2:
            SSD1306_SetPosition(0, 7);
            SSD1306_DrawString("->");
            break;
        default:
            SSD1306_SetPosition(0, 7);
            SSD1306_DrawString("->");
            break;
    }
}

void detectButton(bool button){
    if (button) {
        if (curr_screen == WELCOME_SCREEN){
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        } else if ( (curr_screen == MENU_SCREEN) & ( (pot_choice % 5) < 4) ) {
            curr_screen = CARBON_SCREEN;
            _delay_ms(100);
//        } else if ( (curr_screen == MENU_SCREEN) & ( (pot_choice % 5) == 4) ) {
//            curr_screen = DEBUG_SCREEN;
//            _delay_ms(100);
        } else if ( (curr_screen == MENU_SCREEN) & ( (pot_choice % 5) == 4) ) {
            curr_screen = SENSOR_SCREEN;
            _delay_ms(100);
        } else if (curr_screen == SENSOR_SCREEN) {
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        } else if (curr_screen == CARBON_SCREEN) {
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        } else if ((curr_screen == DEBUG_SCREEN) & ( (pot_choice % 3) == 2) ) {
            curr_screen = MENU_SCREEN;
            _delay_ms(100);
        }
    }
}

/*
serial_init - Initialize the USART port
*/
void serial_init ( unsigned short ubrr ) {
    UBRR0 = ubrr ; // Set baud rate
    UCSR0B |= (1 << TXEN0 ); // Turn on transmitter
    UCSR0B |= (1 << RXEN0 ); // Turn on receiver
    UCSR0C = (3 << UCSZ00 ); // Set for async . operation , no parity ,
    // one stop bit , 8 data bits
}

/*
serial_out - Output a byte to the USART0 port
*/
void serial_out ( char ch )
{
    while (( UCSR0A & (1 << UDRE0 )) == 0);
    UDR0 = ch ;
}

/*
serial_in - Read a byte from the USART0 and return it
*/
char serial_in ()
{
    if ( !( UCSR0A & (1 << RXC0 )) ){return NULL;}
    return UDR0;
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
int main(void)
{
    // init ssd1306
    SSD1306_Init (addr);
    SSD1306_ClearScreen();

    // init ports
    // PD2 = sound, PD3 = motion
    DDRD &= ~(1 << DDD2 | 1 << DDD3 | 1 << DDD4);
    DDRC &= ~(1 << PC1 | 1 << PC2);
    // PD6 = buzzer
    DDRD |= (1 << DDD6 | 1 << DDD7);
    PORTD &= ~(1 << PD6 | 1 << PD7);

    // init ADC
    // ADMUX: ref=AVCC, result=10bit, input=ADC1(potentiometer)
    ADMUX &= ~(1 << REFS1 | 1 << ADLAR | 1 << MUX3 | 1 << MUX2 | 1 << MUX1);
    ADMUX |=  (1 << REFS0 | 1 << MUX0);
    // ADCSRA: prescalar=128
    ADCSRA |= (1 << ADEN | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0);

    // init PWM
    // buzzer
//    TCCR0A |=  (1 << COM0A1 | 1 << WGM01 | 1 << WGM00);
//    TCCR0A &= ~(1 << COM0A0);
//    TCCR0B |=  (1 << CS02);
//    TCCR0B &= ~(1 << WGM02 | 1 << CS01 | 1 << CS00);
//    OCR0A = 0;

    i2c_init(BDIV);
    struct rtc_time timeData;

    // init vars
    bool sound_signal = false;
    bool motion_signal = false;
    unsigned short pot; // potentiometer ADC1
    unsigned short pressure; // pressure ADC2
    char pressure_str[4];
    char pot_str[4];
    servoClose();

//    rtc_run();

//    // init serial
//    serial_init(3); //ubrr=7372800/(16*115200)-1
//    char message;

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

//        unsigned char status;
//        unsigned char command[2] = {0x26, 0x0f};
//        unsigned char response[3];

//        i2c_init(BDIV);
//        status = i2c_io(VOC_I2C_ADDR, NULL, 0, command, 2, response, 3);
//        SSD1306_SetPosition(0,6);
//        SSD1306_DrawString((char *)response);


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


//        message = serial_in();

        // open lid when both sound and motion detected
        if (motion_signal & sound_signal) {
            playSound();
            servoOpen();
            _delay_ms(100);
            // close lid after motion turns OFF
            while (motion_signal) {
                if ((PIND & (1 << PD3)) == 0) {
                    motion_signal = false;
                }
            }
            servoClose();
            _delay_ms(100);
        }

        // update LCD
        if (curr_screen == WELCOME_SCREEN){
            showWelcome(timeData);
        }

        if (curr_screen == MENU_SCREEN) {
            // display menu
            showMenu();
        }

        if (curr_screen == DEBUG_SCREEN) {
            showDebug();
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
