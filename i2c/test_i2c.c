#define FOSC 7372800
#define BDIV ( FOSC / 100000 - 16) / 2 + 1
#define VOC_ADDR 0x59

int main(void) {

    unsigned char status;
    

    i2c_init(BDIV);

    while(1) {

    }
}