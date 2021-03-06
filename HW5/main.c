
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

#define ADDR 0b0100000


// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void initExp();
void writei2c(unsigned char reg, unsigned char val);
unsigned char readi2c();



int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here

    //Set A4 as output for LED
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
    
    //Set B4 as input for LED
    TRISBbits.TRISB4 = 1;
    
    initExp();
    
    
    __builtin_enable_interrupts();

    
    
    
    
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk

        
       
        
        
        if((readi2c() & 0b10000000) == 0b10000000){
           writei2c(0x0A,0b00000000);
           
        }
        else{
           writei2c(0x0A,0b00000001);
        }
        
        
        
        
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<6000000){
            
        
        }
    
        LATAbits.LATA4 = !LATAbits.LATA4;
        
     
        
       

    }
    
}

void initExp(){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();

    writei2c(0x00,0b11110000);
    writei2c(0x0A,0b00001111);
    

}




void writei2c(unsigned char reg, unsigned char val){
    i2c_master_start();
    i2c_master_send(ADDR <<1 | 0);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
}


unsigned char readi2c(){
i2c_master_start(); // make the start bit
i2c_master_send((ADDR << 1) | 0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing
i2c_master_send(0x09); // the register to read from
i2c_master_restart(); // make the restart bit
i2c_master_send((ADDR << 1) | 1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading
unsigned char r = i2c_master_recv(); // save the value returned
i2c_master_ack(1); // make the ack so the slave knows we got it
i2c_master_stop(); // make the stop bit

return r;

}