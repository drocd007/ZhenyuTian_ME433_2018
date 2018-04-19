#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

#include <math.h>


#define CS LATBbits.LATB8       // chip select pin

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use primary oscillator with pll
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
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void init_spi1();
void setVoltage(char a, int v);
unsigned char spi_io(unsigned char o);



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
    LATAbits.LATA4 = 1;

    //Set B4 as input for LED
    TRISBbits.TRISB4 = 1;

    
    RPA1Rbits.RPA1R = 0b0011;
    
    __builtin_enable_interrupts();


    init_spi1();
    
    int i = 0;
    int j = 0;
    float g = 0;
    float f =0;
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk


        _CP0_SET_COUNT(0);
       
        
         
        if (j<100){
            g = j * 10.23; 
        }
        else if (j >= 100 & j < 200) {
            g = 2046.0 - (j*10.23);   
        }
        else {
            j = 0;
            g = j * 10.23; 
        }
      
        f = 512.0 + 512.0 * sin(i*3.14/50);
        
        i++;
        j++;
        
        setVoltage(1,(int)g);
        setVoltage(0,(int)f);


        
        while(_CP0_GET_COUNT()<24000){
            
        }



    }
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}


void init_spi1() {

  TRISBbits.TRISB8 = 0;
  CS = 1;

  // Master - SPI1, pins are: SDI4(F4), SDO4(F5), SCK4(F13).
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here

  // setup spi1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 1000;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
}

void setVoltage(char a, int v){

    unsigned short t;

    t = a << 15;

    t = t | 0b0111000000000000;

    t = t | ((v & 0b1111111111) << 2);

                            // send a ram set status command.
  CS = 0;                   // enable the ram
  spi_io(t>>8);             // ram write status
  spi_io(t&0xFF);           // sequential mode (mode = 0b01), hold disabled (hold = 0)
  CS = 1;                   // finish the command
}
