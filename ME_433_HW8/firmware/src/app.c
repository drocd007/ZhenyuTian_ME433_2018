/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include "ST7735.h"
#include<stdio.h> //get sprintf


    int q = 0;
    unsigned short data[10];

 
    signed short  accelX, accelZ;
    
    int percentX, percentZ;
    
    int range = 32768;
    
    char message[30];
    char message1[30];
    char message2[30];

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************


#define ADDR 0b1101011
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

void initExp(){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();

    writei2c(0x10,0b10000010);
    writei2c(0x11,0b10001000);
    writei2c(0x12,0b00000100);
    

}

void writei2c(unsigned char reg, unsigned char val){
    i2c_master_start();
    i2c_master_send(ADDR <<1 | 0);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
}


unsigned char readi2c(unsigned char reg1){
    i2c_master_start(); // make the start bit
    i2c_master_send((ADDR << 1) | 0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing
    i2c_master_send(reg1); // the register to read from
    i2c_master_restart(); // make the restart bit
    i2c_master_send((ADDR << 1) | 1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading
    unsigned char r = i2c_master_recv(); // save the value returned
    i2c_master_ack(1); // make the ack so the slave knows we got it 
    i2c_master_stop(); // make the stop bit

    return r;

}


void I2C_read_multiple(unsigned char reg2, unsigned short * data, int length){
    
    int i;

    
    i2c_master_start(); // make the start bit
    i2c_master_send((ADDR << 1) | 0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing
    i2c_master_send(reg2); // the register to read from
    i2c_master_restart(); // make the restart bit
    i2c_master_send((ADDR << 1) | 1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading
    
    for (i=0;i<length;i++){    
        data[i]= i2c_master_recv();
        
        if (i<length-1){
            i2c_master_ack(0);
        }

        else{
            i2c_master_ack(1);
        }
        
    }
    i2c_master_stop();
    
    
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    //Set A4 as output for LED
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    
    //Set B4 as input for LED
    TRISBbits.TRISB4 = 1;
    


    initExp();
    LCD_init();
    LCD_clearScreen(BLACK);
    
   
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            
            I2C_read_multiple(0x20, data, 14);
            
            accelX = data[8] | (data[9] << 8);
            accelZ = data[12] | (data[13] << 8);
            
            
            sprintf(message1, "X Accel %d      ", accelX);
            LCD_drawString(10,5,message1, WHITE, BLACK);
            
            sprintf(message2, "Z Accel %d      ", accelZ);
            LCD_drawString(10,20,message2, WHITE, BLACK);

            
            percentX = 50*accelX/range;
            percentZ = 50*accelZ/range;
            
            
            if (accelX>0){
                LCD_drawProgressBox(64,80,2,percentX,50-percentX,WHITE,BLUE);
                LCD_drawProgressBox(14,80,2,0,50,WHITE,BLUE);
                
            }
            
            if (accelX<0){
                LCD_drawProgressBox(14,80,2,50+percentX, -percentX,BLUE,WHITE);
                LCD_drawProgressBox(64,80,2,0,50,WHITE,BLUE);
                  
            }
            
            if (accelZ>0){
                LCD_drawProgressBoxVertical(63,30,2,50-percentZ,percentZ,BLUE,WHITE);
                LCD_drawProgressBoxVertical(63,80,2,0,50,WHITE,BLUE);
            }
            
            if (accelZ<0){
                LCD_drawProgressBoxVertical(63,30,2,50,0,BLUE,WHITE);
                LCD_drawProgressBoxVertical(63,80,2,-percentZ,50+percentZ,WHITE,BLUE);
            }
           
      
        
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT()<2400000){
            }
            LATAbits.LATA4 = !LATAbits.LATA4;
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
