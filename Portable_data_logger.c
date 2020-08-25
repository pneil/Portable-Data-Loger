//-----------------------------------------------------------------------------
// Neelkanth Patel
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
// PF1 drives an NPN transistor that powers the red LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include <ctype.h>
#include <stdio.h>
#include <math.h>


#define LED         (*((volatile uint32_t *)(0x40025000 + 0x02*4)))
#define mode        (*((volatile uint32_t *)0x40031004))
#define MAX_CHARS 80
#define RESET 0x05FA0005
#define readAddress (*((volatile uint32_t *) (n_Add)))

uint32_t flash_Data;
int count = 0,i=0,S,P,T,y=0;
float H,V,n_Th;
char uartStr[81];
int offset[20];
int fieldCount=0;
char type[20];
uint8_t month,sec,hour,day,min,a,b,c,d,e;
uint16_t year,f,D;                                  //Years
uint32_t secondO;                                   //Old snap of HIBRTCC
uint32_t secondN,new_HIB,first_Second;              //New snap of HIBRTCC
uint16_t raw;
uint32_t value;
float Voltage,Temperature;
char str[10];
char INPUT,EDGE;
bool t_flag,hyst_flag = false,inv_B = 0;
char MODE;
uint16_t space;
uint8_t nPage,currentPage,offsetF;
uint32_t data,n_Add,DATA_E,rd_Data;
uint16_t h_D,h_Data;
uint8_t block=0,offsetE;
bool s_Flag = 0 , hib_Flag = 0;
float tempp,Voltt;
uint32_t s_Flash,input_Flash,mode_Flash,edge_Flash,hr_Flash,month_Flash,day_Flash,year_Flash,period_Flash,HIB_FLASH;
uint8_t min_Flash,sec_Flash;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

/*isCommand to check the inbuld command with user typed command wih arguements*/
bool isCommand(char *commandName,int min_arg)
{
    if(strcmp(&uartStr[offset[0]],commandName)==0)
    {
        if (fieldCount>min_arg)
        {
            return true;
        }
    }
    return false;

}

int getValue(int argNumber)

{
    return atoi(&uartStr[offset[argNumber+1]]);
}

char* getString(int argNumber)
{
    return (&uartStr[offset[argNumber+1]]);

}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);                       // wait if uart0 tx fifo full
    UART0_DR_R = c;                                          // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
     putcUart0(str[i]);
}


void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R |= 0;

    // Enable GPIO port F ,A,B,C and E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC;
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    SYSCTL_RCGCHIB_R |=  1;                          // turn-on Hibernation clock
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;         // turn-on Analog comparitor
    SYSCTL_RCGCEEPROM_R |= 1;                        // turn-on EEPROM

    // Hibernation module
    HIB_IM_R |= HIB_IM_RTCALT0  ;
    HIB_CTL_R |= HIB_CTL_CLK32EN | HIB_CTL_RTCEN ;

    // Configure AN0 as an analog input
    GPIO_PORTE_AFSEL_R |= 0x01;                      // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x01;                       // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x01;                      // turn on analog operation on pin PE3

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                                     // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                                 // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;                             // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 3;                                                // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0 | ADC_SSCTL3_TS0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                                  // enable SS3 for operation

    // Configure LED pin
    GPIO_PORTF_DIR_R = 0x02;//RED_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = 0x02;//RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x02;// RED_LED_MASK;  // enable LED

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;// select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure GPIO B  SPI configuration
    // Configure SSI2 pins for SPI configuration
    GPIO_PORTB_DIR_R |= 0xB0;                           // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0xB0;                          // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0xB0;                         // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xB0;                           // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                         // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                     // select master mode
    SSI2_CC_R = 0;                                      // select system clock as the clock source
    SSI2_CPSR_R = 40;                                   // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;

    //Configure GPIO C for analog comparitor
    //GPIO_PORTC_LOCK_R = 0;
    //GPIO_PORTC_CR_R |= 0xC0;
    GPIO_PORTC_DIR_R &= ~ 0xC0;                             // make bits 6 and 7 inputs
    GPIO_PORTC_DEN_R &= ~ 0xC0;                             // Diable data on 6 and 7
    GPIO_PORTC_AFSEL_R |= 0xC0;                             //turn no auxilary fucntion of pin PC6 and PC7
    GPIO_PORTC_AMSEL_R|=0xC0;                               //Analog mode select

    /*GPIO_PORTF_DIR_R |= 0x10;
    GPIO_PORTF_DEN_R |= 0x10;*/

    //Configure EEPROM :
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    while((EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING)==1);
    if( (EEPROM_EESUPP_PRETRY || EEPROM_EESUPP_ERETRY)==1)
    {
        SYSCTL_SREEPROM_R = 1;
        SYSCTL_SREEPROM_R = 0;
        while((EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING)==1)
        if( (EEPROM_EESUPP_PRETRY || EEPROM_EESUPP_ERETRY)==1);
    }
    else
    {
        SYSCTL_SREEPROM_R |= 1;
        __asm("             NOP");
        __asm("             NOP");
        __asm("             NOP");
        __asm("             NOP");
        __asm("             NOP");
        __asm("             NOP");
        while((EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING)==1);
        while( (EEPROM_EESUPP_PRETRY || EEPROM_EESUPP_ERETRY)==1);
    }

}

/*Approximate busy waiting (in units of microseconds), given a 40 MHz system clock*/
void waitMilisecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void analogComp()
{
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_PIN0;
    COMP_ACCTL0_R |= COMP_ACCTL0_ISEN_RISE;

               if (EDGE == 'p')
                   {
                   //COMP_ACCTL0_R |= COMP_ACCTL0_ISEN_RISE;
                           COMP_ACCTL0_R |= COMP_ACCTL1_CINV;
                           waitMilisecond(1000);
                           COMP_ACMIS_R  = COMP_ACMIS_IN0;
                   }
               else if( EDGE == 'n')
                   {
                   COMP_ACCTL0_R &= ~COMP_ACCTL0_ISEN_RISE;
                   COMP_ACCTL0_R |= COMP_ACCTL0_ISEN_FALL;
                   COMP_ACCTL0_R |= COMP_ACCTL1_CINV;
                           waitMilisecond(1000);
                           COMP_ACMIS_R  = COMP_ACMIS_IN0;
                   }

           COMP_ACMIS_R |= COMP_ACMIS_IN0;                              //interrupt status masked
           NVIC_EN0_R |= (1 << (INT_COMP0 - 16));                       //enable interrupt
           COMP_ACINTEN_R |= COMP_ACINTEN_IN0;                          //enable interrupt
           waitMilisecond(100000);
}

bool checkLeapYear(int year)                                            //Function to check Leap year
{
   return (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0));
}

void addSecond()
{
/*checks for last values of sec,min,hour,date,month
if sec is last value 60 the sec becomes 0 and min increment&
if min is last value 60 the min becomes 0 and hour increment&
if hour is last value 23 the hour becomes 0 and min, sec is 0
so every attribute validated and one second added*/

    if(sec == 59 )
    {
        if( min == 59 )
        {
            if(hour == 23)
            {
                hour=0;
                if((day==31&&(month==1 || month==3 || month==5 || month==7 || month==8 || month==10 || month==12))||(day==30&&(month==4 || month==6 || month==9 || month==11))||(day==28&&month==2&& !checkLeapYear(year))||(day==29&&month==2&& checkLeapYear(year)))
                {
                    day=1;
                    if(month==12)
                    {   month =1;
                        year++;
                    }
                    else
                    {
                        month++;
                    }
                }
                else
                    day++;
            }
            else
                hour++;
            min=0;
            sec=0;
        }
        else
        {
            min++;
            sec=0;
        }
    }
    else
    {
        sec++;
    }
    //secondO=secondN;
}

/*Function will add elapsed time and give out new date and time*/
void readTimeDate()
{
    uint32_t elapsedTime;
    secondN=HIB_FLASH;
    elapsedTime = secondN-secondO;
    for(i=0;i<elapsedTime;i++)
    addSecond();
}
/*Print Time and Date*/
void printTimeDate()
{
    putsUart0("\n\r");
    readTimeDate();
    sprintf(str, "Time: [%d : %d : %d]\r\n",hour,min,sec);              //Print Time
    putsUart0(str);
    sprintf(str, "Date: [%d/ %d/ %u] \r\n",month,day,year);                  //Print Date
    putsUart0(str);
    putsUart0("\n\r");
}

void turnOnTimer1()
{
            // Configure Timer 1 for keyboard service
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
            TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
            TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
            TIMER1_TAILR_R = value;                          // set load depends upon th  periodic sample user wnated
            TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
            NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
            TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                             // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);                   // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                                   // get single result from the FIFO
}


//128-255 pages to log DATA,
//STEP 17
//calculate the space needed for all samples
// samples * space/sample
//Trigger mode : HIBRTCC register content = calculate 32 bit + Voltage = calculate 16 bit ==== 48 bit / sample
// Total space = S x 48 (round it to 64 bit)
//Periodic mode = Store voltage / Temp and seperately will we store HIBRTCC at start = 16 bit / sample
//Total space =  S x 16 + 32bit ( 16 bit of voltage or temp and 32 bit for HIB_RTCC_R value)
// ERASE as many pages as needed
// Total space / 1024 = Num of pages e.g 1000 x 48 = 48000/1024 = 46 pages
//128 to Pages ( need to stop)
//STEP 18
uint8_t getFirstPage()
{
    currentPage = 128;
    return currentPage;
}
uint8_t getNextPage()
{
    HIB_DATA_R =  currentPage;                                  //Store current page in HIB_DATA_R i.e. battery backed memory
    //getFirstPage();
    currentPage ++;
    return currentPage;
}
/*Get next address in memory where we can write data : Step 18*/
uint32_t getNextAddress()
{
         HIB_DATA_R = offsetF;                                      // Save the offset of the flash memory into the battery backed memory
         /*n_Add = (currentPage*1024 + offsetF);                    // make new address using address of current + new offset
         offsetF = (offsetF + 1)% 1024;                             // Increment the offset
         return n_Add;                                              // return new address*/
         if( offsetF == 1024 )
         {
             offsetF = 0;
             currentPage = getNextPage();
         }
         n_Add = (currentPage*1024 + offsetF);                      // make new address using address of current + new offset
         offsetF = (offsetF + 1);                                   // Increment the offset
         return n_Add;
}

void newAdd()
{
    getNextAddress();
    getNextAddress();
    getNextAddress();
    getNextAddress();
}
//Function to write a data into indicate
void wrDataPeriodic()
{
    if ( MODE == 'p' || MODE == 't')
    {
    if( INPUT == 'v')
    {
        FLASH_FMA_R = n_Add;
        FLASH_FMD_R = Voltage*100;
        FLASH_FMC_R = 0xA4420000 | FLASH_FMC_WRITE;
        while(FLASH_FMC_R & FLASH_FMC_WRITE);
    }
    else if ( INPUT == 't')
    {
        FLASH_FMA_R = n_Add;
        FLASH_FMD_R = (Temperature)*100;
        FLASH_FMC_R = (0xA4420000 | FLASH_FMC_WRITE);
        while(FLASH_FMC_R & FLASH_FMC_WRITE);
    }
    newAdd();
    }
}

/*TImer1ISR to store sample value on interrupt on given periodic time*/
void timer1Isr()
{

        if (INPUT == 't')
        {
                raw = readAdc0Ss3();
                Temperature = 147.5 - ((75 * 3.3 * raw) / 4096);
                S=S-1;
                wrDataPeriodic();                                           //write it in Flash memory
                sprintf(str, "Temperature:  %3.3f\r\n",Temperature);      //Print the  internal temperature
                putsUart0(str);
                //printTimeDate();

        }

        else if ( INPUT == 'v')
        {
                raw = readAdc0Ss3();
                Voltage = raw*3.3/4096;                                     // Vin = R*Vref+/2^b (b=12 bit)
                //write data into registeR
                S=S-1;
                wrDataPeriodic();                                           //Write it in flash memory
                //sprintf(str, "Voltage:        %3.3f\r\n", Voltage);       //Print the  measured voltage
                //putsUart0(str);
                //printTimeDate();
                //INPUT = 2;
        }

    if ( S == 0)
    {
            TIMER1_TAMR_R = 0;                                              //Turn off periodic mode
            TIMER1_IMR_R = 0;                                               //Clear interrupt mask register
    }

            TIMER1_ICR_R = TIMER_ICR_TATOCINT;                              //clear interrupt

}

void comparator0Isr()
{
    if(t_flag == true && hyst_flag == false)
    {
        if ( S!=0)
        {
             raw = readAdc0Ss3();
             Voltage = raw*3.3/4096;
              S=S-1;
                  wrDataPeriodic();
                  sprintf(str, "Voltage:  %3.3f \r\n", Voltage);               //Print the  internal temperature
                  putsUart0(str);
             }

        else
        {
            COMP_ACINTEN_R &= ~COMP_ACINTEN_IN0;                            //Disable interrupt
        }
        COMP_ACMIS_R=0x01;
        waitMilisecond(10000);//providing an interrupt to the interrupt controller
    }
    // trigger with hysteresis
    if ( t_flag == true && hyst_flag == true )
    {
        // print sampple for first trigger condition
        if ( inv_B == 0 )
        {
            raw = readAdc0Ss3();
            Voltage = raw*3.3/4096;
            S=S-1;
            wrDataPeriodic();
            SSI2_DR_R=h_Data;
            while(SSI2_SR_R & SSI_SR_BSY);
            //h1_flag=false;
            inv_B = 1;
            waitMilisecond(100000);

            if(S == 0)
            {
                COMP_ACINTEN_R &= ~1;
                //h1_flag=0;
                inv_B=0;
                SSI2_DR_R=h_Data;
            }
        }
        else if (inv_B == 1 )
        {
            SSI2_DR_R=h_Data;
            while(SSI2_SR_R & SSI_SR_BSY);
            inv_B = 0;
        }
        COMP_ACMIS_R  = COMP_ACMIS_IN0;
        waitMilisecond(10000);
    }

}

void setUpHib()
{
        HIB_RTCM0_R = T + HIB_RTCC_R;
        HIB_IM_R &= ~ HIB_IM_WC;
        HIB_IC_R = HIB_IC_RTCALT0;
        while((HIB_CTL_R & HIB_CTL_WRC)==0)
        NVIC_EN1_R |= 1<<(INT_HIBERNATE-16-32);
        HIB_IC_R = HIB_IC_RTCALT0;
        while((HIB_CTL_R & HIB_CTL_WRC)==0)
        HIB_IM_R |= HIB_IM_RTCALT0;
}

void hibernationIsr()
{
        if (INPUT == 't')
        {
                raw = readAdc0Ss3();
                Temperature = 147.5 - ((75 * 3.3 * raw) / 4096);
                S=S-1;
                wrDataPeriodic();                                           //write it in Flash memory
        }

        else if ( INPUT == 'v')
        {
                raw = readAdc0Ss3();
                Voltage = raw*3.3/4096;                                     // Vin = R*Vref+/2^b (b=12 bit)
                //write data into registeR
                S=S-1;
                wrDataPeriodic();                                           //Write it in flash memory
        }
        setUpHib();

    if ( S == 0)
    {
        HIB_IM_R &= ~ HIB_IC_RTCALT0;                                             //Clear interrupt mask register
    }

    HIB_IC_R = HIB_IC_RTCALT0;                              //clear interrupt

}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getUart0String()
{
    count=0;

    while ( uartStr != 0x00 )                           //will run till string reaches NULL character

    {
        char c = getcUart0();


        if( c == 0x08 )                                 // if c = backspace check count and take action accordingly
        {
           if ( count >0 )                              // if count is greater then 0 then minus a count to go back to privous charachater
           {
               count--;
           }
           continue;
        }


        if ( c == 0x0D )                                 // if c = enter i.e carrage return, add NULL character and exit program
        {
            uartStr[count] = 0x00;
            break;
        }

        if ( c >= 0x20 )                                 // anything greater than spacebar i.e 0x20 is unprintable:
        {
            if (c >= 'A' && c <='Z')                     // Converting Upper case into Lower
            {
                uartStr[count++] = c+32;
            }
            else
            {
                uartStr[count++] = c;
            }

            if ( count == MAX_CHARS )                    // If max character is enter by user then put NULL character and EXIT the program
            {
                uartStr[count] = 0x00;
                break;
            }
        }
    }


}
/*Function will parse the string
 * will check and increment the character till any delimeter reaches
 * This will also Add "NULL" when the set condition does not match*/
void parseUart0String()
{
    uint8_t len=strlen(uartStr);

            for(i=0;i<len;i++)
            {
                if((((uartStr[i]>=45 && uartStr[i]<=57))||(uartStr[i]>='a' && uartStr[i]<='z')) && (((uartStr[i+1]>=45 && uartStr[i+1]<=57))||(uartStr[i+1]>='a' && uartStr[i+1]<='z')))
                {
                    offset[fieldCount]=i;
                    i++;
                    while((((uartStr[i]>=45 && uartStr[i]<=57))||(uartStr[i]>='a' && uartStr[i]<='z')) && (((uartStr[i+1]>=45 && uartStr[i+1]<=57))||(uartStr[i+1]>='a' && uartStr[i+1]<='z')))
                    {
                        i++;
                    }
                    fieldCount++;
                }
                else
                {
                    offset[fieldCount]=i+1;
                }
              }

           for(i=0;i<len;i++)
                   {
                        if((((uartStr[i]>=45 && uartStr[i]<=57))||(uartStr[i]>='a' && uartStr[i]<='z')))
                       {
                           uartStr[i]=uartStr[i];
                        }
                       else
                       {
                            uartStr[i]=0x00;
                        }
                   }
}
void calcSpace()
{
    if ( MODE == 'p')
    {
        space = S * 32 + 32;                                        //Periodic mode :32 bit for the voltage or temp value and extra 32 is for HIBRTCC register
    }
    else if ( MODE == 't')
        {
            space =  S * 64;                                    // Trigger mode :32 bit only for voltage samples
        }
    //Erase page:
    //Calculate total page needed:
    nPage = ceil(space / 1024*8);                               //Total page needed to store data

    getFirstPage();                                             //Get first page and erase it
    FLASH_FMA_R = currentPage * 1024;                           //Feed address of first page into FMA register
    FLASH_FMC_R = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;            //WRKEY bit and ERASE bits are set
    while(FLASH_FMC_R & FLASH_FMC_ERASE);

    for(i = 0; i < nPage ;i++)
    {
        data = 1024 * getNextPage();
        FLASH_FMA_R = data * 1024;
        FLASH_FMC_R = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;
        while(FLASH_FMC_R & FLASH_FMC_ERASE);
    }
}

uint32_t getFirstAddress()
{
    getFirstPage();
    offsetF = 0;
    n_Add = (currentPage*1024 + offsetF);                           // make new address using address of current + new offset
    offsetF=offsetF + 1;
    return n_Add;
}

void writeEeprom()
{
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
                     EEPROM_EEBLOCK_R  = block;
                     EEPROM_EEOFFSET_R = offsetE;
                     EEPROM_EERDWR_R   = DATA_E;
                     //length_offset++;
}

int main(void)
{
          // Initialize hardware
          initHw();

          //Toggle red LED every half second
          LED = 0x02;                                              //RED LED is turn on for half second
          waitMilisecond(500000);
          LED = 0x00;                                              //RED LED Is turn of for half second
          waitMilisecond(500000);

          putsUart0("\r\n NEEKLANTH PATEL: 1001571998");
          putsUart0("\r\n EE 5314 PROJECT");
          putsUart0("\r\n\n");

      while(true)
      {

          fieldCount = 0;
          getUart0String();                                     //Step 2
          parseUart0String();                                   //Step 3
          
          raw = readAdc0Ss3();                                               //read analog value

         if ( isCommand("reset",0) )
         {
             putsUart0("\n");
             NVIC_APINT_R= RESET;                                           //STEP 5: Resets the core and all on-chip peripherals except the Debug interface.
         }

         else  if ( isCommand("temp",0))                                     // Command to get Internal Temperature
         {
             raw = readAdc0Ss3();
             Temperature = 147.5 - ((75 * 3.3 * raw) / 4096);
             /*sprintf(str, "Temperature:       %3.3f\r\n", Temperature);     //Print the  internal temperature
             putsUart0(str);*/

         }

         else   if ( isCommand("voltage",0))                                  // Command to get Voltage
         {
             raw = readAdc0Ss3();
             Voltage = raw*3.3/4096;                                        // Vin = R*Vref+/2^b (b=12 bit)
             sprintf(str, "Voltage:        %3.3f\r\n", Voltage);            //Print the  measured voltage
             putsUart0(str);
         }

         else   if(isCommand("input",1))

         {
                     putsUart0("\r\n");
                    char* v = getString(0);

                if((strcmp(v,"temp"))==0)
                    {
                        ADC0_SSCTL3_R = ADC_SSCTL3_TS0 | ADC_SSCTL3_END0;
                        INPUT='t';
                        DATA_E = INPUT;
                        offsetE = 10;
                        writeEeprom();
                    }
                else if ((strcmp(v,"voltage"))==0)
                    {
                        ADC0_SSCTL3_R &= ~ ADC_SSCTL3_TS0;
                        INPUT='v';
                        DATA_E = INPUT;
                        offsetE = 10;
                        writeEeprom();
                    }
         }

         else  if(isCommand("sample",1))
         {
             putsUart0("\r\n");
             S = getValue(0);
             DATA_E = S;
             //block = 0;
             offsetE = 0;
             writeEeprom();
         }

         else    if(isCommand("edge",1))
         {
             putsUart0("\r\n");
             char* v = getString(0);

             if ((strcmp(v,"positive")==0))

                 {
                     EDGE = 'p';
                     DATA_E = EDGE;
                     offsetE = 9;
                     writeEeprom();
                 }

             else if ((strcmp(v,"negative")==0))

                {
                     EDGE = 'n';
                     DATA_E = EDGE;
                     offsetE = 4;
                     writeEeprom();
                }
         }

         else    if(isCommand("sleep",1))
         {
             putsUart0("\r\n");
             char* v = getString(0);

             if ((strcmp(v,"on")==0))

                {
                 NVIC_SYS_CTRL_R &= ~NVIC_SYS_CTRL_SLEEPDEEP;
                }
             else if ((strcmp(v,"off")==0))

                {
                    NVIC_SYS_CTRL_R &= ~NVIC_SYS_CTRL_SLEEPDEEP;
                }

          }

         else   if(isCommand("leveling",1))
         {
             putsUart0("\r\n");
             char* v = getString(0);

             if ((strcmp(v,"on")==0))

               {
                   NVIC_SYS_CTRL_R |= NVIC_SYS_CTRL_SLEEPDEEP;
               }

             else if ((strcmp(v,"on")==0))

              {
                  NVIC_SYS_CTRL_R |= NVIC_SYS_CTRL_SLEEPDEEP;
              }

         }

         else   if(isCommand("time",3))
         {
             putsUart0("\r\n");

             hour =  getValue(0) ;                  //Hour                h=a,m=b,c=s
             a=hour;
                         DATA_E = a;
                         offsetE = 3;
                         writeEeprom();
             min  = getValue(1) ;                   //Min
             b=min;
                         offsetE = 4;
                         DATA_E =  b;
                         writeEeprom();
             sec  = getValue(2) ;                   //Second
             c=sec;
                          offsetE = 5;
                          DATA_E =  c;
                          writeEeprom();
             secondO = HIB_RTCC_R;
             first_Second = secondO;
                         offsetE = 1;
                         DATA_E =  secondO;
                         writeEeprom();
         }

         else  if(isCommand("date",3))
         {
             putsUart0("\r\n");
             month = getValue(0);                   //Month                mm=d dd=e yy=f
             d=month;
             offsetE = 6;
                          DATA_E =  d;
                          writeEeprom();
             day= getValue(1);                      //Days
             e=day;
             offsetE = 7;
                          DATA_E =  e;
                          writeEeprom();
             year = getValue(2);                    //Years
             f=year;
             offsetE = 8;
                          DATA_E =  f;
                          writeEeprom();
         }

         else  if(isCommand("periodic",1))                                                //STEP 11 TImer1ISR to take number of sample at given period
         {
             calcSpace();
             getFirstAddress();
             new_HIB = HIB_RTCC_R;
             FLASH_FMA_R = n_Add;
             FLASH_FMD_R = new_HIB;
             FLASH_FMC_R = 0xA4420000 | FLASH_FMC_WRITE;
             while(FLASH_FMC_R & FLASH_FMC_WRITE);
             newAdd();
             MODE = 'p';
             DATA_E = MODE;
             offsetE = 2;
             writeEeprom();
             putsUart0("\r\n");
             T = atof(getString(0));
             DATA_E = T;
             offsetE = 11;
             writeEeprom();
             if ( T < 2 )
             {
                 value = 40000000*T;
                 turnOnTimer1();
             }
             else
                 setUpHib();
         }

         else   if(isCommand("threshold",1))                                               //STEP 12 DAC voltage
         {
              GPIO_PORTA_PUR_R |= 0x02;
              uint16_t DATA;
              putsUart0("\r\n");
              V = atof(getString(0));
              D = (V * 4096)/2.048;
              DATA = 12288 + D ;
              SSI2_DR_R = DATA;                                                     // write data intp data register
          }

         else  if(isCommand("trigger",0))                                          //STEP 13    with-out hysteresis                                                                                  //Pre condition : Input volt > Sample Numbers > Edge positive or negative
          {
              calcSpace();                                                         //Clear the space in flash
              getFirstAddress();
              MODE = 't';
              DATA_E = MODE;
              offsetE = 2;
              writeEeprom();
               putsUart0("\r\n");                                                   //Set DAC set up step 12
               t_flag = true;
               if ( INPUT == 'v')                                                   //Check the input is voltage or Print ERROR on uart0
               {                                                                    //set up comparitor invert bit to match the edge
                  analogComp();                                                     //Comparator0ISR
               }
               else
                putsUart0("\r\n ERROR:Input is Voltage ");
           }

         else  if (isCommand("hystersis",1))
         {
                         H=atof(getString(0));                                                  // Get Hysteresis
                         putsUart0("\r\n");
                         hyst_flag = 1;
                         if ( EDGE == 'p')
                         {
                             n_Th = V + H;                                                      //If the edge is positive then the upper limit of the threshold will be threshold + Hysteresis
                         }
                         else if ( EDGE == 'n')
                         {
                             n_Th = V - H;                                                      //IF the edge is negative then the upper limit of the threshold will be threhsold - Hysteresis
                         }
                         h_D = (n_Th * 4096)/2.048;
                         h_Data = 12288 + h_D;
         }
/*Stop commad will stop all the interrupts and sampling process*/
         else if(isCommand("stop",0))
         {
             putsUart0("\r\n");
             TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
             COMP_ACINTEN_R &= ~COMP_ACINTEN_IN0;
             HIB_IM_R &= ~HIB_IM_RTCALT0;
             putsUart0("\n controller has stopped sampling \r\r\n");
         }
/*DATA command will print all the store data in FLash and EEPROM*/
         else if (isCommand("data",0))
         {
            putsUart0("\n *****EEPROM DATA*****\n\n\r");
                    for (i=0;i<=11;i++)
                         {
                                         while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
                                         EEPROM_EEBLOCK_R  = block;
                                         EEPROM_EEOFFSET_R = i;

                                         switch(EEPROM_EEOFFSET_R)
                                         {
                                         case 0:
                                             s_Flash =  EEPROM_EERDWR_R;
                                             break;
                                         case 1:
                                             rd_Data = EEPROM_EERDWR_R;
                                             break;
                                         case 2:
                                             mode_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 3:
                                             hr_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 4:
                                             min_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 5:
                                             sec_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 6:
                                             month_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 7:
                                             day_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 8:
                                             year_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 9:
                                             edge_Flash =EEPROM_EERDWR_R;
                                             break;
                                         case 10:
                                             input_Flash = EEPROM_EERDWR_R;
                                             break;
                                         case 11:
                                             period_Flash = EEPROM_EERDWR_R;
                                             break;
                                         }
                         }

                    sprintf(str, " Sample   %u \r\n",s_Flash);
                    putsUart0(str);
                    sprintf(str, " HIB_RTCC   %u \r\n",rd_Data);
                    putsUart0(str);

                    if ( mode_Flash == 112)
                    {
                        putsUart0("\n Mode is Periodic ");
                        sprintf(str, " with period of  %u \r\n",period_Flash);
                        putsUart0(str);

                        if ( input_Flash == 118 )
                        {
                            putsUart0("\n Input is Voltage \n\r");
                        }
                        else if ( input_Flash == 116 )
                        {
                            putsUart0("\n Input is Temperature \n\r");
                        }
                    }
                    else if ( mode_Flash == 116)
                    {
                        putsUart0("\n Mode is Trigger \n\r");

                       if ( edge_Flash == 112)
                       {
                           putsUart0("\n edge is Positive \n\r");
                       }
                       else if ( edge_Flash == 110)
                       {
                           putsUart0("\n edge is Negative \n\r");
                       }
                    }
                    sprintf(str, " Time  [%d : %d : %d] \r\n",hr_Flash,min_Flash,sec_Flash);
                    putsUart0(str);
                    sprintf(str, " Date  [%d / %d / %u] \r\n",month_Flash,day_Flash,year_Flash);
                    putsUart0(str);


                    putsUart0("\n\n *****FLASH DATA***** \r\n\n\n");

                    getFirstAddress();
                    if( mode_Flash == 112 )
                                 {
                                    putsUart0("\n Periodic mode asserted on following date and time\n\r ");
                                                                 HIB_FLASH = readAddress;
                                                                 readTimeDate();
                                                                 printTimeDate();
                                                                 newAdd();
                                     if( input_Flash == 118 )
                                     {
                                         for ( i = 0 ; i<s_Flash; i++)
                                                      {

                                                      Voltt = readAddress ;
                                                      sprintf(str, " Voltage[V]   %f \r\n",Voltt/100);
                                                      putsUart0(str);
                                                      newAdd();
                                                      }
                                     }
                                     else if ( input_Flash == 116 )
                                     {
                                         for ( i = 0 ; i<s_Flash; i++)
                                                      {
                                                      tempp= readAddress ;
                                                      sprintf(str, " Temperature[c]   %f \r\n",tempp/100);
                                                      putsUart0(str);
                                                      newAdd();
                                                      }
                                     }
                                 }
                                 else if ( mode_Flash == 116)
                                 {
                                     for ( y = 0 ; y<s_Flash; y++)
                                                  {
                                                  Voltt = readAddress ;
                                                  sprintf(str, " Voltage [V]   %f \r\n",Voltt/100);
                                                  putsUart0(str);
                                                  newAdd();
                                                  }
                                 }
         }

         else
             putsUart0("\n\n !! Command is not valid !! \n\n\r");
      }

}
