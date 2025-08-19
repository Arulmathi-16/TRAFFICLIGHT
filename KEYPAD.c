// PIC16F877A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = EXTRC     // Oscillator Selection bits (RC oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define _XTAL_FREQ 8000000  // Define crystal frequency for delay

void init(void);
void Lcd_command(unsigned char );
void Lcd_data(unsigned char );
void keypad_output(void);

unsigned char i,y;
unsigned char array[7] = {"KEYPAD:"};

void main()
{
    init();                 // Initialize ports and LCD

    Lcd_command(0x80);      // Set cursor to 1st row, 3rd position
    for(y = 0; y < 7; y++)
    {
        Lcd_data(array[y]); // Display "keypad:"
    }

    Lcd_command(0xC0);      // Move cursor to 2nd line

    while(1)
    {
        keypad_output();    // Continuously scan keypad and display pressed key
    }
}

void init()
{
    TRISC = 0x00;   // PORTC as output (LCD control)
    TRISD = 0x00;   // PORTD as output (LCD data)
    TRISB = 0xF0;   // RB7-RB4 input (columns), RB3-RB0 output (rows)
    PORTB = 0x0F;   // Initialize PORTB

    OPTION_REG&=0X7F; // Enable PORTB internal pull-ups

    // LCD Initialization sequence
    Lcd_command(0x30); 
    __delay_ms(100);
    
    Lcd_command(0x30);          
    __delay_ms(100);
    
    Lcd_command(0x30);
    __delay_ms(100);
    
    Lcd_command(0x38);  // 8-bit mode, 2 lines, 5x7 font
    __delay_ms(100);

    Lcd_command(0x0C);  // Display ON, Cursor OFF
    __delay_ms(100);

    Lcd_command(0x01);  // Clear display
    __delay_ms(100);

    Lcd_command(0x06);  // Entry mode set (increment cursor)
    __delay_ms(100);
}

void Lcd_command(unsigned char i)
{
    PORTC &= ~0x08;   // RS = 0 for command (RC3)
    PORTD = i;      // Put command on data port
    PORTC |= 0x01;    // E = 1 (RC0)
    __delay_ms(100);
    PORTC &= ~0x01;   // E = 0
    __delay_ms(100);
}

void Lcd_data(unsigned char i)
{
    PORTC |= 0x08;    // RS = 1 for data (RC3)
    PORTD = i;     // Put data on data port
    PORTC |= 0x01;    // E = 1 (RC0)
    __delay_ms(100);
    PORTC &= ~0x01;   // E = 0
    __delay_ms(100);
}

void keypad_output()
{
    TRISB = 0xF0;             // Upper nibble inputs, lower nibble outputs
    PORTB = 0x0F;             // Initialize rows high

    // Scan Row 1 (RB0 LOW)
    PORTB = 0x0E; // RB0=0, others=1
   
    if (RB4 == 0) 
    { 
        Lcd_data('7'); 
        while(RB4==0); 
        __delay_ms(100);
    }
    if (RB5 == 0) 
    { 
        Lcd_data('8'); 
        while(RB5==0);
        __delay_ms(100);
    }
    if (RB6 == 0) 
    { 
        Lcd_data('9'); 
        while(RB6==0); 
        __delay_ms(100); 
    }
    if (RB7 == 0) 
    { 
        Lcd_data('%');
        while(RB7==0); 
        __delay_ms(100);
    }

    // Scan Row 2 (RB1 LOW)
    PORTB = 0x0D; // RB1=0
   
    if (RB4 == 0) 
    { 
        Lcd_data('4'); 
        while(RB4==0); 
        __delay_ms(100); 
    }
    if (RB5 == 0) 
    { Lcd_data('5');
    while(RB5==0); 
    __delay_ms(100);
    }
    if (RB6 == 0) 
    { Lcd_data('6');
    while(RB6==0);
    __delay_ms(100);
    }
    if (RB7 == 0) 
    { 
        Lcd_data('X');
        while(RB7==0); 
        __delay_ms(100);
    }

    // Scan Row 3 (RB2 LOW)
    PORTB = 0x0B; // RB2=0
    
    if (RB4 == 0) {
        Lcd_data('1');
        while(RB4==0); 
        __delay_ms(100); 
    }
    if (RB5 == 0) {
        Lcd_data('2'); 
        while(RB5==0);
        __delay_ms(100); 
    }
    if (RB6 == 0)
    { 
        Lcd_data('3');
        while(RB6==0); 
        __delay_ms(100);
    }
    if (RB7 == 0) 
    { Lcd_data('-'); 
    while(RB7==0); 
    __delay_ms(100);
    }

    // Scan Row 4 (RB3 LOW)Lcd_command(0x30); 
    PORTB = 0x07; // RB3=0
    
    if (RB4 == 0)
    { 
        Lcd_data('#');
        while(RB4==0); 
        __delay_ms(100);
    }
    if (RB5 == 0) 
    { 
        Lcd_data('0');
        while(RB5==0);
        __delay_ms(100);
    }
    if (RB6 == 0) 
    { 
        Lcd_data('=');
        while(RB6==0);
        __delay_ms(100);
    }
    if (RB7 == 0) 
    { 
        Lcd_data('+');
        while(RB7==0); 
        __delay_ms(100); 
    }
}
