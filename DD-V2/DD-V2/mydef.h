/*
 * mydef.h
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 

#include <atmel_start.h>
//
//// CONFIG1
//#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
//#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
//#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
//#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
//#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
//#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
//#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
//#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
//#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
//
//// CONFIG2
//#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
//#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
//#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
//#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
//#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#ifndef MYDEF_H
#define MYDEF_H



#ifndef NULL
#define NULL                          0
#endif

#define EEDATA_INIT_FLAG              0
#define EEDATA_RAW_FLAG               1

//#define SYS_FREQ        (unsigned long)(_XTAL_FREQ/4000)  //
//#define FCY             SYS_FREQ/4

//#define delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000)))
//#define delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000)))


#define _BV(bit)                      (1LL << (bit))

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;  
typedef enum _BIT { CLEAR = 0, SET }  BIT;

#define uint unsigned  int
#define uchar unsigned  char
#define UCHAR unsigned  char
#define CHAR  signed char
#define UCHAR unsigned char
#define UINT  unsigned int
#define INT   signed int
#define USHORT  unsigned int
#define SHORT   signed int
#define ULONG unsigned long
#define LONG signed long


typedef unsigned char                u8;
typedef  signed char                 s8;
typedef unsigned int                 u16;
typedef signed int                    s16;
typedef unsigned long                u32;
typedef signed long                  s32;
typedef unsigned long long           u64;
typedef signed long long             s64;

typedef unsigned char                UINT8;
typedef  signed char                 INT8;


typedef unsigned int                 UINT16;
typedef signed int                    INT16;

typedef unsigned long                UINT32;
typedef signed long                  INT32;

typedef unsigned long long           UINT64;
typedef signed long long             INT64;


typedef float                         FLOAT32;
typedef long double                   FLOAT64;         


#define MIN(x, y)                      (((x) < (y))? (x):(y))
#define MAX(x, y)                      (((x) > (y))? (x):(y))
#define ABSSUB(x, y)  	                (MAX(x, y) - MIN(x, y))


//#define SLEEP()         Sleep()
//#define IDLE()          Idle()
//#define CLRWDT()        ClrWdt()
//#define NOP()           Nop()
#define ENABLEWDT();    {WDTCONbits.SWDTEN = 1;}
#define DISABLEWDT();   {WDTCONbits.SWDTEN = 0;}
//#define RESET();        Reset();

typedef union _UNION8
{
    UINT8 U8;
    
    struct
    {
        unsigned bit0 : 1;
        unsigned bit1 : 1;
        unsigned bit2 : 1;
        unsigned bit3 : 1;
        unsigned bit4 : 1;
        unsigned bit5 : 1;
        unsigned bit6 : 1;
        unsigned bit7 : 1;
    };
}UNION8;

// A union type for byte or word access for 16 bit values.
typedef union _UNION16
{
    UINT16 U16;
    UINT8  U8[2];
    struct
    {
        unsigned bit0  : 1;
        unsigned bit1  : 1;
        unsigned bit2  : 1;
        unsigned bit3  : 1;
        unsigned bit4  : 1;
        unsigned bit5  : 1;
        unsigned bit6  : 1;
        unsigned bit7  : 1;
        unsigned bit8  : 1;
        unsigned bit9  : 1;
        unsigned bit10 : 1;
        unsigned bit11 : 1;
        unsigned bit12 : 1;
        unsigned bit13 : 1;    
        unsigned bit14 : 1;
        unsigned bit15 : 1;
    };
} UNION16;

// A union type for byte, word, or dword access for 32 bit values.
typedef union _UNION32 
{
    UINT32 U32;

    struct 
    {
        UINT16 LS16;
        UINT16 MS16;
    };

    UINT8 U8[4];
}UNION32;

// A union type for byte, word, or dword access for 64 bit values.
typedef union _UNION64 
{
    UINT32 U32[2];
    UINT16 U16[4];
    UINT8  U8[8];
}UNION64;
     
#endif // MYDEF_H



