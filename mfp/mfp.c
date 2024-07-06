//
// firmware for b68k CPU board MFP chip
//   https://github.com/Lougous/b68k-cpu
//
// target: PIC18F27K42
// 

// PIC18F24K42 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (HS (crystal oscillator) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT enabled regardless of sleep)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include <stdint.h>
#include <stdio.h>
//#include <string.h>

#define _XTAL_FREQ       64000000UL  // 16 MHz (default configuration)

// debug UART
void putch(char data) {
    while( ! U1ERRIRbits.TXMTIF)          // wait until the transmitter is ready
        continue;
    U1TXB = data;                     // send one character
}

#define CEn              (PORTBbits.RB0)

#define RSTn             (PORTBbits.RB3)
#define ASSERT_RSTn      TRISBbits.TRISB3 = 0;   // output - RST# asserted
#define DEASSERT_RSTn    TRISBbits.TRISB3 = 1;   // input - RST# deasserted

#define ACKn             (PORTBbits.RB4)
#define ASSERT_ACKn      TRISBbits.TRISB4 = 0;   // output - ACK# asserted
#define DEASSERT_ACKn    TRISBbits.TRISB4 = 1;   // input - ACK# deasserted

#define IRQn             (PORTBbits.RB5)
#define ASSERT_IRQn      TRISBbits.TRISB5 = 0;   // output - IRQ# asserted
#define DEASSERT_IRQn    TRISBbits.TRISB5 = 1;   // input - IRQ# deasserted

#define WEn              (PORTCbits.RC0)

#define A0               (PORTCbits.RC1)

#define SYS_IRQn             (PORTCbits.RC2)
#define ASSERT_SYS_IRQn      TRISCbits.TRISC2 = 0;   // output - IRQ# asserted
#define DEASSERT_SYS_IRQn    TRISCbits.TRISC2 = 1;   // input - IRQ# deasserted

//#define IOEXT0           (PORTCbits.RC2)
#define IOEXT1           (PORTCbits.RC3)
#define IOEXT2_TX        (PORTCbits.RC7)
#define IOEXT3_RX        (PORTCbits.RC6)

#define BSTP             IOEXT3_RX

#define KB_DAT           (PORTCbits.RC4)
#define KB_CLK           (PORTBbits.RB1)

#define MS_DAT           (PORTCbits.RC5)
#define MS_DAT_DIR       (TRISCbits.TRISC5)

#define MS_CLK           (PORTBbits.RB2)
#define MS_CLK_DIR       (TRISBbits.TRISB2)

// boot file content
#include "../../../software/system/kernel/system-img.c"
#include "../../../software/bootstrap/bootstrap-img.c"

volatile unsigned short kbd_dat;
volatile unsigned short kbd_sin;

volatile unsigned char  ms_recv_mode;
volatile unsigned short ms_dat;
volatile unsigned short ms_sin;
volatile unsigned short ms_parity;
volatile unsigned short ms_cmd;
volatile unsigned char  ms_bcnt;

volatile unsigned char uart_rxbuf[256];
volatile unsigned char uart_rxb_w;
volatile unsigned char uart_rxb_r;

#define UART_RUDF_BIT    0x01     // receive FIFO underflow
#define UART_ROVF_BIT    0x02     // receive FIFO overflow
volatile unsigned char uart_sts;

#define IRQ_RTC_MASK   0x01
#define IRQ_KBD_MASK   0x02
#define IRQ_MS_MASK    0x04
#define IRQ_UART_MASK  0x08
volatile unsigned char irq_cfg;
volatile unsigned char irq_sts;

#define SYS_TMR_IRQ_MASK   0x01
#define SYS_SELF_IRQ_MASK  0x40
#define SYS_RESET_MASK     0x80
volatile unsigned char sys_cfg;
volatile unsigned char sys_sts;

// interrupt handler for external interrupt #1
void __interrupt (irq(INT1),high_priority) _IRQ_handler_keyboard(void)
{
    uint8_t din = KB_DAT;

    // kind of SW filtering
    // ensure clock is really low: it should always be, but sometimes not ...
	if (KB_CLK == 0) {
		// interrupt 1: keyboard
		kbd_sin = (((unsigned short)din) << 15) | (kbd_sin >> 1);

        if (!(kbd_sin & 32)) {
            if (irq_cfg & IRQ_KBD_MASK) {
                ASSERT_IRQn;
                irq_sts |= IRQ_KBD_MASK;
            }
            
			kbd_dat = kbd_sin;
			kbd_sin = 0xffff;
		}

	}
    
	// clear interrupt
	PIR5bits.INT1IF = 0;
}

// interrupt handler for external interrupt #1
void __interrupt (irq(INT2),high_priority) _IRQ_handler_mouse(void)
{
    uint8_t din = MS_DAT;
    
    // kind of SW filtering
    // ensure clock is really low: it should always be, but sometimes not ...
	if (MS_CLK == 0) {

        if (ms_recv_mode == 1) {
        	// mouse => microcontroller
        	ms_sin = (((unsigned short)din) << 15) | (ms_sin >> 1);

            if (!(ms_sin & 32)) {
                if (irq_cfg & IRQ_MS_MASK) {
                    ASSERT_IRQn;
                    irq_sts |= IRQ_MS_MASK;
                }
            
                ms_dat = ms_sin;
                ms_sin = 0xffff;
            }

        } else {
            // microcontroller => mouse
            MS_DAT_DIR = ms_sin & 1;
            ms_sin = 0x8000 | (ms_sin >> 1);

            ms_bcnt++;
            
            if (ms_bcnt == 11) {
            	// end of message
                //__delay_ms(100);
                ms_recv_mode = 1;
            }
        }
    }
    
    // clear interrupt
	PIR7bits.INT2IF = 0;
}


// interrupt handler for timer 2: mouse command send start
void __interrupt(irq(IRQ_TMR2),high_priority) _IRQ_handler_TMR2(void)
{
    // Clear the interrupt flag
    PIR4bits.TMR2IF = 0;
 
    // re-enable external interrupt as next clock pulses will be initiated by 
    // the device
	PIR7bits.INT2IF = 0;
    PIE7bits.INT2IE = 1;
    
    // drive data low - start bit
    MS_DAT_DIR = 0;  // data = output

    __delay_us(10);
    
    // release the clock
    MS_CLK_DIR = 1;  // clock = input

    // rest of emission managed with interrupt generated by clock falling edge
}

// interrupt handler for RTC
void __interrupt(irq(IRQ_TMR0),high_priority) _IRQ_handler_TMR0(void)
{
    // Clear the interrupt flag
    PIR3bits.TMR0IF = 0;
     
    // rise IRQ (or not))
    if (irq_cfg & IRQ_RTC_MASK) {
        irq_sts |= IRQ_RTC_MASK;
        ASSERT_IRQn;
    }
}

// interrupt handler for system timer
void __interrupt(irq(IRQ_TMR1),high_priority) _IRQ_handler_TMR1(void)
{
    // restart timer counter (useful for self interrupt only)
    TMR1L = 0;
    TMR1H = 0;

    // Clear the interrupt flag
    PIR4bits.TMR1IF = 0;
     
    // rise IRQ (or not))
    if (sys_cfg & SYS_TMR_IRQ_MASK) {
        sys_sts |= SYS_TMR_IRQ_MASK;
        ASSERT_SYS_IRQn;
    }
    
}

// interrupt handler for serial input
void __interrupt (irq(U1RX),high_priority) _Interrupt(void) {
    if (uart_rxb_w + 1 == uart_rxb_r) {
        uart_sts |= UART_ROVF_BIT;
    } else {
        uart_rxbuf[uart_rxb_w++] = U1RXB;
    }

    //uart_sts &= ~UART_REMPTY_BIT;
    
    if (irq_cfg & IRQ_UART_MASK) {
        irq_sts |= IRQ_UART_MASK;
        ASSERT_IRQn;
    }
}

void main(void) {
    
    ////////////////////////////////////////////////////////////////////////////
    // pins configuration
    ////////////////////////////////////////////////////////////////////////////
    // enable digital mode for all IOs
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    LATBbits.LATB3 = 0;     // GND = RST# asserted (when output)
    LATBbits.LATB4 = 0;     // GND = ACK# asserted (when output)
    LATBbits.LATB5 = 0;     // GND = IRQ# asserted (when output)
    LATCbits.LATC2 = 0;     // GND = SYS_IRQ# asserted (when output)

    // hold board in reset
    ASSERT_RSTn;
    DEASSERT_IRQn;
    DEASSERT_SYS_IRQn;
    DEASSERT_ACKn;
    
    ////////////////////////////////////////////////////////////////////////////
    // chip configuration
    ////////////////////////////////////////////////////////////////////////////
    OSCCON1 = 0x70;
    
    ////////////////////////////////////////////////////////////////////////////
    // debug UART
    ////////////////////////////////////////////////////////////////////////////
    uart_rxb_w = 0;
    uart_rxb_r = 0;
    uart_sts   = 0;
    
    // init uart 1
    U1CON0bits.MODE=0x0; //0x0=8bit
    U1CON0bits.RXEN=0; //disable receiver
    U1CON0bits.TXEN=0; //disable transmitter
    U1CON0bits.ABDEN=0; //disable auto baud
    U1CON0bits.BRGS=0; //baud high speed mode (4 clocks) 0=low speed (16 clocks) (recommended 0)

    //setup control register 1
    U1CON1bits.SENDB=0; //break transmission disabled
    U1CON1bits.BRKOVR=0; //break transmission disabled
    U1CON1bits.RXBIMD=0; //receive break interrupt mode 
    U1CON1bits.WUE=0; //wake up enabled bit 0=receiver operates normally
    U1CON1bits.ON=0; //serial port disabled for now

    //setup control register 2
    U1CON2bits.FLO=0x0; //no flow control
    U1CON2bits.TXPOL=0; //transmit polarity control bit 0=not inverted
    U1CON2bits.C0EN=0; //checksum mode select 
    U1CON2bits.STP=0x0; //stopbit mode control 0=1 stopbit
    U1CON2bits.RXPOL=0; //receive polarity control bit 0=not inverted 
    U1CON2bits.RUNOVF=0; //run during overflow control bit

    //setup baud rate
    U1BRGL=34;             // BRGS = 0 => BRG = (fCPU / (16 * baud)) - 1
    U1BRGH=0;

#if 0
    //set uart 1 pins to extension connector (J5)
    // TX: RC7 (pin 18)
    TRISCbits.TRISC7 = 0;   // output
    RC7PPS=0b00010011;
  
    // RX: RC6 (pin 17)
    TRISCbits.TRISC6 = 1;   // input
    U1RXPPS=0b00010110;
#else
    //set uart 1 pins to debug connector (J7)
    // TX: RB6 (pin 27)
    TRISBbits.TRISB6 = 0;   // output
    RB6PPS=0b00010011;
  
    // RX: RB7 (pin 28)
    TRISBbits.TRISB7 = 1;   // input
    U1RXPPS=0b00001111;    
#endif
    
    // enable interrupt
    PIE3bits.U1RXIE = 1;
    
    // enable
    U1CON1bits.ON=1;   //serial port enabled
    U1CON0bits.TXEN=1;
    U1CON0bits.RXEN=1;

    
    ////////////////////////////////////////////////////////////////////////////
    // keyboard GPIO + external interrupt
    ////////////////////////////////////////////////////////////////////////////
    kbd_dat = 0xffff;
	kbd_sin = 0xffff;

    INT1PPS = 0b01001;  // RB1 (pin 22)
	TRISCbits.TRISC4  = 1;  // RC4 (pin 15) is input (PS/2 data)
    
	INTCON0bits.INT1EDG = 0;  // interrupt on falling edge
//	INTCON0bits.INT1EDG = 1;  // interrupt on rising edge
	IPR5bits.INT1IP     = 1;  // high priority
	PIE5bits.INT1IE     = 1;  // interrupt enabled

    
    ////////////////////////////////////////////////////////////////////////////
    // mouse GPIO + external interrupt
    ////////////////////////////////////////////////////////////////////////////
    ms_dat = 0xffff;
	ms_sin = 0xffff;
	ms_recv_mode = 1;
    
    INT2PPS = 0b01010;   // RB2 (pin 23)
    LATBbits.LATB2 = 0;  // PS/2 clock : GND = asserted (when output)
	MS_DAT_DIR     = 1;  // RC5 (pin 16) is input (PS/2 data)
    LATBbits.LATB5 = 0;  // PS/2 data : GND = asserted (when output)
    
	INTCON0bits.INT2EDG = 0;  // interrupt on falling edge
//	INTCON0bits.INT2EDG = 1;  // interrupt on rising edge
	IPR7bits.INT2IP     = 1;  // high priority
	PIE7bits.INT2IE     = 1;  // interrupt enabled


    ////////////////////////////////////////////////////////////////////////////
    // RTC timer + interrupt
    ////////////////////////////////////////////////////////////////////////////
    irq_cfg = 0;
    irq_sts = 0;
    
    T0CON0bits.EN   = 1;  // Enable TMR0
    T0CON0bits.MD16 = 0;  // 8-bits mode
    T0CON1bits.CS   = 5;  // 500 kHz
    T0CON1bits.CKPS = 1;  // 1 => 1:2 => ~ 1kHz
    
    TMR0H = 249;
//    TMR0L = 0;
    
    IPR3bits.TMR0IP = 1;  // Make TMR0 interrupt high priority
    PIE3bits.TMR0IE = 1;  // Enable TMR0 interrupt
    
    ////////////////////////////////////////////////////////////////////////////
    // system timer + interrupt
    ////////////////////////////////////////////////////////////////////////////
    sys_cfg = 0;
    sys_sts = 0;
    
    T1CLKbits.CS   = 1;  // Fosc/4  16MHz

    T1GCON   = 0;  // no gating

    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.CKPS = 0;  // 0 => 1:1
    
    // period: 64k x 62.5ns ~= 4 ms
    
    T1CONbits.ON   = 1;  // Enable TMR1
  
    IPR4bits.TMR1IP = 1;  // Make TMR1 interrupt high priority
    PIE4bits.TMR1IE = 1;  // Enable TMR1 interrupt

    ////////////////////////////////////////////////////////////////////////////
    // one shot timer for PS/2 mouse commands
    ////////////////////////////////////////////////////////////////////////////
    T2CLKbits.CS = 5;     // MFINTOSC (500 kHz)
    T2PR = 100;           // 200 us
    T2TMR = 0;
    T2HLTbits.MODE = 8;   // one shot
    IPR4bits.TMR2IP = 1;  // Make TMR2 interrupt high priority
    PIE4bits.TMR2IE = 1;  // Enable TMR2 interrupt
    
    ////////////////////////////////////////////////////////////////////////////
    // delay for PSU ramp-up
    ////////////////////////////////////////////////////////////////////////////
    __delay_ms(500);

    ////////////////////////////////////////////////////////////////////////////
    // general interrupt
    ////////////////////////////////////////////////////////////////////////////
    INTCON0bits.IPEN = 0;  // high/low priority
    INTCON0bits.GIE = 1;   // enable unmasked interrupts
     
    ////////////////////////////////////////////////////////////////////////////
    // allows some time to select bootstrap by pressing 's' on keyboard
    ////////////////////////////////////////////////////////////////////////////
    __delay_ms(500);
   
    const unsigned char *flash_base = (const unsigned char *)&system_bin[0];
    unsigned char dbg_mode = 0;
    
    if (((unsigned char)(kbd_dat >> 6)) == 0x1B) {
        // PS/2 code for 's'
        printf("[MFP] bootstrap ROM\n");
        flash_base = (const unsigned char *)&bootstrap_bin[0];
        dbg_mode = 1;
    } else if (((unsigned char)(kbd_dat >> 6)) == 0x23) {
        // PS/2 code for 'd'
        dbg_mode = 1;        
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // bootstrap
    ////////////////////////////////////////////////////////////////////////////
    unsigned short flash_address = 0;

    while (flash_address < 512) {
        
    if (dbg_mode) printf("[MFP] +rst\n");
    ASSERT_RSTn;
    DEASSERT_ACKn;
    DEASSERT_IRQn;
    DEASSERT_SYS_IRQn;

    __delay_ms(300);  // at least 100 ms
    
    if (dbg_mode) printf("[MFP] -rst\n");
    DEASSERT_RSTn;
    
     // ensure reset not asserted by someone else
    while (! RSTn);

    __delay_us(1);  // must not exceed tRAS max (10 us)

    // read bootstrap option pin

#define ACK_PULSE_READ_EXTRA  { TRISA = 0; TRISA = 0;  ASSERT_ACKn; while (! CEn); __delay_us(2); TRISA = 0xFF;  DEASSERT_ACKn; }
#define ACK_PULSE_READ  { TRISA = 0;  ASSERT_ACKn; while (! CEn); TRISA = 0xFF;  DEASSERT_ACKn; }
#define ACK_PULSE_WRITE  { ASSERT_ACKn; while (! CEn); DEASSERT_ACKn; }

    // load 512 bytes boot
    for (flash_address = 0; flash_address < 512; flash_address++) {
       
        if (! CEn) {
            // swap odd/even (first DMA access with LDS# => MSB)
            LATA = flash_base[(flash_address & 0xFFFE) | ((~flash_address) & 1)];
            
            // read data needs to be hold on bus for up to 500 ns (8 MHz CPU clock)
            // this is specific to boot strap IO access
            ACK_PULSE_READ_EXTRA;
            
            __delay_us(2);
        } else {
            if (dbg_mode) printf("[MFP] boot stopped: %Xh\n", flash_address);
            __delay_ms(1000);
        }
    }
    
    }
    
    if (dbg_mode) printf("[MFP] BS OK\n");

    unsigned char address = 0;
    
    unsigned char post = 0;
    
    while (1) {
        while (CEn);

        //printf("%i @%i=%xh\n", WEn, A0, PORTA);
        
        if (A0 == 0) {
            // data mode
            switch (address) {
                case 0:
                    // B68K_MFP_REG_FLASH_DATA
                    // flash data, read only
                    if (WEn) {
                        LATA = flash_base[flash_address];
                        ACK_PULSE_READ;
                        flash_address++;
                    } else {
                        // cannot write
                        ACK_PULSE_WRITE;
                    }

                    break;
                    
                case 3:
                    // B68K_MFP_REG_TICKS
                    // ticks counter in range 0 to 249, read only
                    if (WEn) {
                        LATA = TMR0H;
                        ACK_PULSE_READ;
                    } else {
                        // cannot write
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                
                case 4:
                    // B68K_MFP_REG_UART_STS
                    if (WEn) {
                        //LATA = (U1ERRIR & 0x80) + (U1FIFO & 0x02);
                        LATA = (U1ERRIR & 0x80) + uart_sts;
                        ACK_PULSE_READ;
                    } else {
                        // reset flags
                        uart_sts &= ~(PORTA);
                        ACK_PULSE_WRITE;
                    }

                    break;
                    
                case 5:
                    // MFP_REG_UART_DATA
                    if (WEn) {
                        LATA = uart_rxbuf[uart_rxb_r];
                        ACK_PULSE_READ;
                        
                        // disable interrupts while messing around with irq_sts
                        // and uart globals
                        INTCON0bits.GIE = 0;
                        
                        if (uart_rxb_w != uart_rxb_r) {
                            uart_rxb_r++;
                        } else {
                            // underflow
                            uart_sts |= UART_RUDF_BIT;
                        }
                        
                        if (uart_rxb_w == uart_rxb_r) {
                            //uart_sts |= UART_REMPTY_BIT;
                            irq_sts &= ~IRQ_UART_MASK;
                            
                            if (irq_sts == 0) {
                                // no more pending interrupts
                                DEASSERT_IRQn;
                            }
                        }
                        
                        INTCON0bits.GIE = 1;
                    } else {
                        U1TXB = PORTA;
                        ACK_PULSE_WRITE;
                    }

                    break;
                    
                case 7:
                    // MFP_REG_POST
                    if (WEn) {
                        LATA = post;
                        ACK_PULSE_READ;
                    } else {
                        post = PORTA;
                        ACK_PULSE_WRITE;
                        printf("[MFP] POST=%02Xh\n", post);
                    }

                    break;
                    
                case 8:
                    // MFP_REG_PS2P0_STS
                    if (WEn) {
                        LATA = (kbd_dat >> 5) & 1;
                        ACK_PULSE_READ;
                    } else {
                        // cannot write
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 9:
                    // MFP_REG_PS2P0_DATA
                    if (WEn) {
                        LATA = (kbd_dat >> 6) & 0xff;
                        ACK_PULSE_READ;
                        kbd_dat = 0xffff;
                    } else {
                        // cannot write
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 10:
                    // MFP_REG_PS2P1_STS
                    if (WEn) {
                        LATA = (ms_dat >> 5) & 1;
                        ACK_PULSE_READ;
                    } else {
                        // latch parity
                        ms_parity = PORTA & 2 ? 0x100 : 0x000;
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 11:
                    // MFP_REG_PS2P1_DATA
                    if (WEn) {
                        LATA = (ms_dat >> 6) & 0xff;
                        ACK_PULSE_READ;
                        ms_dat = 0xffff;
                    } else {
                        // cannot write
                        if (ms_recv_mode) {
                            ms_cmd = (unsigned short)PORTA;
                            ACK_PULSE_WRITE;
                        
                            // disable external interrupt so that clock state
                            // can be set to 0 by host without raising external
                            // interrupt
                            PIE7bits.INT2IE = 0;
	
                            // data to serialize out (with parity)
                            ms_sin = ms_cmd | ms_parity | 0xfe00;
    
                            ms_recv_mode = 0;
                            ms_bcnt = 0;
	
                            // drive clock low
                            MS_CLK_DIR = 0;  // clock = output

                            // start 100 us+ timer
                            T2CON = 0x80;
                        } else {
                            ACK_PULSE_WRITE;
                        }

                        // remaining operations will be clock-interrupt driven
                    }
                    
                    break;
                    
                case 12:
                    // B68K_MFP_REG_SYSCFG
                    if (WEn) {
                        LATA = sys_cfg;
                        ACK_PULSE_READ;
                    } else {
                        sys_cfg = PORTA;
    
                        if (sys_cfg & SYS_SELF_IRQ_MASK) {
                            // self immediate interrupt request
                            sys_cfg &= ~SYS_SELF_IRQ_MASK;  // clear request bit
                            INTCON0bits.GIE = 0;
                            ASSERT_SYS_IRQn;
                            sys_sts |= SYS_SELF_IRQ_MASK;
                            INTCON0bits.GIE = 1;
                        }
    
                        ACK_PULSE_WRITE;
                        
                        if (sys_cfg & SYS_RESET_MASK) {
                            // reset request
                            printf("[MFP] reset !\n");
                            RESET();
                        }
                    }
                    break;
                    
                case 13:
                    // B68K_MFP_REG_SYSSTS
                    if (WEn) {
                        LATA = sys_sts;
                        ACK_PULSE_READ;
                    } else {
                        unsigned char sts = PORTA;
                        ACK_PULSE_WRITE;
                        
                        // disable interrupts while messing around with sys_sts
                        INTCON0bits.GIE = 0;

                        sys_sts &= ~sts;
                        
                        //printf("[MFP] sts=%Xh\n", sys_sts);

                        if (sys_sts == 0) {
                            // no more pending interrupts
                            DEASSERT_SYS_IRQn;
                        }
                        
                        INTCON0bits.GIE = 1;
                    }
                        
                    break;
                    
                case 14:
                    // B68K_MFP_REG_IRQCFG
                    if (WEn) {
                        LATA = irq_cfg;
                        ACK_PULSE_READ;
                    } else {
                        irq_cfg = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                        
                    break;
                    
                case 15:
                    // B68K_MFP_REG_IRQSTS
                    if (WEn) {
                        LATA = irq_sts;
                        ACK_PULSE_READ;
                    } else {
                        unsigned char sts = PORTA;
                        
                        // disable interrupts while messing around with irq_sts
                        INTCON0bits.GIE = 0;

                        ACK_PULSE_WRITE;

                        irq_sts &= ~sts;
                        
                        //printf("[MFP] sts=%Xh\n", irq_sts);

                        if (irq_sts == 0) {
                            // no more pending interrupts
                            DEASSERT_IRQn;
                        }
                        
                        INTCON0bits.GIE = 1;
                    }
                        
                    break;
                    
                default:
                    // 
//                    if (dbg_mode) { 
                        printf("[MFP] bad @ %Xh\n", address);
                        while(1);
//                    }
                    break;
                    
            }
        } else {
            // change/read address
            if (WEn == 0) {
                // write
                address = PORTA;
                ACK_PULSE_WRITE;
                //printf("w@%i\n", address);
            } else {
                // read
                //printf("r@%i\n", address);
                LATA = address;
                ACK_PULSE_READ;
            }
        }
        
    }
        
}
