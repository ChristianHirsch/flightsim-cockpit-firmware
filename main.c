/* Name: main.c
 * Project: hid-mouse, a very simple HID example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.

We use VID/PID 0x046D/0xC00E which is taken from a Logitech mouse. Don't
publish any hardware using these IDs! This is for demonstration only!
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[44] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (GamePad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xA1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM
    0x29, 0x08,                    //     USAGE_MAXIMUM
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0xff,                    //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xC0,                          //   END_COLLECTION
    0xC0,                          // END COLLECTION
};
/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */

int  p_in[6] = {PC4, PC3, PC2, PC1, PB2, PC0};
volatile uint8_t * const p_ip[6] = {&PINC, &PINC, &PINC, &PINC, &PINB, &PINC};
int  p_ou[5] = {PD1, PD0, PC5, PB5, PB4};
volatile uint8_t * const p_op[5] = {&PORTD, &PORTD, &PORTC, &PORTB, &PORTB};

typedef struct{
    uchar   switches;
    uchar   encoder[2];
}report_t;

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

void init_inputs()
{
    // output ^= 1, input ^= 0

    // PORTB
    DDRB &= ~(0x07);    // set PB0, PB1, PB2 to input
    DDRB |=  (0x38);    // set PB3, PB4, PB5 to output
    
    // PORTC
    DDRC &= ~(0x1F);    // set PC0-4 to input
    DDRC |=  (0x20);    // set PC5 to output
    
    // PORTD
    DDRD &= ~(0xF0);    // set PD4-7 to input
    DDRD |=  (0x03);    // set PD0-1 to output

    reportBuffer.switches = 0x00;
    reportBuffer.encoder[0] = 0xFF / 2;
    reportBuffer.encoder[1] = 0xFF / 2;
}

void init_pwm()
{
    DDRB  |=  (1 << PB3); // done in init inputs
    //PORTB &= ~(1 << PB3);

    // set PWM for 50% duty cycle
    OCR2A = 128;

    // set non-inverting mode
    //TCCR2A |= (1 << COM0A1);

    // set fast PWM Mode
    TCCR2A |= (1 << WGM21) | (1 << WGM20);

    // set prescaler to 8 and starts PWM
    TCCR2B |= (1 << CS21);
}

void read_inputs()
{
    for(int i=0; i<6; i++)
    {
        if(*p_ip[i] & (1 << p_in[i]))
        {
            reportBuffer.switches |=  (1<<i);
        }
        else
        {
            reportBuffer.switches &= ~(1<<i);
        }
    }

    static char pd4_debounce = 0;
    static char pd4_set = 0;
    static char pd5_debounce = 0;
    static char pd5_set = 0;

    if(PIND & (1 << PD4))
        pd4_debounce++;
    else
    {
        pd4_set = 0;
        pd4_debounce = 0;
    }

    if(PIND & (1 << PD5))
        pd5_debounce++;
    else
    {
        pd5_set = 0;
        pd5_debounce = 0;
    }

    if(pd4_debounce > 3 && !pd4_set)
    {
        pd4_set = 1;
        reportBuffer.switches ^= (1 << 6);
    }
    
    if(pd5_debounce > 3 && !pd5_set)
    {
        pd5_set = 1;
        reportBuffer.switches ^= (1 << 7);
    }
}

void write_outputs()
{
    for(int i=0; i < 5; i++)
    {
        if(reportBuffer.switches & (1 << i))
            *(p_op[i]) |=  (1<<p_ou[i]);
        else
            *(p_op[i]) &= ~(1<<p_ou[i]);
    }

    // set PWM value
    OCR2A = reportBuffer.encoder[0];
    // enable PWM on PB3
    if(reportBuffer.switches & (1 << 6))
        TCCR2A |= 1<<COM2A1;
    // disable PWM on PB3
    else
        TCCR2A &= ~(1<<COM2A1);
}

void read_encoder1()
{
    static int encoder_last = 0;
    int pin_a = (PINB & (1 << PB0)) != 0;
    int pin_b = (PINB & (1 << PB1)) != 0;
    int encoder_value = (pin_a | (pin_b << 1)) * 3;
    encoder_value = ((encoder_value & 0x01) << 1) | ((encoder_value & 0x02) >> 1);

    int diff = encoder_value - encoder_last;
    encoder_last = encoder_value;
    if(diff != 0)
    {
        if(diff == -1 || diff > 1)
        {
            if(reportBuffer.encoder[0] > 0)
                reportBuffer.encoder[0]--;
        }
        else if(diff == 1 || diff < -1)
        {
            if(reportBuffer.encoder[0] < 255)
                reportBuffer.encoder[0]++;
        }

    }
}

void read_encoder2()
{
    static int encoder_last = 0;
    int pin_a = (PIND & (1 << PD6)) != 0;
    int pin_b = (PIND & (1 << PD7)) != 0;
    int encoder_value = (pin_a | (pin_b << 1)) * 3;
    encoder_value = ((encoder_value & 0x01) << 1) | ((encoder_value & 0x02) >> 1);

    int diff = encoder_value - encoder_last;
    encoder_last = encoder_value;
    if(diff != 0)
    {
        if(diff == -1 || diff > 1)
        {
            if(reportBuffer.encoder[1] > 0)
                reportBuffer.encoder[1]--;
        }
        else if(diff == 1 || diff < -1)
        {
            if(reportBuffer.encoder[1] < 255)
                reportBuffer.encoder[1]++;
        }

    }
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    uchar   i;
    init_inputs();

    wdt_enable(WDTO_1S);
    /* If you don't use the watchdog, replace the call above with a wdt_disable().
     * On newer devices, the status of the watchdog (on/off, period) is PRESERVED
     * OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    odDebugInit();
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */

    init_pwm();

    /* main event loop */
    for(;;)
    {
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */

        read_inputs();
        read_encoder1();
        read_encoder2();
        write_outputs();

        wdt_reset();
        usbPoll();
        if(usbInterruptIsReady())
        {
            /* called after every poll of the interrupt endpoint */
            DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */

            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        }
    }
}

/* ------------------------------------------------------------------------- */
