#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#define DELAY_TIME  1

unsigned char secondLow=0,secondHigh=0,minuteLow=0,minuteHigh=0,hourLow=0,hourHigh=0;

ISR(TIMER1_COMPA_vect)
{
    secondLow++;
	if (secondLow==10)
    {
        secondLow=0;
        secondHigh++;
    }
    if (secondHigh==6 && secondLow ==0) {
        secondHigh=0;
        minuteLow++;
    }
    if (minuteLow==10) {
        minuteLow=0;
        minuteHigh++;
    }
    if (minuteHigh==6 && minuteLow ==0) {
        minuteHigh=0;
        hourLow++;
    }
    if (hourLow==10) {
        hourLow=0;
        hourHigh++;
    }
    if (hourHigh==6 && hourLow ==0) {
        secondLow=secondHigh=minuteLow=minuteHigh=hourLow=hourHigh=0;
    }

}


/* Configure Interrupt INT0 with falling edge. Connect a push button with pull-up. If a falling edge detected the Stop Watch time should be reset */
ISR(INT0_vect)
{
    secondLow = secondHigh = minuteLow = minuteHigh = hourLow = hourHigh =0;
}

/* Configure Interrupt INT1 with raising edge. Connect a push button with pull-down. If a raising edge detected the Stop Watch time should be paused. */
ISR(INT1_vect){
    TCCR1B&= ~(1<<CS12)&~(1<<CS11)&~(1<<CS10);                  // Stop clock
}

/* Configure Interrupt INT2 with falling edge. Connect a push button with pull-up. If a falling edge detected the Stop Watch time should be resumed. */
ISR(INT2_vect){
    TCCR1B |= (1<<CS12) | (1<<CS10);                  // resume counting
}


void init()
{
    MCUCSR&= ~(1<<ISC2);            	// falling edge for interrupt int2
    MCUCR|=(1<<ISC01);              // falling edge for interrupt int0
    MCUCR|=(1<<ISC11)|(1<<ISC10);  // rising edge for interrupt int1
    //timer 
    TCCR1A = (1<<FOC1A);                            // non pwm mode
    TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12);   // 1024 prescalar, compare mode
    TCNT1 = 0;               
    OCR1A = 977;            // Compare value
    TIMSK |= (1<<OCIE1A);    // Enable Module Interrupt
    GICR|=(1<<INT1)|(1<<INT2)|(1<<INT0);  // Enable interrupt request for each
    SREG |= (1<<7);            // Enable I-BIT
}

/* Project:
*clock is 1Mhz.
*6 common anode 7 segments connected to 7447, which is connected to PC0:3.
*each 7seg must be multiplexed using its enable through pins connected to the MCU PA0:7.
*Configure Interrupt INT0 with falling edge. Connect a push button with pull-up. If a falling edge detected the Stop Watch time should be reset.
*Configure Interrupt INT1 with raising edge. Connect a push button with pull-down. If a raising edge detected the Stop Watch time should be paused.
*Configure Interrupt INT2 with falling edge. Connect a push button with pull-up. If a falling edge detected the Stop Watch time should be resumed.
 */
// int0 -> PD2, int1 -> PD3, int2 -> PB2
int main(void)
{
    //PORTC -> DECODER
    //PORTA -> 7SEGMENT starting from 0:6  0x3f
    DDRB&= ~(1<<2);                              // input button at PB2 for int2
    DDRD&= ~(1<<2)&~(1<<3);                          // input button at PD2,3 for int0,int1
    PORTD|=(1<<2);                           // SET pull up for PD2
    PORTB|=(1<<2);                               // SET pull up for PB2

    DDRC|=0X0F;                                 // set output pins for the 7seg decoder
    PORTC&= 0XF0;

    DDRA|=0X3F;                             // SET output enable pins for the 7 segments
    PORTA|=0X3F;

    init();


    for (;;) 
    {
        PORTA&= ~0X3F;                         // turn off all 7segment
        PORTC=(DDRC&0XF0)|secondLow;                 // set decoder input to equal secondLow
        PORTA|= (1<<0);
        _delay_ms(DELAY_TIME);
        PORTA&= ~0X3F;                         // turn off all 7segments
        PORTC=(DDRC&0XF0)|secondHigh;                 // set decoder input to equal secondLow
        PORTA|= (1<<1);
        _delay_ms(DELAY_TIME);
        PORTA&= ~0X3F;                         // turn off all 7segments
        PORTC=(DDRC&0XF0)|minuteLow;                 // set decoder input to equal secondLow
        PORTA|= (1<<2);
        _delay_ms(DELAY_TIME);
        
        PORTA&= ~0X3F;                         // turn off all 7segments
        PORTC=(DDRC&0XF0)|minuteHigh;                 // set decoder input to equal secondLow
        PORTA|= (1<<3);
        _delay_ms(DELAY_TIME);
        PORTA&= ~0X3F;                         // turn off all 7segments
        PORTC=(DDRC&0XF0)|hourLow;                 // set decoder input to equal secondLow
        PORTA|= (1<<4);
        _delay_ms(DELAY_TIME);
        PORTA&= ~0X3F;                         // turn off all 7segments
        PORTC=(DDRC&0XF0)|hourHigh;                 // set decoder input to equal secondLow
        PORTA|= (1<<5);
        _delay_ms(DELAY_TIME);
    }
}




