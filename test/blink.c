#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define USI_SCK PB2
#define USI_DI PB0
#define USI_DO PB1
#define RESET PB5
#define PWM_OUT PB4
#define TRUE 1
#define FALSE 0

static volatile uint8_t changeCounter = 0;
static volatile uint8_t dutyCycle = 0;
static volatile uint8_t storedData = 0;
static volatile uint8_t transferInProgress = FALSE;
const uint8_t MAX_DUTY_CYCLE = 249;

ISR(TIMER1_COMPB_vect) {
    OCR1B = dutyCycle;
}

ISR(USI_START_vect) {
    transferInProgress = TRUE;
    PORTB ^= (1<<PB3);
    USISR |= (1<<USISIF);
}

// handle SPI transaction complete
ISR(USI_OVF_vect) {
    // clearing interrupt flags
    USISR |= (1<<USIOIF);
    transferInProgress = FALSE;
    dutyCycle = USIDR;
    if (dutyCycle > MAX_DUTY_CYCLE) dutyCycle = MAX_DUTY_CYCLE;
}

int spiWrite(uint8_t data) {
    if (transferInProgress) {
        return 1; // TODO: Wait for transfer complete?
    }
    USIDR = data;
    return 0;
}

int main() {

    // set PB3 as output and initialize to LOW
    DDRB |= (1<<PB3);
    PORTB &= ~(1<<PB3);

    // set prescaler to 8
    TCCR1 |= (1<<CS12);

    // set PB4 as output and initialize to LOW
    DDRB |= (1<<PB4);
    PORTB &= ~(1<<PB4);

    // set PB4 to PWM output and setup OC1B counter to be high starting from 0
    GTCCR |= (1<<PWM1B) | (1<<COM1B1);

    // set TOP value
    OCR1C = 249;

    // set initial duty cycle
    OCR1B = dutyCycle;

    // interrupt when OC1B reaches OCR1B
    TIMSK |= (1<<OCIE1B);

    // set USI_DO pin as output
    DDRB |= (1<<USI_DO);

    // interrupt for SPI transfer begin and complete, setting external clock, 3 wire mode
    USICR |= (1<<USIOIE) | (1<<USISIE) | (1<<USICS1) | (1<<USIWM0);

    // enable interrupts
    sei();

    while (1) {
        //spiWrite(1);
        _delay_ms(100);
    }
    return 0;
}