#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <avr/io.h>

#include <IRremote.h>

#define PRONTO_IR_SOURCE 0 // Pronto code byte 0
#define PRONTO_FREQ_CODE 1 // Pronto code byte 1
#define PRONTO_SEQUENCE1_LENGTH 2 // Pronto code byte 2
#define PRONTO_SEQUENCE2_LENGTH 3 // Pronto code byte 3
#define PRONTO_CODE_START 4 // Pronto code byte 4

static const uint16_t *ir_code = NULL;
static uint16_t ir_cycle_count = 0;
static uint32_t ir_total_cycle_count = 0;
static uint8_t ir_seq_index = 0;
static uint8_t ir_led_state = 0;

int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;

const uint16_t DENON_POWA[68] = {0x0000, 0x006d, 0x0000, 0x0020, 0x000b, 0x001d, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x06f0, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0651};
const uint16_t DENON_VOL_UP[68]= {0x0000, 0x006d, 0x0000, 0x0020, 0x000b, 0x001d, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x06a1, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x06a1};
const uint16_t DENON_VOL_DOWN[68] = {0x0000, 0x006d, 0x0000, 0x0020, 0x000b, 0x001d, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x06a1, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x06a1};
void ir_on()
{
  TCCR1A |= (1<<COM1A1) + (1<<COM1A0);
  ir_led_state = 1;
}

void ir_off()
{
  TCCR1A &= ((~(1<<COM1A1)) & (~(1<<COM1A0)) );
  ir_led_state = 0;
}

void ir_toggle()
{
  if (ir_led_state)
    ir_off();
  else
    ir_on();
}

void send_pronto(uint16_t *code)
{
  ir_code = code;
  PORTB &= ~_BV(1); // Turn output off
  DDRB |= _BV(1); // Set it as output
  TCCR1A = 0x00; // Reset the pwm
  TCCR1B = 0x00;
  //printf_P(PSTR("FREQ CODE: %hd\r\n"), code[PRONTO_FREQ_CODE]);
  uint16_t top = ( (F_CPU/1000000.0) * code[PRONTO_FREQ_CODE] * 0.241246 ) - 1;
  //printf_P(PSTR("top: %hu\n\r"), top);
  ICR1 = top;
  OCR1A = top >> 1;
  TCCR1A = (1<<WGM11);
  TCCR1B = (1<<WGM13) | (1<<WGM12);
  TCNT1 = 0x0000;
  TIFR1 = 0x00;
  TIMSK1 = 1 << TOIE1;
  ir_seq_index = PRONTO_CODE_START;
  ir_cycle_count = 0;
  ir_on();
  TCCR1B |= (1<<CS10);
}

#define TOTAL_CYCLES 20000 // Turns off after this number of cycles. About 2 seconds
// FIXME: Turn off after having sent
ISR(TIMER1_OVF_vect)
{
  uint16_t sequenceIndexEnd;
  uint16_t repeatSequenceIndexStart;
  ir_total_cycle_count++;
  ir_cycle_count++;

  if (ir_cycle_count== ir_code[ir_seq_index]) {
    ir_toggle();
    ir_cycle_count = 0;
    ir_seq_index++;
    sequenceIndexEnd = PRONTO_CODE_START +
      (ir_code[PRONTO_SEQUENCE1_LENGTH]<<1) +
      (ir_code[PRONTO_SEQUENCE2_LENGTH]<<1);

    repeatSequenceIndexStart = PRONTO_CODE_START +
      (ir_code[PRONTO_SEQUENCE1_LENGTH]<<1);

    if (ir_seq_index >= sequenceIndexEnd ) {
      ir_seq_index = repeatSequenceIndexStart;

      if(ir_total_cycle_count>TOTAL_CYCLES) {
        ir_off();
        TCCR1B &= ~(1<<CS10);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("READY");

  irrecv.enableIRIn();
}

void loop()
{
  if(irrecv.decode(&results)){
    Serial.print("Received code NEC ");
    Serial.println(results.value, HEX);

    if(results.decode_type == NEC) {
      if(results.value == 0x20DF11EE) {
        send_pronto(DENON_POWA);
      } else if(results.value == 0x20DF41BE) {
        send_pronto(DENON_VOL_UP);
      } else if(results.value == 0x20DFC13E) {
        send_pronto(DENON_VOL_DOWN);
      }
    }

    irrecv.resume();
  }
}
