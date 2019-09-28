#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <avr/io.h>

#include <IRremote.h>

#define IR_PORT PORTB
#define IR_PIN PINB
#define IR_DDR DDRB
#define IR_BV _BV(1)
#define IR_OCR OCR1A
#define IR_TCCRnA TCCR1A
#define IR_TCCRnB TCCR1B
#define IR_TCNTn TCNT1
#define IR_TIFRn TIFR1
#define IR_TIMSKn TIMSK1
#define IR_TOIEn TOIE1
#define IR_ICRn ICR1
#define IR_OCRn OCR1A
#define IR_COMn0 COM1A0
#define IR_COMn1 COM1A1
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

const int DENON_POWA[68] = {0x0000, 0x006d, 0x0000, 0x0020, 0x000b, 0x001d, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x06f0, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x001e, 0x000a, 0x001e, 0x000a, 0x0046, 0x000a, 0x0046, 0x000a, 0x0651};

void ir_on()
{
  IR_TCCRnA |= (1<<IR_COMn1) + (1<<IR_COMn0);
  ir_led_state = 1;
}

void ir_off()
{
  IR_TCCRnA &= ((~(1<<IR_COMn1)) & (~(1<<IR_COMn0)) );
  ir_led_state = 0;
}

void ir_toggle()
{
  if (ir_led_state)
    ir_off();
  else
    ir_on();
}

void ir_start(uint16_t *code)
{
  ir_code = code;
  IR_PORT &= ~IR_BV; // Turn output off
  IR_DDR |= IR_BV; // Set it as output
  IR_TCCRnA = 0x00; // Reset the pwm
  IR_TCCRnB = 0x00;
  //printf_P(PSTR("FREQ CODE: %hd\r\n"), code[PRONTO_FREQ_CODE]);
  uint16_t top = ( (F_CPU/1000000.0) * code[PRONTO_FREQ_CODE] * 0.241246 ) - 1;
  //printf_P(PSTR("top: %hu\n\r"), top);
  IR_ICRn = top;
  IR_OCRn = top >> 1;
  IR_TCCRnA = (1<<WGM11);
  IR_TCCRnB = (1<<WGM13) | (1<<WGM12);
  IR_TCNTn = 0x0000;
  IR_TIFRn = 0x00;
  IR_TIMSKn = 1 << IR_TOIEn;
  ir_seq_index = PRONTO_CODE_START;
  ir_cycle_count = 0;
  ir_on();
  IR_TCCRnB |= (1<<CS10);
}

#define TOTAL_CYCLES 80000 // Turns off after this number of
// cycles. About 2 seconds
// FIXME: Turn off after having sent
ISR(TIMER1_OVF_vect) {
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

void ir_stop()
{
  IR_TCCRnA = 0x00; // Reset the pwm
  IR_TCCRnB = 0x00;
}

const uint16_t inputLength = 512;

void setup() {
  Serial.begin(9600);
  Serial.println("READY");

  irrecv.enableIRIn();
}

void loop()
{
  if (irrecv.decode(&results) && results.decode_type == NEC) {
    Serial.print("Received code NEC ");
    Serial.println(results.value, HEX);

    if(results.value == 0x20DF10EF) {
      // powa
      Serial.println("Sent Denon power code");
      ir_start(DENON_POWA);
    } /*else if(results.value == 0x20DF40BF) {
      // volume up
    } else if(results.value == 0x20DFC03F) {
      // volume down
    }*/
    
    irrecv.resume();
  }
}
