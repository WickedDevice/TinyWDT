#include <SoftwareSerial.h>
#include <util/crc16.h>
#include <avr/wdt.h>
  
#define INITIALIZATION_MESSAGE_LENGTH 6              // 2 uint16_t of payload and 1 uint16 of CRC
#define HEADER_FIRST_BYTE  0x5A
#define HEADER_SECOND_BYTE 0x96

const int host_reset_pin   = 0;                        // PB0 = DIG0
const int configure_tx_pin = 1;                        // PB1 = DIG1 (not connected to anything, defined only for the benefit of SoftSerial 
const int led_pin          = 2;                        // PB2 = DIG2
const int pet_input_pin    = 3;                        // PB3 = DIG3
const int debug_pin        = 4;                        // PB4 = DIG4
boolean debug              = false;
uint8_t debug_state        = 0;

#define LED_SHORT_BLINK_DURATION_MS      (50)
#define PET_WATCHDOG_RELEASE_DURATION_MS (5)

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void){
    MCUSR = 0;
    wdt_disable();
    return;
}
#define soft_reset()        \
do                          \
{                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

uint32_t ms_without_being_pet = 0;
uint32_t min_wait_period_after_petting_ms = 0;
uint32_t maximum_wait_period_after_petting_ms = 0;
volatile uint8_t led_on_duration_ms = 0;
volatile uint8_t check_for_pet_timer_ms = 0;
volatile uint8_t once_per_millisecond_timer_ms = 0;
boolean first_pet = true; // no minimum window constraint on the first pet

// this ISR is set up to fire once a millisecond
// and only impacts 1-byte volatile variables
// in order to avoid the requirement of locking access
const uint8_t timer_preload_value = 131; // 256 - 131 = 125 ticks @ 8MHz/64 = 1ms
ISR(TIM1_OVF_vect){
  TCNT1 = timer_preload_value;
  
  if(led_on_duration_ms > 0){
    led_on_duration_ms--; 
  }
 
  if(check_for_pet_timer_ms > 0){
    check_for_pet_timer_ms--; 
  }    
  
  if(once_per_millisecond_timer_ms > 0){
    once_per_millisecond_timer_ms--; 
  }  
}

SoftwareSerial mySerial(pet_input_pin, configure_tx_pin); // DIG3 = RX, DIG1 = TX (not connected)

uint16_t checkcrc(uint8_t * buffer);
boolean validateInitializationMessage(uint8_t * buffer);
uint16_t bufferToUint16(uint8_t * buffer);
void perform_reset_sequence(void);
void blinkLedFast(uint8_t n);
void blinkLedSlow(uint8_t n);

void setup(){
  uint8_t buffer[INITIALIZATION_MESSAGE_LENGTH];
  uint8_t buffer_index = 0;
  uint8_t header_byte_num = 0;
    
  TCCR1 = 0x07; // divide by 1024
  TCNT1 = timer_preload_value;
  TIMSK = _BV(TOIE1);    
    
  pinMode(host_reset_pin, INPUT);
  pinMode(pet_input_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  blinkLedFast(2);

  if(debug){
    pinMode(debug_pin, OUTPUT); 
    digitalWrite(debug_pin, debug_state);    
  }

  
  mySerial.begin(4800);
  
  for(;;){
  
    // handle what happens the hcheck_for_pet_timer_msost is reset by something else
    if(digitalRead(host_reset_pin) == 0){
      while(digitalRead(host_reset_pin) == 0){
        continue; 
      }
      soft_reset();
    }    
    
    if(mySerial.available()){
      uint8_t rx_char = mySerial.read();
      if(header_byte_num == 0){
        if(rx_char == HEADER_FIRST_BYTE){
          header_byte_num++;
        }
      }
      else if(header_byte_num == 1){
        if(rx_char == HEADER_SECOND_BYTE){
          header_byte_num++;       
        }
        else{         
          header_byte_num = 0; 
        }
      }
      else{
        buffer[buffer_index++] = rx_char;
      }
    }
     
    if(buffer_index == INITIALIZATION_MESSAGE_LENGTH){     
      break; 
    }
  } 
  
  if(validateInitializationMessage(buffer)){
    blinkLedFast(2);  
  }
  else{
    // misconfigured, reset the host
    blinkLedSlow(2);
    perform_reset_sequence();
  }
  
  mySerial.end();  
  
}

void loop(){    
  
  // handle what happens when the host is reset by something else
  if(digitalRead(host_reset_pin) == 0){
    while(digitalRead(host_reset_pin) == 0){
      continue; 
    }
    soft_reset();
  }
  
  // the once per millisecond_timer_ms task runs to
  // increment a counter every millisecond, regardless
  // of other activity, and the counter can be cleared by 
  // the check_for_pet_timer_ms task
  if(once_per_millisecond_timer_ms == 0) {
    once_per_millisecond_timer_ms = 1;
    
    if(debug){
      debug_state = 1 - debug_state;
      digitalWrite(debug_pin, debug_state); 
    }    
    
    ms_without_being_pet++; // increase the counter        
  }
  
  // the check_for_pet_timer_ms task runs to 
  // 1 - determine if the watchdog is currently being pet, and restart the ms_without_being_pet if necessary
  // 2 - issue a reset if an early pet is detected
  // 3 - issue a reset if too long has passed without a pet
  if(check_for_pet_timer_ms == 0){
    check_for_pet_timer_ms = 1;
    
    // issue a reset if either: 
    // CONDITION #1: the input signal is LOW and the counter is lower than minimum_wait_period_after_petting_ms  
    if(digitalRead(pet_input_pin) == 0){ // CONDITION #1   
      if(ms_without_being_pet < min_wait_period_after_petting_ms){
        if(!first_pet){
          // the pet signal arrived too early (on the second, or later, pet)    
          perform_reset_sequence(); 
        }        
      }
              
      // pet signal must be within the allowable window            
      ms_without_being_pet = 0; // timing the new window starts from now     
      led_on_duration_ms = LED_SHORT_BLINK_DURATION_MS; // turn the LED on for 50ms      
      check_for_pet_timer_ms = PET_WATCHDOG_RELEASE_DURATION_MS; // ignore currently being pet for 5ms          
      first_pet = false; // by definition, it's no longer the first pet         
    }
    // or CONDITION #2: the counter is higher than maximum_wait_period_after_petting and not currently being pet
    else{ // if(digitalRead(pet_input_pin) == 1)
      if(ms_without_being_pet >= maximum_wait_period_after_petting_ms){ 
        // the pet signal arrived too late      
        perform_reset_sequence(); 
      }
    }   
  }
  
  // manage the LED
  if(led_on_duration_ms > 0){
    digitalWrite(led_pin, HIGH);   //turn on the LED  
  }
  else{
    digitalWrite(led_pin, LOW);    //turn off the LED    
  }
}

boolean validateInitializationMessage(uint8_t * buffer){
  uint16_t calculated_checksum = 0, received_checksum = 0;
  boolean valid = true;
  
  min_wait_period_after_petting_ms = bufferToUint16(buffer);
  maximum_wait_period_after_petting_ms = bufferToUint16(buffer+2);
  calculated_checksum = checkcrc(buffer);
  received_checksum = bufferToUint16(buffer + INITIALIZATION_MESSAGE_LENGTH - 2);

  if(calculated_checksum != received_checksum){
    valid = false;
  }
  
  if(min_wait_period_after_petting_ms >= maximum_wait_period_after_petting_ms){
    valid = false; 
  }
  
  // the minimum wait interval
  if(min_wait_period_after_petting_ms < 5){
    valid = false; 
  }
  
  return valid;
}

uint16_t bufferToUint16(uint8_t * buffer){
  // 0x12, 0x34 becomes 0x1234 
  uint16_t value = buffer[1];
  value <<= 8;
  value |= buffer[0];

  return value;  
}

uint16_t checkcrc(uint8_t * buffer){
  uint8_t crc = 0, ii = 0;

  for (ii = 0; ii < INITIALIZATION_MESSAGE_LENGTH - 2; ii++){
    crc = _crc16_update(crc, buffer[ii]);
  }

  return crc; 
}

void perform_reset_sequence(void){
  // blink the LED fast three times
  blinkLedFast(3);
    
  pinMode(host_reset_pin, OUTPUT); 
  digitalWrite(host_reset_pin, LOW);

  delayMicroseconds(10);     // minimum pulse width on reset is 2.5us according ot the datasheet
                             // so 10us should be plenty
  
  pinMode(host_reset_pin, INPUT);

  soft_reset();              // reset the watchdog timer so the dance can begin again  
                             // this is *really* important or the host will try and send a serial message
                             // that will be interpretted as random petting behavior and result in a
                             // vicious cycle of restarts
}

void blinkLedFast(uint8_t n){
  uint8_t ii = 0;
  for(ii = 0; ii < n; ii++){
    digitalWrite(led_pin, HIGH); // led on
    delay(50);                   // wait on 
    digitalWrite(led_pin, LOW);  // led off
    delay(150);                  // wait off
  } 
}

void blinkLedSlow(uint8_t n){
  uint8_t ii = 0;
  for(ii = 0; ii < n; ii++){
    digitalWrite(led_pin, HIGH); // led on
    delay(250);                  // wait on 
    digitalWrite(led_pin, LOW);  // led off
    delay(250);                  // wait off
  } 
}

