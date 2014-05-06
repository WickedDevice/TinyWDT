#include <SoftwareSerial.h>
#include <util/crc16.h>
#include <avr/wdt.h>
                                                     // COMMENTS FOR ATTINY45
#define INITIALIZATION_TIMEOUT_MS 10000L             // no watchdog resets will occur or 10 seconds following a reset
#define INITIALIZATION_MESSAGE_LENGTH 6              // 2 uint16_t of payload and 1 uint16 of CRC
#define HEADER_FIRST_BYTE  0x5A
#define HEADER_SECOND_BYTE 0x96

//#define NANODE_HOST
//#define DEBUG

/* THESE VALUES ARE FOR A NANODE USING A1 as the INTERFACE PIN and RESET CONTROL on A0 */
#ifdef NANODE_HOST
  #define RESET_DDR DDRC                               // PORTB is the reset / configuration port
  #define RESET_PORT PORTC                             // 
  #define RESET_INPUT PINC                             //
  #define RESET_PIN 0                                  // PB0 is connected ot the reset pin of the host controller

  #define PETTING_ARDUINO_PIN 15                       // PB3 (DIG3) is the input pin
  #define SOFTSERIAL_JUNK_PIN 1                        // PB1 (DIG1) is not connected to anything
  #define PETTING_INPUT PINC
  #define PETTING_PIN  1

  #define LED_INVERTED
  #define LED_DDR  DDRD                                // PORTB is the LED port
  #define LED_PORT PORTD                               // 
  #define LED_PIN  6                                   // PB2 is connected to an LED
#endif

/* THESE SETTINGS ARE FOR AN ATTiny45 */
#ifndef NANODE_HOST
  #define RESET_DDR DDRB                               // PORTB is the reset / configuration port
  #define RESET_PORT PORTB                             // 
  #define RESET_INPUT PINB                             //
  #define RESET_PIN 0                                  // PB0 is connected ot the reset pin of the host controller

  #define PETTING_ARDUINO_PIN 3                        // PB3 (DIG3) is the input pin
  #define SOFTSERIAL_JUNK_PIN 1                        // PB1 (DIG1) is not connected to anything
  #define PETTING_INPUT PINB
  #define PETTING_PIN  3

  #define LED_DDR  DDRB                                // PORTB is the LED port
  #define LED_PORT PORTB                               // 
  #define LED_PIN  2                                   // PB2 is connected to an LED
#endif

#ifdef LED_INVERTED
  #define LED_OFF do{LED_PORT |= _BV(LED_PIN);}while(0)
  #define LED_ON do{LED_PORT &= ~_BV(LED_PIN);}while(0)  
#else
  #define LED_ON do{LED_PORT |= _BV(LED_PIN);}while(0)
  #define LED_OFF do{LED_PORT &= ~_BV(LED_PIN);}while(0)
#endif
#define RESET_LOW do{RESET_DDR   |= _BV(RESET_PIN);RESET_PORT  &= ~_BV(RESET_PIN);} while(0)
#define RESET_HIGHZ do{RESET_DDR  &= ~_BV(RESET_PIN);}while(0)
#define HOST_IN_RESET ((RESET_INPUT & _BV(RESET_PIN)) == 0) // True if the host is in reset, i.e. someone / thing is pressing the reset button
#define CURRENTLY_BEING_PET ((PETTING_INPUT & _BV(PETTING_PIN)) == 0)

  
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

uint16_t counter = 0;
uint16_t minimum_wait_period_after_petting_ms = 0;
uint16_t maximum_wait_period_after_petting_ms = 0;
uint8_t wdt_enabled = 0;
uint8_t led_on = 0;

long previousMicros = 0;
long interval = 1000; //micros
uint8_t first_pet = 1; // no minimum window constraint on the first pet

SoftwareSerial mySerial(PETTING_ARDUINO_PIN, SOFTSERIAL_JUNK_PIN); // DIG3 = RX, DIG1 = TX (not connected)

uint16_t checkcrc(uint8_t * buffer);
uint8_t validateInitializationMessage(uint8_t * buffer);
uint16_t bufferToUint16(uint8_t * buffer);
void perform_reset_sequence(void);
void blinkLedFast(uint8_t n);
void blinkLedSlow(uint8_t n);

void setup(){
  uint32_t currentMillis = millis();
  uint32_t timeoutMillis = currentMillis + INITIALIZATION_TIMEOUT_MS;  
  uint8_t buffer[INITIALIZATION_MESSAGE_LENGTH];
  uint8_t buffer_index = 0;
  uint8_t header_byte_num = 0;
  
  
  // set all pins to input
  LED_DDR = _BV(LED_PIN);  // all pins are inputs, except LED pin
  
#ifdef DEBUG  
  Serial.begin(115200);
  Serial.println("Hello World");
#endif

  blinkLedFast(2);

  mySerial.begin(4800);
  
  while(currentMillis < timeoutMillis){
     currentMillis = millis();
     if(mySerial.available()){
       uint8_t rx_char = mySerial.read();
       if(header_byte_num == 0){
         if(rx_char == HEADER_FIRST_BYTE){
           header_byte_num++;
#ifdef DEBUG
           Serial.println("Received First Header Byte");
         }
         else{
           Serial.println("Not First Header Byte"); 
#endif
         }
       }
       else if(header_byte_num == 1){
         if(rx_char == HEADER_SECOND_BYTE){
           header_byte_num++; 
#ifdef DEBUG
           Serial.println("Received Second Header Byte");
#endif           
         }
         else{
#ifdef DEBUG
           Serial.println("Received Erroneous Second Header Byte");
#endif           
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
  
#ifdef DEBUG
  Serial.print("Num Message Bytes Received: ");
  Serial.println(buffer_index);
  Serial.print("Buffer: ");
  for(buffer_index = 0; buffer_index < INITIALIZATION_MESSAGE_LENGTH; buffer_index++){
    if(buffer[buffer_index] < 16) Serial.print("0");
    Serial.print(buffer[buffer_index], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif    
  
  if((buffer_index == INITIALIZATION_MESSAGE_LENGTH) && validateInitializationMessage(buffer)){
     wdt_enabled = 1;
     blinkLedFast(2);
     
#ifdef DEBUG       
     Serial.println("WDT Function Enabled");     
     Serial.print("Minimum Window: ");
     Serial.println(minimum_wait_period_after_petting_ms);
     Serial.print("Maximum Window: ");
     Serial.println(maximum_wait_period_after_petting_ms);
#endif
  }
  else{
     if(buffer_index == 0){       
       blinkLedSlow(3); // indicate no initialization message received
#ifdef DEBUG  
       Serial.println("No Initialization Message Received");            
#endif
     }
     else if(buffer_index < INITIALIZATION_MESSAGE_LENGTH){
       blinkLedSlow(4); // indicate initialization message too short
#ifdef DEBUG       
       Serial.println("Too Short Initialization Message Received");                   
#endif
     }
     else{
       blinkLedSlow(5); // indicate validation failed
#ifdef DEBUG         
       Serial.println("Invalid Initialization Message Received");                                   
#endif       
     }
  }
  
  mySerial.end();  
}

void loop(){  
  uint8_t currently_being_pet = 0;
  unsigned long currentMicros = micros();
  
  // handle what happens the host is reset by something else
  if(HOST_IN_RESET){
    while(HOST_IN_RESET){
      continue; 
    }
    soft_reset();
  }
  
  if(wdt_enabled){
    
    if(CURRENTLY_BEING_PET){
      currently_being_pet = 1;
    }
    
    // scheduled once per millisecond (interval = 1000)
    if(currentMicros - previousMicros > interval){
      previousMicros = currentMicros;  
      
      counter++; // increase the counter
      if(led_on > 0){
        led_on--; 
      }
      
#ifdef DEBUG
      if((counter % 1000) == 0){
        Serial.print("."); 
      }
#endif
      
      // issue a reset if either: 
      // CONDITION #1: the input signal is LOW and the counter is lower than minimum_wait_period_after_petting_ms
      // or CONDITION #2: the counter is higher than maximum_wait_period_after_petting
      if( currently_being_pet && counter < minimum_wait_period_after_petting_ms && first_pet == 0){ // CONDITION #1
#ifdef DEBUG        
        Serial.println("Resetting host because of minimum window constraint");    
        Serial.print("Counter Value = "); 
        Serial.println(counter);        
#endif        
        perform_reset_sequence(); 
      }
      else if(counter >= maximum_wait_period_after_petting_ms){
#ifdef DEBUG          
        Serial.println("Resetting host because of maximum window constraint");                                   
#endif        
        perform_reset_sequence(); 
      } 
    }
    
    if(currently_being_pet){ // didn't issue a so reset the watchdog counter
#ifdef DEBUG    
      Serial.println("Received Pet Signal, resetting counter");
#endif      
      counter = 0; 
      currently_being_pet = 1;
      first_pet = 0;
      led_on = 50; // turn the LED on for 50ms
      delay(100);   // give the host some time to release the petting signal before starting the count again
    }        
    
    // manage the LED
    if(led_on){
      LED_ON;  //turn on the LED
    }
    else{
      LED_OFF; //turn off the LED
    }
  }
}

uint8_t validateInitializationMessage(uint8_t * buffer){
  uint16_t calculated_checksum = 0, received_checksum = 0;
  
  minimum_wait_period_after_petting_ms = bufferToUint16(buffer);
  maximum_wait_period_after_petting_ms = bufferToUint16(buffer+2);
  calculated_checksum = checkcrc(buffer);
  received_checksum = bufferToUint16(buffer + INITIALIZATION_MESSAGE_LENGTH - 2);
  
#ifdef DEBUG
  Serial.print("minimum_wait_period_after_petting_ms = ");
  Serial.println(minimum_wait_period_after_petting_ms);
  Serial.print("maximum_wait_period_after_petting_ms = ");
  Serial.println(maximum_wait_period_after_petting_ms);
  Serial.print("calculated_checksum = ");
  Serial.println(calculated_checksum, HEX);
  Serial.print("received_checksum = ");
  Serial.println(received_checksum, HEX);
#endif
    
  if(calculated_checksum == received_checksum){
    return 1;
  }
  else{
    return 0; 
  }
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
    
  RESET_LOW;

  delay(10);                 // minimum pulse width on reset is 2.5us according ot the datasheet
                             // so 10ms should be plenty
  
  RESET_HIGHZ;

  soft_reset();              // reset the watchdog timer so the dance can begin again  
                             // this is *really* important or the host will try and send a serial message
                             // that will be interpretted as random petting behavior and result in a
                             // vicious cycle of restarts
}

void blinkLedFast(uint8_t n){
  uint8_t ii = 0;
  for(ii = 0; ii < n; ii++){
    LED_ON; // led on
    delay(50);              // wait on 
    LED_OFF; // led off
    delay(150);             // wait off
  } 
}

void blinkLedSlow(uint8_t n){
  uint8_t ii = 0;
  for(ii = 0; ii < n; ii++){
    LED_ON;  // led on
    delay(250);             // wait on 
    LED_OFF; // led off
    delay(250);             // wait off
  } 
}

