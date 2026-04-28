
//Default password after first EEPROM initialization
//Current password 2468
#include <avr/io.h> //Part of standard C libraries and also not external
#include <avr/interrupt.h>
#include <stdint.h> //standard header file and not external
#define EEPROM_PIN_BASE 0 //
#define EEPROM_ADDR 10 //
#define EEPROM_VAL 0xA5 //This prevents the reading of corrupted or 
//uninitialized data when the Arduino goes through a power cycle
#define PASS_LENGTH 4 //password length
#define TIMEOUT_MS 15000 //15 seconds for timeout
#define ERROR_MS 1500 //Delay to hold error state
//#define ROW_NUM 4 /**/
//#define COL_NUM 3
#define DELAY_T 5
#define SERIAL_B 9600 //Baud rate for Serial
#define SERVO_PIN 9 //Pin that controls 
#define LOCKED_US    600
#define UNLOCKED_US  2400
//Keypad defines
#define SCAN_A 4
#define SCAN_B 3
#define SENSE_PIN A0
#define WAKE_PIN 2
#define IDLE_THRESHOLD 950
//Blocked Servo defines
#define SERVO_SENSE_PIN A1
#define BLOCK_CURRENT_THRESHOLD 120
#define BLOCK_TIME_MS 250
#define MOVE_TIMEOUT_MS 1000
#define SETTLE_IGNORE_MS 120
//LED defines
//(All led's connected with 15k Ohm resistor each, then connected to ground)
#define LED_RED 10
#define LED_GREEN 11
#define LED_YELLOW 12
//HASH defines
#define EEPROM_MARKER_ADDR 10
#define EEPROM_MARKER_VAL 0xA5
#define EEPROM_SALT_ADDR 20 // 8 bytes
#define EEPROM_HASH_ADDR 30 //32 bytes
#define SALT_LENGTH 8
#define HASH_LENGTH 32
#define SALT_NOISE_PIN A2 //Using floating analog pin for noise
#define ROTR(x,n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x,y,z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22))
#define EP1(x) (ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25))
#define SIG0(x) (ROTR(x,7) ^ ROTR(x,18) ^  ((x) >> 3)  )
#define SIG1(x) (ROTR(x,17) ^ ROTR(x,19) ^ ((x) >> 10 ))

//LED Struct
struct LEDBlink{
  int pin;
  unsigned long last_toggle_ms;
  bool is_on;
};
LEDBlink redLED = {LED_RED, 0, false};
LEDBlink greenLED = {LED_GREEN, 0, false};
LEDBlink yellowLED = {LED_YELLOW, 0, false};

//Start of state machine
enum State{
  SLEEP, //Currently standing as a placeholder until Milestone 3
  LOCKED, //Servo motor in locked mode
  INPUT_STATE, //collects and reads the digits from the keypad, * to clear, # to submit
  CHECK_PW, //Validates the entry[] against a saved password
  UNLOCKING, //MILESTONE 3A
  UNLOCKED, //Servo motor in unlocked mode, use '#' to lock
  LOCKING, //MILESTONE 3A
  ERROR, //shows the error feedback, then returns to INPUT_STATE
  REPROGRAM //State to reprogram the password
};
unsigned long motion_start_ms = 0;
unsigned long high_current_start_ms = 0;
State previous_state = LOCKED;
State state = LOCKED;
//char saved_password[PASS_LENGTH + 1] = {0}; //No longer hardcoded
char entry[PASS_LENGTH + 1] = {0};
uint8_t entryLength = 0;

unsigned long last_input_ms = 0;
unsigned long  error_start_ms = 0;
//Keypad settings here
char dec_phase_1(int adc){
  if(adc < 354){return '1';}
  if(adc < 380){return '2';}
  if(adc < 405){return '4';}
  if(adc < 442){return '5';}
  if(adc < 485){return '3';}
  if(adc < 600){return '6';}
  return '\0';
}
char dec_phase_2(int adc){
  if(adc < 353){return '7';}
  if((adc < 390)){return '8';}
  if(adc < 416){return '*';}
  if(adc < 447){return '0';}
  if(adc < 490){return '9';}
  if(adc < 650){return '#';}
  return '\0';
}
char get_key(){
  //Phase 1
  digitalWrite(SCAN_A, LOW);
  digitalWrite (SCAN_B, HIGH);
  delayMicroseconds(20);
  int adcA= analogRead(SENSE_PIN);

  //Phase 2
  digitalWrite(SCAN_A, HIGH);
  digitalWrite(SCAN_B, LOW);
  delayMicroseconds(20);
  int adcB = analogRead(SENSE_PIN);
  if(adcA < IDLE_THRESHOLD){
    return dec_phase_1(adcA);
  }
  if(adcB < IDLE_THRESHOLD){
    return dec_phase_2(adcB);
  }
  return '\0';
}
char get_key_event(){
  static bool hold = false;
  char c = get_key();
  if(c != '\0' && !hold){
    delay(20);
    char confirm = get_key();
    if(confirm == c){
      hold = true;
      return c;
    }
  }
  if(c == '\0'){
    hold = false;
  }
  return '\0';
}

//Keypad functions here


//SERVO Functions HERE
static inline void setServoPulseUs(uint16_t pulse_us) {
  // Timer1 tick = 0.5 us when prescaler = 8 (16MHz / 8 = 2MHz)
  // ticks = pulse_us / 0.5us = pulse_us * 2
  uint16_t ticks = pulse_us * 2;


  // Safety clamp so you don't command insane widths
  if (ticks < 1000) ticks = 1000;      // 500 us
  if (ticks > 5000) ticks = 5000;      // 2500 us


  OCR1A = ticks;
}


void setupTimer1_50Hz() {
  pinMode(SERVO_PIN, OUTPUT);


  // Stop Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;


  // Fast PWM, TOP = ICR1  (Mode 14: WGM13:0 = 1110)
  // Clear OC1A on compare match, set at BOTTOM (non-inverting)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12);


  // Prescaler = 8
  TCCR1B |= (1 << CS11);


  // TOP for 50Hz:
  // 16MHz / 8 = 2MHz => 0.5us per tick
  // 20ms / 0.5us = 40000 ticks -> TOP = 39999
  ICR1 = 39999;


  // Start at LOCKED
  setServoPulseUs(LOCKED_US);
}

static inline void servo_set_locked(){ //Function to set the servo motor to the locked position
  setServoPulseUs(LOCKED_US);
}

static inline void servo_set_unlocked(){ //Function to set the servo motor to the unlocked position
  setServoPulseUs(UNLOCKED_US);
}
//SERVO SETTINGS HERE
void entry_clear(){ //This clears the password entered in the system
  entryLength = 0; //Reset entryLength to zero
  entry[0] = '\0';
}
void entry_append(char digit){ // Appends a null character to the password 
  if(entryLength < PASS_LENGTH){ //In order to create a "finished" password
    entry[entryLength++] = digit;
    entry[entryLength] = '\0'; //Will make the last digit a nullifying character
  }
}
bool entryComplete(){ //Bool function to check if the password is 4 digits
  if(entryLength == PASS_LENGTH){
    return true;
  } else {
    return false;
  }
}


//EEPROM functions

/* Instead of using eeprom functions, instead we have to use the registers:
EEAR: Address Register
EEDR: Data Register
EECR: Control Register


If we can use the functions, I'll check back in with Milburn bc it would make code
much shorter.
*/

static inline uint8_t eeprom_read_byte(uint16_t addr){ //This returns a byte from Arduino saved data
  while (EECR & (1 << EEPE)) {
    }
    //Waits while the EEPROM is busy
    EEAR = addr; //Set EEPROM address
    EECR |= (1 << EERE); //This triggers the actual read itself
    return EEDR; //return data
  
}
static inline void eeprom_write_byte(uint16_t addr, uint8_t data){ //This will write a byte to the Arduino saved data
  while(EECR & (1 << EEPE)){

  } //Waits while the EEPROM is busy
    EEAR = addr; //Sets the EEPROm address
    EEDR = data; //Sets the data from parameter
    EECR |= (1 << EEMPE); //master write enable
    EECR |= (1 << EEPE); //start writing
  
}

//Start of sleep mode settings

volatile bool woke_up = false; //Bool to determine if the system is awake or not
ISR(INT0_vect){
  woke_up = true;
}
void setup_wake_interrupt(){
  //Make D2 = PD2 = INT0
  //Setting PD2 as an input
  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);
 EICRA &= ~(1 << ISC01);
 EICRA &= ~(1 << ISC00);
  EIFR |= (1 << INTF0);
  EIMSK |= (1 << INT0);
  sei();
}
void enter_sleep_mode(){
  LED_off();
  EIFR |= (1 << INTF0);
  EIMSK |= (1 << INT0);
  EICRA &= ~(1 << ISC01);
  EICRA &= ~(1 << ISC00);
  SMCR &= ~((1 << SM2) | (1 << SM1) | (1 << SM0));
  SMCR |= (1 << SM1);
  SMCR |= (1 << SE);
  sei();
  asm volatile("sleep");
  SMCR &= ~(1 << SE);
  EIMSK &= ~(1 << INT0);
}
void disable_unused_modules_before_sleep(){
  ADCSRA &= ~(1 << ADEN);
  PRR |= (1 << PRADC);
  PRR |= (1 << PRSPI);
  PRR |= (1 <<PRTWI);
  PRR |= (1 << PRUSART0);
  PRR |= (1 << PRTIM1);
}
void enable_modules_after_wake(){
  
  PRR &= ~(1 << PRADC);
  PRR &= ~(1 << PRSPI);
  PRR &= ~(1 <<PRTWI);
  PRR &= ~(1 << PRUSART0);
  PRR &= ~(1 << PRTIM1);
  ADCSRA |= (1 << ADEN);
  Serial.begin(SERIAL_B);
  setupTimer1_50Hz();
  servo_set_locked();
  setup_wake_interrupt();
  last_input_ms = millis();
  //servo_setup();
}
//Servo Blocking Code

int read_servo_current_adc(){
  return analogRead(SERVO_SENSE_PIN);
}
void start_unlock_motion(){
  previous_state = LOCKED;
  servo_set_unlocked();
  motion_start_ms = millis();
  high_current_start_ms = 0;
  state = UNLOCKING;
}
void start_lock_motion(){
  previous_state = UNLOCKED;
  servo_set_locked();
  motion_start_ms = millis();
  high_current_start_ms = 0;
  state = LOCKING;
}
bool servo_blocked_current(){
  unsigned long elapsed = millis() - motion_start_ms;
  int current_adc = read_servo_current_adc();
  if(elapsed < SETTLE_IGNORE_MS){
    high_current_start_ms = 0;
    return false;
  }
  if(current_adc > BLOCK_CURRENT_THRESHOLD){
    if(high_current_start_ms == 0){
      high_current_start_ms = millis();
    }
    if((millis()- high_current_start_ms) >= BLOCK_TIME_MS){
      return true;
    }

  } else {
    high_current_start_ms = 0;
  }
  return false;
}
//LED functions


void blink_led(LEDBlink* led, unsigned long interval){
  unsigned long now =millis();
  if(now - led->last_toggle_ms >= interval){
    led->last_toggle_ms = now;
    led->is_on = !led->is_on;
    digitalWrite(led->pin, led->is_on ? HIGH : LOW);
  }
}
void pulse_led(int pin, unsigned long period_ms, unsigned long on_time_ms){
  unsigned long t = millis() % period_ms;
  if(t < on_time_ms){
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin, LOW);
  }
}
void blink_alternate(LEDBlink* led1, LEDBlink* led2, unsigned long interval){
  unsigned long now = millis();
  if(now - led1->last_toggle_ms >= interval){
    led1->last_toggle_ms = now;
    led1->is_on = !led1->is_on;
    led2->is_on = !led1->is_on;
    digitalWrite(led1->pin, led1->is_on ? HIGH : LOW);
    digitalWrite(led2->pin, led2->is_on ? HIGH : LOW);

  }
}
void LED_off(){ //Turns all the LED's off
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
}
void LED_showInput(){ //Input = YELLOW
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  pulse_led(LED_YELLOW, 2000, 75);
}
void LED_showLocked(){ //Locked = RED LED
//digitalWrite(&LED_RED, 1000); //slower blink
pulse_led(LED_RED, 2000, 50);
}
void LED_showUnlocked(){ //Unlocked = GREEN

  //digitalWrite(LED_GREEN, HIGH);
  pulse_led(LED_GREEN, 2000, 50);
}
void show_error(){ //Blinking Red
  blink_alternate(&redLED, &yellowLED, 150);
}
void show_reprogram(){ //Reprogram green and red
  
  //digitalWrite(LED_RED, HIGH);
  //digitalWrite(LED_GREEN, HIGH);
  //digitalWrite(LED_YELLOW, HIGH);
  pulse_led(LED_RED, 2000, 50);
  pulse_led(LED_GREEN, 2000, 50);


}
//START OF HASHING FUNCTIONS

bool hash_equal(const uint8_t* a, const uint8_t* b, uint8_t length){
uint8_t i;
for(i = 0; i < length; ++i){
  if(a[i] != b[i]){
    return false;
  
  }
}
return true;
}
void generate_salt(uint8_t* salt, uint8_t length){
uint32_t seed_pool = 0x12345678UL;
uint8_t i;
uint8_t j;
for (i = 0; i < length; ++i){
  uint8_t byteVal = 0;
  for(j = 0; j < 8; ++j){
    uint16_t adc = analogRead(SALT_NOISE_PIN);
    uint32_t t1 = micros();
    uint32_t t2 = millis();
    uint16_t timer_mix = TCNT1; //When timer1 is running include it's present count
    seed_pool ^= ((uint32_t)adc << 16);
    seed_pool ^= t1;
    seed_pool ^= ((uint32_t)t2 << 8);
    seed_pool ^= timer_mix;
    seed_pool ^= (seed_pool << 13);
    seed_pool ^= (seed_pool >> 17);
    seed_pool ^= (seed_pool << 5);
    byteVal <<= 1;
    byteVal |= (uint8_t)(seed_pool & 0x01);
    delayMicroseconds(7 + (adc & 0x0F));
  }
  salt[i] = byteVal;
}
}

void print_bytes_hex(const uint8_t *data, uint8_t length){
  uint8_t i = 0;
  for(i = 0; i < length; ++i){
    if(data[i] < 16){
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}


static const uint32_t k[64]= { //Contains all the 64 round constants for SHA Encryption
0x428a2f98UL, 0x71374491UL, 0xb5c0fbcfUL, 0xe9b5dba5UL,
0x3956c25bUL, 0x59f111f1UL, 0x923f82a4UL, 0xab1c5ed5UL,
0xd807aa98UL, 0x12835b01UL, 0x243185beUL, 0x550c7dc3UL,
0x72be5d74UL, 0x80deb1feUL, 0x9bdc06a7UL, 0xc19bf174UL,
0xe49b69c1UL, 0xefbe4786UL, 0x0fc19dc6UL, 0x240ca1ccUL,
0x2de92c6fUL, 0x4a7484aaUL, 0x5cb0a9dcUL, 0x76f988daUL,
0x983e5152UL, 0xa831c66dUL, 0xb00327c8UL, 0xbf597fc7UL,
0xc6e00bf3UL, 0xd5a79147UL, 0x06ca6351UL, 0x14292967UL,
0x27b70a85UL, 0x2e1b2138UL, 0x4d2c6dfcUL, 0x53380d13UL,
0x650a7354UL, 0x766a0abbUL, 0x81c2c92eUL, 0x92722c85UL,
0xa2bfe8a1UL, 0xa81a664bUL, 0xc24b8b70UL, 0xc76c51a3UL,
0xd192e819UL, 0xd6990624UL, 0xf40e3585UL, 0x106aa070UL,
0x19a4c116UL, 0x1e376c08UL, 0x2748774cUL, 0x34b0bcb5UL,
0x391c0cb3UL, 0x4ed8aa4aUL, 0x5b9cca4fUL, 0x682e6ff3UL,
0x748f82eeUL, 0x78a5636fUL, 0x84c87814UL, 0x8cc70208UL,
0x90befffaUL, 0xa4506cebUL, 0xbef9a3f7UL, 0xc67178f2UL
};
//https://stackoverflow.com/questions/11937192/sha-256-pseuedocode#:~:text=Core%20Algorithm%20Steps:%20The%20SHA%2D256%20process%20begins,hex_return(hash_2)%2C%20hex_return(hash_3)%2C%20hex_return(hash_4)%2C%20hex_return(hash_5)%2C%20hex_return(hash_6)%2C%20hex_return(hash_7))%5Cn%20return(final_hash)%5Cn%60%60%60
void sha256(const uint8_t* data, size_t len, uint8_t hash[32]){
uint32_t h[8] = {
   0x6a09e667UL, 0xbb67ae85UL, 0x3c6ef372UL, 0xa54ff53aUL,
    0x510e527fUL, 0x9b05688cUL, 0x1f83d9abUL, 0x5be0cd19UL
};
size_t new_len = len + 1;
while((new_len % 64) != 56){
  new_len++;
}
size_t total_len = new_len + 8;
uint8_t msg[64];
uint16_t i;
 for(i = 0; i < total_len; ++i){
  msg[i] = 0;
 }
 for(i = 0; i < len; ++i){
  msg[i] = data[i];
 }
 msg[len] = 0x80;
 uint64_t bit_len = (uint64_t)len * 8ULL;
 for(i = 0; i < 8; ++i){
  msg[total_len - 1 - i] = (uint8_t)(bit_len >> (8 * i));
 }
 for(size_t chunk = 0; chunk < total_len; chunk += 64){
  uint32_t w[64];
  for(i = 0; i < 16; i++){
  uint16_t j = chunk + (i * 4);
  w[i] = ((uint32_t)msg[j] << 24) | ((uint32_t)msg[j+1] << 16) | ((uint32_t)msg[j+2] << 8) |((uint32_t)msg[j+3]);

 }

 for(i = 16; i < 64; ++i){
  w[i] = SIG1(w[i - 2]) + w[i - 7] + SIG0(w[i - 15]) + w[i - 16];
 }
 
 uint32_t a = h[0];
 uint32_t b = h[1];
 uint32_t c = h[2];
 uint32_t d = h[3];
 uint32_t e = h[4];
 uint32_t f = h[5];
 uint32_t g = h[6];
 uint32_t hh = h[7];
 
 for(i = 0; i < 64; i++){
  uint32_t t1 = hh + EP1(e) + CH(e, f, g) + k[i] + w[i];
  uint32_t t2 =  EP0(a) + MAJ(a, b, c);
  hh = g;
  g = f;
   f = e;
   e = d + t1;
   d = c;
   c = b;
   b = a;
   a = t1 + t2;
 }

h[0] += a;
h[1] += b;
h[2] += c;
h[3] += d;
h[4] += e;
h[5] += f;
h[6] += g;
h[7] += hh;
 }
for(i = 0; i < 8; i++){
  hash[i * 4 + 0] = (uint8_t)(h[i] >> 24);
  hash[i * 4 + 1] = (uint8_t)(h[i] >> 16);
  hash[i * 4 + 2] = (uint8_t)(h[i] >> 8);
  hash[i * 4 + 3] = (uint8_t)(h[i]);

  }
}
void hash_pin_w_salt(const char* pin, const uint8_t* salt, uint8_t out_hash[32]){
uint8_t buf[SALT_LENGTH + PASS_LENGTH];
uint8_t i;
for (i = 0; i < SALT_LENGTH; ++i){
  buf[i] = salt[i];
}
for(i = 0; i < PASS_LENGTH;++i){
  buf[SALT_LENGTH + i] = (uint8_t)pin[i];
}
sha256(buf, SALT_LENGTH + PASS_LENGTH, out_hash);
}
void save_credentials_to_EEPROM(const char* pin){
uint8_t salt[SALT_LENGTH];
uint8_t hash[HASH_LENGTH];
generate_salt(salt, SALT_LENGTH);
hash_pin_w_salt(pin, salt, hash);
save_hash_salt_EEPROM(salt, hash);
eeprom_write_byte(EEPROM_MARKER_ADDR, EEPROM_MARKER_VAL);
}
//void load_credentials_from_EEPROM(const char *pin){
//}

void save_hash_salt_EEPROM(const uint8_t* salt, const uint8_t *hash){
uint8_t i;
for(i = 0; i < SALT_LENGTH; ++i){
  eeprom_write_byte(EEPROM_SALT_ADDR + i, salt[i]);
}
for(i = 0; i < HASH_LENGTH;++i){
  eeprom_write_byte(EEPROM_HASH_ADDR + i, hash[i]);
}
}
void load_hash_salt_EEPROM(uint8_t* salt, uint8_t* hash){
uint8_t i;
for(i = 0; i < SALT_LENGTH; ++i){
  salt[i] = eeprom_read_byte(EEPROM_SALT_ADDR + i);
}
for(i = 0; i < HASH_LENGTH; ++i){
  hash[i] = eeprom_read_byte(EEPROM_HASH_ADDR + i);
}
}
bool entry_matches_stored_hash(const char* pin){
uint8_t stored_salt[SALT_LENGTH];
uint8_t stored_hash[HASH_LENGTH];
uint8_t computed_hash[HASH_LENGTH];
load_hash_salt_EEPROM(stored_salt, stored_hash);
hash_pin_w_salt(pin, stored_salt, computed_hash);
return hash_equal(stored_hash, computed_hash, HASH_LENGTH);
}
void initialize_default_pw(){
if(eeprom_read_byte(EEPROM_MARKER_ADDR) != (EEPROM_MARKER_VAL)){
  save_credentials_to_EEPROM("1234"); //Initialize password as "1234"
}
}








//END OF HASHING FUNCTIONS
void setup(){
delay(100);
Serial.begin(SERIAL_B);
delay(100);
last_input_ms = millis(); //Using millis to measure elapsed time
setup_wake_interrupt();
//KEYPAD pinMode setup from 2a
//  for (int row = 0; row < ROW_NUM; ++row){
//     pinMode(pinRows[row], OUTPUT); //Setting the rows to become outputs
//     digitalWrite(pinRows[row], HIGH); //Making these rows high
//   }

//   for(int col = 0; col < COL_NUM; ++col){
//     pinMode(pinCols[col], INPUT_PULLUP);//Setting the columns to becomes 
//   }
pinMode(SCAN_A,OUTPUT);
pinMode(SCAN_B, OUTPUT);

digitalWrite(SCAN_A, HIGH);
digitalWrite(SCAN_B,HIGH);
pinMode(SERVO_SENSE_PIN, INPUT);
//SERVO setup from 2b
setupTimer1_50Hz();
servo_set_locked();

//eeprom_initialize_if_needed(saved_password);
initialize_default_pw();
//LED 
pinMode(LED_RED, OUTPUT);
pinMode(LED_GREEN, OUTPUT);
pinMode(LED_YELLOW, OUTPUT);

/* DEBUG to check what password is saved beforehand
Serial.print("Saved Pin loaded='");
Serial.print(saved_password);
Serial.println("'");
Serial.print("Saved PIN bytes: ");
for(int i = 0; i < PASS_LENGTH; ++i){
  Serial.print((uint8_t)saved_password[i], HEX);
  Serial.print(" ");
}
Serial.println();
*/
Serial.println("Locker is ready. Enter 4 digit PIN:");

}

void loop(){
  
  char key = get_key_event();
  if(key != '\0'){
    last_input_ms = millis(); //Whenever the user presses a key, last_input_ms records the time
  }
  if((millis()- last_input_ms) > TIMEOUT_MS){
    if(state == UNLOCKED){
      Serial.println("Timeout while unlocked. Locking before sleep...");
      start_lock_motion();
    } //If more than fifteen seconds, time out 
   else if(state == LOCKED || state == INPUT_STATE || state == ERROR){
    state = SLEEP;
   }
  }
  
  switch(state){
    case LOCKED:
    LED_showLocked();
    if(key!= '\0'){ //Press a key first to wake up the system
     state = INPUT_STATE;
    }
    //state = INPUT_STATE;
    break;

    case INPUT_STATE:
    LED_showInput();
    if(key == '\0'){ break;}
    if(key >= '0' && key <= '9'){ //Bounds for password
      entry_append(key);
      Serial.print("Entered: ");
      Serial.println(entry); 

    } else if(key == '*'){ //* = cleared password
      entry_clear();
      Serial.println("Cleared.");
    } else if(key == '#'){ //# = submit password
    Serial.println("Submitted");
      state = CHECK_PW;
    }

    //For future, I will probably update this part to 
    //automatically submit when the max password length is reached.
    
    break;

    case CHECK_PW:
    LED_off();
    if(!entryComplete()){ //System does not validate until password is complete
      Serial.println("Password Incomplete. Try again: ");
      state = INPUT_STATE;//return to input 
      break;
    }
    
    //compare entered to saved password
    
    if(entry_matches_stored_hash(entry)){
      Serial.println("Correct Password. Unlocking...");
      start_unlock_motion();
      
    } else {
      Serial.println("Incorrect Password");
      error_start_ms = millis();
      state = ERROR;
    }
    entry_clear(); //Cleared after checking
    break;

    case UNLOCKED:
    LED_showUnlocked();
    if (key == '*'){ // Using "*" as the lock button
      Serial.println("Locking...");
     // servo_set_locked();
      //state = LOCKED;
      entry_clear();
      start_lock_motion();
    } else if(key == '#'){
      Serial.println("Enter new password:");
      entry_clear();
      state = REPROGRAM; //
    }
    
    break;

  case ERROR:
  show_error();
 //Blinking red and yellow LED
  if((millis()-error_start_ms) > ERROR_MS){ //Hold the error state
  
    Serial.println("Error, try again:");
    state = INPUT_STATE; //Allows the user to try again
  }
    break;
    
  case SLEEP: //Placeholder for Milestone 3
  LED_off();
  Serial.println("Entering sleep...");
  Serial.flush();
  disable_unused_modules_before_sleep();
  enter_sleep_mode();
  enable_modules_after_wake();
  Serial.println("Woke up.");
  state = LOCKED;
  break;

  default:
  state = LOCKED;
  break;

  case REPROGRAM: //Reprogram is much like the input, with the exception that 
                  //it saves the result to EEPROM, which allows you to make a new
                  //PIN number
  show_reprogram();
  if(key == '\0'){
    break;
  }
  if(key >= '0' && key <= '9'){
    entry_append(key);
    Serial.print("New PIN: ");
    Serial.println(entry);
    
  } else if( key == '*'){
    entry_clear();
    Serial.println("Cleared new PIN.");
  } else if(key == '#'){
    if(!entryComplete()){
      Serial.println("New PIN incomplete. Enter 4 digits.");
      break;
    }
    //save_pw_to_eeprom(entry);//Save the entered password to EEPROM
    //eeprom_write_byte(EEPROM_ADDR, EEPROM_VAL); // Write the marker again when changing the password
   // load_pw_from_eeprom(saved_password); //Load the now active password from EEPROM
    save_credentials_to_EEPROM(entry);
    Serial.println("New PIN saved. Now locked.");
    entry_clear();
    servo_set_locked();
    state = LOCKED;
  }
  break;
  case UNLOCKING:
  if(servo_blocked_current()){
    Serial.println("Blocked while unlocking. Reverting");
    servo_set_locked();
    state = LOCKED;
  } else if((millis()- motion_start_ms) >= MOVE_TIMEOUT_MS){
    LED_showUnlocked();
    Serial.println("Unlock complete.");
    state = UNLOCKED;
  }
  break;
  case LOCKING:
  if(servo_blocked_current()){
    Serial.println("Blocked while locking. Reverting.");
    servo_set_unlocked();
    state = UNLOCKED;
  } else if((millis()-motion_start_ms) >= MOVE_TIMEOUT_MS){
    LED_showLocked();
    Serial.println("Lock complete.");
    state = LOCKED;
  }
  break;
  } //end of switch statement
}



