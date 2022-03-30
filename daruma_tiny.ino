//PCB Darumu Badge code
//Written for Spence Konde's ATtiny84/44/24 core:
//https://github.com/SpenceKonde/ATTinyCore
//Software PWM library from:
//https://github.com/Palatis/arduino-softpwm
//Some code referenced from:
//https://github.com/FastLED/FastLED

#include <SoftPWM.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <util/atomic.h>

// Set fuses with this AVRDUDE command:
// Fuse burn will fail unless LED on pin 9 (SCK) is disconnected
// avrdude -V -pattiny24 -cusbtiny -U lfuse:w:0xe2:m -U hfuse:w:0x77:m  -U efuse:w:0xff:m

const uint8_t NUM_LED = 10; // number of LEDs

bool partyMode = false; // never sleep and run pattern indefinitely

//SOFTPWM_DEFINE_PINMODE(CHANNEL, DDRX, PORTX, BIT);
//CHANNEL is library address and number of PWM channels
//DDRX is register location for INPUT or OUTPUT (Data Direction Register)
//PORTX is whether pin is located on A or B
//BIT is which bit on the PORTX register each pin is set by
//Reference:
//https://www.arduino.cc/en/Reference/PortManipulation
//https://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny24A-44A-84A-DataSheet-DS40002269A.pdf


//Define the ATtiny84 pins according to the board for softPWM
//Pin numbering according to:
//https://raw.githubusercontent.com/SpenceKonde/ATTinyCore/master/avr/extras/ATtiny_x4.png
SOFTPWM_DEFINE_CHANNEL(0, DDRB, PORTB, PORTB0);  //ATTINY84 pin 2
SOFTPWM_DEFINE_CHANNEL(1, DDRB, PORTB, PORTB1);  //ATTINY84 pin 3
//SOFTPWM_DEFINE_CHANNEL(0, DDRB, PORTB, PORTB3);  //ATTINY84 pin 4 RESET PIN
SOFTPWM_DEFINE_CHANNEL(2, DDRB, PORTB, PORTB2);  //ATTINY84 pin 5
SOFTPWM_DEFINE_CHANNEL(3, DDRA, PORTA, PORTA7);  //ATTINY84 pin 6
SOFTPWM_DEFINE_CHANNEL(4, DDRA, PORTA, PORTA6);  //ATTINY84 pin 7
SOFTPWM_DEFINE_CHANNEL(5, DDRA, PORTA, PORTA5);  //ATTINY84 pin 8
SOFTPWM_DEFINE_CHANNEL(6, DDRA, PORTA, PORTA4);  //ATTINY84 pin 9
SOFTPWM_DEFINE_CHANNEL(7, DDRA, PORTA, PORTA3);  //ATTINY84 pin 10
SOFTPWM_DEFINE_CHANNEL(8, DDRA, PORTA, PORTA2);  //ATTINY84 pin 11
//SOFTPWM_DEFINE_CHANNEL(0, DDRA, PORTA, PORTA1);  //ATTINY84 pin 12 UART TX PIN
SOFTPWM_DEFINE_CHANNEL(9, DDRA, PORTA, PORTA0);  //ATTINY84 pin 13

//Define number of PWM channels, levels of brightness (255 max)
SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(NUM_LED, 255);

//Set an output brightness ceiling for each LED
//Lowering these improves battery life at expense of visible brightness
//Lower individual values to normalize different color LEDs (dim blues to match reds)
//Must be <= Palatis::SoftPWM.PWMlevels()
uint8_t maxBright = 200;

//Buffer of brightness values for each LED softpwm object
uint8_t buffer[NUM_LED] = { 0 };

//Variables used for fading functions
uint8_t fadeBuffer[NUM_LED] = { 0 }; // buffer to hold faded values
volatile uint16_t fadeTimer = 0; // how many times a fade has been applied
uint16_t fadeStart = 300; // how long to wait before fading out

//Variables used for input and debouncing
uint16_t button = 0;
uint8_t buttonReg = 0; // shift register for button debouncing
uint8_t shakeReg = 0; // shift register for shake sensor debouncing
bool extendFade = false; // decide if the pattern display is extended


//Implements a state machine to handle timing for patterns and fading of LEDs
class Timing {
    unsigned int currentMillis; // time since startup in ms
    unsigned int previousMillis; // last time timer went off

  public:
    uint8_t millisInterval; // set delay on pattern updates

    Timing() // constructor
    {
      unsigned int millisInterval = 1;
    }

    bool Update()
    {
      currentMillis = millis();
      if (currentMillis - previousMillis >= millisInterval) {

        previousMillis = currentMillis; // remember time for next update check
        return true; // time to update
      }
      else {
        return false; // do not update
      }

    }
};

Timing pattern;
Timing fader;
Timing input;

void setup() {
  //Begin with specified pwm frequency in hz
  Palatis::SoftPWM.begin(180);

  ADCSRA = 0; // turn off ADC for power saving

  //Configure button input
  DDRA &= ~0b00000010; // Make pin 12 INPUT
  PORTA |= 0b00000010; // Make pin 12 PULLUP

  //Configure interrupts
  //https://thewanderingengineer.com/2014/08/11/pin-change-interrupts-on-attiny85/
  GIMSK = 0b00100000;    // turns on pin change interrupts
  PCMSK1 &= ~0b00001000;    // turn off interrupts on pin PB3 RESET

  //Configure interrupt pin
  //pinMode(interruptPin, INPUT);
  DDRB &= ~0b00001000; // Make pin PB3 INPUT
  PORTB |= 0b00001000; // Make pin PB3 PULLUP

  //Check if button is held on startup
  partyMode = !(PINA & 0b00000010);

  button = EEPROM.read(10); // remember last set pattern

  sei();  // enable interrupts to wake from sleep

  wdt_enable(WDTO_8S); // turn on watchdog timer
}

void loop() {

  if ( partyMode == true ) {
    extendFade = true;
  }

  //  Serial.println(fadeTimer);
  if ( input.Update() == true ) {
    input.millisInterval = 5; // set polling speed

    if ( debounce( !( PINA & 0b00000010 ), buttonReg ) == true ) { // read button at pin 12
      button++;
      extendFade = true;

      input.millisInterval = 3000; // set polling speed
    }

    if ( debounce( !( PINB & 0b00001000 ), shakeReg ) == true ) { // read shake sensor at pin 12
      extendFade = true;
    }

  }

  //Pick patterns based on button input and iterate pattern states
  if ( pattern.Update() == true ) {
    switch (button) {
      case 0:
        sinScroll(10, 3);
        break;
      case 1:
        eyes(30);
        break;
      case 2:
        eyeRing(10, 8);
        break;
      case 3:
        sinScroll(8, 5);
        break;
      case 4:
        eyeKnight(10, 40);
        break;
      case 5:
        chase(250, 2);
        break;
      default:
        chase(1, 2); // repeat last pattern to prevent visual delay on pattern rollover
        button = 0;
        //        Serial.println("button rollover");
    }
  }


  //Fade LED values before output while waking and sleeping
  if ( fader.Update() == true ) {
    fadeTimer++;

    // fade LEDs in on start
    if ( fadeTimer < 255 ) {
      fader.millisInterval = 5; // set fade in speed
      fadeIn (fadeTimer);
    }

    //before fade out, check if we need to extend the pattern display
    else if ( fadeTimer == fadeStart ) {
      wdt_reset();
      //if movement detected, reset timer and continue loop
      if ( extendFade == true) {
        extendFade = false;
        fadeTimer = 255; // extend LED pattern display
      }
    }

    // fade LEDs out before sleeping
    else if ( fadeTimer > fadeStart ) {
      fader.millisInterval = 5; // set fade out speed
      fadeOut (fadeTimer);
    }
  }


  // push LED buffer states to output
  if ( ( fadeTimer < 255 || fadeTimer > fadeStart ) ) {
    showAll(fadeBuffer); // show faded values at startup or before sleep
  }
  else {
    showAll(buffer);
  }


  // Sleep after fade out has turned all LED's off
  if ( fadeTimer >= fadeStart && allOff( fadeBuffer ) == true ) {
    wdt_reset();
    wdt_disable(); // disable WDT to not interrupt sleep
    EEPROM.update(10, button); // save button state before sleeping
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    PCMSK1 |= 0b00001000;    // turn on interrupts on pin PB3 RESET tiny
    sleep_cpu();
    // Code resumes from sleep here
    PCMSK1 &= ~0b00001000;    // turn off interrupts on pin PB3 RESET
    extendFade = false;
    wdt_enable(WDTO_8S);
    fadeTimer = 0; // restart fade sequence upon wake up
  }

}



//Interrupt service routine to wake the mcu from sleep
//Called when shake sensor detects motion
ISR(PCINT1_vect) { // attiny

  if ( MCUCR & 0b00100000 == 1) { // if the sleep enable bit is set (if currently sleeping)
    sleep_disable(); // wake up
  }

}



//A variant of ring() that recreates the Knight Rider car light
void eyeKnight(uint8_t interval, uint8_t trail) {
  static uint8_t count = 0;
  pattern.millisInterval = interval;

  static uint8_t eyeOrder[NUM_LED] = { 0, 1, 2, 3, 4, 9, 8, 7, 6, 5 }; // only 9 LEDs for test

  //Go through every LED, and decay its brightness level to create a fading trail
  for (uint8_t i = 0; i < NUM_LED; i++) {
    buffer[i] = buffer[i] * (255 - trail) / 255;
  }

  if ( buffer[ eyeOrder[ count ] ] <= 80) { // Move to next LED in eyeOrder[] once the leader has decayed
    count = count + 1;
    if (count > NUM_LED - 1 || count < 0) { // Loop back to first LED
      count = 0;
    }
    buffer[ eyeOrder[ count ] ] = 255;
  }
}



//Create copies of buffer[] values and fade them up as timer increases
void fadeIn (uint16_t timer) {
  for (uint8_t i = 0; i < NUM_LED; i++) {
    fadeBuffer[i] = ( buffer[i] *  timer ) / 255;
  }
}



//Create copies of buffer[] values and fade them down as timer increases
void fadeOut (uint16_t timer) {
  for (uint8_t i = 0; i < NUM_LED; i++) {
    if ( (fadeStart + 255) - timer <= 20 ) { // fade more quickly just before turning off
      fadeBuffer[i] = ( fadeBuffer[i] * 250 ) / 255; // stop reading new buffer[] values
      //            fader.millisInterval = 15;
      //      fader.millisInterval = (fadeStart + 255) - timer; // increase fade speed towards end
    }
    else {
      fadeBuffer[i] = ( buffer[i] * ( (fadeStart + 255) - timer  ) ) / 255;
    }
  }
}



//Check if a passed array is all zeroes
//Used to make sure all LEDs are off before sleeping
bool allOff(uint8_t checkBuffer[NUM_LED])
{

  if ( fadeTimer <= fadeStart ) { // if the LED not being fading out
    return false;
  }

  for ( uint8_t i = 0; i < NUM_LED; i++ ) {
    if ( checkBuffer[i] != 0 ) {
      return false; // stop checking if nonzero is found
    }
  }

  return true; // if no nonzeros are found, then entire buffer must be zero

}



//Shift register debounce routine from http://www.ganssle.com/debouncing-pt2.htm
//https://www.best-microcontroller-projects.com/easy_switch_debounce.html
bool debounce(uint8_t pin, uint8_t &shifter) {
  shifter = (shifter << 1) | !pin | 0b11110000; // left shift current state of button into reg
  if (shifter == 0b11111000) {
    //    pressButton++;
    //    if (fadeTimer >= 255) { // if pattern is changed and initial fade in is over
    //      fadeTimer = 255; // reset fade timer to keep LEDs on while picking patterns
    //    }
    return true;
  }
  return false;
}



//A sine wave is scrolled around all LEDs
//freq changes the number of waves present
void sinScroll(uint8_t interval, uint8_t freq) {
  static uint8_t phase = 20; // Used to shift the phase of the wave
  // Start at 20 for smoother turn on
  pattern.millisInterval = interval; // Set pattern update rate

  phase += 1;
  for (uint8_t i = 0; i < NUM_LED; i++) {
    buffer[i] = ease8InOutQuad((i * ((freq * 256) / NUM_LED)) + phase); // From FastLED sinelon
  }
}



//A lit LED scrolls around the board leaving a slowly fading trail behind it
//Reminiscent of a ring oscillator
void ring(uint8_t interval, uint8_t trail) {
  static uint8_t count = 0;//track the position of the leading LED
  pattern.millisInterval = interval;

  for (uint8_t i = 0; i < NUM_LED; i++) { // Decays the brightness of LEDs to fade the trail
    buffer[i] = buffer[i] * ((255.0 - trail) / 255.0);
  }

  if (buffer[count] <= 128) { // Move to next LED once the leader has decayed to half brightness
    //clockwise rotation
    count++;
    if (count > NUM_LED - 1) { // Loop back to first LED
      count = 0;
    }
    /*
      //counterclockwise rotation
      count--;
      if(count == 255){ // Loop back to first LED
      count = NUM_LED -1;
      }
    */
    buffer[count] = 255;
  }
}



//A symmetrical variant of ring() that moves through both of the daruma's eyes
void eyeRing(uint8_t interval, uint8_t trail) {
  static uint8_t count = 0;
  pattern.millisInterval = interval;

  for (uint8_t i = 0; i < NUM_LED; i++) { // Decays the brightness of LEDs to fade the trail
    buffer[i] = buffer[i] * (255 - trail) / 255;
  }
  if (buffer[count] <= 128) { // Move to next LED once the leader has decayed to half brightness
    //Clockwise rotation
    count++;
    if (count > (NUM_LED - 1) / 2) { // Loop back to first LED
      count = 0;
    }

    /*
      //Counterclockwise rotation
      count--;
      if(count == 255){//Loop back to first LED
      count = NUM_LED -1;
      }
    */
    buffer[count] = 255; // Sets leader LED on one side
    buffer[(NUM_LED - 1) - count] = 255; // Mirrors leader LED on other side
  }
}



//A variant of ring() that recreates the Knight Rider car light
void knight(uint8_t interval, uint8_t trail) {
  static int8_t flip = 1;//Flips the scrolling LED direction
  static uint8_t count = 0;
  pattern.millisInterval = interval;

  //Go through every LED, and decay its brightness level to create a fading trail
  for (uint8_t i = 0; i < NUM_LED; i++) {
    buffer[i] = buffer[i] * (255 - trail) / 255;
  }

  if (buffer[count] <= 80) { // Move to next LED once the leader has decayed
    count = count + flip;
    if (count >= NUM_LED - 1 || count <= 0) { // Loop back to first LED
      flip = -flip;
      //count = 0;
    }
    buffer[count] = 255;
  }
}



//A plain theatre chase that phase shifts a square wave across the LEDs
//gap determines LED grouping eg:3 means every 3rd will be lit
void chase(uint8_t interval, uint8_t gap) {
  static uint8_t phase = 0;
  pattern.millisInterval = interval;

  phase += 1;
  for (uint8_t i = 0; i < NUM_LED; i++) {
    if ((i + phase) % gap == 0) {
      buffer[i] = 255;
    }
    else {
      buffer[i] =  0;
    }
  }
}



//A theatre chase that phase shifts a sine wave across the LEDs
void sinChase(uint8_t interval, uint8_t gap) {
  static uint8_t phase = 0;
  static uint8_t pat = 1;
  pattern.millisInterval = interval;

  phase += 5; // Increase this by factors of 255 to increase execution speed
  if (phase == 255) { // Switches which LEDs are turned on
    pat++;
    phase = 0;
  }
  for (uint8_t i = 0; i < NUM_LED; i++) {
    if ((i + pat) % gap == 0) { // Every % #-th LED will turn on
      buffer[i] = ease8InOutQuad(phase);
    }

    else {
      buffer[i] = 0;
    }
  }
}



//Alternate a full cycle of a wave between the daruma's eyes and the other LEDs
void eyes(uint8_t interval) {
  static uint8_t phase = 0;
  static bool look = 0;//Flip whether the eyes are on or off
  pattern.millisInterval = interval;

  phase += 5;//Increase this by factors of 255 to increase execution speed
  if (phase == 255) { //Switches which LEDs are turned on
    phase = 0;
    look = !look;
  }
  for (uint8_t i = 0; i < NUM_LED; i++) {
    if (look == 0 && (i == 4 || i == 5)) { //Turn on the eyes
      buffer[i] = ease8InOutQuad(phase);
    }
    else if (look == 1 && (i != 4 && i != 5)) { //Turn on everything except the eyes
      buffer[i] = ease8InOutQuad(phase);
    }
    else { // Turn off the LEDs not
      buffer[i] = 0;
    }
  }
}



//Apple's breathing LED effect
//This functiong is a gigantic 1KiB and will probably not be used
void breathe(uint8_t interval) {
  pattern.millisInterval = interval;

  for (uint8_t i = 0; i < NUM_LED; i++) {
    buffer[i] = ((exp (sin (millis() / 2000.0 * PI)) - 0.36787944) * 108.0);//breathing LEDs effect
  }
}



//This function takes in a variable between 0 to 255 and outputs
//a quadratic approximation of a sine wave.
//(Increasing from 0-127, decreasing from 128-255)
uint8_t ease8InOutQuad(uint8_t tri) {
  //First, generate a triangle wave
  if (tri >= 128) {
    tri = 255 - tri;
  }
  tri = tri << 1;

  //Next, convert the triangle wave into a sine wave
  uint8_t quadHalf = tri;
  if (quadHalf >= 128) {
    quadHalf = 255 - quadHalf;
  }

  //Then ease the sine wave for a more even transition
  uint8_t quadFull  = (quadHalf * (quadHalf + 1)) / 256;
  uint8_t quadEased = quadFull << 1;
  if (tri >= 128) {
    quadEased = 255 - quadEased;
  }

  return quadEased;
}



//Push all LED buffer values to PWM output
void showAll(uint8_t output[NUM_LED]) {
  for (uint8_t i = 0; i < NUM_LED; i++) {
    //Rescale buffer values according to maxBright and push to output
    //This is a smaller reimplementation of map()
    //      Palatis::SoftPWM.set(i, (output[i] * maxBright[i] / 255));
    Palatis::SoftPWM.set(i, (output[i] *  maxBright / 255  ));
  }
}
