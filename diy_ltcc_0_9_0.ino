/*

	this code is to be considered experimental and is provided "as is", without warranty of any kind, 
	express or implied, including but not limited to fitness for a particular purpose.
	if your LT1 comes apart using it i will not be held responsible
	you were warned
	
	this code is licensed under GNU General Public Licencse v3 - please see file: LICENSE in repository

*/
#define VERSION "0.9.0"

// only select one of the below options
//#define SERIAL_DEBUG 1
#define LOGGING 1
//#define INPUT_TEST 1

// or one or both for simulator mode
//#define SIMULATE 1
//#define DETAIL_SIM 1      // causes detailed uart logging of simulation routines

// if you require a tach driver on A0 uncomment - note the A0 output is still used / configured for INPUT_TEST mode
//#define TACH_DRIVER 1

#define DWELL_TGT 4.5       // dwell target in milliseconds - edit to your liking (using 4.5ms for standard ls2 coils)
#define CRANK_DWELL_TGT 3.2 // dwell target for cranking / startup
#define VOLT_COMP_8 2.4     // adds / subtracts from dwell target at given input voltage
#define VOLT_COMP_9 1.5
#define VOLT_COMP_10 0.9
#define VOLT_COMP_11 0.4
#define VOLT_COMP_13 -0.2
#define VOLT_COMP_14 -0.4
#define VOLT_COMP_15 -0.7
#define VOLT_COMP_16 -0.9

#define BUILD_ENV_DPTI 1 // comment out if you get compiler errors about undeclared digitalPinToInterrupt() function

#define NUM_CYLINDERS 8

// mainly informational / used in setup() do not change unless you know what you're doing
#define PCM_EST 2
#define OPTI_HI_RES 3
#define OPTI_LO_RES 8   // must use pin 8 for capture interrupt
#define TACH_SIGNAL A0  // statically assigned to tachOutput[] structure

#define CRANKING_SPK_ADV 10 // arbitrary value to use before we pickup commanded sa from the pcm
#define LR_OVERFLOW 20      // low res timer overflow limit - engine has stalled if timer1 overflows this many times (16 x 32676us = 522.816 ms between tdc signals - about 29rpm)

typedef struct {      // the structure for the dwell table
  uint8_t dwell8V;
  uint8_t dwell9V;
  uint8_t dwell10V;
  uint8_t dwell11V;
  uint8_t dwell12V;
  uint8_t dwell13V;
  uint8_t dwell14V;
  uint8_t dwell15V;
  uint8_t dwell16V;
} dwell;

#define RPM_DIVS 34
#define CRANK_RPM_DIVS 9 // these low rpm cells get lesser CRANK_DWELL_TGT

dwell dwellTable[RPM_DIVS]; // the dwell table - populated during setup() contains dwell in degrees required to meet DWELL_TGT at different rpms

int rpmReference[RPM_DIVS] = { 25, 50, 75, 100, 125, 150, 175, 200, // the rpm reference
  400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 
  3000, 3200, 3400, 3600, 3800, 4000, 4400, 4800, 5200, 5600, 6000, 6400, 6800 };

uint64_t rpmTable[RPM_DIVS];  // the rpm lookup for the dwell table - populated during setup() contains rpm reference in microseconds per 90* (between low res signals)

volatile int8_t sequenceIndex = -1;     // if negative we've lost sequence, else pointer to the current cylinder in the sequencer[] struct
volatile boolean invertSeq;             // invert sequence detection - when true the engine came to rest with the low res trigger in a gap (beam not broken)
volatile uint8_t hrDegrees = 90;        // the high resolution counter - counts down from 90 after each negative edge low res pulse
volatile uint8_t saDegrees;             // stored by the EST line interrupt this is a snapshot of most recent spark advance in degrees btdc
volatile uint8_t dwellDegrees;          // current dwell requirement in degrees
volatile uint8_t estEnabler;            // count degrees while est is high inside high res isr - used to debounce est input

volatile uint8_t errCount;              // for below error and lag statistics
uint8_t errorDegs;                      // hrDegrees should be 0 when the low res TDC signal is caught - this tracks cumulative # of errors per last 10 revolutions
uint8_t dwellLag;                       // difference between targeted dwell and when dwell actually started - cumulative over last 10 revolutions
#define ERROR_LOGS 10                   // match constant for resetting errCount and above statistics

volatile uint8_t lrSemaphoreRising;      // triggers sequence detection in main loop
volatile uint8_t lrSemaphoreFalling;     // used for sequence detection when invertSeq = true

volatile unsigned long lrTimeHi;        // microseconds between low res falling edges also used as semaphore to trigger dwell lookup in loop()
volatile uint8_t lrHi_overflow;         // the timer1 overflow counter - 16 bit unsigned overflows at 65535 - timer1 prescaler is 8 so one "tick" every .5 us
uint64_t lrHi_overflow_const = 32767;   // used to calculate the most recent time interval bewteen low res rising edge signals for rpm lookup
uint64_t rpmTimeHi;                     // holds the low-res interval - calculated in loop()

uint8_t timer2OFC;                      // timer2 overflow counter
uint8_t timer2OFCompare = 8;            // scale this multiplier with rpm to reduce uart lag
//#define T2CMPCOUNT 12                   // count to 13
volatile boolean logFlag;               // flag to prompt log data output and voltage detection

typedef struct {
  int cylNo;          // user readable cylinder no
  int armed;          // set to 1 = armed for dwell, set to 0 = est signal caught - prevents dwell routine from dwelling recently fired coil
  uint8_t *portAddr;  // used at a pointer to the output registers so coil drivers can be controlled directly (faster than using digital[write/read]() arduino functions)
  byte bitMask;       // the bit mask of each output pin - see the atmega datasheet for more info on this and timers
} cylinders;

cylinders sequencer[NUM_CYLINDERS] = {
  {1, 1, &PORTD, B00010000},  // arduino d4
  {8, 1, &PORTB, B00000010},  // arduino d9
  {4, 1, &PORTB, B00001000},  // arduino d11
  {3, 1, &PORTD, B00100000},  // arduino d5
  {6, 1, &PORTB, B00000100},  // arduino d10
  {5, 1, &PORTD, B01000000},  // arduino d6
  {7, 1, &PORTD, B10000000},  // arduino d7
  {2, 1, &PORTB, B00010000},  // arduino d12
};

// a pointer to the tach output (default A0) - using the cylinders struct because it's convenient
cylinders tachOutput[1] = {
  {0, 0, &PORTC, B00000001}
};

void setup() {

  // configure misc output pins
  pinMode(13, OUTPUT);      // pin d13 used as a general information led - on = system ready, off = running, blinking = ???, etc.
  digitalWrite(13, LOW);

  pinMode(TACH_SIGNAL, OUTPUT);
  #ifdef TACH_DRIVER
  digitalWrite(TACH_SIGNAL, HIGH);
  #endif

  // disable digital buffers on unused adc pins
  bitSet (DIDR0, ADC1D);  // disable digital buffer on A1
  bitSet (DIDR0, ADC2D);  // disable digital buffer on A2
  bitSet (DIDR0, ADC3D);  // disable digital buffer on A3
  bitSet (DIDR0, ADC4D);  // disable digital buffer on A4
  bitSet (DIDR0, ADC5D);  // disable digital buffer on A5
  
  // configure input pins and enable pullups
  pinMode(OPTI_HI_RES, INPUT_PULLUP);
  pinMode(OPTI_LO_RES, INPUT_PULLUP);
  pinMode(PCM_EST, INPUT_PULLUP);
  
  // TODO setup adc input for voltage comparator
  
  
  // configure output pins
  DDRD = DDRD | B11110000; // configure PD4 - PD7 (arduino d4 - d7) as outputs
  DDRB = DDRB | B00011110; // configure PB1 - PB5 (arduino d9 - d12) as outputs
  
  Serial.begin(115200);
  Serial.print("diy-ltcc-");
  Serial.println(VERSION);

  populateRPMTimes();
  populateDwellDegrees();
  
  lrSemaphoreRising = 0;
  lrSemaphoreFalling = 0;
  saDegrees = CRANKING_SPK_ADV;

  cli(); // disable interrupts
  
  // attach the high res interrupt to count crankshaft degrees
  #ifdef BUILD_ENV_DPTI
    attachInterrupt(digitalPinToInterrupt(OPTI_HI_RES), isr_hi_res, CHANGE);
  #else
    attachInterrupt(1, isr_hi_res, CHANGE);
  #endif 

  // attach the EST line interrupt to service routine to trigger ignition
  #ifdef BUILD_ENV_DPTI
    attachInterrupt(digitalPinToInterrupt(PCM_EST), isr_est, FALLING);
  #else
    attachInterrupt(0, isr_est, FALLING);
  #endif

  #ifndef SIMULATE
    // configure timer1 for capture interrupt
    TCCR1A = 0; // clear timer1 registers
    TCCR1B = 0; 
    if ( digitalRead(OPTI_LO_RES) ) {
      TCCR1B &= ~(1<<ICES1);            // set first capture on falling edge (see declaration of invertSeq variable for more info)
      invertSeq = true;
    } else {
      TCCR1B |= (1<<ICES1);             // set first capture on rising edge
      invertSeq = false;
    }
    TCCR1A &= 0b11111100;               // set WGM21 & WGM20 to 0 (see datasheet pg. 155).
    TCCR1B &= 0b11110111;               // set WGM22 to 0 (see pg. 158).
    TCCR1B |= (1<<CS11);                // prescaler 8 - gives timer1 .5 us resolution per tick
    TIMSK1 |= (1<<ICIE1)|(1<<TOIE1);    // enable timer in input capture mode, and overflow interrupts
  #endif

  // configure timer2 for variable interval
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 192; //220*1024*7/16mhz = 0.099904 s - see timer2OFCompare definition (timer 2 multiplexer value)
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1<<CS12)|(1<<CS10); // 1024 prescaler   
  TIMSK2 |= (1 << OCIE2A);
  
  sei();

}


void isr_est() {

#ifndef SIMULATE
  // store the current spark advance
  if (estEnabler > 2) {
    if (hrDegrees > 90 ) {
      saDegrees = 0;
    } else {
      saDegrees = hrDegrees;
    }
    #ifdef INPUT_TEST
      if (digitalRead(PCM_EST)) {
        *tachOutput[0].portAddr |= tachOutput[0].bitMask; // drive the tach output high      
      } else {
        tachOutput[0].portAddr &= ~tachOutput[0].bitMask; // drive the tach output low
        Serial.print("saDegrees=");
        Serial.println(saDegrees);
      }    
    #else
      // turn off the coil output to trigger spark event
      if (sequenceIndex > -1) {
        *sequencer[sequenceIndex].portAddr &= ~sequencer[sequenceIndex].bitMask;
        #ifdef SERIAL_DEBUG
        Serial.print("c");
        Serial.print(sequencer[sequenceIndex].cylNo);
        Serial.print(" fire ");
        Serial.print(hrDegrees);
        Serial.print(" ");
        Serial.println(estEnabler);
        #endif
      }
      #ifdef TACH_DRIVER
        // drive tach output line high
        *tachOutput[0].portAddr |= tachOutput[0].bitMask;
      #endif
    #endif
  }
#endif
  
}


void isr_hi_res() {   // hi res change interrupt - counts crankshaft degrees

  hrDegrees--;
  // if PCM_EST line is high count degrees into estEnabler to debounce
  uint8_t estState = PIND & B00000100; // read D2 state
  if (estState) estEnabler++;
  #ifdef INPUT_TEST
    if (digitalRead(OPTI_HI_RES)) {
      *sequencer[4].portAddr |= sequencer[4].bitMask; // drive #6 coil pin high
    } else {
      *sequencer[4].portAddr &= ~sequencer[4].bitMask; // drive #6 coil pin low
    }
  #endif
  
}


ISR(TIMER1_OVF_vect) {  // timer1 overflow interrupt vector 
  
  lrHi_overflow++;

  if ( lrHi_overflow > LR_OVERFLOW ) { 
    
    // timeout on low res pulse
    lrHi_overflow = LR_OVERFLOW + 1;
    lrTimeHi = 0;
    
    // at this point the engine has stalled or the low res signal has failed, shut everything down
    hrDegrees = 90;
    sequenceIndex = -1;
    lrSemaphoreRising = 0;
    lrSemaphoreFalling = 0;
    saDegrees = CRANKING_SPK_ADV;
    if( bit_is_set(TCCR1B ,ICES1) ) { // waiting on rising edge
      invertSeq = false;
    } else {
      invertSeq = true;
    }
    digitalWrite(13, HIGH);
    
  }
    
}


ISR(TIMER1_CAPT_vect) { // timer1 capture interrupt vector - measures microseconds between low res rising edge (TDC) signals
  
  if( bit_is_set(TCCR1B, ICES1) ) { // trigger on rising edge of low res signal

    #ifdef INPUT_TEST
      *sequencer[1].portAddr |= sequencer[1].bitMask; // drive #8 coil pin high
      Serial.print("hrDegRise=");
      Serial.println(hrDegrees);
      hrDegrees = 90;
    #else
      if (ICR1 == 0) {
        lrTimeHi = 1;           // .5us error if we happen to land on zero
      } else {
        lrTimeHi = ICR1;        // push the timer value into the semaphore variable
      }
      TCNT1 = 0;                // reset the count register
      if ( sequenceIndex > -1 ) {
        sequenceIndex++;        // increment the sequence index
        if (sequenceIndex == NUM_CYLINDERS) {
          sequenceIndex = 0;
        }
      } 
      lrSemaphoreRising = hrDegrees;  // for sequence detection       
      hrDegrees = 90;                 // reset the degree counter
      #ifdef TACH_DRIVER
        *tachOutput[0].portAddr &= ~tachOutput[0].bitMask; // drive the tach output low - needs testing - may not give wide enough pulsewidth at high rpm for accurate tach readings
      #endif
      
    #endif
    
  } else {                         // trigger on falling edge

    #ifdef INPUT_TEST
      *sequencer[1].portAddr &= ~sequencer[1].bitMask; // drive #8 coil pin low
      Serial.print("hrDegFall=");
      Serial.println(hrDegrees);
    #else
      lrSemaphoreFalling = hrDegrees; // for sequence detection
    #endif
    
  }
  
  TCCR1B ^= _BV(ICES1); // toggle <TODO find name of this register> bit value so interrupt triggers on the other edge
  
}


ISR (TIMER2_COMPA_vect) { // general purpose timer for scheduling adc conversions and uart output
  // Timer 2 has reached its comparison value
  timer2OFC++;
  if (timer2OFC >= timer2OFCompare) {
    timer2OFC = 0;
    logFlag = true;
  }
  
}


void loop() {
  
  #ifdef SERIAL_DEBUG
    static bool initialized;
    if ( ! initialized ) {
       Serial.print("ready in: ");
       Serial.print(millis());
       Serial.println("ms");
       initialized = true;
    }
  #endif
  
  static uint64_t rpmTimeLast = 0; // for reference in getDwellDegrees lookup function
  static uint8_t rpmIndex = 0;     // for human readable rpm and dwell lookup hint
  boolean stalled = false;
  
  char serial_out[20];

  #ifdef SIMULATE

    // sim mode branch for testing functionality
    static uint8_t simMode = 5;
    static uint64_t loopTimer;    // comparator for switching low res pulses
    static uint64_t loopInterval = 180; // how long between low res pulses
    static uint32_t simTime = 0;  // comparator for switching between simulation modes
    static uint64_t displayInterval = 180; // scale to slow display of sequence
    static uint64_t lowResInterval = loopInterval * 90;      // how many micros per low res falling edge for dwell lookup
    uint64_t thisLoop;
    static uint8_t revCounter = 0;
    
    char str[30];

    uint32_t thisSim = millis();
    if ( (thisSim > simTime + 7000) || saDegrees == CRANKING_SPK_ADV ) {

      simTime = thisSim;
      simMode++;
      if ( simMode == 6 ) simMode = 0;
      
      switch(simMode) {
        case 0: // 925 rpm, 26 deg spark, 12v, real time
          loopInterval = 180; // microseconds per degree
          saDegrees = 28;     // spark advance
          lowResInterval = loopInterval * 90;
          displayInterval = loopInterval;
           Serial.print("simulation ");
           Serial.print(simMode + 1);
           Serial.println(" 925 rpm, 26 deg spark, 12v, real time");
           Serial.print("hrSignal time: ");
           Serial.println(uintToStr(displayInterval, str));
          break;
        case 1: // 925 rpm, 26 deg spark, 12v, real time
          loopInterval = 180; // microseconds per degree
          saDegrees = 28;     // spark advance
          lowResInterval = loopInterval * 90;
          displayInterval = loopInterval * 16; // 1/16 time
           Serial.print("simulation ");
           Serial.print(simMode + 1);
           Serial.println(" 925 rpm, 26 deg spark, 12v, 1/16 time");
           Serial.print("hrSignal time: ");
           Serial.println(uintToStr(displayInterval, str));
          break;
        case 2: // 3500 rpm, 35 deg spark, 12v, real time
          loopInterval = 48;  // microseconds per degree
          saDegrees = 35;     // spark advance
          lowResInterval = loopInterval * 90;
          displayInterval = loopInterval;
           Serial.print("simulation ");
           Serial.print(simMode + 1);
           Serial.println(" 3500 rpm, 35 deg spark, 12v, real time");
           Serial.print("hrSignal time: ");
           Serial.println(uintToStr(displayInterval, str));
          break;
        case 3: // 3500 rpm, 35 deg spark, 12v, real time
          loopInterval = 48;  // microseconds per degree
          saDegrees = 35;     // spark advance
          lowResInterval = loopInterval * 90;
          displayInterval = loopInterval * 32; // 1/32 time
           Serial.print("simulation ");
           Serial.print(simMode + 1);
           Serial.println(" 3500 rpm, 35 deg spark, 12v, 1/32 time");
           Serial.print("hrSignal time: ");
           Serial.println(uintToStr(displayInterval, str));
          break;
        case 4: // 7000 rpm, 40 deg spark, 12v, real time
          loopInterval = 24;  // microseconds per degree
          saDegrees = 40;     // spark advance
          lowResInterval = loopInterval * 90;
          displayInterval = loopInterval;
           Serial.print("simulation ");
           Serial.print(simMode + 1);
           Serial.println(" 7000 rpm, 40 deg spark, 12v, real time");
           Serial.print("hrSignal time: ");
           Serial.println(uintToStr(displayInterval, str));
          break;
        case 5: // 7000 rpm, 40 deg spark, 12v, real time
          loopInterval = 24;  // microseconds per degree
          saDegrees = 40;     // spark advance
          lowResInterval = loopInterval * 90;
          displayInterval = loopInterval * 64; // 1/64 time
           Serial.print("simulation ");
           Serial.print(simMode + 1);
           Serial.println(" 7000 rpm, 40 deg spark, 12v, 1/64 time");
           Serial.print("hrSignal time: ");
           Serial.println(uintToStr(displayInterval, str));
          break;
      }
      dwellDegrees = getDwell(lowResInterval, rpmTimeLast, 12, rpmIndex);
      rpmTimeLast = lowResInterval;
      Serial.print("dwellDegrees = ");
      Serial.println(dwellDegrees);
      
    }

    thisLoop = micros();
    if ( thisLoop - loopTimer > displayInterval ) {

      // this "tick" represents a high-res change (1 crankshaft degree)
      loopTimer = thisLoop;
      hrDegrees--;
      if ( hrDegrees == 0 ) {
        hrDegrees = 90;
        sequencer[sequenceIndex].armed = 1; // armed for next cycle
        sequenceIndex++;
        *tachOutput[0].portAddr &= ~tachOutput[0].bitMask; // drive the tach output low
        if ( sequenceIndex == NUM_CYLINDERS ) sequenceIndex = 0;
        revCounter++;
        if (revCounter == 8) {
          revCounter = 0;
          logFlag = true;
        }
        /*Serial.print("lrSignal: ");
        Serial.println(uintToStr(lowResInt, str));*/
        dwellDegrees = getDwell(lowResInterval, rpmTimeLast, 12, rpmIndex);
        rpmTimeLast = lowResInterval;
      }

      uint8_t regValue = *sequencer[sequenceIndex].portAddr & sequencer[sequenceIndex].bitMask;
      if ((hrDegrees <= saDegrees) && ( regValue )) {
        // turn off the dwelling coil
        #ifdef DETAIL_SIM
        Serial.print("firing coil at ");
        Serial.println(hrDegrees);
        #endif
        *sequencer[sequenceIndex].portAddr &= ~sequencer[sequenceIndex].bitMask;
        sequencer[sequenceIndex].armed = 0;
        *tachOutput[0].portAddr |= tachOutput[0].bitMask; // drive the tach output high
      }

      if (dwellDegrees > 0)
        dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex);
       
    }
    // end simulator branch
    
  #else

    // begin main branch
    if ( sequenceIndex < 0 ) {

      // we haven't detected sequence yet so do that first
      int8_t nextCyl = -1;
      if (invertSeq) {    // engine stopped in a low res trigger gap (beam not broken)
        if ((lrSemaphoreRising != 0) && (invertSeq)) {   // we have a rising edge degree count
                                                         
          #ifdef SERIAL_DEBUG
            Serial.print("detect inv, degRising=");
            Serial.print(lrSemaphoreRising);
            Serial.print(", degFalling=");
            Serial.println(lrSemaphoreFalling);
          #endif
          nextCyl = detectSeqInverse(lrSemaphoreRising, lrSemaphoreFalling);
          if (nextCyl < 0) {
            // clear the falling semaphore and invert bit to detect normally on next falling edge
            invertSeq = false;
            lrSemaphoreFalling = 0;
          }
         
        } 
      } else {          // engine stopped on a low res trigger (beam broken)
        if ((lrSemaphoreFalling != 0) && (! invertSeq)) { // we have a falling edge degree count
  
          #ifdef SERIAL_DEBUG
            Serial.print("detect norm, degFalling=");
            Serial.println(lrSemaphoreFalling);
          #endif
          nextCyl = detectSequence(lrSemaphoreFalling);
          if (nextCyl < 0) lrSemaphoreFalling = 0;       // clear the falling semaphore and wait for next falling edge
          
        }
      }

      if (nextCyl > -1) {

        // sequence caught
        sequenceIndex = nextCyl;
        dwellDegrees = 1; // minimum default dwell for first coil firing
        digitalWrite(13, LOW);
        #ifdef SERIAL_DEBUG
           Serial.println("sequenced");
        #endif

      }
      
    }
    // end sequence detection

    // check if we need to dwell coils (#1)
    if (dwellDegrees > 0) dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex);
    
    if ( lrTimeHi != 0 ) {

      // low res semaphore caught
      
      stalled = false;

      // calculate interval between low res rising edges in microseconds
      rpmTimeHi = (lrHi_overflow * lrHi_overflow_const) + uint64_t (lrTimeHi / 2); // might consider doing this calculation less freq at higher rpm
      cli(); // disable interrupts to clear the lrTimeHi semaphore - this could be problematic
      lrTimeHi = 0;
      sei();
      
      if (lrHi_overflow > LR_OVERFLOW) stalled = true;
      
      lrHi_overflow = 0;
      
      if (sequenceIndex < 0) {
        // not sequenced yet, retrieve dwell from lowest rpm cell
        dwellDegrees = getDwell(rpmTable[0] + 1, 0, 12, rpmIndex);
      } else {
        dwellDegrees = getDwell(rpmTimeHi, rpmTimeLast, 12, rpmIndex); // might want to consider doing this less freq at higher rpms
        // re-arm the previous coil
        if (sequenceIndex == 0) {
          sequencer[NUM_CYLINDERS - 1].armed = 1;
        } else {
          sequencer[sequenceIndex - 1].armed = 1;
        }
        
        // clear the est debouncer
        estEnabler = 0;

        // scale the logging and voltage sense interval with rpmIndex
        if (rpmIndex > 8) timer2OFCompare = rpmIndex;
         
      }
      rpmTimeLast = rpmTimeHi; // this is used as a hint to getDwell() so it doesn't iterate the entire array after each TDC signal
      
      #ifdef SERIAL_DEBUG
        if (rpmTimeHi > 8500) {
          cylInfoUart();
        }
      #endif

      // accumulate error degrees
      if (lrSemaphoreRising > 90) {
        // wraparound
        errorDegs += 256 - lrSemaphoreRising;
      } else {
        errorDegs += lrSemaphoreRising;
      }
       
    }

    if (dwellDegrees > 0) {
      
      // check if we need to dwell coils (#2), and if stalled
      if (stalled) {
        // stalled - shut down any dwelling coils
        #ifndef INPUT_TEST
        for (uint8_t i = 0; i < NUM_CYLINDERS; i++) {
          *sequencer[i].portAddr &= ~sequencer[i].bitMask;
          sequencer[i].armed = 1;
        }
        #endif
        dwellDegrees = 0;
        rpmIndex = 0;
        #ifdef SERIAL_DEBUG
          Serial.println("stalled");
        #endif
      } else {
        dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex);
      }
    }
    
  #endif


  // output logging data - triggered by timer2 (TODO: verify this timer #!!!)
  if (logFlag) {

    // TODO voltage / bandgap / cpu temp detection
    
    logFlag = false;
    #ifdef LOGGING
    if ( sequenceIndex < 0 ) {
      sprintf(serial_out,"R%d:S%d:D%d:C%d:E%d:L%d", rpmReference[rpmIndex], saDegrees, dwellDegrees, 0, errorDegs, dwellLag);
    } else {
      sprintf(serial_out,"R%d:S%d:D%d:C%d:E%d:L%d", rpmReference[rpmIndex], saDegrees, dwellDegrees, sequencer[sequenceIndex].cylNo, errorDegs, dwellLag);
    }
    Serial.println(serial_out);
    errCount++;
    #endif
    
  }

  #ifndef SIMULATE
    // check if we need to dwell coils (#3)
    if (dwellDegrees > 0) dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex);
  #endif
  
  // update / clear error statistics
  if (errCount >= ERROR_LOGS) {
    errCount = 0;
    errorDegs = 0;
    dwellLag = 0;
  }
  
}


void populateRPMTimes() {
  
    // convert rpm to microseconds per 90 degrees crank rotation (high res signal)
  #ifdef SERIAL_DEBUG
    Serial.println("dwell table");
    Serial.print("RPM\t");
  #endif
  
  for (int i=0; i<RPM_DIVS; i++) {
    #ifdef SERIAL_DEBUG
      Serial.print(rpmReference[i]);
      Serial.print("\t");
    #endif
    float temp = rpmReference[i] * 0.016666666667;
    rpmTable[i] = uint64_t (250000 / temp);
  }
  
  #ifdef SERIAL_DEBUG
    Serial.println("");
    Serial.print("us/90*\t");
  #endif
  
  char str[30];
  for (int i=0; i<RPM_DIVS; i++) {
    #ifdef SERIAL_DEBUG
      Serial.print(uintToStr(rpmTable[i], str));
      Serial.print("\t");
    #endif
  }
  
  #ifdef SERIAL_DEBUG
    Serial.println("");
  #endif
    
}


int8_t detectSeqInverse(uint8_t risingDegrees, uint8_t fallingDegrees) {

  // note: returns the index of sequencer array, not cylNo
  uint8_t seqDegrees = fallingDegrees - risingDegrees;
  if ((seqDegrees > 82) && (seqDegrees < 90)) {
    
    return -1;
    
  } else {
                         // 46 deg of broken beam #8
    if ((seqDegrees > 42) && (seqDegrees < 50)) {

      // next cyl #8
      return 1;
                         // 56 deg of broken beam #2
    } else if ((seqDegrees > 52) && (seqDegrees < 60)) {

      // next cyl #2
      return 7;
                         // 66 deg of broken beam #5
    } else if ((seqDegrees > 62) && (seqDegrees < 70)) {

      // next cyl #5
      return 5;
                         // 76 deg of broken beam #3
    } else if ((seqDegrees > 72) && (seqDegrees < 80)) {

      // next cyl #3
      return 3;
      
    }
  
  }
  return 0;

}


int8_t detectSequence(uint8_t seqDegrees) {

  // note: returns the index of sequencer array, not cylNo
  if ((seqDegrees > 82) && (seqDegrees < 90)) {

    return -1;
    
  } else {
                       // 90 - 14 = 76 #4
    if ((seqDegrees > 72) && (seqDegrees < 80)) {

      // next cyl #4
      return 2;
                      //  90 - 24 = 66 #6
    } else if ((seqDegrees > 62) && (seqDegrees < 70)) {

      // next cyl #6
      return 4;
                      // 90 - 34 = 56 #7
    } else if ((seqDegrees > 52) && (seqDegrees < 60)) {

      // next cyl #7
      return 6;
                      // 90 - 44 = 46 #1
    } else if ((seqDegrees > 42) && (seqDegrees < 50)) {

      // next cyl #1
      return 0;
     
    }
  
  }
  return 0;
  
}


void populateDwellDegrees() {
  
  // compensate dwell for voltage and convert to degrees
  #ifdef SERIAL_DEBUG
  Serial.print("8v");
  Serial.print("\t");
  #endif
  for (int i=0; i<RPM_DIVS; i++) {

    if (i < CRANK_RPM_DIVS) { // CRANK_DWELL_TGT
      dwellTable[i].dwell8V = int ((CRANK_DWELL_TGT + VOLT_COMP_8) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell8V == 0) dwellTable[i].dwell8V = 1;
      dwellTable[i].dwell9V = int ((CRANK_DWELL_TGT + VOLT_COMP_9) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell9V == 0) dwellTable[i].dwell9V = 1;
      dwellTable[i].dwell10V = int ((CRANK_DWELL_TGT + VOLT_COMP_10) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell10V == 0) dwellTable[i].dwell10V = 1;
      dwellTable[i].dwell11V = int ((CRANK_DWELL_TGT + VOLT_COMP_11) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell11V == 0) dwellTable[i].dwell11V = 1;
      dwellTable[i].dwell12V = int (CRANK_DWELL_TGT * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell12V == 0) dwellTable[i].dwell12V = 1;
      dwellTable[i].dwell13V = int ((CRANK_DWELL_TGT + VOLT_COMP_13) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell13V == 0) dwellTable[i].dwell13V = 1;
      dwellTable[i].dwell14V = int ((CRANK_DWELL_TGT + VOLT_COMP_14) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell14V == 0) dwellTable[i].dwell14V = 1;
      dwellTable[i].dwell15V = int ((CRANK_DWELL_TGT + VOLT_COMP_15) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell15V == 0) dwellTable[i].dwell15V = 1;
      dwellTable[i].dwell16V = int ((CRANK_DWELL_TGT + VOLT_COMP_16) * 1000) / (rpmTable[i] / 90);
      if (dwellTable[i].dwell16V == 0) dwellTable[i].dwell16V = 1;
    } else {
      dwellTable[i].dwell8V = int ((DWELL_TGT + VOLT_COMP_8) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell9V = int ((DWELL_TGT + VOLT_COMP_9) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell10V = int ((DWELL_TGT + VOLT_COMP_10) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell11V = int ((DWELL_TGT + VOLT_COMP_11) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell12V = int (DWELL_TGT * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell13V = int ((DWELL_TGT + VOLT_COMP_13) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell14V = int ((DWELL_TGT + VOLT_COMP_14) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell15V = int ((DWELL_TGT + VOLT_COMP_15) * 1000) / (rpmTable[i] / 90);
      dwellTable[i].dwell16V = int ((DWELL_TGT + VOLT_COMP_16) * 1000) / (rpmTable[i] / 90);
    }
    #ifdef SERIAL_DEBUG
    Serial.print(dwellTable[i].dwell8V);
    Serial.print('\t');
    #endif
  }
  #ifdef SERIAL_DEBUG
  Serial.println("");
  Serial.print("9v");
  Serial.print('\t');
  
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell9V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("10v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell10V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("11v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell11V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("12v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell12V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("13v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell13V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("14v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell14V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("15v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell15V);
    Serial.print('\t');
  }
  Serial.println("");
  Serial.print("16v");
  Serial.print('\t');
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(dwellTable[i].dwell16V);
    Serial.print('\t');
  }
  Serial.println("");
  #endif
}


char * uintToStr( const uint64_t num, char *str )
{

  // for printing 64 bit integers
  uint8_t i = 0;
  uint64_t n = num;
 
  do
    i++;
  while ( n /= 10 );
 
  str[i] = '\0';
  n = num;
 
  do
    str[--i] = ( n % 10 ) + '0';
  while ( n /= 10 );

  return str;
  
}


uint8_t getDwell(uint64_t rpmMicros, uint64_t rpmMicrosLast, uint8_t systemVolts, uint8_t &dwellIndex) {
  
  // lookup the degrees of dwell required for the current rpm from the dwell table
  boolean dwellMatch = false;
  
  if ( rpmMicrosLast == 0 ) {

    // no hint to work from, iterate the whole array
    for (uint8_t indexTmp = 0; indexTmp < RPM_DIVS; indexTmp++ ) {
      /*Serial.print(uintToStr(rpmTable[indexTmp], str));
      Serial.print(" <> ");
      Serial.println(uintToStr(rpmMicros  , str));*/
      if (rpmTable[indexTmp] < rpmMicros) {
        if (indexTmp > 0) {
          dwellIndex = indexTmp - 1;
        }
        dwellMatch =  true;
        break;
      }
    }

    if ( ! dwellMatch )
      dwellIndex = RPM_DIVS - 1;
    
  } else {

    // use the rpmMicrosLast hint to look near last known rpm and save some iterations / instructions
    if ( rpmMicrosLast < rpmMicros ) {
      
      // engine is decelerating
      
      //char buf[30];
      //Serial.print(uintToStr(rpmMicrosLast, str));
      //Serial.print(" < ");
      //Serial.println(uintToStr(rpmMicros, str));
      //Serial.println("decel");
      
      if (dwellIndex > 0) {
        if (rpmMicros > rpmTable[dwellIndex - 1]) {
          while ( ! dwellMatch ){
            
            dwellIndex--;
            if (dwellIndex == 0) {
              dwellMatch = true;
              break;
            }
            if (rpmMicros < rpmTable[dwellIndex]) {
              dwellMatch = true;
              break;
            }
            
          }
        }
      }
      
    } else if (rpmMicrosLast > rpmMicros) {
      
      // engine is accellerating
      // TODO - may want to consider fortifying dwell by some degrees (possibly need another lookup table referenced to rpm)
      // for accelleration conditions (hint taken from megasquirt sequencer coil selection http://www.megamanual.com/seq/coils.htm)
      //Serial.println("accel");
      if (dwellIndex < (RPM_DIVS - 1)) {
        if (rpmMicros < rpmTable[dwellIndex + 1]) {
          while ( ! dwellMatch ){
            
            dwellIndex++;
            if (dwellIndex == RPM_DIVS) {
              dwellIndex = RPM_DIVS - 1;
              dwellMatch = true;
              break;
            }
            if (rpmTable[dwellIndex] < rpmMicros) {
              dwellMatch = true;
              dwellIndex--;
              break;
            }
          }
        }
      }
      
    }
    
  }

  //Serial.println(dwellIndex);
  return getDwellAtVoltage(dwellIndex, systemVolts);
  
}


void dwellCoils(uint8_t dwellDeg, uint8_t sparkAdvDeg, uint8_t posDeg, int8_t seqIndex) {

  // look at the current crankshaft location (posDeg = degrees BTDC)
  // and determine if it's time to start dwelling a coil
  
  for (uint8_t lookAhead = 0; lookAhead < 4; lookAhead++) {

    // if the current coil is not armed step out of this branch
    if (sequencer[seqIndex].armed == 1) {

      if (posDeg > 90) posDeg = 0;
      
      //if (((sparkAdvDeg + dwellDeg) >= (lookAhead * 90)) && ((sparkAdvDeg + dwellDeg) < ((lookAhead + 1) * 90))
      //  && (sparkAdvDeg < posDeg)) {
      
      if (((sparkAdvDeg + dwellDeg) >= (lookAhead * 90)) && ((sparkAdvDeg + dwellDeg) < ((lookAhead + 1) * 90))) {
        
        // check if dwell is required based on current hrDegrees count (passed in as posDeg)
        if ((posDeg + (lookAhead * 90)) <= (sparkAdvDeg + dwellDeg)) {

          // begin dwelling currently indexed coil
          *sequencer[seqIndex].portAddr |= sequencer[seqIndex].bitMask;
          sequencer[seqIndex].armed = 0;
          switch (lookAhead) {
            case 0:
              dwellLag += (sparkAdvDeg + dwellDeg) - posDeg;
              break;
            case 1:
              dwellLag += ((sparkAdvDeg + dwellDeg) - posDeg) - 90;
              break;
            case 2:
              dwellLag += ((sparkAdvDeg + dwellDeg) - posDeg) - 180;
              break;
            case 3:
              dwellLag += ((sparkAdvDeg + dwellDeg) - posDeg) - 270;
          }
          #ifdef SERIAL_DEBUG
            Serial.print("c");
            Serial.print(sequencer[seqIndex].cylNo);
            Serial.print(" dwl ");
            Serial.print(posDeg);
            Serial.print(" err ");
            Serial.print(dwellLag);
            Serial.print(" la ");
            Serial.println(lookAhead);
          #endif
          
        }
      }
    }
    
    // check if need to consider dwelling next cylinder
    if ((sparkAdvDeg + dwellDeg) >= ((lookAhead + 1) * 90)) {
      seqIndex++;
      //lookAhead++;
      if ( seqIndex == NUM_CYLINDERS )
        seqIndex = 0;
    } else {
      break;
    }
 
  }
  
}


uint8_t getDwellAtVoltage(uint8_t dwellInd, uint8_t sysVolts) {

  // returns dwell degrees from the dwellTable at given sysVolts
  // voltages in switch tree are in ordered my most likely to least likely
  // if there's a more elegant or faster way to do this i'm open to suggestions
  switch (sysVolts) {

    case (13):
      return dwellTable[dwellInd].dwell13V;
      break;
      
    case (14):
      return dwellTable[dwellInd].dwell14V;
      break;

    case (12):
      return dwellTable[dwellInd].dwell12V;
      break;

    case (11):
      return dwellTable[dwellInd].dwell11V;
      break;

    case (15):
      return dwellTable[dwellInd].dwell15V;
      break;

    case (10):
      return dwellTable[dwellInd].dwell10V;
      break;

    case (9):
      return dwellTable[dwellInd].dwell9V;
      break;

    case (8):
      return dwellTable[dwellInd].dwell8V;
      break;

    case (16):
      return dwellTable[dwellInd].dwell16V;
      break;
   
  }  
}

void cylInfoUart() {
  
  char str[20];
  Serial.print("cyl=");
  if (sequenceIndex < 0) {
    Serial.println(sequenceIndex);
  } else {
    Serial.println(sequencer[sequenceIndex].cylNo);
  }
  Serial.print("sa=");
  Serial.println(saDegrees); 
  Serial.print("dwl=");
  Serial.println(dwellDegrees);
  Serial.print("lri=");
  Serial.println(uintToStr(rpmTimeHi, str));
  Serial.print("deg=");
  Serial.println(lrSemaphoreRising);

}
