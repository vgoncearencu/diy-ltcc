////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  this code is to be considered experimental and is provided "as is", without warranty of any kind, 
//  express or implied, including but not limited to fitness for a particular purpose.
//  if your LT1 comes apart using it i will not be held responsible
//  you were warned
//  
//  this software is licensed under GNU General Public Licencse v3 - please see file: LICENSE in repository
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define VERSION "0.9.7"

// only select one of the below options (or none)
//#define SERIAL_DEBUG 1    // dumps debugging level info to the uart - not intended for normal operation as the uart out buffer may overrun and crash the mcu
//#define LOGGING 1         // streams operational stats to the uart
//#define INPUT_TEST 1      // mirrors state of inputs to leds - EST illuminates the tach driver, opti hi-res cyl #6, opti lo-res cyl #8
//#define DUMP_TABLES 1     // generate static initializers for the dwell tables

// or one or both for simulator mode
//#define SIMULATE 1
//#define DETAIL_SIM 1      // causes detailed uart logging of simulation routines

// if you require a tach driver on A0 uncomment - note the A0 output is still used / configured for INPUT_TEST mode
//#define TACH_DRIVER 1


// coil constants - uncomment the ones you have or create your own dwell tables from test data

// for GM 12658183 coils
#define DWELL_TGT 4.5       // dwell target in milliseconds - edit to your liking (using 4.5ms @ 12v for standard ls2 coils)
#define CRANK_DWELL_TGT 3.2 // dwell target for cranking / startup
#define ACCEL_COMP 0.5      // for accell comp table

// for ls1 / D580 coils
//#define DWELL_TGT 6.1       // dwell target in milliseconds - edit to your liking
//#define CRANK_DWELL_TGT 3.6 // dwell target for cranking / startup
//#define ACCEL_COMP 0.6      // for accell comp table

#define VOLT_COMP_8 2.4     // adds / subtracts from dwell target at given input voltage
#define VOLT_COMP_9 1.5
#define VOLT_COMP_10 0.9
#define VOLT_COMP_11 0.4
#define VOLT_COMP_12 0.0
#define VOLT_COMP_13 -0.2
#define VOLT_COMP_14 -0.5
#define VOLT_COMP_15 -0.7
#define VOLT_COMP_16 -0.9
#define VOLT_COMP_17 -1.0

#define BUILD_ENV_DPTI 1 // comment out if you get compiler errors about undeclared digitalPinToInterrupt() function

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//  end of typical user controls - do not edit below this line unless you know what you're doing
//
//////////////////////////////////////////////////////////////////////////////////////////////////

// some pin definitions - mainly informational / used in setup() but most port control is handled w/ direct reads / writes (faster)
#define PCM_EST 2
#define OPTI_HI_RES 3
#define OPTI_LO_RES 8   // must use pin 8 for capture interrupt
#define TACH_SIGNAL A0  // statically assigned to tachOutput[] structure
#define IGN_VSENSE 5    // voltage sensing on adc pin 5

#define NUM_CYLINDERS 8
#define MAX_SPK_ADV 46      // for calculating maximum dwell begin angle (dwellDegrees + accelComp + MAX_SPK_ADV)
#define CRANKING_SPK_ADV 10 // arbitrary value to use before we pickup commanded sa from the pcm
#define LR_OVERFLOW 20      // low res timer overflow limit - engine has stalled if timer1 overflows this many times (21 x 32676us = 686.196 ms between tdc signals - about 22rpm)

#define VOLT_DIVS 10      // the # of voltage rows in the dwell arrays see VOLT_COMP_* declarations
#define VOLT_OFFSET 8     // the starting voltage offset (8v = dwell[0][rpm]) so lookup is dwell[volts - volt_offset][rpm]
#define RPM_DIVS 34       // the # of rpm cols in the dwell arrays
#define CRANK_RPM_DIVS 9  // these low rpm cells (25-200rpm) get lesser CRANK_DWELL_TGT

#define RPM_DIV_1 25
#define RPM_DIV_2 50
#define RPM_DIV_3 75
#define RPM_DIV_4 100
#define RPM_DIV_5 125
#define RPM_DIV_6 150
#define RPM_DIV_7 175
#define RPM_DIV_8 200
#define RPM_DIV_9 400
#define RPM_DIV_10 600
#define RPM_DIV_11 800
#define RPM_DIV_12 1000
#define RPM_DIV_13 1200
#define RPM_DIV_14 1400
#define RPM_DIV_15 1600
#define RPM_DIV_16 1800
#define RPM_DIV_17 2000
#define RPM_DIV_18 2200
#define RPM_DIV_19 2400
#define RPM_DIV_20 2600
#define RPM_DIV_21 2800
#define RPM_DIV_22 3000
#define RPM_DIV_23 3200
#define RPM_DIV_24 3400
#define RPM_DIV_25 3600
#define RPM_DIV_26 3800
#define RPM_DIV_27 4000
#define RPM_DIV_28 4400
#define RPM_DIV_29 4800
#define RPM_DIV_30 5200
#define RPM_DIV_31 5600
#define RPM_DIV_32 6000
#define RPM_DIV_33 6400
#define RPM_DIV_34 6800

#define RPM_TO_US(x) (( 250000UL * 60UL / ( x ) )) // Convert RPM to microseconds per 90* (us between low res signals)

// (((CRANK_DWELL_TGT + VOLT_COMP_8) * 1000.0) / (float (rpmTable[i] / 90)) + 0.5)
#define CALC_DWELL(rpm, volts) (((DWELL_TGT + VOLT_COMP_ ## volts ) * 90000UL / (RPM_TO_US(rpm))))

#define CALC_CRANK_DWELL(rpm, volts) ((int _CRANK_DWELL(rpm, volts) == 0) ? 1:_CRANK_DWELL(rpm, volts))

#define _CRANK_DWELL(rpm, volts) (((CRANK_DWELL_TGT + VOLT_COMP_ ## volts ) * 90000UL / (RPM_TO_US(rpm))))

#define CALC_ACCEL_COMP(rpm) (((ACCEL_COMP) * 90000UL / (RPM_TO_US(rpm))))

#define CALC_DWELLBEGIN(rpm, volts, spk_adv) ((CALC_DWELL(rpm, volts) + CALC_ACCEL_COMP(rpm) + spk_adv))

#define CALC_LOOKAHEAD(rpm, volts) ((CALC_DWELLBEGIN(rpm, volts, MAX_SPK_ADV) / 90) + HAS_REMAINDER(int CALC_DWELLBEGIN(rpm, volts, MAX_SPK_ADV)) + 1)

#define HAS_REMAINDER(degs) ((degs % 90==0) ? 1:0)


#define FILL_DWELL_COLUMN(rpm) { (CALC_DWELL(rpm, 8)), \
                                 (CALC_DWELL(rpm, 9)),  \
                                 (CALC_DWELL(rpm, 10)), \
                                 (CALC_DWELL(rpm, 11)), \
                                 (CALC_DWELL(rpm, 12)), \
                                 (CALC_DWELL(rpm, 13)), \
                                 (CALC_DWELL(rpm, 14)), \
                                 (CALC_DWELL(rpm, 15)), \
                                 (CALC_DWELL(rpm, 16)), \
                                 (CALC_DWELL(rpm, 17))} 

#define FILL_CRANK_DWELL_COLUMN(rpm) { (CALC_CRANK_DWELL(rpm, 8)), \
                                       (CALC_CRANK_DWELL(rpm, 9)),  \
                                       (CALC_CRANK_DWELL(rpm, 10)), \
                                       (CALC_CRANK_DWELL(rpm, 11)), \
                                       (CALC_CRANK_DWELL(rpm, 12)), \
                                       (CALC_CRANK_DWELL(rpm, 13)), \
                                       (CALC_CRANK_DWELL(rpm, 14)), \
                                       (CALC_CRANK_DWELL(rpm, 15)), \
                                       (CALC_CRANK_DWELL(rpm, 16)), \
                                       (CALC_CRANK_DWELL(rpm, 17))}

// the dwell table - contains dwell in degrees required to meet DWELL_TGT at different rpms
const uint16_t PROGMEM dwellTable[RPM_DIVS][VOLT_DIVS] = {FILL_CRANK_DWELL_COLUMN(RPM_DIV_1), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_2), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_3), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_4), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_5), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_6), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_7), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_8), 
                                            FILL_CRANK_DWELL_COLUMN(RPM_DIV_9), 
                                            FILL_DWELL_COLUMN(RPM_DIV_10),      
                                            FILL_DWELL_COLUMN(RPM_DIV_11),      
                                            FILL_DWELL_COLUMN(RPM_DIV_12),      
                                            FILL_DWELL_COLUMN(RPM_DIV_13),      
                                            FILL_DWELL_COLUMN(RPM_DIV_14),      
                                            FILL_DWELL_COLUMN(RPM_DIV_15),      
                                            FILL_DWELL_COLUMN(RPM_DIV_16),      
                                            FILL_DWELL_COLUMN(RPM_DIV_17),      
                                            FILL_DWELL_COLUMN(RPM_DIV_18),      
                                            FILL_DWELL_COLUMN(RPM_DIV_19),      
                                            FILL_DWELL_COLUMN(RPM_DIV_20),      
                                            FILL_DWELL_COLUMN(RPM_DIV_21),      
                                            FILL_DWELL_COLUMN(RPM_DIV_22),      
                                            FILL_DWELL_COLUMN(RPM_DIV_23),      
                                            FILL_DWELL_COLUMN(RPM_DIV_24),      
                                            FILL_DWELL_COLUMN(RPM_DIV_25),      
                                            FILL_DWELL_COLUMN(RPM_DIV_26),    
                                            FILL_DWELL_COLUMN(RPM_DIV_27),      
                                            FILL_DWELL_COLUMN(RPM_DIV_28),      
                                            FILL_DWELL_COLUMN(RPM_DIV_29),      
                                            FILL_DWELL_COLUMN(RPM_DIV_30),      
                                            FILL_DWELL_COLUMN(RPM_DIV_31),      
                                            FILL_DWELL_COLUMN(RPM_DIV_32),      
                                            FILL_DWELL_COLUMN(RPM_DIV_33),      
                                            FILL_DWELL_COLUMN(RPM_DIV_34)};

#ifdef LOGGING
const uint16_t rpmReference[RPM_DIVS] PROGMEM = {RPM_DIV_1,
                                                 RPM_DIV_2,
                                                 RPM_DIV_3,
                                                 RPM_DIV_4,
                                                 RPM_DIV_5,
                                                 RPM_DIV_6,
                                                 RPM_DIV_7,
                                                 RPM_DIV_8,
                                                 RPM_DIV_9,
                                                 RPM_DIV_10,
                                                 RPM_DIV_11,
                                                 RPM_DIV_12,
                                                 RPM_DIV_13,
                                                 RPM_DIV_14,
                                                 RPM_DIV_15,
                                                 RPM_DIV_16,
                                                 RPM_DIV_17,
                                                 RPM_DIV_18,
                                                 RPM_DIV_19,
                                                 RPM_DIV_20,
                                                 RPM_DIV_21,
                                                 RPM_DIV_22,
                                                 RPM_DIV_23,
                                                 RPM_DIV_24,
                                                 RPM_DIV_25,
                                                 RPM_DIV_26,
                                                 RPM_DIV_27,
                                                 RPM_DIV_28,
                                                 RPM_DIV_29,
                                                 RPM_DIV_30,
                                                 RPM_DIV_31,
                                                 RPM_DIV_32,
                                                 RPM_DIV_33,
                                                 RPM_DIV_34};
#endif

// the rpm lookup for the dwell table - contains rpm reference in microseconds per 90* (between low res signals)
const PROGMEM uint64_t rpmTable[RPM_DIVS] = {RPM_TO_US(RPM_DIV_1),
                                             RPM_TO_US(RPM_DIV_2),
                                             RPM_TO_US(RPM_DIV_3),
                                             RPM_TO_US(RPM_DIV_4),
                                             RPM_TO_US(RPM_DIV_5),
                                             RPM_TO_US(RPM_DIV_6),
                                             RPM_TO_US(RPM_DIV_7),
                                             RPM_TO_US(RPM_DIV_8),
                                             RPM_TO_US(RPM_DIV_9),
                                             RPM_TO_US(RPM_DIV_10),
                                             RPM_TO_US(RPM_DIV_11),
                                             RPM_TO_US(RPM_DIV_12),
                                             RPM_TO_US(RPM_DIV_13),
                                             RPM_TO_US(RPM_DIV_14),
                                             RPM_TO_US(RPM_DIV_15),
                                             RPM_TO_US(RPM_DIV_16),
                                             RPM_TO_US(RPM_DIV_17),
                                             RPM_TO_US(RPM_DIV_18),
                                             RPM_TO_US(RPM_DIV_19),
                                             RPM_TO_US(RPM_DIV_20),
                                             RPM_TO_US(RPM_DIV_21),
                                             RPM_TO_US(RPM_DIV_22),
                                             RPM_TO_US(RPM_DIV_23),
                                             RPM_TO_US(RPM_DIV_24),
                                             RPM_TO_US(RPM_DIV_25),
                                             RPM_TO_US(RPM_DIV_26),
                                             RPM_TO_US(RPM_DIV_27),
                                             RPM_TO_US(RPM_DIV_28),
                                             RPM_TO_US(RPM_DIV_29),
                                             RPM_TO_US(RPM_DIV_30),
                                             RPM_TO_US(RPM_DIV_31),
                                             RPM_TO_US(RPM_DIV_32),
                                             RPM_TO_US(RPM_DIV_33),
                                             RPM_TO_US(RPM_DIV_34)};


const uint8_t PROGMEM accelComp[RPM_DIVS] = {CALC_ACCEL_COMP(RPM_DIV_1),
                                             CALC_ACCEL_COMP(RPM_DIV_2),
                                             CALC_ACCEL_COMP(RPM_DIV_3),
                                             CALC_ACCEL_COMP(RPM_DIV_4),
                                             CALC_ACCEL_COMP(RPM_DIV_5),
                                             CALC_ACCEL_COMP(RPM_DIV_6),
                                             CALC_ACCEL_COMP(RPM_DIV_7),
                                             CALC_ACCEL_COMP(RPM_DIV_8),
                                             CALC_ACCEL_COMP(RPM_DIV_9),
                                             CALC_ACCEL_COMP(RPM_DIV_10),
                                             CALC_ACCEL_COMP(RPM_DIV_11),
                                             CALC_ACCEL_COMP(RPM_DIV_12),
                                             CALC_ACCEL_COMP(RPM_DIV_13),
                                             CALC_ACCEL_COMP(RPM_DIV_14),
                                             CALC_ACCEL_COMP(RPM_DIV_15),
                                             CALC_ACCEL_COMP(RPM_DIV_16),
                                             CALC_ACCEL_COMP(RPM_DIV_17),
                                             CALC_ACCEL_COMP(RPM_DIV_18),
                                             CALC_ACCEL_COMP(RPM_DIV_19),
                                             CALC_ACCEL_COMP(RPM_DIV_20),
                                             CALC_ACCEL_COMP(RPM_DIV_21),
                                             CALC_ACCEL_COMP(RPM_DIV_22),
                                             CALC_ACCEL_COMP(RPM_DIV_23),
                                             CALC_ACCEL_COMP(RPM_DIV_24),
                                             CALC_ACCEL_COMP(RPM_DIV_25),
                                             CALC_ACCEL_COMP(RPM_DIV_26),
                                             CALC_ACCEL_COMP(RPM_DIV_27),
                                             CALC_ACCEL_COMP(RPM_DIV_28),
                                             CALC_ACCEL_COMP(RPM_DIV_29),
                                             CALC_ACCEL_COMP(RPM_DIV_30),
                                             CALC_ACCEL_COMP(RPM_DIV_31),
                                             CALC_ACCEL_COMP(RPM_DIV_32),
                                             CALC_ACCEL_COMP(RPM_DIV_33),
                                             CALC_ACCEL_COMP(RPM_DIV_34)};

const uint8_t PROGMEM maxLookAhead[VOLT_DIVS] = { CALC_LOOKAHEAD(RPM_DIV_34, 8),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 9),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 10),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 11),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 12),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 13),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 14),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 15),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 16),
                                                   CALC_LOOKAHEAD(RPM_DIV_34, 17)};

volatile int8_t sequenceIndex = -1;     // if negative we've lost sequence, else pointer to the current cylinder in the sequencer[] struct
volatile boolean invertSeq;             // invert sequence detection - when true the engine came to rest with the low res trigger in a gap (beam not broken)
volatile uint8_t hrDegrees = 90;        // the high resolution counter - counts down from 90 after each falling edge low res pulse
volatile uint8_t saDegrees;             // stored by the EST line interrupt this is a snapshot of most recent spark advance in degrees btdc
volatile uint8_t estEnabler;            // count degrees while est is high inside high res isr - used to debounce est input

volatile uint8_t errCount;              // for below error and lag statistics
uint8_t errorDegs;                      // hrDegrees should be 0 when the low res TDC signal is caught - this tracks cumulative # of errors per last 10 revolutions
uint8_t dwellLag;                       // difference between targeted dwell and when dwell actually started - cumulative over last 10 revolutions
#define ERROR_LOGS 10                   // match constant for resetting errCount and above statistics

volatile uint8_t lrSemaphoreRising;     // triggers sequence detection in main loop
volatile uint8_t lrSemaphoreFalling;    // used for sequence detection when invertSeq = true

volatile unsigned long lrTimeHi;        // microseconds between low res falling edges also used as semaphore to trigger dwell lookup in loop()
volatile uint8_t lrHi_overflow;         // the timer1 overflow counter - 16 bit unsigned overflows at 65535 - timer1 prescaler is 8 so one "tick" every .5 us
uint64_t lrHi_overflow_const = 32767;   // used to calculate the most recent time interval bewteen low res rising edge signals for rpm lookup
uint64_t rpmTimeHi;                     // holds the low-res interval - calculated in loop()

//float ignVolts_f;                       // precision voltage for logging
uint16_t ignMillivolts = 0;             // system voltage represented in millivolts
uint8_t ignVolts_gross = 10;            // ignVolts_f rounded to single byte integer - initialize to 10 for sane dwell lookup before first adc conversion
uint8_t ignVolts_dec;                   // voltage lsb

//const float voltFormula = 0.017361806;  // = (vref / (r2 / (r1 + r2))) / 1024  NOTE: for best precision calculate from measurements of vref, r1 and r2 with calibrated equipment
const float mvFormula = 1.7361806;      // = ((vref / (r2 / (r1 + r2))) / 1024) * 100 NOTE: for best precision calculate from measurements of vref, r1 and r2 with calibrated equipment

uint8_t timer2OFC;                      // timer2 overflow counter
uint8_t timer2OFCompare = 8;            // scale this multiplier with rpm to reduce uart lag
volatile boolean logFlag;               // flag to prompt log data output and voltage detection

typedef struct {
  int cylNo;          // user readable cylinder no
  int armed;          // set to 1 = armed for dwell, set to 0 = est signal caught - prevents dwell routine from dwelling recently fired coil
  volatile uint8_t *portAddr;  // used at a pointer to the output registers so coil drivers can be controlled directly (faster than using digital[write/read]() arduino functions)
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
  {0, 0, &PORTC, B00000001}   // arduino A0
};

void setup() {

  // configure misc output pins
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
  pinMode(PCM_EST, INPUT);
  
  // setup adc input for voltage comparator
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);    // prescaler 128 - "slowest" prescaler gives most accuracy 
  ADMUX  =  bit (REFS0) | (IGN_VSENSE & 0x07);          // select AVcc for aref and select input port
  
  // configure output pins
  DDRD = DDRD | B11110000; // configure PD4 - PD7 (arduino d4 - d7) as outputs
  DDRB = DDRB | B00111110; // configure PB1 - PB5 (arduino d9 - d12) as outputs
  
  Serial.begin(115200);
  Serial.print(F("diy-ltcc-"));
  Serial.println(VERSION);

  #ifdef DUMP_TABLES
    dumpRPMTable();
    dumpDwellTable();
    dumpAccelComp();
    dumpLookAhead();
  #endif
  
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
  OCR2A = 192; //192*1024*9/16mhz = 0.110592 s - see timer2OFCompare definition (timer 2 multiplexer) NOTE: overflow counter 9 (zero indexed 8) scales with rpmIndex so logging
  TCCR2A |= (1 << WGM21);                                                                    // output and voltage detection / conversion happens less often at higher rpm
  TCCR2B |= (1<<CS12)|(1<<CS10); // 1024 prescaler   
  TIMSK2 |= (1 << OCIE2A);
  
  sei();

}


void isr_est() {

#ifndef SIMULATE // disable if simulating b/c if left open this line picks up noise from the outputs, messing with saDegrees
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
    // set pin 13 high
    PORTB |= B00100000;
    
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
        *tachOutput[0].portAddr &= ~tachOutput[0].bitMask; // drive the tach output low - needs testing
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

  static uint16_t dwellDegrees;     // current dwell requirement in degrees
  static uint64_t rpmTimeLast = 0;  // for reference in getDwellDegrees lookup function
  static uint8_t rpmIndex = 0;      // for human readable rpm and dwell lookup hint
  static boolean adcConv = false;   // for adc conversion request state machine
  static uint8_t voltsIndex = 0;    // for voltage lookups and maxLookAhead[] in dwellCoils
  boolean stalled = false;

  #ifdef SIMULATE

    // sim mode branch for testing functionality
    static uint8_t simMode = 5;
    static uint64_t loopTimer;              // comparator for switching low res pulses
    static uint64_t loopInterval = 180;     // how long between low res pulses
    static uint32_t simTime = 0;            // comparator for switching between simulation modes
    static uint64_t displayInterval = 180;  // scale to slow display of sequence
    static uint64_t lowResInterval = loopInterval * 90;      // how many micros per low res falling edge for dwell lookup
    uint64_t thisLoop;
    static uint8_t revCounter = 0;
    voltsIndex = 14 - VOLT_OFFSET;          // simulate @ 14.4v
    
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
      dwellDegrees = getDwell(lowResInterval, rpmTimeLast, voltsIndex, rpmIndex);
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
        dwellDegrees = getDwell(lowResInterval, rpmTimeLast, voltsIndex, rpmIndex);
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
        dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
       
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
        // turn off pin 13 led
        PORTB &= ~B00100000;
        #ifdef SERIAL_DEBUG
           Serial.println("sequenced");
        #endif

      }
      
    }
    // end sequence detection

    // check if we need to dwell coils (#1)
    if (dwellDegrees > 0) dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
    
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
        dwellDegrees = getDwell(pgm_read_dword(&rpmTable[0]) + 1, 0, voltsIndex, rpmIndex);
      } else {
        dwellDegrees = getDwell(rpmTimeHi, rpmTimeLast, voltsIndex, rpmIndex); // might want to consider doing this less freq at higher rpms
        // re-arm the previous coil and make sure it fired (we're past tdc)
        // TODO - this could potentially keep the engine running albeit at 0 degrees advance
        // if we lose the est signal
        if (sequenceIndex == 0) {
          *sequencer[NUM_CYLINDERS - 1].portAddr &= ~sequencer[NUM_CYLINDERS - 1].bitMask;
          sequencer[NUM_CYLINDERS - 1].armed = 1;
        } else {
          *sequencer[sequenceIndex - 1].portAddr &= ~sequencer[sequenceIndex - 1].bitMask;
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
          cylInfoUart(dwellDegrees);
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
        /*// stalled - shut down any dwelling coils - NOTE this doesn't seem to work (i.e. when no EST signal to fire coils) - will blow ign fuse if too many coils dwell at once
        #ifndef INPUT_TEST
        for (uint8_t i = 0; i < NUM_CYLINDERS; i++) {
          *sequencer[i].portAddr &= ~sequencer[i].bitMask;
          sequencer[i].armed = 1;
        }
        #endif */
        // TODO - test by removing est signal, injector power, and mechanically stalling w/ clutch
        // engine should restart without switching ign off
        dwellDegrees = 0;
        rpmIndex = 0;
        #ifdef SERIAL_DEBUG
          Serial.println("stalled");
        #endif
      } else {
        dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
      }
    }
    
  #endif


  // output logging data and request adc conversion periodically - triggered by timer2 (TODO: verify this timer #!!!)
  if (logFlag) {

    // ign voltage detection
    if (! adcConv) {
      bitSet (ADCSRA, ADSC);  // start a conversion
      adcConv = true;
    } else {
      // the ADC clears the bit when done
      if (bit_is_clear(ADCSRA, ADSC)) {
        adcConv = false;
        //ignVolts_f = float (ADC) * voltFormula;
        //ignVolts_gross = uint8_t (ignVolts_f + 0.5);
        // not sure if this is any faster than converting to float
        ignMillivolts = (ADC) * mvFormula;
        ignVolts_gross = ignMillivolts / 100;
        ignVolts_dec = ignMillivolts - (ignVolts_gross * 100);
        
        // find the index for use in lookups
        if (ignVolts_gross > 7 && ignVolts_gross < 18) {
          voltsIndex = ignVolts_gross - VOLT_OFFSET;
          if (ignVolts_dec > 49) {
            // rounding
            if (ignVolts_gross == 17) {
              voltsIndex = VOLT_DIVS - 1;
            } else {
              voltsIndex = voltsIndex + 1;
            }
          }          
        } else {
          if (ignVolts_gross < 8) voltsIndex = 0;
          if (ignVolts_gross > 17) voltsIndex = VOLT_DIVS - 1;
        }
      }
    }
        
    logFlag = false;
    #ifdef LOGGING
    char serial_out[35]; //Rnnnn:Snn:Dnnn:Cn:Vnn.nn:Ennn:Lnnn - if you add anything here make sure to increase the serial_out buffer size accordingly (used by sprintf)
    const char outfmt_std[31] PROGMEM = "R%d:S%d:D%d:C%d:V%d.%d:E%d:L%d\0";
    const char outfmt_0[32] PROGMEM = "R%d:S%d:D%d:C%d:V%d.0%d:E%d:L%d\0";
    uint8_t cylNo;
    if ( sequenceIndex < 0 ) {
      cylNo = 0;
    } else {
      cylNo = sequencer[sequenceIndex].cylNo;
    }
    if (ignVolts_dec < 10) {
      sprintf(serial_out, outfmt_0, pgm_read_word(&rpmReference[rpmIndex]), saDegrees, dwellDegrees, cylNo, ignVolts_gross, ignVolts_dec, errorDegs, dwellLag);
    } else {
      sprintf(serial_out, outfmt_std, pgm_read_word(&rpmReference[rpmIndex]), saDegrees, dwellDegrees, cylNo, ignVolts_gross, ignVolts_dec, errorDegs, dwellLag);
    }
    Serial.println(serial_out);
    errCount++;
    #endif
    
  }

  #ifndef SIMULATE
    // check if we need to dwell coils (#3)
    if (dwellDegrees > 0) dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
  #endif
  
  // update / clear error statistics
  if (errCount >= ERROR_LOGS) {
    errCount = 0;
    errorDegs = 0;
    dwellLag = 0;
  }
  
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


#ifdef DUMP_TABLES

void dumpRPMTable() {
  
  // dump rpmTable to uart
  Serial.println("dumping uint64_t rpmTable[RPM_DIVS] PROGMEM = {");
  Serial.print("  ");
  
  char str[30];
  for (int i=0; i<RPM_DIVS; i++) {
    Serial.print(uintToStr(pgm_read_dword(&rpmTable[i]), str));
    if (i < RPM_DIVS - 1) Serial.print(",  ");
  }
  
  Serial.println(" };");
  Serial.println("");
    
}


void dumpDwellTable() {
  
  // print out the dwell table to uart
  //Serial.println();
  Serial.println("dumping uint16_t dwellTable[RPM_DIVS][VOLT_DIVS] PROGMEM = {");
  Serial.print("  { ");
  for (int v=0; v<VOLT_DIVS; v++) {
    for (int i=0; i<RPM_DIVS; i++) {
      Serial.print(pgm_read_word(&dwellTable[i][v]));
      if (i < RPM_DIVS - 1) Serial.print(",  ");
    }
    if (v == VOLT_DIVS - 1) {
      Serial.println(" }");
      Serial.println("};");
      Serial.println("");
    } else {
      Serial.println(" },");
      Serial.print("  { ");
    }
  }

}


void dumpAccelComp() {
  
  // compensate dwell adder for voltage and convert to degrees
  Serial.print("dumping uint8_t accelComp[RPM_DIVS] PROGMEM = { ");
  for (int i=0; i<RPM_DIVS; i++) {
    
    //accelComp[i] = uint8_t ((ACCEL_COMP * 1000.0) / float (pgm_read_dword(&rpmTable[i]) / 90) + 0.5);
    Serial.print(pgm_read_byte(&accelComp[i]));
    if (i < RPM_DIVS - 1) Serial.print(",  ");
  }
  Serial.println(" };");

}

void dumpLookAhead() {
  
  // dump maxLookAhead[]
  Serial.print("dumping uint8_t maxLookAhead[VOLT_DIVS] PROGMEM = { ");
  for (int i=0; i<VOLT_DIVS; i++) {
    
    Serial.print(pgm_read_byte(&maxLookAhead[i]));
    if (i < VOLT_DIVS - 1) Serial.print(",  ");
  }
  Serial.println(" };");

}
#endif

char * uintToStr( const uint64_t num, char *str )
{
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


uint16_t getDwell(uint64_t rpmMicros, uint64_t rpmMicrosLast, uint8_t voltIndex_l, uint8_t &dwellIndex) {
  
  // lookup the degrees of dwell required for the current rpm from the dwell table
  boolean dwellMatch = false;
  uint8_t accelCompAdder = 0;
  
  if ( rpmMicrosLast == 0 ) {

    // no hint to work from, iterate the whole array
    for (uint8_t indexTmp = 0; indexTmp < RPM_DIVS; indexTmp++ ) {
      /*Serial.print(uintToStr(rpmTable[indexTmp], str));
      Serial.print(" <> ");
      Serial.println(uintToStr(rpmMicros  , str));*/
      if (pgm_read_dword(&rpmTable[indexTmp]) < rpmMicros) {
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
        if (rpmMicros > pgm_read_dword(&rpmTable[dwellIndex - 1])) {
          while ( ! dwellMatch ){
            
            dwellIndex--;
            if (dwellIndex == 0) {
              dwellMatch = true;
              break;
            }
            if (rpmMicros < pgm_read_dword(&rpmTable[dwellIndex])) {
              dwellMatch = true;
              break;
            }
            
          }
        }
      }
      
    } else if (rpmMicrosLast > rpmMicros) {
      
      // engine is accellerating
      //Serial.println("accel");
      if (dwellIndex < (RPM_DIVS - 1)) {
        if (rpmMicros < pgm_read_dword(&rpmTable[dwellIndex + 1])) {
          while ( ! dwellMatch ){
            
            dwellIndex++;
            if (dwellIndex == RPM_DIVS) {
              dwellIndex = RPM_DIVS - 1;
              dwellMatch = true;
              break;
            }
            if (pgm_read_dword(&rpmTable[dwellIndex]) < rpmMicros) {
              dwellMatch = true;
              dwellIndex--;
              break;
            }
          }
        }
      }
      // add some accel compensation
      // for accelleration conditions (hint taken from megasquirt sequencer coil selection http://www.megamanual.com/seq/coils.htm)
      if (dwellIndex > CRANK_RPM_DIVS) accelCompAdder = pgm_read_byte(&accelComp[dwellIndex]);
      
    }
    
  }

  return pgm_read_word(&dwellTable[dwellIndex][voltIndex_l]) + accelCompAdder;
  
}


void dwellCoils(uint16_t dwellDeg, uint8_t sparkAdvDeg, uint8_t posDeg, int8_t seqIndex, uint8_t voltIndex_l) {

  // look at the current crankshaft location (posDeg = degrees BTDC)
  // and determine if it's time to start dwelling a coil
  uint8_t maxLA = pgm_read_byte(&maxLookAhead[voltIndex_l]);
  for (uint8_t lookAhead = 0; lookAhead < maxLA; lookAhead++) {

    // if the current coil is not armed step out of this branch
    if (sequencer[seqIndex].armed == 1) {

      if (posDeg > 90) posDeg = 0;
      
      if (((sparkAdvDeg + dwellDeg) >= (lookAhead * 90)) && ((sparkAdvDeg + dwellDeg) < ((lookAhead + 1) * 90))) {
        
        // check if dwell is required based on current hrDegrees count (passed in as posDeg)
        if ((posDeg + (lookAhead * 90)) <= (sparkAdvDeg + dwellDeg)) {

          // begin dwelling currently indexed coil
          *sequencer[seqIndex].portAddr |= sequencer[seqIndex].bitMask;
          sequencer[seqIndex].armed = 0;
          dwellLag += ((sparkAdvDeg + dwellDeg) - posDeg) - (lookAhead * 90);
          
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
      if ( seqIndex == NUM_CYLINDERS )
        seqIndex = 0;
    } else {
      break;
    }
 
  }
  
}


void cylInfoUart(uint16_t dwellDeg) {
  
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
  Serial.println(dwellDeg);
  Serial.print("lri=");
  Serial.println(uintToStr(rpmTimeHi, str));
  Serial.print("deg=");
  Serial.println(lrSemaphoreRising);

}
