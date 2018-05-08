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


#define VERSION "0.9.12"

// only select one of the below options (or none)
//#define SERIAL_DEBUG 1    // dumps debugging level info to the uart - not intended for normal operation as the uart out buffer may overrun and crash the mcu
//#define LOGGING 1         // streams operational stats to the uart
//#define INPUT_TEST 1      // mirrors state of inputs to leds - EST illuminates the tach driver, opti hi-res cyl #6, opti lo-res cyl #8
//#define DUMP_TABLES 1     // dump all the tables to uart to verify values
//#define PROFILE 1         // for profiling program execution

// if you require a tach driver on A0 uncomment - note the A0 output is still used / configured for INPUT_TEST mode
//#define TACH_DRIVER 1

// ion sense trigger on A1
//#define ION_TRIGGER 1
//#define ION_TRG_DEGREE 74

// coil constants - uncomment the ones you have or create your own dwell tables from test data

// for GM 12658183 coils / D585
#define DWELL_TGT 4.5       // dwell target in milliseconds - edit to your liking (using 4.5ms @ 12v for standard ls2 coils)
#define CRANK_DWELL_TGT 3.2 // dwell target for cranking / startup
#define ACCEL_COMP 0.5      // for accell comp table

// for ls1 / D580 coils
//#define DWELL_TGT 6.1       // dwell target in milliseconds - edit to your liking
//#define CRANK_DWELL_TGT 3.6 // dwell target for cranking / startup
//#define ACCEL_COMP 0.6      // for accell comp table

#define VOLT_COMP_8 2.4     // adds / subtracts ms from dwell target at given input voltage
#define VOLT_COMP_9 1.5
#define VOLT_COMP_10 0.9
#define VOLT_COMP_11 0.4
#define VOLT_COMP_12 0.0
#define VOLT_COMP_13 -0.2
#define VOLT_COMP_14 -0.5
#define VOLT_COMP_15 -0.7
#define VOLT_COMP_16 -0.9
#define VOLT_COMP_17 -1.0

//#define VOLT_CONVERSION 0.017361806  // vref / (r2 / (r1 + r2))) / 1024          5 / (3900 / (10000 + 3900)) / 1024 
#define VOLT_CONVERSION 0.017518997

#define MAP_MIN_THOLD 350   // minimum map value for accel compensation - this should be above your normal idle map by about 50 points
#define MAP_DELTA 40        // minimum delta to consider accel event - for hysteresis

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
#define TACH_SIGNAL A0  // for pin setup only
#define ION_SIGNAL A1   // for pin setup only
#define IGN_VSENSE 5    // voltage sensing on adc pin 5
#define MAP_VSENSE 4    // map voltage sensing on adc pin 4

#define NUM_CYLINDERS 8
#define MAX_SPK_ADV 46      // for calculating maximum dwell begin angle (dwellDegrees + accelComp + MAX_SPK_ADV)
#define CRANKING_SPK_ADV 10 // arbitrary value to use before we pickup commanded sa from the pcm
#define LR_OVERFLOW 2       // low res timer overflow limit - engine has stalled if timer1 overflows this many times 2 x ((2x65535)x4us) = 524.28 ms between tdc signals - about 26rpm)

#define VOLT_DIVS 10      // the # of voltage rows in the dwell arrays see VOLT_COMP_* declarations
#define VOLT_OFFSET 8     // the starting voltage offset (8v = dwell[0][rpm]) so lookup is dwell[volts - volt_offset][rpm]
#define RPM_DIVS 31       // the # of rpm cols in the dwell arrays
#define CRANK_RPM_DIVS 6  // these low rpm cells (25-200rpm) get lesser CRANK_DWELL_TGT
#define ACCEL_RPM_DIV 13  // lowest rpm to start applying accel compensation

#define RPM_DIV_1 80
#define RPM_DIV_2 110
#define RPM_DIV_3 140
#define RPM_DIV_4 170
#define RPM_DIV_5 200
#define RPM_DIV_6 400
#define RPM_DIV_7 600
#define RPM_DIV_8 800
#define RPM_DIV_9 1000
#define RPM_DIV_10 1200
#define RPM_DIV_11 1400
#define RPM_DIV_12 1600
#define RPM_DIV_13 1800
#define RPM_DIV_14 2000
#define RPM_DIV_15 2200
#define RPM_DIV_16 2400
#define RPM_DIV_17 2600
#define RPM_DIV_18 2800
#define RPM_DIV_19 3000
#define RPM_DIV_20 3200
#define RPM_DIV_21 3400
#define RPM_DIV_22 3600
#define RPM_DIV_23 3800
#define RPM_DIV_24 4000
#define RPM_DIV_25 4400
#define RPM_DIV_26 4800
#define RPM_DIV_27 5200
#define RPM_DIV_28 5600
#define RPM_DIV_29 6000
#define RPM_DIV_30 6400
#define RPM_DIV_31 6800

#define RPM_TO_US(x) (( 250000UL * 60UL / ( x ) )) // Convert RPM to microseconds per 90* (us between low res signals)
//#define RPM_TO_TMR1_TICKS(x) (( 15625UL * 60UL / ( x) )) // convert RPM to timer1 ticks per 90* - 16us per tick using 256 prescaler
#define RPM_TO_TMR1_TICKS(x) (( 62500UL * 60UL / ( x) )) // convert RPM to timer1 ticks per 90* - 4us per tick using 64 prescaler

// (((CRANK_DWELL_TGT + VOLT_COMP_8) * 1000.0) / (float (rpmTable[i] / 90)) + 0.5)
#define CALC_DWELL(rpm, volts) (((DWELL_TGT + VOLT_COMP_ ## volts ) * 90000UL / (RPM_TO_US(rpm))))

#define CALC_CRANK_DWELL(rpm, volts) ((int _CRANK_DWELL(rpm, volts) == 0) ? 1:_CRANK_DWELL(rpm, volts))

#define _CRANK_DWELL(rpm, volts) (((CRANK_DWELL_TGT + VOLT_COMP_ ## volts ) * 90000UL / (RPM_TO_US(rpm))))

#define CALC_ACCEL_COMP(rpm) (((ACCEL_COMP) * 90000UL / (RPM_TO_US(rpm))))

#define CALC_DWELLBEGIN(rpm, volts, spk_adv) ((CALC_DWELL(rpm, volts) + CALC_ACCEL_COMP(rpm) + spk_adv))

#define CALC_LOOKAHEAD(rpm, volts) ((CALC_DWELLBEGIN(rpm, volts, MAX_SPK_ADV) / 90) + HAS_REMAINDER(int CALC_DWELLBEGIN(rpm, volts, MAX_SPK_ADV)) + 1)

#define HAS_REMAINDER(degs) ((degs % 90==0) ? 1:0)

#define CALC_ADC_THOLD(volts) ((volts / VOLT_CONVERSION))

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
                                            FILL_DWELL_COLUMN(RPM_DIV_6), 
                                            FILL_DWELL_COLUMN(RPM_DIV_7), 
                                            FILL_DWELL_COLUMN(RPM_DIV_8), 
                                            FILL_DWELL_COLUMN(RPM_DIV_9), 
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
                                            FILL_DWELL_COLUMN(RPM_DIV_31)};
                                            

// the rpm lookup for the dwell table - contains rpm reference in microseconds per 90* (between low res signals)
//                                                                <timer 1 ticks (16us each)>  
const PROGMEM uint16_t rpmTable[RPM_DIVS] = {RPM_TO_TMR1_TICKS(RPM_DIV_1),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_2),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_3),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_4),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_5),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_6),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_7),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_8),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_9),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_10),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_11),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_12),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_13),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_14),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_15),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_16),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_17),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_18),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_19),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_20),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_21),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_22),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_23),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_24),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_25),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_26),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_27),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_28),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_29),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_30),
                                             RPM_TO_TMR1_TICKS(RPM_DIV_31)};


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
                                             CALC_ACCEL_COMP(RPM_DIV_31)};

const uint8_t PROGMEM maxLookAhead[VOLT_DIVS] = { CALC_LOOKAHEAD(RPM_DIV_31, 8),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 9),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 10),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 11),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 12),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 13),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 14),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 15),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 16),
                                                   CALC_LOOKAHEAD(RPM_DIV_31, 17)};

const uint16_t PROGMEM voltsTable[VOLT_DIVS] = { CALC_ADC_THOLD(7.5),
                                                 CALC_ADC_THOLD(8.5),
                                                 CALC_ADC_THOLD(9.5),
                                                 CALC_ADC_THOLD(10.5),
                                                 CALC_ADC_THOLD(11.5),
                                                 CALC_ADC_THOLD(12.5),
                                                 CALC_ADC_THOLD(13.5),
                                                 CALC_ADC_THOLD(14.5),
                                                 CALC_ADC_THOLD(15.5),
                                                 CALC_ADC_THOLD(16.5)};

volatile int8_t sequenceIndex = -1;     // if negative we've lost sequence, else pointer to the current cylinder in the sequencer[] struct
volatile boolean invertSeq;             // invert sequence detection - when true the engine came to rest with the low res trigger in a gap (beam not broken)
volatile uint8_t hrDegrees = 90;        // the high resolution counter - counts down from 90 after each falling edge low res pulse
volatile uint8_t saDegrees;             // stored by the EST line interrupt this is a snapshot of most recent spark advance in degrees btdc
volatile uint8_t estEnabler;            // count degrees while est is high inside high res isr - used to debounce est input

//volatile uint8_t errCount;              // for below error and lag statistics
uint8_t errorDegs;                      // hrDegrees should be 0 when the low res TDC signal is caught - this tracks cumulative # of errors per last 10 logged messages
uint8_t dwellLag;                       // difference between targeted dwell and when dwell actually started - cumulative over last 10 logged messages
#define ERROR_REVS 40                   // match constant for resetting error tracking statistics - 4 counts = 1 crank revolution
//#define ERROR_LOGS 10                   // match constant for resetting errCount and above statistics

volatile uint8_t lrSemaphoreRising;     // triggers sequence detection in main loop
volatile uint8_t lrSemaphoreFalling;    // used for sequence detection when invertSeq = true

volatile uint16_t lrTimeHi;             // low res falling edge timer - time between tdc signals
volatile uint8_t tdcSemaphore;          // semaphore to trigger dwell lookup in loop()
volatile boolean stalled = false;       // set when timer1 overflows 2x
volatile uint8_t lrHi_overflow;         // the timer1 overflow counter used for stall detection

volatile uint8_t tmr2Mux;               // for 1s timekeeping
volatile uint8_t tmr2SkipStep = 0;    // for correcting loss
volatile uint8_t tmr2Counter;           // for counting ticks
volatile uint16_t tmr2Seconds;          // for tracking seconds
volatile uint8_t tmr2UartMux;           // for setting logFlag - 3 * 4.08ms = uart message every 12.24ms
volatile boolean logFlag;               // flag semaphore to prompt uart data output

typedef struct {
  int cylNo;          // user readable cylinder no
  int armed;          // set to 1 = armed for dwell, set to 0 = est signal caught - prevents dwell routine from dwelling recently fired coil
  volatile uint8_t *portAddr;  // used as a pointer to the output registers so coil drivers can be controlled directly (faster than using digital[write/read]() arduino functions)
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


void setup() {

  // configure misc output pins
  pinMode(TACH_SIGNAL, OUTPUT);
  #ifdef TACH_DRIVER
  digitalWrite(TACH_SIGNAL, HIGH);
  #endif
  pinMode(ION_SIGNAL, OUTPUT);

  // disable digital buffers on unused adc pins
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
  ADMUX  =  bit (REFS0) | (IGN_VSENSE & 0x07);        // select AVcc for aref and select input port
  //ADMUX  =  bit (REFS0) | (MAP_VSENSE & B00000111);     // select AVcc for aref and select input port
  
  // configure output pins
  DDRD = DDRD | B11110000; // configure PD4 - PD7 (arduino d4 - d7) as outputs
  DDRB = DDRB | B00111110; // configure PB1 - PB5 (arduino d9 - d12) as outputs
  
  Serial.begin(115200);
  Serial.print(F("diy-ltcc-"));
  Serial.println(VERSION);

  #ifdef DUMP_TABLES
    dumpRPMTable();
    dumpVoltsTable();
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

  // configure timer1 for capture interrupt
  TCCR1A = 0; // clear timer1 config registers
  TCCR1B = 0; 
  
  //if ( digitalRead(OPTI_LO_RES) ) {
  if (PINB & B00000001) {
    TCCR1B &= ~(1<<ICES1);            // set first capture on falling edge (see declaration of invertSeq variable for more info)
    invertSeq = true;
  } else {
    TCCR1B |= (1<<ICES1);             // set first capture on rising edge
    invertSeq = false;
  }
    
  //TCCR1B |= (1<<CS12);                // prescaler 256 - gives timer1 16 us resolution per tick - RPM formula 937500 / lrTimeHi
  TCCR1B |= (1<<CS11)|(1<<CS10);     // prescaler 64 - gives timer1 4us resolution per tick - RPM formula 3750000 / lrTimeHi 
  TCCR1B |= (1<<ICNC1);              // enable noise canceller
  TIMSK1 |= (1<<ICIE1)|(1<<TOIE1);   // enable input capture and overflow interrupts

  // configure timer2 for profiling and timekeeping
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;

  TCCR2B |= (1<<CS22)|(1<<CS21);    // prescaler 256 - gives timer1 16 us resolution per tick - 4.08ms per overflow - 245 overflows = 0.9996s
  TIMSK2 |= (1 << TOIE2);           // enable overflow interrupt L counting 1 additional tick per 10s interval corrects loss to 28ms / hr or ~ 0.7s / day
  
  sei();
  
  bitSet (ADCSRA, ADSC);  // request first adc conversion

}


void isr_est() {

  // this isr is time critical, keep to minimum number of instructions here
  // store the current spark advance
  if (estEnabler > 2) {
    if (hrDegrees > 90 ) {
      saDegrees = 0;
    } else {
      saDegrees = hrDegrees;
    }
    #ifdef INPUT_TEST
      if (digitalRead(PCM_EST)) {
        //*tachOutput[0].portAddr |= tachOutput[0].bitMask; // drive the tach output high
        PORTC |= B00000001; // drive A0 high
      } else {
        //tachOutput[0].portAddr &= ~tachOutput[0].bitMask; // drive the tach output low
        PORTC &= ~B00000001; // drive A0 low
        Serial.print("saDegrees=");
        Serial.println(saDegrees);
      }    
    #else
      // turn off the coil output to trigger spark event
      if (sequenceIndex > -1) {
        *sequencer[sequenceIndex].portAddr &= ~sequencer[sequenceIndex].bitMask;
        // clear the est debouncer
        estEnabler = 0;
        #ifdef SERIAL_DEBUG
          sei();
          Serial.print("c");
          Serial.print(sequencer[sequenceIndex].cylNo);
          Serial.print(" fire ");
          Serial.print(hrDegrees);
          Serial.print(" ");
          Serial.println(estEnabler);
        #endif
      }
      #ifdef TACH_DRIVER
        sei();
        // drive tach output line high
        //*tachOutput[0].portAddr |= tachOutput[0].bitMask;
        PORTC |= B00000001; // drive A0 high
      #endif
    #endif
  }
  
}


void isr_hi_res() {   // hi res change interrupt - counts crankshaft degrees

  // this isr is time critical, keep to minimum number of instructions here
  hrDegrees--;
  // if PCM_EST line is high count degrees into estEnabler to debounce
  uint8_t estState = PIND & B00000100; // read D2 state 
  if (estState) estEnabler++;
  #ifdef ION_TRIGGER
    if (hrDegrees == ION_TRG_DEGREE) 
      PORTC |= B00000010; // drive A1 high
      //PORTC &= ~B00000010; // drive A1 low
  #endif
  
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

  if ( lrHi_overflow >= LR_OVERFLOW ) { 
    // timeout on low res pulse - not terribly time critical b/c engine has stalled
    lrHi_overflow = LR_OVERFLOW + 1;
    lrTimeHi = 0;
    
    // at this point the engine has stalled or the low res signal has failed, shut everything down
    stalled = true;
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
    // set pin 13 high - yellow status led on solid
    PORTB |= B00100000;
  }
  
}


ISR(TIMER1_CAPT_vect) { // timer1 capture interrupt vector - measures ticks between low res rising edge (TDC) signals

  // this isr is time critical, keep to minimum number of instructions here
  if( bit_is_set(TCCR1B, ICES1) ) { // triggered on rising edge of low res signal

    #ifdef INPUT_TEST
      *sequencer[1].portAddr |= sequencer[1].bitMask; // drive #8 coil pin high
      Serial.print("hrDegRise=");
      Serial.println(hrDegrees);
      hrDegrees = 90;
    #else
      lrSemaphoreRising = hrDegrees;  // degree count @ tdc signal - for communicating with main loop and sequence detection
      hrDegrees = 90;           // reset the degree counter
      sei();                    // re-enable global interrupts asap - everything after this is non-time critical
      //========================================================================================================
      tdcSemaphore = 1;;        // for communicating with main loop
      lrTimeHi = ICR1;          // push the timer value into the semaphore variable
      TCNT1 = 0;                // reset the count register
      
      if ( sequenceIndex > -1 ) {   // TODO - consider moving to isr_est() ???
        sequenceIndex++;        // increment the sequence index
        if (sequenceIndex == NUM_CYLINDERS) {
          sequenceIndex = 0;
        }
      }
      /* WTF? 
      lrSemaphoreRising = hrDegrees;  // for sequence detection       
      hrDegrees = 90;                 // reset the degree counter */
      #ifdef TACH_DRIVER
        PORTC &= ~B00000001; // drive A0 low
      #endif
      
    #endif
    
  } else {                         // triggered on falling edge

    #ifdef INPUT_TEST
      *sequencer[1].portAddr &= ~sequencer[1].bitMask; // drive #8 coil pin low
      Serial.print("hrDegFall=");
      Serial.println(hrDegrees);
    #else
      lrSemaphoreFalling = hrDegrees; // for sequence detection
    #endif
    
  }
  
  TCCR1B ^= _BV(ICES1); // toggle ICR edge select bit value so interrupt triggers on the other edge
  
}


ISR (TIMER2_OVF_vect) { // general purpose timer for timekeeping, scheduling uart output and profiling
  
  // Timer 2 has overflowed - 256 prescaler gives 4.08ms per overflow - *245 = 0.9996
  tmr2Counter++;
  sei(); // enable interrupts so this will nest and execute later if needed

  if (tmr2SkipStep == 9) {
    if (tmr2Counter == 246) {
      tmr2Counter = 0;
      tmr2Seconds++;
      tmr2SkipStep = 0;
      //Serial.println("-");
    }
    
  } else {
    if (tmr2Counter == 245 ) {
      tmr2Counter = 0;
      tmr2Seconds++;
      tmr2SkipStep++;
      //Serial.println(tmr2Seconds);
    }
    
  }
    
  tmr2UartMux++;
  if (tmr2UartMux == 29) {
    tmr2UartMux = 0;
    //logFlag = true; // NOTE - uncomment to experiment with time based logging versus tdc count triggered logging
  }

}


void loop() {
  
  #ifdef SERIAL_DEBUG
    static bool initialized;
    if ( ! initialized ) {
       Serial.print("ready in: ");
       Serial.print(millis()); // millis() probably won't work with timer0 de-configured... :-O
       Serial.println("ms");
       initialized = true;
    }
  #endif

  static uint16_t dwellDegrees;     // current dwell requirement in degrees
  //static uint32_t rpmTimeLast = 0;  // for reference in getDwellDegrees lookup function
  static uint16_t rpmTimeLast = 0;  // for reference in getDwellDegrees lookup function
  static uint8_t rpmIndex = 0;      // for human readable rpm and dwell lookup hint
  static uint8_t voltsIndex = 0;    // for voltage lookups and maxLookAhead[] in dwellCoils
  static uint16_t rpmTimeHi;        // holds the most recent low-res interval
  static uint16_t adcVoltsRaw;      // ignition voltage in adc raw format
  static uint16_t adcVoltsRawLast;  // for voltage lookup hint
  static uint16_t adcMapRaw;        // map sensor input in adc raw format
  static uint16_t adcMapRawLast;    // map sensor hysteresis
  static uint8_t tdcCount;          // for tracking engine revolutions (4 tdc ticks = 1 rev)
  
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
      // turn off pin 13 status led
      PORTB &= ~B00100000;
      // reset runtime
      tmr2Seconds = 0;
      tmr2Counter = 0;
      tmr2SkipStep = 0;
      #ifdef SERIAL_DEBUG
         Serial.println("sequenced");
      #endif

    }
    
  }
  // end sequence detection

  // check if we need to dwell coils (#1)
  if (dwellDegrees > 0) dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
  
  //if ( lrTimeHi != 0 ) {
  if ( tdcSemaphore ) {
  //if ( lrSemaphoreRising ) {

    // low res tdc semaphore caught
    //lrCountLocal = lrSemaphoreRising;
    tdcSemaphore = 0;
    stalled = false;
    rpmTimeHi = lrTimeHi;
    tdcCount++;      
    lrHi_overflow = 0;

    voltsIndex = getVoltageIndex(adcVoltsRaw, adcVoltsRawLast, voltsIndex);
    adcVoltsRawLast = adcVoltsRaw;
    if (sequenceIndex < 0) {
      // not sequenced yet, retrieve dwell from lowest rpm cell
      dwellDegrees = getDwell(pgm_read_word(&rpmTable[0]) + 1, 0, voltsIndex, rpmIndex, adcMapRaw, adcMapRawLast);
    } else {
      dwellDegrees = getDwell(rpmTimeHi, rpmTimeLast, voltsIndex, rpmIndex, adcMapRaw, adcMapRawLast); // might want to consider doing this less freq at higher rpms
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
      //estEnabler = 0; note: moved to isr_est()

      // scale the logging and voltage sense interval with rpmIndex
      //if (rpmIndex > 8) timer2OFCompare = rpmIndex;

      #ifdef ION_TRIGGER
        //PORTC |= B00000010; // drive A1 high
        PORTC &= ~B00000010; // drive A1 low
      #endif
       
    }
    rpmTimeLast = rpmTimeHi;   // this is used as a hint to getDwell() so it doesn't iterate the entire array (rpmTable) after each TDC signal
    adcMapRawLast = adcMapRaw; // copy current map to hysteresis variable
    
    #ifdef SERIAL_DEBUG
      if (rpmTimeHi > 531) { // only output debugging info below certain rpm TODO make this a constant / macro
        cylInfoUart(dwellDegrees, rpmTimeHi); // TODO - need to pass low res rising degrees into this
      }
    #endif

    // accumulate error degrees
    if (lrSemaphoreRising > 90) {
      // wraparound
      errorDegs += 255 - lrSemaphoreRising;
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
      tdcCount = 0;
      dwellDegrees = 0;
      rpmIndex = 0;
      #ifdef SERIAL_DEBUG
        Serial.println("stalled");
      #endif
    } else {
      dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
    }
  }
  
  // read results of adc conversion
  
  // the ADC clears the bit when done
  if (bit_is_clear(ADCSRA, ADSC)) {

    uint8_t adMux = ADMUX & B00000111;
    if ( adMux == IGN_VSENSE ) {
      adcVoltsRaw = ADC;
      ADMUX  =  bit (REFS0) | (MAP_VSENSE & B00000111);
    } else {
      adcMapRaw = ADC;
      ADMUX  =  bit (REFS0) | (IGN_VSENSE & B00000111);
    }

    bitSet (ADCSRA, ADSC);  // start a conversion
  }

/*
#ifdef LOGGING
  // output logging data periodically - triggered by timer2
  if (logFlag) {

    logFlag = false;
              
    uint8_t cylNo;
    if ( sequenceIndex < 0 ) {
      cylNo = 0;
    } else {
      cylNo = sequencer[sequenceIndex].cylNo;
    }

    Serial.print("R");
    Serial.print(rpmTimeHi);
    Serial.print(":S");
    Serial.print(saDegrees);
    Serial.print(":D");
    Serial.print(dwellDegrees);
    Serial.print(":C");
    Serial.print(cylNo);
    Serial.print(":M");
    Serial.print(adcMapRaw);
    Serial.print(":V");
    Serial.print(adcVoltsRaw);
    Serial.print(":F");
    Serial.print(tdcCount);
    Serial.print(":E");
    Serial.print(errorDegs);
    Serial.print(":L");
    Serial.print(dwellLag);
    Serial.print(":T");
    Serial.println(tmr2Seconds);
    
  }
#endif */

  // check if we need to dwell coils (#3)
  //if (dwellDegrees > 0) dwellCoils(dwellDegrees, saDegrees, hrDegrees, sequenceIndex, voltsIndex);
  
  // update / clear error statistics every N revolutions
  if (tdcCount >= ERROR_REVS) {

    #ifdef LOGGING
        uint8_t cylNo;
        if ( sequenceIndex < 0 ) {
          cylNo = 0;
        } else {
          cylNo = sequencer[sequenceIndex].cylNo;
        }
    
        Serial.print("R");
        Serial.print(rpmTimeHi);
        Serial.print(":A");
        Serial.print(saDegrees);
        Serial.print(":D");
        Serial.print(dwellDegrees);
        Serial.print(":C");
        Serial.print(cylNo);
        Serial.print(":M");
        Serial.print(adcMapRaw);
        Serial.print(":V");
        Serial.print(adcVoltsRaw);
        Serial.print(":T");
        Serial.print(tdcCount);
        Serial.print(":E");
        Serial.print(errorDegs);
        Serial.print(":L");
        Serial.print(dwellLag);
        Serial.print(":S");
        Serial.println(tmr2Seconds);
    #endif
    
    tdcCount = 0;
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
  Serial.println("dumping uint16_t rpmTable[RPM_DIVS] PROGMEM = {");
  Serial.print("  ");
  
  char str[30];
  for (int i=0; i<RPM_DIVS; i++) {
    //Serial.print(uintToStr(pgm_read_word(&rpmTable[i]), str));
    Serial.print(pgm_read_word(&rpmTable[i]));
    if (i < RPM_DIVS - 1) Serial.print(",  ");
  }
  
  Serial.println(" };");
  Serial.println("");
    
}


void dumpVoltsTable() {
  
  // dump voltsTable[]
  Serial.print("dumping uint16_t voltsTable[VOLT_DIVS] PROGMEM = { ");
  for (int i=0; i<VOLT_DIVS; i++) {
    
    Serial.print(pgm_read_word(&voltsTable[i]));
    if (i < VOLT_DIVS - 1) Serial.print(",  ");
  }
  Serial.println(" };");

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
    
    //accelComp[i] = uint8_t ((ACCEL_COMP * 1000.0) / float (pgm_read_word(&rpmTable[i]) / 90) + 0.5);
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

#ifdef DUMP_TABLES //|| SERIAL_DEBUG || SIMULATE
char * uintToStr( const uint32_t num, char *str )
{
  uint8_t i = 0;
  uint32_t n = num;
 
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
#endif

uint16_t getDwell(uint16_t rpmTicks, uint16_t rpmTicksLast, uint8_t voltIndex_l, uint8_t &dwellIndex, uint16_t mapCurrent, uint16_t mapPrevious) {
  
  // lookup the degrees of dwell required for the current rpm from the dwell table
  boolean dwellMatch = false;
  uint8_t accelCompAdder = 0;
  
  if ( rpmTicksLast == 0 ) {

    // no hint to work from, iterate the whole array
    for (uint8_t indexTmp = 0; indexTmp < RPM_DIVS; indexTmp++ ) {
      /*Serial.print(uintToStr(rpmTable[indexTmp], str));
      Serial.print(" <> ");
      Serial.println(uintToStr(rpmTicks  , str));*/
      if (pgm_read_word(&rpmTable[indexTmp]) < rpmTicks) {
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

    // use the rpmTicksLast hint to look near last known rpm and save some iterations / instructions
    if ( rpmTicksLast < rpmTicks ) {
      
      // engine is decelerating
      
      //char buf[30];
      //Serial.print(uintToStr(rpmTicksLast, str));
      //Serial.print(" < ");
      //Serial.println(uintToStr(rpmTicks, str));
      //Serial.println("decel");
      
      if (dwellIndex > 0) {
        if (rpmTicks > pgm_read_word(&rpmTable[dwellIndex - 1])) {
          while ( ! dwellMatch ){
            
            dwellIndex--;
            if (dwellIndex == 0) {
              dwellMatch = true;
              break;
            }
            if (rpmTicks < pgm_read_word(&rpmTable[dwellIndex])) {
              dwellMatch = true;
              break;
            }
            
          }
        }
      }
      
    } else if (rpmTicksLast > rpmTicks) {
      
      // engine is accellerating
      //Serial.println("accel");
      if (dwellIndex < (RPM_DIVS - 1)) {
        if (rpmTicks < pgm_read_word(&rpmTable[dwellIndex + 1])) {
          while ( ! dwellMatch ){
            
            dwellIndex++;
            if (dwellIndex == RPM_DIVS) {
              dwellIndex = RPM_DIVS - 1;
              dwellMatch = true;
              break;
            }
            if (pgm_read_word(&rpmTable[dwellIndex]) < rpmTicks) {
              dwellMatch = true;
              dwellIndex--;
              break;
            }
          }
        }
      }
      // add some accel compensation
      // for accelleration conditions (hint taken from megasquirt sequencer coil selection http://www.megamanual.com/seq/coils.htm)
      if (mapCurrent < MAP_MIN_THOLD) {
        if (mapCurrent < (mapPrevious - MAP_DELTA)) 
          accelCompAdder = pgm_read_byte(&accelComp[dwellIndex]);
      }    
    }
    
  }

  return pgm_read_word(&dwellTable[dwellIndex][voltIndex_l]) + accelCompAdder;
  
}


uint8_t getVoltageIndex(uint16_t adcValue, uint16_t adcValueLast, uint8_t voltsIndex) {
  
  // lookup the raw adc value in the voltsTable and return index
  boolean voltsMatch = false;
  
  if ( adcValueLast == 0 ) {

    // no hint to work from, iterate the whole array
    for (uint8_t indexTmp = 0; indexTmp < VOLT_DIVS; indexTmp++ ) {
      if (pgm_read_word(&voltsTable[indexTmp]) > adcValue) {
        if (indexTmp > 0) {
          voltsIndex = indexTmp - 1;
        }
        voltsMatch =  true;
        break;
      }
    }

    if ( ! voltsMatch )
      voltsIndex = VOLT_DIVS - 1;
    
  } else {
    
    // use the adcValueLast hint to look near last known adc value and save some iterations / instructions
    if ( adcValueLast > adcValue ) {
      
      // voltage decreased
      
      if (voltsIndex > 0) {
        if (adcValue < pgm_read_word(&voltsTable[voltsIndex - 1])) {
          while ( ! voltsMatch ){
            
            voltsIndex--;
            if (voltsIndex == 0) {
              voltsMatch = true;
              break;
            }
            if (adcValue > pgm_read_word(&voltsTable[voltsIndex])) {
              voltsMatch = true;
              voltsIndex++;
              break;
            }
            
          }
        }
      }
      
    } else if (adcValueLast < adcValue) {
      
      // voltage increased
      if (voltsIndex < (VOLT_DIVS - 1)) {
        if (adcValue > pgm_read_word(&voltsTable[voltsIndex + 1])) {
          while ( ! voltsMatch ){
            
            voltsIndex++;
            if (voltsIndex == VOLT_DIVS) {
              voltsIndex = VOLT_DIVS - 1;
              voltsMatch = true;
              break;
            }
            if (pgm_read_word(&voltsTable[voltsIndex]) > adcValue) {
              voltsMatch = true;
              break;
            }
          }
        }
      }      
    }
    
  }

  return voltsIndex;
  
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

#ifdef SERIAL_DEBUG
void cylInfoUart(uint16_t dwellDeg, uint16_t rpmTicks) {
  
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
  //Serial.println(uintToStr(rpmTimeHi, str));
  Serial.println(rpmTicks);
  Serial.print("deg=");
  Serial.println(lrSemaphoreRising); // TODO need to pass this variable in now

}
#endif
