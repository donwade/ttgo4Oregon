#include "RFM69OOK.h"
#include "RFM69OOKregisters.h"
#include <SPI.h>

#define  SimpleFIFO_NONVOLATILE
#include <SimpleFIFO.h>

RFM69OOK radio (RF69OOK_SPI_CS, RF69OOK_IRQ_PIN, true, RF69OOK_IRQ_NUM, RF69OOK_RESET_PIN);

//#define xRF69OOK_SPI_CS  18
//#define xRF69OOK_IRQ_PIN 32
//#define xRF69OOK_IRQ_NUM 32
//#define xRF69OOK_RESET_PIN
//RFM69OOK radio (RF69OOK_SPI_CS, RF69OOK_IRQ_PIN, true, RF69OOK_IRQ_NUM, RF69OOK_RESET_PIN);

unsigned long cnt;

typedef enum { LOOK4SYNC = 'a', LOOK4PREAMBLE_PH1='b', LOOK4PREAMBLE_PH2='c', COLLECT_DATA='d', PROCESS_PACKET='e' } OREGON_STATE;
volatile  OREGON_STATE currentState = LOOK4SYNC;
volatile  OREGON_STATE nextState = LOOK4SYNC;


#define MAX_BITS_IN_DATASTREAM 115

#define FREQ_WIDTH 1700000
#define FREQ_MID 433920000
#define FREQ_MIN (FREQ_MID-FREQ_WIDTH/2)
#define FREQ_MAX (FREQ_MID+FREQ_WIDTH/2)

#define FREQ_OREGON5a  433863800
#define FREQ_OREGON4a  433879300
#define FREQ_OREGONXa  433890300
//#define FREQ_STRONG    (432034000)
#define FREQ_STRONG    (43203400)
#define FREQ_STEP 5000

#define MIN_DB_PWR 15   // 15 = off
#define MAX_DB_PWR 32   // ouch!
#define ELEMENTS(x) (sizeof(x)/sizeof(x[0]))

// no overlap! :)
#define SHORT_ON_LEFT   250
#define SHORT_ON_RIGHT  710  //600
#define LONG_ON_LEFT    711  //690
#define LONG_ON_RIGHT  1220  //1000

#define SHORT_OFF_LEFT  250  //400
#define SHORT_OFF_RIGHT 775  //749
#define LONG_OFF_LEFT   780  //950
#define LONG_OFF_RIGHT 1300


static volatile uint32_t interruptCount = 0, lastCount = 0;

uint32_t baseFrequencyHz = FREQ_OREGON4a;

signed char basePowerDb = (MIN_DB_PWR + MAX_DB_PWR)/2;

typedef enum {POWER, FREQUENCY, UNKNOWN} umode_e;
umode_e uMode = UNKNOWN;


unsigned short longestSyncRun = 0;

bool bForceReport;
int16_t noiseFloorDb2 = 0;
int ookNoiseFloor = 0;
bool bTxON = false;
//---------------------------------------------------------------------
void loadNgo(uint32_t freq)
{
    radio.setFrequency(freq);
}
//---------------------------------------------------------------------
void Scanner(uint32_t freq, uint32_t width, uint16_t stepsize)
{
    radio.setBandwidthHz(stepsize);
    //radio.writeReg(REG_TESTDAGC, 0);  // move rssi to manual
    radio.setRSSIThreshold(-128);

    //freq = (freq/stepsize) * stepsize;

    Serial.printf("scanning from %d to %d stepsize %d, @ %d steps \n", freq-width/2, freq+width/2, stepsize, width/stepsize);

    for (uint32_t f = freq - width/2; f < freq + width/2; f += stepsize)
    {
        radio.setFrequency(f);
        procKeyboard();
        for (int i = 0; i < 3; i++)
        {
            delay(10);
            int16_t rssi = radio.readRSSI(0) - noiseFloorDb2;
            int32_t fei  = radio.getFEIvalue();
            int32_t afc  = radio.getAFCvalue();
            char extra =  (f == freq) ? 'c' : ' ';

            //if (! fei || !afc)
            Serial.printf("%c %8u Hz rssi=%3d fei=%5d afc=%5d\n", extra , f, rssi, fei, afc);
        }
        Serial.println();
    }
    radio.writeReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);  // move rssi to automatic
    Serial.printf("done. Press r to reset\n");
    while(procKeyboard());
}
//---------------------------------------------------------------------
const float bandwidthOptions[] = {  1.3, 1.6, 2.0, 2.6, 3.1,
                                    3.9, 5.2, 6.3, 7.8, 10.4,
                                    12.5, 15.6, 20.8, 25.0,
                                    31.3, 41.7, 50.0, 62.5,
                                    83.3, 100.0, 125.0, 166.7,
                                    200.0, 250.0}; // ook values

signed char bwIndex = 9; // pick 10.4 as default
bool setBandwidthTweek(signed char direction)
{
    if ( direction < -1 || direction > +1) return false;
    bwIndex += direction;
    if (bwIndex < 0) bwIndex = 0;
    if (bwIndex == ELEMENTS(bandwidthOptions)) bwIndex--;
    unsigned int bw = bandwidthOptions[bwIndex] * 1000;
    radio.setBandwidthHz(bw);
    Serial.printf("set bandwidth = %d (%d)\n", bw, bwIndex);
    return true;
}
//---------------------------------------------------------------------
bool procKeyboard()
{
    if (Serial.available() > 0) {
      // read the incoming byte:
      int c = Serial.read();

      switch (c)
      {
        case 'b':
            setBandwidthTweek(-1);
        break;

        case 'B':
            setBandwidthTweek(+1);
        break;

        case 'T':
          bTxON = 1;
        break;

        case ' ':
          reportHistory();
        break;

          case '-':
            switch (uMode)
            {
                case POWER:
                    basePowerDb--;
                    if (basePowerDb < MIN_DB_PWR) basePowerDb = MIN_DB_PWR;
                    radio.setPowerLevel(basePowerDb);
                    Serial.printf("-power = %d\n", basePowerDb);
                break;
                case FREQUENCY:
                    baseFrequencyHz -= FREQ_STEP;
                    if (baseFrequencyHz < FREQ_MIN) baseFrequencyHz= FREQ_MIN;
                    loadNgo(baseFrequencyHz);
                    Serial.printf("-frequency = %d\n", baseFrequencyHz);
                break;
                default:
                    Serial.println("define power(p) or frequency(f)");
                break;
            }
          break;

          case '+':
          case '=':
              switch (uMode)
              {
                  case POWER:
                      basePowerDb++;
                      if (basePowerDb > MAX_DB_PWR) basePowerDb = MAX_DB_PWR;
                      radio.setPowerLevel(basePowerDb);
                      Serial.printf("+power = %d\n", basePowerDb);
                  break;
                  case FREQUENCY:
                      baseFrequencyHz += FREQ_STEP;
                      if (baseFrequencyHz > FREQ_MAX) baseFrequencyHz= FREQ_MAX;
                      loadNgo(baseFrequencyHz);
                      Serial.printf("-frequency = %d\n", baseFrequencyHz);
                  break;
                  default:
                      Serial.println("define power(p) or frequency(f)");
                  break;

               }
               break;
          break;

          case 'r':
                esp_restart();
          break;

          case 's':
              Scanner(FREQ_STRONG, 100000, 1000);
          break;

          case 'p':
          case 'P':
            Serial.printf("select power\n");
            uMode = POWER;
          break;

          case 'f':
          case 'F':
            uMode = FREQUENCY;
            Serial.printf("select frequency\n");
          break;

          default:
              Serial.printf("unknown command %c ?\n", c);
              Serial.printf("transmitter off.\n", c);
              bTxON = 0;
          break;

          }
      }
    return true;
   }
//---------------------------------------------------------------------
// setting RXBW = 1.3 causes signal to be missed even if slightly off freq
// setting RXBW = 250khz catches and reovers most but true bw is 1.7mhz

#define BASE_RATE 1000
#define REG_BASE_RATE (32000000 / BASE_RATE)
#define HI(x) (REG_BASE_RATE >> 8)
#define LO(x) (REG_BASE_RATE & 0xFF)

bool auxInit()
{
  const byte CONFIG[][2] =
  {
    //DIO0 is the only IRQ we're using
    { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 | RF_DIOMAPPING1_DIO2_01 },
    { REG_DIOMAPPING2, RF_DIOMAPPING2_DIO5_01 | RF_DIOMAPPING2_DIO4_10},

    { REG_RXBW,       0x04 },       //   DCC=4% BW=250.0 khz  exp=0 mant=24 (wide open)

/*
    { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUS  // allow SYNC preamble detection
                   | RF_DATAMODUL_MODULATIONTYPE_OOK
                   | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    { REG_BITRATEMSB, HI(REG_BASE_RATE) },  // for bit sync detector
    { REG_BITRATELSB, LO(REG_BASE_RATE) },
    { REG_BITRATEMSB, 0x03 },       //  bitrate: 0x3D1 =32768 Hz
    { REG_BITRATELSB, 0xD1 },       //  bitrate  0x682B = 1200Hz
      // table 14
    { REG_RXBW,       0x57 },       //  DCC=4% BW= 1.3 khz exp=7 mant=24
    { REG_RXBW,       0x56 },       //  DCC=4% BW= 2.6 khz exp=6 mant=24
    { REG_RXBW,       0x55 },       //  DCC=4% BW= 5.2 khz exp=5 mant=24
    { REG_RXBW,       0x54 },       //  DCC=4% BW=10.4 khz exp=4 mant=24
*/
     {255, 0}
  };

  for (byte i = 0; CONFIG[i][0] != 255; i++)
    radio.writeReg(CONFIG[i][0], CONFIG[i][1]);

  Serial.println("radio init done");
  return true;
}
//---------------------------------------------------------------------

int16_t setSquelchByRssi(uint32_t frequency)
{
    uint32_t    restored;
    int16_t     testSetting = 0;

    bool        bSquelchOpen = 0;
    int16_t     lockedSetting = -255;
    int16_t     rssiFloor = -255;  // assume noise is perfect.

    const uint8_t num_samples = 4;

    restored = radio.getFrequency();

    loadNgo(frequency); // nobody out here :)

    for (testSetting =-1 ; testSetting > -255; testSetting--)
    {
        int16_t rssi = 0;
        bSquelchOpen = false;

        radio.setRSSIThreshold(testSetting);
        for (int i = 0; i < num_samples; i++)
        {
            delayMicros(1000);
            rssi += radio.readRSSI(0);
            bSquelchOpen |= (radio.readReg(REG_IRQFLAGS1) & 0x40);
        }
        log_d("knob = %d rssi = %d squelch = %d", testSetting, rssi/num_samples,  bSquelchOpen);
        if (bSquelchOpen && lockedSetting == -255)
        {
            lockedSetting = testSetting;
            rssiFloor = rssi / num_samples;
        }
    }

    log_d("Setting squelch to %d and rssi was %d\n", lockedSetting+1 , rssiFloor);

    lockedSetting++;                       // last setting just before squelch opened.
    radio.setRSSIThreshold(lockedSetting); //leave squelched, just a hair above the noise floor
    loadNgo(restored);
    return rssiFloor;

}


//---------------------------------------------------------------------
bool ook_monFreq(uint32_t freq)
{
    uint32_t restoreFreq;
    static int16_t last_rssi = 0;
    static bool last_squelch;


    Serial.printf("monitoring frequency %u\n", freq);

    restoreFreq = radio.getFrequency();

    loadNgo(freq);

    while (1)
    {
        int16_t rssi = radio.readRSSI(0);
        bool bSquelchOpen = (radio.readReg(REG_IRQFLAGS1) & 0x40);
        delayMicros(100);

        if (rssi != last_rssi ||  bSquelchOpen != last_squelch)
        {
            Serial.printf("freq %u rssi = %d vs %d squelch = %d vs %d \n", freq, rssi, last_rssi, bSquelchOpen, last_squelch);
            last_rssi = rssi;
            last_squelch = bSquelchOpen;
        }
    }
    loadNgo(restoreFreq);
}

//---------------------------------------------------------------------
#define FLIGHT_LEN 300
char flightRecorderL1[FLIGHT_LEN];
static uint32_t flightIndexL1 = 0;

char flightRecorderL2[FLIGHT_LEN];
static uint32_t flightIndexL2 = 0;

char flightRecorderL3[FLIGHT_LEN];
static uint32_t flightIndexL3 = 0;

char flightRecorderL4[FLIGHT_LEN];
static uint32_t flightIndexL4 = 0;

char flightRecorderL5[FLIGHT_LEN];
static uint32_t flightIndexL5 = 0;
//---------------------------------------------------------------------
void clearFlightRecorders()
{
   memset(flightRecorderL1, 0, sizeof(flightRecorderL1));
   memset(flightRecorderL2, 0, sizeof(flightRecorderL2));
   memset(flightRecorderL3, 0, sizeof(flightRecorderL3));
   memset(flightRecorderL4, 0, sizeof(flightRecorderL4));
   memset(flightRecorderL5, 0, sizeof(flightRecorderL5));
}
//---------------------------------------------------------------------
void dumpHex(char *who, unsigned int len)
{
    for(int i= 0; i < len; i++) Serial.printf("%02X", who[i]);
    Serial.println();
}
//---------------------------------------------------------------------
void dumpFlightRecorder()
{
    Serial.printf("L1[%3d] = %s\n", strlen(flightRecorderL1), flightRecorderL1 );
    Serial.printf("L2[%3d] = %s\n", strlen(flightRecorderL2), flightRecorderL2 );
    Serial.printf("L3[%3d] = %s\n", strlen(flightRecorderL3), flightRecorderL3 );
    Serial.printf("L4[%3d] = %s\n", strlen(flightRecorderL4), flightRecorderL4 );
    Serial.printf("L5[%3d] = %s\n", strlen(flightRecorderL5), flightRecorderL5 );
    dumpHex(flightRecorderL5, flightIndexL5);
    flightIndexL1 = 0;
    flightIndexL2 = 0;
    flightIndexL3 = 0;
    flightIndexL4 = 0;
    flightIndexL5 = 0;
}

//---------------------------------------------------------------------
#define RX_GATE_PIN05 34
static unsigned long endTime = 0;
static uint32_t edgeCount;

bool bIsWindowOpen(void)
{

    if (endTime)
    {
        if ((micros() > endTime) )
        {
            edgeCount = interruptCount;
            endTime = 0;
            enableStopwatch();
            currentState = LOOK4SYNC;

            dumpFlightRecorder();
            clearFlightRecorders();
            return false;  // no counting
        }
        else
            return true; // keep counting.
    }
    return false;  // no counting
}
//---------------------------------------
void stopWatchIrq(void)
{
    endTime = micros() + 110000;   // 110 ms total
    detachInterrupt(RX_GATE_PIN05); // generate interrupts in RX mode
    interruptCount = 0;
}
//---------------------------------------
void enableStopwatch(void)
{
    pinMode(RX_GATE_PIN05, INPUT);
    attachInterrupt(RX_GATE_PIN05, stopWatchIrq, CHANGE); // generate interrupts in RX mode
}

//---------------------------------------------------------------------
//#define TX
#define SCOPE_PIN15 13
#define SCOPE_PIN09 25

#if 1
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.printf("built %s at %s\n", __FILE__, __TIME__);
    clearFlightRecorders();

    radio.resetHW();
    radio.initialize();
    auxInit();

//    pinMode(SCOPE_PIN15, OUTPUT);
//    digitalWrite(SCOPE_PIN15, LOW);
//    pinMode(SCOPE_PIN09, OUTPUT);
//    digitalWrite(SCOPE_PIN09, LOW);


    //radio.setBandwidthHz(20000); // as seen in gqrx

#ifdef TX
    radio.transmitBegin();
#else
    radio.receiveBegin();
#endif

    radio.setFrequency(baseFrequencyHz);
    uint32_t foo = radio.getFrequency();
    Serial.printf("%u %u----------------------\n", foo, baseFrequencyHz);

    radio.setPowerLevel(basePowerDb);

#ifdef TX
    radio.setMode(RF69OOK_MODE_TX);
#else
    radio.setMode(RF69OOK_MODE_RX);

    //radio.setLnaAtten( 0);
    radio.setLnaAtten(-6);
    //radio.setLnaAtten(-12);
    //radio.setLnaAtten(-24);
    //radio.setLnaAtten(-36);
    //radio.setLnaAtten(-48);
    int8_t gain = radio.getLnaAtten();
    Serial.printf("lna gain set to  %d db", gain);

    radio.setSquelchByOOK(FREQ_MAX);


    radio.setOOKsliceMode(OOK_SLICE_FIXED);     // most accurate
    //radio.setOOKsliceMode(OOK_SLICE_PEAK);       fair mabye good for auto calc
    //radio.setOOKsliceMode(OOK_SLICE_AVERAGE);    fails miserably.


    enableStopwatch();

#endif

    noiseFloorDb2 =  setSquelchByRssi(FREQ_MAX);    // find find noise floor

    //radio.setRSSIThreshold(0xFF);                   // remove rssi squelch, this is ook

    radio.attachUserInterrupt(radioDataIrq);

    loadNgo(baseFrequencyHz);

    Serial.println("setup done.");

}
//---------------------------------------------------------------------
void delayMicros(uint32_t d) {
  uint32_t t = micros() + d ;
  while(micros() < t);
}
//---------------------------------------------------------------------
void txPing (uint32_t time)
{
    radio.send(0);
    delayMicros(time);
    //delay(time);

    radio.send(1);
    //delay(time);
    delayMicros(time);
}

//---------------------------------------------------------------------
//-------------------------------------------------------------
#define MAX_TIME 2048
#define QUANTIZE 50
unsigned OnTimeHistory  [MAX_TIME] = {0};
unsigned OffTimeHistory [MAX_TIME] = {0};

//-------------------------------------------------------------
static unsigned short syncBitCtr;
static unsigned short preambleDownCnt;


typedef enum { SHORT_OFF = 's', SHORT_ON = 'S', LONG_OFF = 'l', LONG_ON = 'L' , BAD_ON = 'B', BAD_OFF ='b'} QUAD_BIT;

typedef struct { bool bit; unsigned long time;} PAIR;

SimpleFIFO<QUAD_BIT, 100> fifo;  // original 10 had bad data overrun :(
SimpleFIFO<PAIR,  10> errors;

static volatile uint32_t afc_ON,afc_OFF, fei_OFF, fei_ON;
static volatile int16_t rssi_ON, rssi_OFF;
static volatile unsigned long intOffTime,intOnTime;


 OREGON_STATE look4Sync(QUAD_BIT radioBit);
 OREGON_STATE look4PreamblePH1(QUAD_BIT radioBit);
 OREGON_STATE look4PreamblePH2(QUAD_BIT radioBit);
 OREGON_STATE collectData(QUAD_BIT radioBit);
 OREGON_STATE processData(QUAD_BIT radioBit);
 QUAD_BIT     classifier (bool databit, unsigned long bitTime);

 //-------------------------------------------------------------

QUAD_BIT classifier (bool databit, unsigned long bitTime)
{
    QUAD_BIT ret = BAD_ON;

    if (databit)
        {
            if (bitTime >=SHORT_ON_LEFT && bitTime <= SHORT_ON_RIGHT)
                ret = SHORT_ON;
            else if (bitTime >= LONG_ON_LEFT && bitTime <= LONG_ON_RIGHT)
                ret = LONG_ON;
            else
            {
                PAIR hold;
                hold.bit = databit;
                hold.time = bitTime;
                errors.enqueue(hold);
                ret = BAD_ON;
            }

        }
    else
        {
            if (bitTime >=SHORT_OFF_LEFT && bitTime <= SHORT_OFF_RIGHT)
                 ret = SHORT_OFF;
             else if (bitTime >= LONG_OFF_LEFT && bitTime <= LONG_OFF_RIGHT)
                 ret = LONG_OFF;
             else
                 {
                     PAIR hold;
                     hold.bit = databit;
                     hold.time = bitTime;
                     errors.enqueue(hold);
                     ret = BAD_OFF;
                 }
        }
    return ret;
}

//-------------------------------------------------------------

OREGON_STATE look4Sync(QUAD_BIT radioBit)
{

    if (radioBit == SHORT_OFF || radioBit == SHORT_ON)
    {
        syncBitCtr++;
    }

    if (radioBit == LONG_OFF || radioBit == LONG_ON)
    {
        longestSyncRun = (longestSyncRun < syncBitCtr) ? syncBitCtr : longestSyncRun;
        syncBitCtr = 0;
    }
    if (syncBitCtr > 40 ) return LOOK4PREAMBLE_PH1;
    return LOOK4SYNC;


}

//-------------------------------------------------------------
OREGON_STATE look4PreamblePH1(QUAD_BIT radioBit)
{
    if (radioBit == SHORT_OFF || radioBit == SHORT_ON)
    {
        // more sync bits keep going
        return LOOK4PREAMBLE_PH1;
    }

    if (radioBit == LONG_OFF || radioBit == LONG_ON)
    {
        // sync stopped, found first preamble bit.
        preambleDownCnt = 3;  // not 4 we've had first bit from last state
        return LOOK4PREAMBLE_PH2;
    }

    // bad bit.
    syncBitCtr = 0;
    Serial.printf("huh? %d\n", radioBit);

    return LOOK4SYNC;

}
//-------------------------------------------------------------
OREGON_STATE look4PreamblePH2(QUAD_BIT radioBit)
{
    if ((radioBit == LONG_OFF || radioBit == LONG_ON))
    {
        preambleDownCnt--;
        if (!preambleDownCnt)
        {
            flightIndexL3 = 0;
            memset(flightRecorderL3, 0, sizeof(flightRecorderL3));

            return COLLECT_DATA;
        }
        // keep chucking the A or 5;
        return LOOK4PREAMBLE_PH2;
    }

    // bad or unexpected bit.
    syncBitCtr = 0;
    return LOOK4SYNC;

}
//-------------------------------------------------------------
OREGON_STATE collectData(QUAD_BIT radioBit)
{
    flightRecorderL3[flightIndexL3++] = radioBit;

    if (flightIndexL3 < MAX_BITS_IN_DATASTREAM && flightIndexL3 < FLIGHT_LEN) return COLLECT_DATA;

    syncBitCtr = 0;
    return PROCESS_PACKET;
}
//-------------------------------------------------------------
#define c2b(x,y) ((x == '1') << y) //ascii char to packed bit

OREGON_STATE processData(QUAD_BIT radioBit)
{
    unsigned halfbit = 1;
    flightIndexL4 = 0;
    int i;

    for (i = 0; i < flightIndexL3; i++)
    {
        if ( flightRecorderL3[i] == 's' || flightRecorderL3[i] == 'S' )
        {
            if (!(halfbit & 1))
            {
                flightRecorderL4[flightIndexL4++] = (flightRecorderL3[i] == 's') ? '0' : '1';
            }
            halfbit++;
        }
        else if ( flightRecorderL3[i] == 'L' || flightRecorderL3[i] == 'l' )
        {
            flightRecorderL4[flightIndexL4++] = (flightRecorderL3[i] == 'l') ? '0' : '1';
            halfbit += 2;
        }
        else
        {
            // its a bad bit, what do we do?
        }
    }

    // good lord, nibbles are reverse endian. I give up.
    flightIndexL5 = 0;
    memset(flightRecorderL5, 0, sizeof(flightRecorderL5));

    for (i = 0; i < flightIndexL4 +8; i+= 8)
    {
        uint16_t hi,lo;
        Serial.printf("in: %c%c%c%c%c%c%c%c ",
            flightRecorderL4[i+0],flightRecorderL4[i+1],flightRecorderL4[i+2],flightRecorderL4[i+3],
            flightRecorderL4[i+4],flightRecorderL4[i+5],flightRecorderL4[i+6],flightRecorderL4[i+7]);

        hi = c2b(flightRecorderL4[i+0],0) | c2b(flightRecorderL4[i+1],1) | c2b(flightRecorderL4[i+2],2) | c2b(flightRecorderL4[i+3],3);
        lo = c2b(flightRecorderL4[i+4],0) | c2b(flightRecorderL4[i+5],1) | c2b(flightRecorderL4[i+6],2) | c2b(flightRecorderL4[i+7],3);
        flightRecorderL5[flightIndexL5++] = (hi << 4) | lo;

        Serial.printf("  0x%2X \n", flightRecorderL5[flightIndexL5-1]);
    }

    return LOOK4SYNC;
}


//-------------------------------------------------------------
PAIR data;
void radioDataIrq(void)
{
    static unsigned long now, old_time, new_time, bitTime, noise_time;

    if (!bIsWindowOpen()) return;


    bool radioBit = radio.poll();
    now = micros();
    bitTime = now - noise_time;

    QUAD_BIT foo = classifier(!radioBit, bitTime);
    fifo.enqueue(foo);

    flightRecorderL1[flightIndexL1] = foo;
    if (flightIndexL1 < FLIGHT_LEN) flightIndexL1++;

    noise_time = now;

    if (radioBit &&bitTime < 180 )  // piece of noise went down and up.
    {
//      digitalWrite(SCOPE_PIN15, HIGH);
        return;  // noise spur let next edge/state own this time.
    }

//    digitalWrite(SCOPE_PIN09, radioBit);

//    digitalWrite(SCOPE_PIN15, LOW);

    bitTime = now - old_time;
    old_time = now;

    interruptCount++;
    if (!(interruptCount % 200)) bForceReport = true;

    if (!radioBit)
    {
        //cant do below in an interrupt, takes 10ms (ouch)
        //afc_ON = radio.getAFCvalue();
        //fei_ON = radio.getFEIvalue();

        rssi_ON = radio.readRSSI(0);

        intOnTime = bitTime;
        if (bitTime < MAX_TIME) OffTimeHistory[ bitTime/QUANTIZE * QUANTIZE ]++;
    }
    else
    {
        //afc_OFF = radio.getAFCvalue();
        //fei_OFF = radio.getFEIvalue();
        rssi_OFF = radio.readRSSI(0);
        intOffTime = bitTime;
        if (bitTime < MAX_TIME) OnTimeHistory[ bitTime/QUANTIZE * QUANTIZE ]++;
    }
}

//---------------------------------------------------------------------
static int count = 0;
static uint8_t lastRssi;
static bool lastSquelch;
static unsigned loopCount = 0;

void loop()
{
  QUAD_BIT item;
  procKeyboard();

  if (errors.count() > 0)
  {
    PAIR stuff = errors.dequeue();
    Serial.printf("bit=%d time=%ld\n", stuff.bit, stuff.time);
  }

  if (fifo.count() > 0)
  {
    item = fifo.dequeue();

    flightRecorderL2[flightIndexL2] = currentState;
    if (flightIndexL2 < FLIGHT_LEN) flightIndexL2++;

    switch (currentState)

    {
        case LOOK4SYNC:
            nextState = look4Sync(item);
        break;

        case LOOK4PREAMBLE_PH1:
            nextState = look4PreamblePH1(item);
        break;

        case LOOK4PREAMBLE_PH2:
            nextState = look4PreamblePH2(item);
        break;

        case COLLECT_DATA:
            nextState = collectData(item);
        break;

        case PROCESS_PACKET:
            nextState = processData(item);
        break;

        default:
            nextState = LOOK4SYNC;
        break;
     }
  }

  if ( currentState != nextState )
  {
    Serial.printf("state change %c to %c\n", (char) currentState, (char) nextState);
    currentState = nextState;
  }


  if (0) //(interruptCount != lastCount || bForceReport )
  {
      bForceReport = false;
      lastCount = interruptCount;
      reportHistory();

//    Serial.printf("%u hz\n", baseFrequencyHz);

//    Serial.printf("ON  t=%ld AFC = %5d FEI=%5d \n", intOnTime,  afc_ON,fei_ON);
//    Serial.printf("OFF t=%5ld AFC = %5d FEI=%5d \n", intOffTime, afc_OFF,fei_OFF);

//    Serial.printf("ON  t=%5ld snr = %5d\n", intOnTime,  rssi_ON  - noiseFloorDb2);
//    Serial.printf("OFF t=%5ld snr = %5d\n", intOffTime, rssi_OFF - noiseFloorDb2);

  }

  //ook_monFreq(FREQ_OREGON4a);

  //radio.send(0); // tx on
  //txPing(800);

  while(0)
  {
      for (uint32_t freq= 433005000; freq < 434790000; freq += 5000)
      {
          radio.setFrequency(freq);
          Serial.printf(" count = %d freq=%d \n", count++, freq);
          delay(100);

          txPing(800);
          txPing(900);
          txPing(1000);
      }
      Serial.printf("-----------\n");
  }
}
#else
void setup(void)
{
     Serial.begin(115200);
    delay(100);
    Serial.println("snoring");
}
void loop (void)
{
    delay(10000);
}
#endif

//---------------------------------------------------------------------
void reportHistory(void)
{
    bool bPendingCRLF = false;
    Serial.printf("--- %d edges ----%d run ------%d packets ----state %d\n", edgeCount, longestSyncRun, flightIndexL3,currentState);
    for (int i = 0; i < MAX_TIME; i += QUANTIZE)
    {
        if ( !OnTimeHistory[i] && !OffTimeHistory[i])
        {
            //Serial.printf(" %4d-%4d  ON = %4d   OFF=%4d\n", i, i+ QUANTIZE-1, OnTimeHistory[i], OffTimeHistory[i]);
            bPendingCRLF = true;
            continue;
        }
        if (bPendingCRLF) Serial.println();
        bPendingCRLF = false;
        Serial.printf("[%4d-%4d] ON =%8d OFF=%8d\n", i, i+ QUANTIZE-1, OnTimeHistory[i], OffTimeHistory[i]);
    }
}

