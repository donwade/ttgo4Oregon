// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <drw_utils.h>
#include <RFM69OOK.h>
#include <RFM69OOKregisters.h>
#include <SPI.h>
#include <esp32-hal-log.h>

//const lmic_pinmap lmic_pins = { ttygo
//  .nss = 18,
//  .rxtx = LMIC_UNUSED_PIN,
//  .rst = 14,
//  .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
//};



RFM69OOK* RFM69OOK::selfPointer;

bool RFM69OOK::initialize()
{
  const byte CONFIG[][2] =
  {
#define LEGACY
#ifdef LEGACY
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_OFF | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, 0x68 }, //  bitrate: 0x3D1 =32768 Hz
    /* 0x04 */ { REG_BITRATELSB, 0x2B},	 //  bitrate  0x682B = 1200Hz
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_4}, // BW: 10.4 kHz
    /* 0x1B */ { REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000 },
    /* 0x1D */ { REG_OOKFIX, 6 }, // Fixed threshold value (in dB) in the OOK demodulator
    /* 0x29 */ { REG_RSSITHRESH, 140 }, // RSSI threshold in dBm = -(REG_RSSITHRESH / 2)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
#else
		{ REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },

		//default:5khz, (FDEV + BitRate/2 <= 500Khz)
		{ REG_FDEVMSB, RF_FDEVMSB_5000},
		{ REG_FDEVLSB, RF_FDEVLSB_5000},

		//(BitRate < 2 * RxBw)
		//~ { REG_RXBW, RF_RXBW_DCCFREQ_001 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0 },

		//(BitRate < 2 * RxBw)
		{ REG_RXBW, RF_RXBW_DCCFREQ_100 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0 },

		//~ { REG_LNA, RF_LNA_GAINSELECT_MAXMINUS6},

		{ REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK},

		//DIO0 is the only IRQ we're using
		{ REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 | RF_DIOMAPPING1_DIO2_01 },
		{ REG_DIOMAPPING2, RF_DIOMAPPING2_DIO5_01 | RF_DIOMAPPING2_DIO4_10},


		// default 3 preamble bytes 0xAAAAAA
		{ REG_PREAMBLELSB, 5 },

		//~ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_SIZE_1 | RF_SYNC_TOL_7 },
		{ REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_SIZE_2 | RF_SYNC_TOL_5 },

		{ REG_SYNCVALUE1, 0xaa },
		{ REG_SYNCVALUE2, 0x66 },

//~
		//~  0x2f  { REG_SYNCVALUE1, 0xaa },      //attempt to make this compatible with sync1 byte of RFM12B lib
		//~  0x2f  { REG_SYNCVALUE2, 0xaa },
		//~  0x2f  { REG_SYNCVALUE3, 0xaa },
		//~  0x2f  { REG_SYNCVALUE4, 0xaa },
		//~  0x2f  { REG_SYNCVALUE5, 0xaa },
		//~  0x2f  { REG_SYNCVALUE6, 0xaa },
		//~  0x2f  { REG_SYNCVALUE7, 0xaa },
		//~  0x2f  { REG_SYNCVALUE8, 0xaa },

		{ REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },

		//TX on FIFO not empty
		{ REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE },

		//RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		{ REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF },

		// // TODO: Should use LOWBETA_ON, but having trouble getting it working
		{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 },

		// AFC Offset for low mod index systems
		{ REG_TESTAFC, 0 },
#endif

    {255, 0}
  };

  resetHW();


  pinMode(_slaveSelectPin, OUTPUT);
  SPI.begin();

  for (byte i = 0; CONFIG[i][0] != 255; i++)
  {
  	log_d("[0x%02X] = 0x%02X", CONFIG[i][0], CONFIG[i][1]);
    writeReg(CONFIG[i][0], CONFIG[i][1]);
  }
  setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  setMode(RF69OOK_MODE_STANDBY);

  readAllRegs();
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

  Serial.println("radio init done");
  readAllRegs();

  selfPointer = this;
  return true;
}

void RFM69OOK::setStandby(void)
{
	setMode(RF69OOK_MODE_STANDBY);
	while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); //  # Wait for ModeReady
}

void RFM69OOK::setDataMode(bool packet, bool bitsync )
{
    unsigned int datamode;

	if (packet)
		datamode = RF_DATAMODUL_DATAMODE_PACKET;
	else
	{
		if (bitsync)
			datamode = RF_DATAMODUL_DATAMODE_CONTINUOUS;
		else
			datamode = RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC;
	}
	writeReg( REG_DATAMODUL, datamode | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00 );// #no shaping
}

void RFM69OOK::setCarrier(float freq)
{
	setFrequencyMHz(freq);
}

bool RFM69OOK::is_noise(char *data)
{
	return true;
#if 0
	rolling_counter = 0
	for c in data:
		if c == 0xFF:
			rolling_counter = 0
		else:
			rolling_counter += 1

		if rolling_counter >= 20:
			return False
#endif
}


void RFM69OOK::setPayloadLength(unsigned int x)
{
	writeReg( REG_PAYLOADLENGTH, x ); // #in variable length mode: the max frame size, not used in TX
}

#define NO_NOISE 2000000
uint8_t  RFM69OOK::setSquelchByOOK(uint32_t calFreq)
{
    int hits;
    uint32_t saveFreq;
	uint8_t ookNoiseFloor;

    saveFreq = getFrequency();
	setFrequency(calFreq);

    writeReg(REG_OOKPEAK, 0); //set fixed threshold OOK squelch
    while (1)
    {
        for (ookNoiseFloor = 0; ookNoiseFloor < 255; ookNoiseFloor++)
        {
            bool test = poll();

            writeReg(REG_OOKFIX, ookNoiseFloor);

            Serial.printf("%d OOK noise ookNoiseFloor at %d db\r", calFreq, ookNoiseFloor);
            for ( hits = 0; hits < NO_NOISE; hits++)
            {
                delayMicroseconds(1);
                bool thisBit = poll();
                if (test == poll()) continue;
                Serial.printf(" count = %d\r", hits);
                break;
            }
            if (hits == NO_NOISE)
            {
                Serial.printf("%d OOK noise ookNoiseFloor %d db (count = %d)----------------------------\n", calFreq, ookNoiseFloor, hits);
                setFrequency(saveFreq);
                return ookNoiseFloor;
            }
        }
        setFrequency(saveFreq);
        return ookNoiseFloor;
    }
}
//---------------------------------------------------------------------
int32_t RFM69OOK::getAFCvalue(void)
{
	uint16_t msb,lsb, temp;

	temp = readReg(REG_AFCFEI);
	writeReg(REG_AFCFEI, temp & ~RF_AFCFEI_AFCAUTO_ON);//afc_ON in manual mode
	writeReg(REG_AFCFEI, temp | RF_AFCFEI_AFC_CLEAR);  //clean result reg
	writeReg(REG_AFCFEI, temp | RF_AFCFEI_AFC_START);  //start conversion
	while (!( readReg(REG_AFCFEI) & RF_AFCFEI_AFC_DONE));

	msb = readReg(REG_AFCMSB);
	lsb = readReg(REG_AFCLSB);
	//msb = 0x10;
	//lsb = 0x00;

	int16_t raw = ((msb << 8) | lsb);
	//Serial.printf("msb=%u lsb=%u raw=%d\n" , msb, lsb, raw);
	int32_t ret = ((int32_t)raw * RF69OOK_FSTEP);
	//int32_t ret = ((int32_t)raw );
	return ret;
}
//---------------------------------------------------------------------
int32_t RFM69OOK::getFEIvalue(void)
{
	uint16_t msb,lsb,temp;
	temp = readReg(REG_AFCFEI);
	temp |= RF_AFCFEI_FEI_START;
	writeReg(REG_AFCFEI, temp);

	while (!( readReg(REG_AFCFEI) & RF_AFCFEI_FEI_DONE));

	msb = readReg(REG_FEIMSB);
	lsb = readReg(REG_FEILSB);
	int16_t raw = (int16_t) ((msb << 8) | lsb);
	int32_t ret = (int32_t)(raw * RF69OOK_FSTEP);
	return ret;
}
//---------------------------------------------------------------------
void RFM69OOK::setOOKsliceMode(slice_t slicetype)
{
	showRegister(REG_OOKPEAK);
    uint8_t control = readReg(REG_OOKPEAK);
    writeReg(REG_OOKPEAK, (control & 0x3F) | slicetype << 6);
	showRegister(REG_OOKPEAK);
}

bool RFM69OOK::calRssiThreshold()
{
	int i;
	for (i = -255; i != 1; i++)
	{
		setRSSIThreshold(i);
		delay(300);
		int16_t rssi = readRSSI(0);

		uint8_t bSquelchOpen = !!(readReg(REG_IRQFLAGS1) & 0x40);

		Serial.printf("rssi = %03d >  threshold = %03d bSquelchOpen = %1d\n", rssi, i , bSquelchOpen);
	}
	return true;
}
//---------------------------------------------------------------------
void RFM69OOK::resetHW()
{
	if (_resetPin)
	{
		pinMode(_resetPin, OUTPUT);
		digitalWrite(_resetPin, HIGH);
		delay(10);
		digitalWrite(_resetPin, LOW);
		log_d("reset RFM by pin %d", _resetPin);
	}
}
//---------------------------------------------------------------------
// Poll for OOK signal
bool RFM69OOK::poll()
{
  return digitalRead(_interruptPin);
}

// Send a 1 or 0 signal in OOK mode
void RFM69OOK::send(bool signal)
{
  digitalWrite(_interruptPin, signal);
}

// Turn the radio into transmission mode
void RFM69OOK::transmitBegin()
{
  setMode(RF69OOK_MODE_TX);
  detachInterrupt(_interruptNum); // not needed in TX mode
  pinMode(_interruptPin, OUTPUT);
}

// Turn the radio back to standby
void RFM69OOK::transmitEnd()
{
  pinMode(_interruptPin, INPUT);
  setMode(RF69OOK_MODE_STANDBY);
}

// Turn the radio into OOK listening mode
void RFM69OOK::receiveBegin()
{
  pinMode(_interruptPin, INPUT);
  attachInterrupt(_interruptNum, RFM69OOK::isr0, CHANGE); // generate interrupts in RX mode
  setMode(RF69OOK_MODE_RX);
  log_d("interupt Pin %d connected to isr %d ATTACHED to isr0", _interruptPin, _interruptNum);
}

// Turn the radio back to standby
void RFM69OOK::receiveEnd()
{
  setMode(RF69OOK_MODE_STANDBY);
  detachInterrupt(_interruptNum); // make sure there're no surprises
  log_d("interrupt Rx pin disconnected. Now in standby");
}

// Handle pin change interrupts in OOK mode
void RFM69OOK::interruptHandler()
{
  if (userInterrupt != NULL) (*userInterrupt)();
}

// Set a user interrupt for all transfer methods in receive mode
// call with NULL to disable the user interrupt handler
void RFM69OOK::attachUserInterrupt(void (*function)())
{
  userInterrupt = function;
}

// return the frequency (in Hz)
uint32_t RFM69OOK::getFrequency()
{
  uint32_t ret = RF69OOK_FSTEP * (((uint32_t)readReg(REG_FRFMSB)<<16) + ((uint16_t)readReg(REG_FRFMID)<<8) + readReg(REG_FRFLSB));
  log_d("setting frequency to %d hz", ret);
  return ret;
}

// Set literal frequency using floating point MHz value
void RFM69OOK::setFrequencyMHz(float f)
{
  setFrequency(f * 1000000);
}

// set the frequency (in Hz)
void RFM69OOK::setFrequency(uint32_t freqHz)
{
  // TODO: p38 hopping sequence may need to be followed in some cases
  log_d("setting frequency to %d hz", freqHz);
  freqHz /= RF69OOK_FSTEP; // divide down by FSTEP to get FRF
  writeReg(REG_FRFMSB, freqHz >> 16);
  writeReg(REG_FRFMID, freqHz >> 8);
  writeReg(REG_FRFLSB, freqHz);
  for (int i= 0; i < 100; i++)
  	if ((readReg(REG_IRQFLAGS1) & 0x10)) break ; // wait for pll lock
}

// Set bitrate
void RFM69OOK::setBitrate(uint32_t bitrate)
{
  log_d("setting bit rate to %d", bitrate);
  bitrate = 32000000 / bitrate; // 32M = XCO freq.
  writeReg(REG_BITRATEMSB, bitrate >> 8);
  writeReg(REG_BITRATELSB, bitrate);
}

// set OOK bandwidth
void RFM69OOK::setBandwidth(uint8_t bw)
{
  writeReg(REG_RXBW, readReg(REG_RXBW) & 0xE0 | bw);
  log_d("setting bandwidth to %d", bw);
}


#define RFM_CLOCK 32000000
void RFM69OOK::setBandwidthHz(uint32_t usrBwHz)
{
    unsigned long RxBwExp;
    unsigned long RxBwMant;
    log_d("set bandwidth to %d\n", usrBwHz);
    //Serial.printf("REG_RXBW in : [0x%X] = 0x%X\n", REG_RXBW, readReg(REG_RXBW));
    for ( RxBwExp = 0; RxBwExp < 8; RxBwExp++)
    {
        for (RxBwMant = 16; RxBwMant < 28; RxBwMant += 4)
        {
            unsigned long bw = (float)RFM_CLOCK / (float)(RxBwMant * ((unsigned long) 1 << (RxBwExp+3)));
            if ( usrBwHz <= bw )
            {
                //Serial.printf(" RxMant = %lu RxBwExp = %lu bw = %lu\n", RxBwMant, RxBwExp, bw);
                unsigned int Mant = RxBwMant/4 - 4;
                uint8_t rmw = readReg(REG_RXBW);
                rmw &= ~0x1F; rmw |= ((Mant << 3) | RxBwExp);
                writeReg(REG_RXBW, rmw);
            }

        }
    }
	log_d("REG_RXBW out: [0x%X] = 0x%X\n", REG_RXBW, readReg(REG_RXBW));
}

// set RSSI threshold
void RFM69OOK::setRSSIThreshold(int16_t rssi)
{
  writeReg(REG_RSSITHRESH, (-rssi ));
  // dwade writeReg(REG_RSSITHRESH, (-rssi << 1));
}

void RFM69OOK::readModWriteRFM(byte reg, byte left, byte right, byte value)
{
	//dprintf("start = %d end = %d value = %X\n", left, right, value);

	byte lmask = ~((1 << left+1) -1);
	byte rmask =  (1 << right) -1;
	byte drop = rmask | lmask;
	//debugByteInBinary(lmask);
	//debugByteInBinary(rmask);
	//debugByteInBinary(drop);
	byte read = readReg(reg);
	//debugByteInBinary(read);
	read &= drop;
	value <<= right;
	byte write = read | value;
	//debugByteInBinary(write);

	writeReg(reg, write);
	//Serial.println("---------------");
}

void RFM69OOK::showBinary(uint8_t what)
{
	for (int i = 0; i < 8; i++)
	{
		Serial.print((what & 0x80) ? '1' : '0');
		if (i == 3) Serial.print('.');
		what <<= 1;
	}
}

uint8_t RFM69OOK::_showRegister(char*name, uint8_t someRegister)
{
	uint8_t reg = readReg(someRegister);

	Serial.printf ("[%s 0x%02X] = 0x%02X  ", name, someRegister, reg);
	showBinary(reg); Serial.println();
	return reg;
}

const int8_t db2rfm[] = { +99, 0, -6, -12, -24, -36, -48 }; //[0]=auto [8]=reserved
void RFM69OOK::setLnaAtten(int8_t setLnaDb)
{
  int toRfm;
  if (setLnaDb < -48) setLnaDb = -48;
  if (setLnaDb > 0 ) setLnaDb = 0;

  // answer = 0 means auto agc (not offered)
  for (toRfm = 0; toRfm < 8; toRfm++)
  {
	if ( setLnaDb >= db2rfm[toRfm] ) break;
  }

  log_d("setLnaAtten usr_val = %d hw_map=%d\n", setLnaDb, db2rfm[toRfm]);

  showRegister(REG_LNA);
  readModWriteRFM(REG_LNA, 2, 0, toRfm);  // set fixed threshold mode
  showRegister(REG_LNA);
}

int8_t RFM69OOK::getLnaAtten(void)
{
  uint8_t index;
  showRegister(REG_LNA);
  index = (readReg(REG_LNA) >> 3) & 7 ;  // read CURRENT active lna atten.
  return db2rfm[index];
}


// set OOK fixed threshold
void RFM69OOK::setFixedThreshold(uint8_t threshold)
{
  writeReg(REG_OOKFIX, threshold);
}

// set sensitivity boost in REG_TESTLNA
// see: http://www.sevenwatt.com/main/rfm69-ook-dagc-sensitivity-boost-and-modulation-index
void RFM69OOK::setSensitivityBoost(uint8_t value)
{
  writeReg(REG_TESTLNA, value);
}

void RFM69OOK::setMode(byte newMode)
{
	log_d("newMode=%d _mode=%d", newMode, _mode);
    if (newMode == _mode) return;

    switch (newMode) {
        case RF69OOK_MODE_TX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      		if (_isRFM69HW) setHighPowerRegs(true);
            break;
        case RF69OOK_MODE_RX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      		if (_isRFM69HW) setHighPowerRegs(false);
            break;
        case RF69OOK_MODE_SYNTH:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69OOK_MODE_STANDBY:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69OOK_MODE_SLEEP:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default:
        	log_e("unknown mode %d", newMode);
	        return;
    }

    // waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
    log_d("waiting for mode %d to set", newMode);
    while (_mode == RF69OOK_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
    log_d("mode %d set ok", newMode);
    _mode = newMode;
}

void RFM69OOK::sleep() {
  log_d("going into sleep mode");
  setMode(RF69OOK_MODE_SLEEP);
}

// set output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
void RFM69OOK::setPowerLevel(byte powerLevel)
{
  _powerLevel = powerLevel > 31 ? 31 : powerLevel;
  writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
  log_d("power level set to %d", _powerLevel);
}

void RFM69OOK::isr0() { selfPointer->interruptHandler(); }

int16_t RFM69OOK::readRSSI(bool forceTrigger) {

  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // Wait for RSSI_Ready
  }

  // dwade return -(readReg(REG_RSSIVALUE) >> 1);
  return -(readReg(REG_RSSIVALUE));
}

byte RFM69OOK::readReg(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

void RFM69OOK::writeReg(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

// Select the transceiver
void RFM69OOK::select() {
  noInterrupts();
  // save current SPI settings
  ////_SPCR = SPCR;
  ////_SPSR = SPSR;
  // set RFM69 SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); //decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  digitalWrite(_slaveSelectPin, LOW);
}

/// UNselect the transceiver chip
void RFM69OOK::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
  // restore SPI settings to what they were before talking to RFM69
  ////SPCR = _SPCR;
  ////SPSR = _SPSR;
  interrupts();
}

/*

Table 10 Power Amplifier Mode Selection Truth Table
  PA0    PA1     PA2                                           Power Range  Pout Formula
   1      0      0		PA0 output on pin RFIO 				 -18 to +13 dBm -18  dBm + OutputPower
   0      1      0      PA1 enabled on pin PA_BOOST			  -2 to +13 dBm -18  dBm + OutputPower
   0      1      1		PA1 and PA2 combined on pin PA_BOOST  +2 to +17 dBm -14  dBm + OutputPower
   1      1      1      PA1+PA2 on PA_BOOST 				  +5 to +20 dBm -11  dBm + OutputPower
Notes - To ensure correct operation at the highest power levels, please make sure to adjust the Over Current Protection

*/

void RFM69OOK::setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only

  log_d("high power tx is set %s", onOff ? "HIGH - PA1&2 ready":"LOW - PA0 only");
}

void RFM69OOK::setHighPowerRegs(bool onOff) {
  log_d("POWER to PA amps is %s", onOff ? "ON":"OFF");
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void RFM69OOK::setCS(byte newSPISlaveSelect) {
  _slaveSelectPin = newSPISlaveSelect;
  pinMode(_slaveSelectPin, OUTPUT);
}

// for debugging
void RFM69OOK::readAllRegs()
{
  byte regVal;
  for (byte regAddr = 1; regAddr <= 0x4F; regAddr++) {
	  regVal = readReg(regAddr);
	  Serial.printf ("[%02X] = 0x%02X ", regAddr, regVal);
	  showBinary(regVal);
	  Serial.println();
  }

  for (byte regAddr = 0x4E; regAddr <= 0x71; regAddr++) {
    regVal = readReg(regAddr);
    Serial.printf ("[%02X] = 0x%02X ", regAddr, regVal);
    showBinary(regVal);
    Serial.println();
   }
}

byte RFM69OOK::readTemperature(byte calFactor)  // returns centigrade
{
  setMode(RF69OOK_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
}                                                            // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void RFM69OOK::rcCalibration()
{
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

