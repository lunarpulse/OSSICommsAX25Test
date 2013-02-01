#include "adf7021n.h"

unsigned char ShiftReg;

#define bitSet(value, bit) ((value) |= (0x01 << (bit)))
#define bitClear(value, bit) ((value) &= ~(0x01 << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define setShiftRegLSB(bitValue) (bitWrite(ShiftReg, 0, bitValue ))

char preamble_count;
unsigned char preamble_found;

volatile uint8_t adf702x_buf[] =  {0x7E,0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0xE0,0x96, 0x94, 0x6C, 0x96, 0xA6, 0xA8, 0xE2,0xAE, 0x92, 0x88, 0x8A, 0x62, 0x40, 0x63,0x03,0xF0, 0x21, 0x30, 0x30, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x4E, 0x2F, 0x30, 0x30,
		0x30, 0x30, 0x30, 0x2E, 0x30, 0x30, 0x57, 0x3E,0x38, 0x76,0x7E};
volatile uint8_t adf702x_rx_buf[RF_MAX];

//static const uint32_t adf7021_tx_reg = 0x01600850;
static unsigned char mode;


uint8_t fcslo, fcshi;
uint16_t fcs;
uint8_t shiftbit;
uint8_t stuff;
uint8_t flag, fcsflag;
#define TX_BUFFER_SIZE	(64)
uint8_t txBuffer[64]={ 0x86,   0xA2,  0x40,  0x40,  0x40,  0x40,  // Dst Address
		0x60, // SSID (call sign)
		0X9E,  0xA6,  0xA6, 0x92, 0x40,   0x40,  // Src Address
		0x61, // SSID (last call sign)
		0x03, // Control
		0xF0, // PID
		0x54,0x65,0x73,0x74, // "Test"
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74,
		0x54,0x65,0x73,0x74};


uint8_t bits_step = 0;
uint16_t bytes_step = 0;
uint8_t inbyte;

typedef enum {
	AX25_START,
	AX25_DATA,
	AX25_FCS,
	AX25_END
} TX_PACKET_FRAME;

TX_PACKET_FRAME ax25_packet_mode;



adf7021n_config txReg;

void sendPacket(void);


/*
Goal:
fOut  =  437.850 MHz
fXTAL  = 19.2MHz
Internal Inductor

R = 5
Integer_N = 228
Fractional_N = 1536
TX_FREQUENCY_DEVIATION = 17
DEMO_CLK_DIVIDE = 5
CDR_CLK_DIVIDE = 100

Realized:
fOUT = 437.850 MHz
fPFD = 3.84 MHz
fDEV = 498.046875 Hz
data rate = 1200 bps
*/


//static const uint32_t adf7021_regs[] = {
//	0x01600850, //r0
//	0x00575011, //r1
//	0x0027F082, //r2 // 12.04dBm: PA setting 63
//	0x371493A3, //r3
//	0x80592C94, //r4
//	0x00003155, //r5
//};


//static const uint32_t adf7021_regs[] = {
//	0x06E029A0, //r0
//	0x00C6D051, //r1
//	0x0027F082, //r2 // 12.04dBm: PA setting 63
//	0x3714FD63, //r3
//	0x00394A94, //r4
//	0x00003155, //r5
//	0x0000020F
//};


void ax25_makePacket(char* dstAddr, char* srcAddr, uint8_t* data, uint8_t dataSize)
{
	volatile uint8_t i;

	// prevent buffer overflow
	if ( dataSize + 16 >= TX_BUFFER_SIZE)
	{
		dataSize = TX_BUFFER_SIZE - 16;
	}

	for ( i = 0 ; i < TX_BUFFER_SIZE ; i++)
	{
		txBuffer[i] = 0;
	}

	// First 16 Bytes data format is fixed
	// Refer to AX.25 UI frame
	// 6 Bytes
	// address field shift 1 bit to the left
	for(i = 0; i < 6 ; i++)
	{
		txBuffer[i] = (uint8_t)dstAddr[i] << 1;
	}

	// 1 Byte
	txBuffer[6] = 0x60;

	// 6 Bytes
	// address field shift 1 bit to the left
	for(i = 0; i < 6 ; i++)
	{
		txBuffer[i+7] = (uint8_t)srcAddr[i] << 1;
	}

	// 3 Bytes
	txBuffer[13] = 0x61;
	txBuffer[14] = 0x03;
	txBuffer[15] = 0xF0;

	for ( i = 0 ; i < dataSize ; i ++)
	{
		txBuffer[i+16] = data[i];
	}
}

void adf7021n_enable_data_interrupt()
{

//	// RX_TXCLK is 1.2
// 	P1OUT |= BIT2; // pull up
//	P1IE |= BIT2; // interrupt enable
//	P1IES |= BIT2; // interrupt hi/lo falling edge
//	P1IFG &= ~BIT2; // P1.2 IFG cleared just in case

	// TX_TXCLK is 2.3
// 	P2OUT |= BIT3; // pull up
 	P2IES |= BIT3; // interrupt hi/lo falling edge
 	P2IFG &= ~BIT3; // P2.3 IFG cleared just in case
 	P2IE |= BIT3; // interrupt enable


	// data communication check!
	//P6DIR |= BIT0;
//	IO_DIRECTION TODO: change IO_DIRECTION
}
void adf7021n_portSetup(void)
{
	P1OUT &= ~(RX_SCLK_PIN + RX_SREAD_PIN + RX_SDATA_PIN);
	P1DIR &= ~(RX_MUXOUT_PIN + RX_CLK_PIN + RX_DATA_PIN + RX_SWD_PIN);
	P1DIR |= (RX_SCLK_PIN + RX_SREAD_PIN + RX_SDATA_PIN);

	P2OUT &= ~(RX_SLE_PIN + RX_CE_PIN + TX_DATA_PIN + TX_SCLK_PIN + TX_SREAD_PIN + TX_SDATA_PIN);
	P2DIR &= ~ (TX_MUXOUT_PIN + TX_CLK_PIN);
	P2DIR |= (RX_SLE_PIN + RX_CE_PIN + TX_DATA_PIN + TX_SCLK_PIN + TX_SREAD_PIN + TX_SDATA_PIN);

	P4OUT &= ~(TX_SLE_PIN + TX_CE_PIN + TX_ON_PIN + RX_ON_PIN + PA_ON_PIN);
	P4DIR |= (TX_SLE_PIN + TX_CE_PIN + TX_ON_PIN + RX_ON_PIN + PA_ON_PIN);
}

void adf7021n_txEnable(void)
{
	P4OUT |= TX_CE_PIN;
}

void adf7021n_txDisable(void)
{
	P4OUT &= ~TX_CE_PIN;
}

void adf7021n_initRegisterZero(void)
{
	txReg.r0.fractional_n = 1536; // change fractional_n to change offset frequency error
	txReg.r0.integer_n = 228;
	txReg.r0.tx_rx = ADF7021N_TX_MODE;
	txReg.r0.uart_mode = ADF7021N_UART_DISABLED;
	txReg.r0.muxout = ADF7021N_MUXOUT_DIGITAL_LOCK_DETECT;
}

void adf7021n_initRegisterOne(void)
{
	txReg.r1.r_counter = 5;
	txReg.r1.clkout_divide = 0; // Pin CLKOUT Off
	txReg.r1.xtal_doubler = ADF7021N_XTAL_DOUBLER_DISABLE;
	txReg.r1.xosc_enable = ADF7021N_XOSC_ENABLE_ON; // for OSC1 pin 0.8Vpp Clipped Sine-wave
	txReg.r1.xtal_bias = 0; // default
	txReg.r1.cp_current = 0; // default
	txReg.r1.vco_enable = ADF7021N_VCO_ENABLE_ON; // on by default
	txReg.r1.rf_divide_by_2 = ADF7021N_RF_DIVIDE_BY_2_ON; // Internal Inductor for 437.850MHz out
	txReg.r1.vco_bias = 0; // default
	txReg.r1.vco_adjust = 0; // default
	txReg.r1.vco_inductor = ADF7021N_VCO_INDUCTOR_INTERNAL; // Internal Inductor for 437.850MHz out
}

void adf7021n_initRegisterTwo(void)
{
	txReg.r2.modulation_scheme = ADF7021N_MODULATION_2FSK;
	txReg.r2.pa_enable = ADF7021N_PA_ENABLE_OFF; // PA off by default
	txReg.r2.pa_ramp = ADF7021N_PA_RAMP_NO_RAMP; // default
	txReg.r2.pa_bias = 0; // default
	txReg.r2.power_amplifier = 0; // off by default
	txReg.r2.tx_frequency_deviation = 17; // for 1200 bps data rate
	txReg.r2.txdata_invert = 0; // normal by default
	txReg.r2.r_cosine_alpha = 0; // 0.5 by default
}

void adf7021n_initRegisterThree(void)
{
	txReg.r3.bbos_clk_divide = 0; // default
	txReg.r3.demod_clk_divide = 5; // for 1200 bps data rate
	txReg.r3.cdr_clk_divide = 100; // for 1200 bps data rate
	txReg.r3.seq_clk_divide = 0; // default
	txReg.r3.agc_clk_divide = 1; // default (0 is invalid value)
}

void adf7021n_initAllTxRegisgers(void)
{
	adf7021n_initRegisterZero();
	adf7021n_initRegisterOne();
	adf7021n_initRegisterTwo();
	adf7021n_initRegisterThree();
}

void adf7021n_txInit(void)
{
	adf7021n_initAllTxRegisgers();
	P2IES |= BIT3; // interrupt hi/lo falling edge

}

void byte_write(uint8_t _register, uint8_t mode)
{
	volatile int i;

	if (mode == TX) {
		for(i = 7; i >= 0; i--)
		{
			P2OUT &= ~TX_SCLK_PIN;
//			IO_SET(TX_SCLK, LOW);
			if(_register & (1<<i)) {
				P2OUT |= TX_SDATA_PIN;
//				IO_SET(TX_SDATA, HIGH);
			}
			else {
				P2OUT &= ~TX_SDATA_PIN;
//				IO_SET(TX_SDATA, LOW);
			}
			__delay_cycles(5);
			P2OUT |= TX_SCLK_PIN;
//			IO_SET(TX_SCLK, HIGH);
			__delay_cycles(15);
		}
		P2OUT &= ~TX_SCLK_PIN;
//		IO_SET(TX_SCLK, LOW);
	} else if (mode == RX) {
		for(i = 7; i >= 0; i--)
		{
			P1OUT &= ~RX_SCLK_PIN;
//			IO_SET(RX_SCLK, LOW);
			if(_register & (1<<i)) {
				P1OUT |= RX_SDATA_PIN;
//				IO_SET(RX_SDATA, HIGH);
			}
			else {
				P1OUT &= ~RX_SDATA_PIN;
//				IO_SET(RX_SDATA, LOW);
			}
			__delay_cycles(5);
			P1OUT |= RX_SCLK_PIN;
//			IO_SET(RX_SCLK, HIGH);
			__delay_cycles(15);
		}
		P1OUT &= ~RX_SCLK_PIN;
//		IO_SET(RX_SCLK, LOW);
	}
}

void adf7021n_regWrite(uint32_t registers, uint8_t mode)
{
	volatile int i;
	unsigned char _register;

	for(i=0;i<sizeof(registers);i++)
	{
		_register = (registers >> 24-(i*8)) & 0xff;
		byte_write(_register, mode);
	}
    // SLE
	if (mode == TX) {
		P4OUT |= TX_SLE_PIN;
//		IO_SET(TX_SLE, HIGH);
		__delay_cycles(10);
		P2OUT &= ~TX_SDATA_PIN;
//		IO_SET(TX_SDATA, LOW);                       // SDATA low
		P4OUT &= ~TX_SLE_PIN;
//		IO_SET(TX_SLE, LOW);
	} else if (mode == RX) {
		P2OUT |= RX_SLE_PIN;
//		IO_SET(RX_SLE, HIGH);
		__delay_cycles(10);
		P1OUT &= ~RX_SDATA_PIN;
//		IO_SET(RX_SDATA, LOW);                       // SDATA low
		P2OUT &= ~RX_SLE_PIN;
//		IO_SET(RX_SLE, LOW);
	}
	__delay_cycles(30);
}

void adf7021n_writeRegisterZero(uint8_t mode)
{
	uint32_t reg =
			(0) |
			((uint32_t)(txReg.r0.fractional_n & 0x7FFF) << 4) |
			((uint32_t)(txReg.r0.integer_n & 0xFF)<< 19) |
			((uint32_t)(txReg.r0.tx_rx & 0x01)<< 27) |
			((uint32_t)(txReg.r0.uart_mode & 0x01)<< 28) |
			((uint32_t)(txReg.r0.muxout & 0x07)<< 29);

	adf7021n_regWrite(reg, mode);
}


void adf7021n_writeRegisterOne(uint8_t mode)
{
	uint32_t reg =
			(1) |
			((uint32_t)(txReg.r1.r_counter & 0x07) << 4) |
			((uint32_t)(txReg.r1.clkout_divide & 0x0F)<< 7) |
			((uint32_t)(txReg.r1.xtal_doubler & 0x01)<< 11) |
			((uint32_t)(txReg.r1.xosc_enable & 0x01)<< 12) |
			((uint32_t)(txReg.r1.xtal_bias & 0x03)<< 13) |
			((uint32_t)(txReg.r1.cp_current & 0x03)<< 15) |
			((uint32_t)(txReg.r1.vco_enable & 0x01)<< 17) |
			((uint32_t)(txReg.r1.rf_divide_by_2 & 0x01)<< 18) |
			((uint32_t)(txReg.r1.vco_bias & 0x0F)<< 19) |
			((uint32_t)(txReg.r1.vco_adjust & 0x03)<< 23) |
			((uint32_t)(txReg.r1.vco_inductor & 0x01)<< 25);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterTwo(uint8_t mode)
{
	uint32_t reg =
			(2) |
			((uint32_t)(txReg.r2.modulation_scheme & 0x07) << 4) |
			((uint32_t)(txReg.r2.pa_enable & 0x01)<< 7) |
			((uint32_t)(txReg.r2.pa_ramp & 0x07)<< 8) |
			((uint32_t)(txReg.r2.pa_bias & 0x03)<< 11) |
			((uint32_t)(txReg.r2.power_amplifier & 0x3F)<< 13) |
			((uint32_t)(txReg.r2.tx_frequency_deviation & 0x1FF)<< 19) |
			((uint32_t)(txReg.r2.txdata_invert & 0x03)<< 28) |
			((uint32_t)(txReg.r2.r_cosine_alpha & 0x01)<< 30);

	adf7021n_regWrite(reg, mode);
}


void adf7021n_writeRegisterThree(uint8_t mode)
{
	uint32_t reg =
			(3) |
			((uint32_t)(txReg.r3.bbos_clk_divide & 0x03) << 4) |
			((uint32_t)(txReg.r3.demod_clk_divide & 0x0F)<< 6) |
			((uint32_t)(txReg.r3.cdr_clk_divide & 0xFF)<< 10) |
			((uint32_t)(txReg.r3.seq_clk_divide & 0xFF)<< 18) |
			((uint32_t)(txReg.r3.agc_clk_divide & 0x3F)<< 26);

	adf7021n_regWrite(reg, mode);
}



void adf7021n_rx()
{
	P4OUT &= ~TX_CE_PIN;
	P2OUT |= RX_CE_PIN;

//	IO_SET(TX_CE, LOW);
//	IO_SET(RX_CE, HIGH);

	// fill adf702x_rx_buf
 	// r4 for 1200 bps
//	adf702x_write(adf7021_regs[1], RX);
//	adf702x_write(adf7021_regs[3], RX);
//	adf702x_write(adf7021_regs[5], RX);
//	adf702x_write(adf7021_regs[0], RX);
//	adf702x_write(adf7021_regs[4], RX);

    //mode = RX;
    mode = IDLE;
}

void adf7021n_setTxFracN(uint16_t fracN)
{
	// TODO: boundary check fracN: 0~32767
	txReg.r0.fractional_n = fracN;
}

uint16_t adf7021n_getTxFracN(void)
{
	return txReg.r0.fractional_n;
}

void adf7021n_setTxMuxout(paramMuxout muxout)
{
	txReg.r0.muxout = muxout;
}

uint8_t adf7021n_getTxMuxout(void)
{
	return txReg.r0.muxout;
}

// no parameter type for this function as we change this value in the code
void adf7021n_setTxVcoBias(uint8_t vcoBias)
{
	txReg.r1.vco_bias = vcoBias;
}

uint8_t adf7021n_getTxVcoBias(void)
{
	return txReg.r1.vco_bias;
}

void adf7021n_setTxVcoAdjust(uint8_t vcoAdjust)
{
	txReg.r1.vco_adjust = vcoAdjust;
}

uint8_t adf7021n_getTxVcoAdjust(void)
{
	return txReg.r1.vco_adjust;
}

void adf7021n_setTxVcoEnableOff(void)
{
	txReg.r1.vco_enable = ADF7021N_VCO_ENABLE_OFF;
}

void adf7021n_setTxVcoEnableOn(void)
{
	txReg.r1.vco_enable = ADF7021N_VCO_ENABLE_ON;
}

void adf7021n_setTxPowerAmp(paramPaRamp paRamp, paramPaBias paBias, uint8_t paLevel)
{
	txReg.r2.pa_ramp = paRamp;
	txReg.r2.pa_bias = paBias;
	txReg.r2.power_amplifier = paLevel; // TODO: boundary check 0 ~ 63
}

uint8_t adf7021n_getTxPALevel(void)
{
	return txReg.r2.power_amplifier;
}

void adf7021n_setTxPowerAmpOn(void)
{
	txReg.r2.pa_enable = ADF7021N_PA_ENABLE_ON;
}

void adf7021n_setTxPowerAmpOff(void)
{
	txReg.r2.pa_enable = ADF7021N_PA_ENABLE_OFF;
}

void adf7021n_tx()
{
	adf7021n_writeRegisterOne(TX);
	//wait 0.7 ms
	adf7021n_writeRegisterThree(TX);
	adf7021n_writeRegisterZero(TX);
	// wait 40 us
	// check PLL to be locked
	adf7021n_writeRegisterTwo(TX);
	mode = TX;
}



void adf7021n_tx1010test()
{
	adf7021n_writeRegisterOne(TX);
	adf7021n_writeRegisterThree(TX);
	adf7021n_writeRegisterZero(TX);
	adf7021n_writeRegisterTwo(TX);
	adf7021n_regWrite(0x0000040F, TX); //R15 1010 test pattern
	mode = TX;
}


void adf7021n_txCarriertest()
{
	adf7021n_writeRegisterOne(TX);
	//wait 0.7 ms
	adf7021n_writeRegisterThree(TX);
	adf7021n_writeRegisterZero(TX);
	// wait 40 us
	// check PLL to be locked
	adf7021n_writeRegisterTwo(TX);
	adf7021n_regWrite(0x0000010F, TX); //R15 Carrier F test pattern

	mode = TX;
}

void adf7021n_txHightest()
{

	adf7021n_writeRegisterOne(TX);
	adf7021n_writeRegisterThree(TX);
	adf7021n_writeRegisterZero(TX);
	adf7021n_writeRegisterTwo(TX);
	adf7021n_regWrite(0x0000020F, TX); //R15 high tone test pattern
	mode = TX;
}


void adf7021n_txLowtest()
{
	adf7021n_writeRegisterOne(TX);
	adf7021n_writeRegisterThree(TX);
	adf7021n_writeRegisterZero(TX);
	adf7021n_writeRegisterTwo(TX);
	adf7021n_regWrite(0x0000030F, TX); //R15 low tone test pattern
	mode = TX;
}



unsigned char adf7021n_getMode()
{
	return mode;
}



void adf7021n_recvStart()
{
	//adf7021n_enable_data_interrupt();
	adf7021n_rx();
}

void TX_PA_PowerOn()
{
	P4OUT |= PA_ON_PIN;
//    IO_DIRECTION(PA_ON, OUTPUT);
//    IO_SET(PA_ON, HIGH);
}

//void adf7021n_setTxPaLevel()
//{
//	// TODO: make change level.
////	adf702x_write(adf7021_regs[2] , TX);
//}


void adf7021n_sendStart(void)
{
	sendPacket();
	P2OUT |= TX_DATA_PIN;//TODO: default data status???? high or low??
//	IO_SET(TX_DATA,HIGH);
	adf7021n_regWrite(0x0000000F, TX);
	adf7021n_tx(); // set ADF register
	TX_PA_PowerOn(); // PA on
	ax25_packet_mode = AX25_START;
	bytes_step=0;
	bits_step=0;
}

void flipout(void)
{
	stuff = 0;
	if(!(P2OUT & BIT4))
		P2OUT |= TX_DATA_PIN;
//		IO_SET(TX_DATA,HIGH); //check TXDATA out register value
	else
		P2OUT &= ~TX_DATA_PIN;
//		IO_SET(TX_DATA,LOW);
}

void fcsbit(uint8_t tbyte)
{
	shiftbit = fcs & 0x0001;
	fcs = fcs >> 1;
	if((shiftbit ^ tbyte) == 0x01)
	{
		fcs = fcs ^ 0x8408;
	}

}

void sendByte()
{
	// NRZI
	volatile uint8_t bt;
	bt = inbyte & 0x01;
	if(ax25_packet_mode == AX25_DATA)
	{
		fcsbit(bt);
	}
	if(bt == 0)
	{
		flipout();
	}
	else
	{
		stuff++;
		if((ax25_packet_mode == AX25_DATA || ax25_packet_mode == AX25_FCS) && (stuff == 5))
		{
			flipout();
			bits_step--;
			return;
		}
	}
	inbyte = inbyte >> 1;
}

void sendPacket(void)
{
	fcslo=fcshi = 0xff;
	fcs = 0xffff;
	stuff = 0;
}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void adf7021n_Data_Tx_handler(void)
{
	switch (P2IFG & BIT3) {
		case BIT3:
			//flag 20 times
			if(ax25_packet_mode == AX25_START)
			{
				// if send 20 times, go to AX25_DATA
				if (bytes_step > 100) {
					ax25_packet_mode = AX25_DATA;
					bytes_step = 0; // prepare to go next mode.
					bits_step = 0;
					//P2IFG &= ~BIT3; // P2.3 IFG cleared
					break;
				}

				if (bits_step == 0) {
					inbyte = 0x7E;
				}
				sendByte();
			} else if(ax25_packet_mode == AX25_DATA) //data
			{
				// if sent all sendData, go to AX25_FCS
				if (bytes_step > sizeof(txBuffer)-1) {
					ax25_packet_mode = AX25_FCS;
					bytes_step=0;
					bits_step = 0;
					//P2IFG &= ~BIT3; // P2.3 IFG cleared
					break;
				}

				// get data at each byte.
				if(bits_step == 0 ) {
					inbyte = txBuffer[bytes_step];
				}
				// send bytes (based on sizeof txBuffer)
				sendByte();

			} else if(ax25_packet_mode == AX25_FCS) //fcs
			{
				if (bytes_step > sizeof(fcs)-1) {
					ax25_packet_mode = AX25_END;
					bytes_step=0;
					bits_step = 0;
					//P2IFG &= ~BIT3; // P2.3 IFG cleared
					break;
				}

				if(bits_step == 0 && bytes_step == 0) {
					fcs = fcs ^ 0xffff;
					fcslo = fcs & 0xff;
					fcshi = (fcs & 0xff00)>>8;
					inbyte = fcslo;

				}
				if (bits_step == 0 && bytes_step == 1 )inbyte = fcshi;
				if (bytes_step == 0) sendByte();
				if (bytes_step == 1 )sendByte();

				// go to AX25_END
			} else if(ax25_packet_mode == AX25_END) //flag 1 times
			{
				// if send 10 times, go to AX25_DATA
				if (bytes_step > 10) {
					// diable CE
					// go to default ax25 packet mode

					P4OUT &= ~TX_CE_PIN;
//					IO_SET(TX_CE, LOW);
					ax25_packet_mode = AX25_START;
					bytes_step = 0; // prepare to go next mode.
					bits_step = 0;
					//P2IFG &= ~BIT3; // P2.3 IFG cleared
					break;
				}

				if (bits_step == 0) {
					inbyte = 0x7E;
				}
				sendByte();
			}

			bits_step++;

			if (bits_step > 7)
			{
				bits_step = 0;
				bytes_step++;
			}
			P2IFG &= ~BIT3; // P2.3 IFG cleared
			break;
		default :
			P2IFG = 0;
			break;
	}
}


// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void adf7021n_Data_Rx_handler(void)
{
	switch (mode) {
		case RX:
			ShiftReg = ShiftReg << 1;
//			setShiftRegLSB(P5IN & BIT0);
			setShiftRegLSB(P1IN & BIT3);

		    bits_step++;

		    if (bits_step == 8) {
		    	bits_step = 0;
		    	switch(bytes_step){
		    		case 0:
		    			if (ShiftReg != SYNC_WORD2) {
		    				mode = IDLE;
		    			}
		    			break;
		    		case 1:
		    			// header 0
		    			break;
		    		case 2:
		    			// header 1
		    			break;
		    		case 3:
		    			// byte size
		    			break;
		    		default:
		    			adf702x_rx_buf[bytes_step-4] = ShiftReg;
		    	}

		        if(bytes_step > 4+6){ // 6 is data length
	        	  P6OUT ^= BIT0;
                  bytes_step = 0;
		          mode = IDLE;
		          // CE LOW?
		        }
		    	bytes_step++;

		    }
		    break;

		case IDLE:
			ShiftReg=ShiftReg<<1;
//			setShiftRegLSB(P5IN & BIT0); RX_DATA
			setShiftRegLSB(P1IN & BIT3);

			if (preamble_found == 1) {
				if (ShiftReg == SYNC_WORD1) {
					bits_step=0;
					bytes_step=0;
					preamble_count=0;
					preamble_found=0;
					mode = RX;
				}
				break;
			}

			if ((ShiftReg == VALID_PREAMBLE_BYTE_1) || (ShiftReg == VALID_PREAMBLE_BYTE_2)) {
				preamble_count++;
			} else {
				preamble_count = 0;
				preamble_found = 0; //false
			}

			if (preamble_count >= 16) {
				preamble_found = 1; //false
			}
			break;
	}
	P1IFG &= ~BIT2; // P1.2 IFG cleared
}
