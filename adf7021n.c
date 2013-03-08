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



adf7021n_config adf7021nReg;


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


void adf7021n_rxEnable(void)
{
	P2OUT |= RX_CE_PIN;
}

void adf7021n_rxDisable(void)
{
	P2OUT &= ~RX_CE_PIN;
}


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


void adf7021n_initRegisterZero(uint8_t mode)
{

	adf7021nReg.r0.fractional_n = 1536; // change fractional_n to change offset frequency error
	adf7021nReg.r0.integer_n = 228;
	adf7021nReg.r0.tx_rx = ADF7021N_TX_MODE;
	adf7021nReg.r0.uart_mode = ADF7021N_UART_DISABLED;
	adf7021nReg.r0.muxout = ADF7021N_MUXOUT_DIGITAL_LOCK_DETECT;

	if (mode == RX)
	{
		adf7021nReg.r0.tx_rx = ADF7021N_RX_MODE;
	}
}

void adf7021n_initRegisterOne(void)
{
	adf7021nReg.r1.r_counter = 5;
	adf7021nReg.r1.clkout_divide = 0; // Pin CLKOUT Off
	adf7021nReg.r1.xtal_doubler = ADF7021N_XTAL_DOUBLER_DISABLE;
	adf7021nReg.r1.xosc_enable = ADF7021N_XOSC_ENABLE_ON; // for OSC1 pin 0.8Vpp Clipped Sine-wave
	adf7021nReg.r1.xtal_bias = ADF7021N_XTAL_BIAS_35uA; // for fast start up
	adf7021nReg.r1.cp_current = 0; // change this after measuring voltage of VCOIN so it can be in the middle
	adf7021nReg.r1.vco_enable = ADF7021N_VCO_ENABLE_ON; // on by default
	adf7021nReg.r1.rf_divide_by_2 = ADF7021N_RF_DIVIDE_BY_2_ON; // Internal Inductor for 437.850MHz out
	adf7021nReg.r1.vco_bias = 8; // default
	adf7021nReg.r1.vco_adjust = 3; // default
	adf7021nReg.r1.vco_inductor = ADF7021N_VCO_INDUCTOR_INTERNAL; // Internal Inductor for 437.850MHz out
}

void adf7021n_initRegisterTwo(void)
{
	adf7021nReg.r2.modulation_scheme = ADF7021N_MODULATION_2FSK;
	adf7021nReg.r2.pa_enable = ADF7021N_PA_ENABLE_OFF; // PA off by default
	adf7021nReg.r2.pa_ramp = ADF7021N_PA_RAMP_NO_RAMP; // default
	adf7021nReg.r2.pa_bias = 0; // default
	adf7021nReg.r2.power_amplifier = 0; // off by default
	adf7021nReg.r2.tx_frequency_deviation = 17; // for 1200 bps data rate
	adf7021nReg.r2.txdata_invert = 0; // normal by default
	adf7021nReg.r2.r_cosine_alpha = 0; // 0.5 by default
}

void adf7021n_initRegisterThree(uint8_t mode)
{
	adf7021nReg.r3.bbos_clk_divide = 16; // default
	adf7021nReg.r3.demod_clk_divide = 5; // for 1200 bps data rate
	adf7021nReg.r3.cdr_clk_divide = 100; // for 1200 bps data rate
	adf7021nReg.r3.seq_clk_divide = 192; // default
	adf7021nReg.r3.agc_clk_divide = 13; // default (0 is invalid value)
}

void adf7021n_initRegisterFour(void)
{
	adf7021nReg.r4.demode_scheme = ADF7021N_2FSK_CORREL_DEMOD;
	adf7021nReg.r4.dot_product = ADF7021N_DOT_PRODUCT_CROSS;
	adf7021nReg.r4.rx_invert = ADF7021N_RX_INVERT_NORMAL;
	adf7021nReg.r4.discriminator_bw = 0;
	adf7021nReg.r4.post_demod_bw = 0;
	adf7021nReg.r4.if_filter_bw = 0;
}

void adf7021n_initRegisterFive(void)
{
	adf7021nReg.r5.if_cal_coarse = ADF7021N_IF_CAL_COARSE_NO_CAL;
	adf7021nReg.r5.if_filter_divider = 0;
	adf7021nReg.r5.if_filer_adjust = 0;
	adf7021nReg.r5.ir_phase_adjust_mag = 0;
	adf7021nReg.r5.ir_phase_adjust_direction = ADF7021N_IR_PHASE_ADJ_DIR_I_CH;
	adf7021nReg.r5.ir_gain_adjust_mag = 0;
	adf7021nReg.r5.ir_gain_adjust_i_q = ADF7021N_IR_GAIN_ADJ_I_CH;
	adf7021nReg.r5.ir_gain_adjust_up_dn = ADF7021N_IR_GAIN_ADJ_UP_DN_GAIN;
}

void adf7021n_initRegisterSix(void)
{
	adf7021nReg.r6.if_fine_cal = ADF7021N_IF_FINE_CAL_DISABLED;
	adf7021nReg.r6.if_cal_lower_tone_divide = 0;
	adf7021nReg.r6.if_cal_upper_tone_divide = 0;
	adf7021nReg.r6.if_cal_dwell_time = 0;
	adf7021nReg.r6.ir_cal_source_drive_level = ADF7021N_IR_CAL_SRC_DRV_LEVEL_OFF;
	adf7021nReg.r6.ir_cal_source_divide_by_2 = ADF7021N_IR_CAL_SOURCE_DIVIDE_2_OFF;
}

void adf7021n_initRegisterSeven(void)
{
	adf7021nReg.r7.adc_mode = ADF7021N_ADC_MODE_RSSI;
	adf7021nReg.r7.readback_mode = ADF7021N_READBACK_MODE_AFC;
	adf7021nReg.r7.readback_select = ADF7021N_READBACK_DISABLED;
}

void adf7021n_initRegisterNine(void)
{
	adf7021nReg.r9.agc_low_threshold = 30; // default value
	adf7021nReg.r9.agc_high_threshold = 70; // default value
	adf7021nReg.r9.agc_mode = ADF7021N_AGC_MODE_AUTO;
	adf7021nReg.r9.lna_gain = ADF7021N_LNA_GAIN_3;
	adf7021nReg.r9.filter_gain = ADF7021N_FILTER_GAIN_8;
	adf7021nReg.r9.filter_current = ADF7021N_FILTER_CURRENT_LOW;
	adf7021nReg.r9.lna_mode = ADF7021N_LNA_MODE_DEFAULT;
	adf7021nReg.r9.lna_bias = 0;
	adf7021nReg.r9.mixer_linearity = ADF7021N_MIXER_LINEARITY_HIGH;
}

void adf7021n_initRegisterTen(void)
{
	adf7021nReg.r10.afc_en = ADF7021N_AFC_EN_OFF;
	adf7021nReg.r10.afc_scaling_factior = 0;
	adf7021nReg.r10.ki = 0;
	adf7021nReg.r10.kp = 0;
	adf7021nReg.r10.max_afc_range = 0;
}

void adf7021n_initRegisterFourteen(void)
{
	adf7021nReg.r14.test_dac_en = ADF7021N_TEST_DAC_EN_OFF;
	adf7021nReg.r14.test_dac_offset = 0;
	adf7021nReg.r14.test_dac_gain = 0;
	adf7021nReg.r14.pulse_extension = 0;
	adf7021nReg.r14.ed_leak_factor = 0;
	adf7021nReg.r14.ed_peak_response = 0;
}

void adf7021n_initRegisterFifteen(void)
{
	adf7021nReg.r15.rx_test_mode = ADF7021N_RX_TEST_NORMAL;
	adf7021nReg.r15.tx_test_mode = ADF7021N_TX_TEST_NORMAL;
	adf7021nReg.r15.sigma_delta_test_mode = 0;
	adf7021nReg.r15.pfd_cp_test_modes = 0;
	adf7021nReg.r15.clk_mux = 0;
	adf7021nReg.r15.pll_test_mode = 0;
	adf7021nReg.r15.analog_test_mode = ADF7021N_AG_TEST_BANDGAP_V;
	adf7021nReg.r15.force_ld_high = 0;
	adf7021nReg.r15.reg_1_pd = 0;
	adf7021nReg.r15.cal_override = 0;
}


void adf7021n_initAllTxRegisgers(void)
{
	adf7021n_initRegisterZero(TX);
	adf7021n_initRegisterOne();
	adf7021n_initRegisterTwo();
	adf7021n_initRegisterThree(TX);
}

void adf7021n_txInit(void)
{
	adf7021n_initAllTxRegisgers();
	P2IES |= BIT3; // interrupt hi/lo falling edge
}




// Register 0 set / get functions

void adf7021n_setFracN(uint16_t fracN)
{
	// TODO: boundary check fracN: 0~32767
	adf7021nReg.r0.fractional_n = fracN;
}

uint16_t adf7021n_getFracN(void)
{
	return adf7021nReg.r0.fractional_n;
}

void adf7021n_setIntegerN(uint8_t intN)
{
	// TODO: boundary check
	adf7021nReg.r0.integer_n = intN;
}

void adf7021n_setTxRx(paramTxRx txRx)
{
	adf7021nReg.r0.tx_rx = txRx;
}

uint8_t adf7021n_getTxRx(void)
{
	return adf7021nReg.r0.tx_rx;
}

void adf7021n_setMuxout(paramMuxout muxout)
{
	adf7021nReg.r0.muxout = muxout;
}

uint8_t adf7021n_getMuxout(void)
{
	return adf7021nReg.r0.muxout;
}

// Register 1 set / get functions
void adf7021n_setRCounter(uint8_t rCounter)
{
	adf7021nReg.r1.r_counter = rCounter;
}


void adf7021n_setChargePumpCurrent (paramCpCurrent cpCurrent)
{
	adf7021nReg.r1.cp_current = cpCurrent;
}

uint8_t adf7021n_getChargePumpCurrent(void)
{
	return adf7021nReg.r1.cp_current;
}


void adf7021n_setVcoEnableOff(void)
{
	adf7021nReg.r1.vco_enable = ADF7021N_VCO_ENABLE_OFF;
}

void adf7021n_setVcoEnableOn(void)
{
	adf7021nReg.r1.vco_enable = ADF7021N_VCO_ENABLE_ON;
}

// no parameter type for this function as we change this value in the code
void adf7021n_setVcoBias(uint8_t vcoBias)
{
	adf7021nReg.r1.vco_bias = vcoBias;
}

uint8_t adf7021n_getVcoBias(void)
{
	return adf7021nReg.r1.vco_bias;
}

void adf7021n_setVcoAdjust(uint8_t vcoAdjust)
{
	adf7021nReg.r1.vco_adjust = vcoAdjust;
}

uint8_t adf7021n_getVcoAdjust(void)
{
	return adf7021nReg.r1.vco_adjust;
}


// Register 2 set / get functions

void adf7021n_setModulationScheme(paramModulation modulationScheme)
{
	adf7021nReg.r2.modulation_scheme = modulationScheme;
}

void adf7021n_setPowerAmpOn(void)
{
	adf7021nReg.r2.pa_enable = ADF7021N_PA_ENABLE_ON;
}

void adf7021n_setPowerAmpOff(void)
{
	adf7021nReg.r2.pa_enable = ADF7021N_PA_ENABLE_OFF;
}

void adf7021n_setPowerAmp(paramPaRamp paRamp, paramPaBias paBias, uint8_t paLevel)
{
	adf7021nReg.r2.pa_ramp = paRamp;
	adf7021nReg.r2.pa_bias = paBias;
	adf7021nReg.r2.power_amplifier = paLevel; // TODO: boundary check 0 ~ 63
}

uint8_t adf7021n_getPALevel(void)
{
	return adf7021nReg.r2.power_amplifier;
}

void adf7021n_setTxFreqDeviation(uint16_t txFreqDev)
{
	adf7021nReg.r2.tx_frequency_deviation = txFreqDev;
}


// Register 3 set / get functions

void adf7021n_setDemodDivider(uint8_t demodDiv)
{
	adf7021nReg.r3.demod_clk_divide = demodDiv;
}


void adf7021n_setCDRDivider(uint8_t cdrDiv)
{
	adf7021nReg.r3.cdr_clk_divide = cdrDiv;
}


// Register 4 set / get functions

void adf7021n_setDemodScheme(paramDemodScheme demodScheme)
{
	adf7021nReg.r4.demode_scheme = demodScheme;
}

void adf7021n_setDotProduct(paramDotProduct dotProduct)
{
	adf7021nReg.r4.dot_product = dotProduct;
}
void adf7021n_setRxInvert(paramRxInvert rxInvert)
{
	adf7021nReg.r4.rx_invert = rxInvert;
}

void adf7021n_setDiscriminatorBW(uint16_t discrimBW)
{
	adf7021nReg.r4.discriminator_bw = discrimBW;
}

void adf7021n_setPostDemodBW(uint16_t postDemodBW)
{
	adf7021nReg.r4.post_demod_bw = postDemodBW;
}

void adf7021n_setIFFilterBW(paramIfFiltBW ifFiltBW)
{
	adf7021nReg.r4.if_filter_bw = ifFiltBW;
}


// Register 5 set / get functions

void adf7021n_setIFCalCoarseON(void)
{
	adf7021nReg.r5.if_cal_coarse = ADF7021N_IF_CAL_COARSE_DO_CAL;
}

void adf7021n_setIFCalCoarseOFF(void)
{
	adf7021nReg.r5.if_cal_coarse = ADF7021N_IF_CAL_COARSE_NO_CAL;
}

void adf7021n_setIFFliterAdj(uint8_t adj)
{
	adf7021nReg.r5.if_filer_adjust = adj;
}

void adf7021n_setIRPhaseAdjMag(uint8_t mag)
{
	adf7021nReg.r5.ir_phase_adjust_mag = mag;
}

void adf7021n_setIRPhasAdjDir_I(void)
{
	adf7021nReg.r5.ir_phase_adjust_direction = ADF7021N_IR_PHASE_ADJ_DIR_I_CH;
}

void adf7021n_setIRPhasAdjDir_Q(void)
{
	adf7021nReg.r5.ir_phase_adjust_direction = ADF7021N_IR_PHASE_ADJ_DIR_Q_CH;
}

void adf7021n_setIRGainAdjMag(uint8_t mag)
{
	adf7021nReg.r5.ir_gain_adjust_mag = mag;
}

void adf7021n_setIRGainAdj_I(void)
{
	adf7021nReg.r5.ir_gain_adjust_i_q = ADF7021N_IR_GAIN_ADJ_I_CH;
}

void adf7021n_setIRGainAdj_Q(void)
{
	adf7021nReg.r5.ir_gain_adjust_i_q = ADF7021N_IR_GAIN_ADJ_Q_CH;
}

void adf7021n_setIRGainAdj_Gain(void)
{
	adf7021nReg.r5.ir_gain_adjust_up_dn = ADF7021N_IR_GAIN_ADJ_UP_DN_GAIN;
}

void adf7021n_setIRGainAdj_Atten(void)
{
	adf7021nReg.r5.ir_gain_adjust_up_dn = ADF7021N_IR_GAIN_ADJ_UP_DN_ATTEN;
}

// Register 6 set / get functions

void adf7021n_setIFCalFineON(void)
{
	adf7021nReg.r6.if_fine_cal = ADF7021N_IF_FINE_CAL_ENABLED;
}

void adf7021n_setIFCalFineOFF(void)
{
	adf7021nReg.r6.if_fine_cal = ADF7021N_IF_FINE_CAL_DISABLED;
}

void adf7021n_setIRCalSrcDrvLevel(paramIRCalSrcDrvLevel level)
{
	adf7021nReg.r6.ir_cal_source_drive_level = level;
}

void adf7021n_setIRCalSrcDiv2ON(void)
{
	adf7021nReg.r6.ir_cal_source_divide_by_2 = ADF7021N_IR_CAL_SOURCE_DIVIDE_2_ON;
}

void adf7021n_setIRCalSrcDiv2OFF(void)
{
	adf7021nReg.r6.ir_cal_source_divide_by_2 = ADF7021N_IR_CAL_SOURCE_DIVIDE_2_OFF;
}

// Register 7 set / get functions


void adf7021n_setADCMode(paramAdcMode adcMode)
{
	adf7021nReg.r7.adc_mode = adcMode;
}

void adf7021n_setReadbackMode(paramReadbackMode readbackMode)
{
	adf7021nReg.r7.readback_mode = readbackMode;
}

void adf7021n_setReadbackSelectON(void)
{
	adf7021nReg.r7.readback_select = ADF7021N_READBACK_ENABLED;
}

void adf7021n_setReadbackSelectOFF(void)
{
	adf7021nReg.r7.readback_select = ADF7021N_READBACK_DISABLED;
}


// Register 9 set / get functions

void adf7021n_setAGCLowThreshold(uint8_t lowThreshold)
{
	adf7021nReg.r9.agc_low_threshold = lowThreshold;
}

uint8_t adf7021n_getAGCLowThreshold(void)
{
	return adf7021nReg.r9.agc_low_threshold;
}

void adf7021n_setAGCHighThreshold(uint8_t highThreshold)
{
	adf7021nReg.r9.agc_high_threshold = highThreshold;
}

uint8_t adf7021n_getAGCHighThreshold(void)
{
	return adf7021nReg.r9.agc_high_threshold;
}

void adf7021n_setAGCMode(paramAgcMode agcMode)
{
	adf7021nReg.r9.agc_mode = agcMode;
}

void adf7021n_setLNAGain(paramLnaGain lnaGain)
{
	adf7021nReg.r9.lna_gain = lnaGain;
}

void adf7021n_setFilterGain(paramFilterGain filtGain)
{
	adf7021nReg.r9.filter_gain = filtGain;
}

void adf7021n_setFilterCurrent_Low(void)
{
	adf7021nReg.r9.filter_current = ADF7021N_FILTER_CURRENT_LOW;
}

void adf7021n_setFilterCurrent_High(void)
{
	adf7021nReg.r9.filter_current = ADF7021N_FILTER_CURRENT_HIGH;
}

void adf7021n_setLNAMode_Default(void)
{
	adf7021nReg.r9.lna_mode = ADF7021N_LNA_MODE_DEFAULT;
}

void adf7021n_setLNAMode_ReducedGain(void)
{
	adf7021nReg.r9.lna_mode = ADF7021N_LNA_MODE_REDUCED_GAIN;
}

void adf7021n_setMixerLinearity_Default(void)
{
	adf7021nReg.r9.mixer_linearity = ADF7021N_MIXER_LINEARITY_DEFAULT;
}

void adf7021n_setMixerLinearity_High(void)
{
	adf7021nReg.r9.mixer_linearity = ADF7021N_MIXER_LINEARITY_HIGH;
}




void byte_write(uint8_t _register, uint8_t mode)
{
	volatile int i;

	if (mode == TX) {
		for(i = 7; i >= 0; i--)
		{
			P2OUT &= ~TX_SCLK_PIN;
			if(_register & (1<<i)) {
				P2OUT |= TX_SDATA_PIN;
			}
			else {
				P2OUT &= ~TX_SDATA_PIN;
			}
			__delay_cycles(5);
			P2OUT |= TX_SCLK_PIN;
			__delay_cycles(15);
		}
		P2OUT &= ~TX_SCLK_PIN;
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
		__delay_cycles(10);
		P2OUT &= ~TX_SDATA_PIN;
		P4OUT &= ~TX_SLE_PIN;
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
			((uint32_t)(adf7021nReg.r0.fractional_n & 0x7FFF) << 4) |
			((uint32_t)(adf7021nReg.r0.integer_n & 0xFF)<< 19) |
			((uint32_t)(adf7021nReg.r0.tx_rx & 0x01)<< 27) |
			((uint32_t)(adf7021nReg.r0.uart_mode & 0x01)<< 28) |
			((uint32_t)(adf7021nReg.r0.muxout & 0x07)<< 29);

	adf7021n_regWrite(reg, mode);
}


void adf7021n_writeRegisterOne(uint8_t mode)
{
	uint32_t reg =
			(1) |
			((uint32_t)(adf7021nReg.r1.r_counter & 0x07) << 4) |
			((uint32_t)(adf7021nReg.r1.clkout_divide & 0x0F)<< 7) |
			((uint32_t)(adf7021nReg.r1.xtal_doubler & 0x01)<< 11) |
			((uint32_t)(adf7021nReg.r1.xosc_enable & 0x01)<< 12) |
			((uint32_t)(adf7021nReg.r1.xtal_bias & 0x03)<< 13) |
			((uint32_t)(adf7021nReg.r1.cp_current & 0x03)<< 15) |
			((uint32_t)(adf7021nReg.r1.vco_enable & 0x01)<< 17) |
			((uint32_t)(adf7021nReg.r1.rf_divide_by_2 & 0x01)<< 18) |
			((uint32_t)(adf7021nReg.r1.vco_bias & 0x0F)<< 19) |
			((uint32_t)(adf7021nReg.r1.vco_adjust & 0x03)<< 23) |
			((uint32_t)(adf7021nReg.r1.vco_inductor & 0x01)<< 25);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterTwo(uint8_t mode)
{
	uint32_t reg =
			(2) |
			((uint32_t)(adf7021nReg.r2.modulation_scheme & 0x07) << 4) |
			((uint32_t)(adf7021nReg.r2.pa_enable & 0x01)<< 7) |
			((uint32_t)(adf7021nReg.r2.pa_ramp & 0x07)<< 8) |
			((uint32_t)(adf7021nReg.r2.pa_bias & 0x03)<< 11) |
			((uint32_t)(adf7021nReg.r2.power_amplifier & 0x3F)<< 13) |
			((uint32_t)(adf7021nReg.r2.tx_frequency_deviation & 0x1FF)<< 19) |
			((uint32_t)(adf7021nReg.r2.txdata_invert & 0x03)<< 28) |
			((uint32_t)(adf7021nReg.r2.r_cosine_alpha & 0x01)<< 30);

	adf7021n_regWrite(reg, mode);
}


void adf7021n_writeRegisterThree(uint8_t mode)
{
	uint32_t reg =
			(3) |
			((uint32_t)(adf7021nReg.r3.bbos_clk_divide & 0x03) << 4) |
			((uint32_t)(adf7021nReg.r3.demod_clk_divide & 0x0F)<< 6) |
			((uint32_t)(adf7021nReg.r3.cdr_clk_divide & 0xFF)<< 10) |
			((uint32_t)(adf7021nReg.r3.seq_clk_divide & 0xFF)<< 18) |
			((uint32_t)(adf7021nReg.r3.agc_clk_divide & 0x3F)<< 26);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterFour(uint8_t mode)
{
	uint32_t reg =
			(4) |
			((uint32_t)(adf7021nReg.r4.demode_scheme & 0x07) << 4) |
			((uint32_t)(adf7021nReg.r4.dot_product & 0x01) << 7) |
			((uint32_t)(adf7021nReg.r4.rx_invert & 0x03)<< 8) |
			((uint32_t)(adf7021nReg.r4.discriminator_bw & 0x7FF)<< 10) |
			((uint32_t)(adf7021nReg.r4.post_demod_bw & 0x7FF)<< 20) |
			((uint32_t)(adf7021nReg.r4.if_filter_bw & 0x03)<< 30);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterFive(uint8_t mode)
{
	uint32_t reg =
			(5) |
			((uint32_t)(adf7021nReg.r5.if_cal_coarse & 0x01) << 4) |
			((uint32_t)(adf7021nReg.r5.if_filter_divider & 0x1FF)<< 5) |
			((uint32_t)(adf7021nReg.r5.if_filer_adjust & 0x3F)<< 14) |
			((uint32_t)(adf7021nReg.r5.ir_phase_adjust_mag & 0x0F)<< 20) |
			((uint32_t)(adf7021nReg.r5.ir_phase_adjust_direction & 0x01)<< 24) |
			((uint32_t)(adf7021nReg.r5.ir_gain_adjust_mag & 0x1F)<< 25) |
			((uint32_t)(adf7021nReg.r5.ir_gain_adjust_i_q & 0x01)<< 30) |
			((uint32_t)(adf7021nReg.r5.ir_gain_adjust_up_dn & 0x01)<< 31);
	adf7021n_regWrite(reg, mode);
}


void adf7021n_writeRegisterSix(uint8_t mode)
{
	uint32_t reg =
			(6) |
			((uint32_t)(adf7021nReg.r6.if_fine_cal & 0x01) << 4) |
			((uint32_t)(adf7021nReg.r6.if_cal_lower_tone_divide & 0xFF)<< 5) |
			((uint32_t)(adf7021nReg.r6.if_cal_upper_tone_divide & 0xFF)<< 13) |
			((uint32_t)(adf7021nReg.r6.if_cal_dwell_time & 0x7F)<< 21) |
			((uint32_t)(adf7021nReg.r6.ir_cal_source_drive_level & 0x03)<< 28) |
			((uint32_t)(adf7021nReg.r6.ir_cal_source_divide_by_2 & 0x01)<< 30);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterSeven(uint8_t mode)
{
	uint32_t reg =
			(7) |
			((uint32_t)(adf7021nReg.r7.adc_mode & 0x03) << 4) |
			((uint32_t)(adf7021nReg.r7.readback_mode & 0x03)<< 6) |
			((uint32_t)(adf7021nReg.r7.readback_select & 0x01)<< 8);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterNine(uint8_t mode)
{
	uint32_t reg =
			(9) |
			((uint32_t)(adf7021nReg.r9.agc_low_threshold & 0x7F) << 4) |
			((uint32_t)(adf7021nReg.r9.agc_high_threshold & 0x7F)<< 11) |
			((uint32_t)(adf7021nReg.r9.agc_mode & 0x03)<< 18) |
			((uint32_t)(adf7021nReg.r9.lna_gain & 0x03)<< 20) |
			((uint32_t)(adf7021nReg.r9.filter_gain & 0x03)<< 22) |
			((uint32_t)(adf7021nReg.r9.filter_current & 0x01)<< 24) |
			((uint32_t)(adf7021nReg.r9.lna_mode & 0x01)<< 25) |
			((uint32_t)(adf7021nReg.r9.lna_bias & 0x03)<< 26) |
			((uint32_t)(adf7021nReg.r9.mixer_linearity & 0x01)<< 28);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterTen(uint8_t mode)
{
	uint32_t reg =
			(10) |
			((uint32_t)(adf7021nReg.r10.afc_en & 0x01) << 4) |
			((uint32_t)(adf7021nReg.r10.afc_scaling_factior & 0xFFF)<< 5) |
			((uint32_t)(adf7021nReg.r10.ki & 0x0F)<< 17) |
			((uint32_t)(adf7021nReg.r10.kp & 0x07)<< 21) |
			((uint32_t)(adf7021nReg.r10.max_afc_range & 0xFF)<< 24);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterFourteen(uint8_t mode)
{
	uint32_t reg =
			(14) |
			((uint32_t)(adf7021nReg.r14.test_dac_en & 0x01) << 4) |
			((uint32_t)(adf7021nReg.r14.test_dac_offset & 0xFFFF)<< 5) |
			((uint32_t)(adf7021nReg.r14.test_dac_gain & 0x0F)<< 21) |
			((uint32_t)(adf7021nReg.r14.pulse_extension & 0x03)<< 25) |
			((uint32_t)(adf7021nReg.r14.ed_leak_factor & 0x07)<< 27) |
			((uint32_t)(adf7021nReg.r14.ed_peak_response & 0x03)<< 30);

	adf7021n_regWrite(reg, mode);
}

void adf7021n_writeRegisterFifteen(uint8_t mode)
{
	uint32_t reg =
			(15) |
			((uint32_t)(adf7021nReg.r15.rx_test_mode & 0x0F) << 4) |
			((uint32_t)(adf7021nReg.r15.tx_test_mode & 0x07)<< 8) |
			((uint32_t)(adf7021nReg.r15.sigma_delta_test_mode & 0x07)<< 11) |
			((uint32_t)(adf7021nReg.r15.pfd_cp_test_modes & 0x07)<< 14) |
			((uint32_t)(adf7021nReg.r15.clk_mux & 0x07)<< 17) |
			((uint32_t)(adf7021nReg.r15.pll_test_mode & 0x0F)<< 20) |
			((uint32_t)(adf7021nReg.r15.analog_test_mode & 0x0F)<< 24) |
			((uint32_t)(adf7021nReg.r15.force_ld_high & 0x01)<< 28) |
			((uint32_t)(adf7021nReg.r15.reg_1_pd & 0x01)<< 29) |
			((uint32_t)(adf7021nReg.r15.cal_override & 0x03)<< 30);

	adf7021n_regWrite(reg, mode);
}


void adf7021n_rx()
{

	adf7021n_writeRegisterOne(RX);
	// wait 0.7 ms
	adf7021n_writeRegisterThree(RX);
	//adf7021n_writeRegisterSix(RX);
	adf7021n_writeRegisterFive(RX);
	// wait 0.2ms or 8.2 ms
	adf7021n_writeRegisterZero(RX);
	// wait 40 us
	adf7021n_writeRegisterFour(RX);

	//adf7021n_writeRegisterTen(RX);

    //mode = RX;
    mode = IDLE;
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
	adf7021n_regWrite(0x0000010F, TX); //R15 Carrier test pattern

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

void adf7021n_txNotest()
{
	adf7021n_writeRegisterOne(TX);
	adf7021n_writeRegisterThree(TX);
	adf7021n_writeRegisterZero(TX);
	adf7021n_writeRegisterTwo(TX);
	adf7021n_regWrite(0x0000000F, TX);
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
//	P2OUT |= TX_DATA_PIN;//TODO: default data status???? high or low??
	P2OUT &= ~TX_DATA_PIN;//TODO: default data status???? high or low??

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
	if(!(P2OUT & TX_DATA_PIN))
		P2OUT |= TX_DATA_PIN;
	else
		P2OUT &= ~TX_DATA_PIN;
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
			//flag 100 times
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
				if (bits_step == 0 && bytes_step == 1)inbyte = fcshi;
				if (bytes_step == 0) sendByte();
				if (bytes_step == 1) sendByte();

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
