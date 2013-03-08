/*
 * adf7021n.h
 *
 *  Created on: 2012. 6. 18.
 *      Author: donghee
 */

#ifndef ADF7021N_H_
#define ADF7021N_H_

#include  <msp430x16x.h>
#include "global.h"


// Port 1
#define RX_MUXOUT_PIN	(BIT1)
#define RX_CLK_PIN 		(BIT2)
#define RX_DATA_PIN 	(BIT3)
#define RX_SWD_PIN 		(BIT4)
#define RX_SCLK_PIN 	(BIT5)
#define RX_SREAD_PIN 	(BIT6)
#define RX_SDATA_PIN 	(BIT7)


// Port 2
#define RX_SLE_PIN 		(BIT0)
#define RX_CE_PIN 		(BIT1)
#define TX_MUXOUT_PIN	(BIT2)
#define TX_CLK_PIN 		(BIT3) // INPUT
#define TX_DATA_PIN 	(BIT4)
#define TX_SCLK_PIN 	(BIT5)
#define TX_SREAD_PIN 	(BIT6)
#define TX_SDATA_PIN 	(BIT7)

// Port 3

// Port 4
#define TX_SLE_PIN 		(BIT0)
#define TX_CE_PIN 		(BIT1)
#define TX_ON_PIN 		(BIT2)
#define RX_ON_PIN 		(BIT3)
#define PA_ON_PIN 		(BIT4)

// Port 5

// Port 6


#define TX_XTAL 		(19200000)
#define RX_XTAL			(19200000)

#define IDLE 0
#define TX 1
#define RX 2

#define RF_MAX 256
#define PREAMBLE_BYTE           0xAA

#define VALID_PREAMBLE_BYTE_1   0x55
#define VALID_PREAMBLE_BYTE_2   0xAA

#define SYNC_WORD1 0xD3                // First byte of sync word
#define SYNC_WORD2 0x91                // Second byte of sync word

#define HEADER_SIZE     4       // 4 bytes header

#define ON  TRUE
#define OFF FALSE

#define adf702x_data       (adf702x_buf + 3)


// Register 0
typedef enum
{
ADF7021N_TX_MODE = 0,
ADF7021N_RX_MODE =1
} paramTxRx;


#define ADF7021N_UART_DISABLED (0)
#define ADF7021N_UART_ENABLED	(1)

typedef enum
{
	ADF7021N_MUXOUT_REGULATOR_READY	 = 0,
	ADF7021N_MUXOUT_FILTER_CAL_COMPLETE = 1,
	ADF7021N_MUXOUT_DIGITAL_LOCK_DETECT = 2,
	ADF7021N_MUXOUT_RSSI_READY = 3,
	ADF7021N_MUXOUT_TX_RX = 4,
	ADF7021N_MUXOUT_LOGIC_ZERO = 5,
	ADF7021N_MUXOUT_TRISTATE = 6,
	ADF7021N_MUXOUT_LOGIC_ONE = 7
} paramMuxout;


// Register 1
#define ADF7021N_XTAL_DOUBLER_DISABLE	(0)
#define ADF7021N_XTAL_DOUBLER_ENABLE	(1)

#define ADF7021N_XOSC_ENABLE_OFF		(0)
#define ADF7021N_XOSC_ENABLE_ON			(1) // for 0.8v p-p clipped sine-wave Oscillator

#define ADF7021N_XTAL_BIAS_20uA			(0)
#define ADF7021N_XTAL_BIAS_25uA			(1)
#define ADF7021N_XTAL_BIAS_30uA			(2)
#define ADF7021N_XTAL_BIAS_35uA			(3)

typedef enum
{
	ADF7021N_CP_CURRENT_0_3mA = 0, // all when Rset = 3.6k Ohm
	ADF7021N_CP_CURRENT_0_9mA = 1,
	ADF7021N_CP_CURRENT_1_5mA = 2,
	ADF7021N_CP_CURRENT_2_1mA = 3
} paramCpCurrent;


#define ADF7021N_VCO_ENABLE_OFF			(0)
#define ADF7021N_VCO_ENABLE_ON			(1)

#define ADF7021N_RF_DIVIDE_BY_2_OFF		(0)
#define ADF7021N_RF_DIVIDE_BY_2_ON		(1)

#define ADF7021N_VCO_INDUCTOR_INTERNAL	(0)
#define ADF7021N_VCO_INDUCTOR_EXTERNAL	(1)

// Register 2
typedef enum
{
	ADF7021N_MODULATION_2FSK = 0,
	ADF7021N_MODULATION_2GFSK = 1,
	ADF7021N_MODULATION_3FSK = 2,
	ADF7021N_MODULATION_4FSK = 3,
	ADF7021N_MODULATION_OVER2FSK = 4,
	ADF7021N_MODULATION_RS2FSK = 5,
	ADF7021N_MODULATION_RS3FSK = 6,
	ADF7021N_MODULATION_RS4FSK =7
} paramModulation;


#define ADF7021N_PA_ENABLE_OFF			(0)
#define ADF7021N_PA_ENABLE_ON			(1)

typedef enum
{
	ADF7021N_PA_RAMP_NO_RAMP = 0,
	ADF7021N_PA_RAMP_256 = 1,
	ADF7021N_PA_RAMP_128 = 2,
	ADF7021N_PA_RAMP_64 = 3,
	ADF7021N_PA_RAMP_32 = 4,
	ADF7021N_PA_RAMP_16 = 5,
	ADF7021N_PA_RAMP_8= 6,
	ADF7021N_PA_RAMP_4 = 7
} paramPaRamp;

typedef enum
{
	ADF7021N_PA_BIAS_5uA = 0,
	ADF7021N_PA_BIAS_7uA = 1,
	ADF7021N_PA_BIAS_9uA = 2,
	ADF7021N_PA_BIAS_11uA =3
} paramPaBias;

#define ADF7021N_TXDATA_INVERT_NORMAL	(0)
#define ADF7021N_TXDATA_INVERT_CLK		(1)
#define ADF7021N_TXDATA_INVERT_DATA		(2)
#define ADF7021N_TXDATA_INVERT_CLK_DATA	(3)

#define ADF7021N_RCOSINE_ALPHA_0_5		(0)
#define ADF7021N_RCOSINE_ALPHA_0_7		(1)

// Register 3
#define ADF7021N_BBOS_CLK_DIVIDE_4		(0)
#define ADF7021N_BBOS_CLK_DIVIDE_8		(1)
#define ADF7021N_BBOS_CLK_DIVIDE_16		(2)
#define ADF7021N_BBOS_CLK_DIVIDE_32		(3)

// Register 4
typedef enum
{
	ADF7021N_2FSK_LINEAR_DEMOD = 0,
	ADF7021N_2FSK_CORREL_DEMOD = 1,
	ADF7021N_3FSK_DEMOD = 2,
	ADF7021N_4FSK_DEMOD = 3
} paramDemodScheme;

typedef enum
{
	ADF7021N_DOT_PRODUCT_CROSS = 0,
	ADF7021N_DOT_PRODUCT_DOT = 1
}paramDotProduct;


typedef enum
{
	ADF7021N_RX_INVERT_NORMAL = 0,
	ADF7021N_RX_INVERT_CLK = 1,
	ADF7021N_RX_INVERT_DATA = 2,
	ADF7021N_RX_INVERT_CLK_DATA = 3
} paramRxInvert;

typedef enum
{
ADF7021N_IF_FILT_BW_9kHz = 0,
ADF7021N_IF_FILT_BW_13_5kHz = 1,
ADF7021N_IF_FILT_BW_18_5kHz = 2
} paramIfFiltBW;



// Register 5
#define ADF7021N_IF_CAL_COARSE_NO_CAL	(0)
#define ADF7021N_IF_CAL_COARSE_DO_CAL	(1)

#define ADF7021N_IR_PHASE_ADJ_DIR_I_CH	(0)
#define ADF7021N_IR_PHASE_ADJ_DIR_Q_CH	(1)

#define ADF7021N_IR_GAIN_ADJ_I_CH		(0)
#define ADF7021N_IR_GAIN_ADJ_Q_CH		(1)

#define ADF7021N_IR_GAIN_ADJ_UP_DN_GAIN	(0)
#define ADF7021N_IR_GAIN_ADJ_UP_DN_ATTEN	(1)

// Register 6
#define ADF7021N_IF_FINE_CAL_DISABLED	(0)
#define ADF7021N_IF_FINE_CAL_ENABLED	(1)
typedef enum
{
	ADF7021N_IR_CAL_SRC_DRV_LEVEL_OFF = 0,
	ADF7021N_IR_CAL_SRC_DRV_LEVEL_LOW = 1,
	ADF7021N_IR_CAL_SRC_DRV_LEVEL_MID = 2,
	ADF7021N_IR_CAL_SRC_DRV_LEVEL_HIGH = 3
} paramIRCalSrcDrvLevel;

#define ADF7021N_IR_CAL_SOURCE_DIVIDE_2_OFF	(0)
#define ADF7021N_IR_CAL_SOURCE_DIVIDE_2_ON	(1)

// Register 7
typedef enum
{
	ADF7021N_ADC_MODE_RSSI = 0,
	ADF7021N_ADC_MODE_BATTERY = 1,
	ADF7021N_ADC_MODE_TEMP = 2,
	ADF7021N_ADC_MODE_EXT_PIN = 3
} paramAdcMode;

typedef enum
{
	ADF7021N_READBACK_MODE_AFC = 0,
	ADF7021N_READBACK_MODE_ADC = 1,
	ADF7021N_READBACK_MODE_FILTER_CAL = 2,
	ADF7021N_READBACK_MODE_SILICON_REV = 3
} paramReadbackMode;

typedef enum
{
	ADF7021N_READBACK_DISABLED = 0,
	ADF7021N_READBACK_ENABLED = 1
} paramReadbackSelect;



// Register 8

// Register 9
typedef enum
{
	ADF7021N_AGC_MODE_AUTO = 0,
	ADF7021N_AGC_MODE_MANUAL = 1,
	ADF7021N_AGC_MODE_FREEZE = 2
} paramAgcMode;

typedef enum
{
	ADF7021N_LNA_GAIN_3= 0,
	ADF7021N_LNA_GAIN_10 = 1,
	ADF7021N_LNA_GAIN_30 = 2
} paramLnaGain;

typedef enum
{
	ADF7021N_FILTER_GAIN_8= 0,
	ADF7021N_FILTER_GAIN_24 = 1,
	ADF7021N_FILTER_GAIN_72 = 2
} paramFilterGain;

#define ADF7021N_FILTER_CURRENT_LOW		(0)
#define ADF7021N_FILTER_CURRENT_HIGH	(1)

#define ADF7021N_LNA_MODE_DEFAULT		(0)
#define ADF7021N_LNA_MODE_REDUCED_GAIN	(1)

#define ADF7021N_MIXER_LINEARITY_DEFAULT	(0)
#define ADF7021N_MIXER_LINEARITY_HIGH		(1)

// Register 10
#define ADF7021N_AFC_EN_OFF				(0)
#define ADF7021N_AFC_EN_ON				(1)

// Register 14
#define ADF7021N_TEST_DAC_EN_OFF		(0)
#define ADF7021N_TEST_DAC_EN_ON			(1)

// Register 15
typedef enum
{
	ADF7021N_RX_TEST_NORMAL = 0,
	ADF7021N_RX_TEST_SCLK_SDATA_I_Q = 1,
	ADF7021N_RX_TEST_REVERSE_I_Q = 2,
	ADF7021N_RX_TEST_I_Q_CLK_DATA = 3,
	ADF7021N_RX_TEST_3FSK_SLICER_DATA = 4,
	ADF7021N_RX_TEST_CORREL_SLICER_DATA = 5,
	ADF7021N_RX_TEST_LINEAR_SLICER_DATA = 6,
	ADF7021N_RX_TEST_SDATA_CDR = 7,
	ADF7021N_RX_TEST_ADD_FILTERING_I_Q = 8,
	ADF7021N_RX_TEST_EN_REG_14_DEMOD_PARAM = 9,
	ADF7021N_RX_TEST_PWR_DN_DDT_ED_IN_T_4_MODE = 10,
	ADF7021N_RX_TEST_ENV_DETECTOR_WDT_DISABLED = 11,
	ADF7021N_RX_TEST_PROHIBIT_CALACTIVE = 13,
	ADF7021N_RX_TEST_FORCE_CALACTIV = 14,
	ADF7021N_RX_TEST_EN_DEMOD_DUR_CAL = 15
} paramRxTestMode;

typedef enum
{
	ADF7021N_TX_TEST_NORMAL = 0,
	ADF7021N_TX_TEST_CARRIER = 1,
	ADF7021N_TX_TEST_HIGH_TONE = 2,
	ADF7021N_TX_TEST_LOW_TONE = 3,
	ADF7021N_TX_TEST_1010 = 4,
	ADF7021N_TX_TEST_PN9 = 5,
	ADF7021N_TX_TEST_SYNC_BYTE_RPT = 6
} paramTxTestMode;

typedef enum
{
	ADF7021N_AG_TEST_BANDGAP_V = 0,
	ADF7021N_AG_TEST_40uA_I_REG4 = 1,
	ADF7021N_AG_TEST_FILT_I_STAGE1 = 2,
	ADF7021N_AG_TEST_FILT_I_STAGE2 = 3,
	ADF7021N_AG_TEST_FILT_I_STAGE1_ = 4,
	ADF7021N_AG_TEST_FILT_Q_STAGE1 = 5,
	ADF7021N_AG_TEST_FILT_Q_STAGE2 = 6,
	ADF7021N_AG_TEST_FILT_Q_STAGE1_ = 7,
	ADF7021N_AG_TEST_ADC_REF_V = 8,
	ADF7021N_AG_TEST_BIAS_I_RSSI_5uA = 9,
	ADF7021N_AG_TEST_FILT_CAL_OSC = 10,
	ADF7021N_AG_TEST_AG_RSSI_I = 11,
	ADF7021N_AG_TEST_OSET_LOOP_POS_VE_FB = 12,
	ADF7021N_AG_TEST_RSSI_RECT_POS = 13,
	ADF7021N_AG_TEST_RSSI_RECT_NEG = 14,
	ADF7021N_AG_TEST_BIAS_I_BB_FILT = 15
} paramAnalogTestMode;


typedef struct {
	struct{
		uint16_t fractional_n;
		uint8_t integer_n;
		uint8_t tx_rx;
		uint8_t uart_mode;
		uint8_t muxout;
	} r0;

	struct{
		uint8_t r_counter;
		uint8_t clkout_divide;
		uint8_t xtal_doubler;
		uint8_t xosc_enable;
		uint8_t xtal_bias;
		uint8_t cp_current;
		uint8_t vco_enable;
		uint8_t rf_divide_by_2;
		uint8_t vco_bias;
		uint8_t vco_adjust;
		uint8_t vco_inductor;
	} r1;

	struct{
		uint8_t modulation_scheme;
		uint8_t pa_enable;
		uint8_t pa_ramp;
		uint8_t pa_bias;
		uint8_t power_amplifier;
		uint16_t tx_frequency_deviation;
		uint8_t txdata_invert;
		uint8_t r_cosine_alpha;
	} r2;

	struct{
		uint8_t bbos_clk_divide;
		uint8_t demod_clk_divide;
		uint8_t cdr_clk_divide;
		uint8_t seq_clk_divide;
		uint8_t agc_clk_divide;
	} r3;

	struct{
		uint8_t demode_scheme;
		uint8_t dot_product;
		uint8_t rx_invert;
		uint16_t discriminator_bw;
		uint16_t post_demod_bw;
		uint8_t if_filter_bw;
	} r4;

	struct{
		uint8_t if_cal_coarse;
		uint16_t if_filter_divider;
		uint8_t if_filer_adjust;
		uint8_t ir_phase_adjust_mag;
		uint8_t ir_phase_adjust_direction;
		uint8_t ir_gain_adjust_gain;
		uint8_t ir_gain_adjust_mag;
		uint8_t ir_gain_adjust_i_q;
		uint8_t ir_gain_adjust_up_dn;
	} r5;

	struct{
		uint8_t if_fine_cal;
		uint8_t if_cal_lower_tone_divide;
		uint8_t if_cal_upper_tone_divide;
		uint8_t if_cal_dwell_time;
		uint8_t ir_cal_source_drive_level;
		uint8_t ir_cal_source_divide_by_2;
	} r6;

	struct{
		uint8_t adc_mode;
		uint8_t readback_mode;
		uint8_t readback_select;
	} r7;

	struct{
		uint8_t agc_low_threshold;
		uint8_t agc_high_threshold;
		uint8_t agc_mode;
		uint8_t lna_gain;
		uint8_t filter_gain;
		uint8_t filter_current;
		uint8_t lna_mode;
		uint8_t lna_bias;
		uint8_t mixer_linearity;
	} r9;

	struct{
		uint8_t afc_en;
		uint16_t afc_scaling_factior;
		uint8_t ki;
		uint8_t kp;
		uint8_t max_afc_range;
	} r10;

	struct{
		uint8_t test_dac_en;
		uint16_t test_dac_offset;
		uint8_t test_dac_gain;
		uint8_t pulse_extension;
		uint8_t ed_leak_factor;
		uint8_t ed_peak_response;
	} r14;

	struct{
		uint8_t rx_test_mode;
		uint8_t tx_test_mode;
		uint8_t sigma_delta_test_mode;
		uint8_t pfd_cp_test_modes;
		uint8_t clk_mux;
		uint8_t pll_test_mode;
		uint8_t analog_test_mode;
		uint8_t force_ld_high;
		uint8_t reg_1_pd;
		uint8_t cal_override;
	} r15;
} adf7021n_config;

void ax25_makePacket(char* dstAddr, char* srcAddr, uint8_t* data, uint8_t dataSize);

void adf7021n_portSetup(void);
void adf7021n_txInit(void);

void adf7021n_txEnable(void);
void adf7021n_txDisable(void);


// Register 0 set / get functions
void adf7021n_setFracN(uint16_t fracN);
uint16_t adf7021n_getFracN(void);
void adf7021n_setIntegerN(uint8_t intN);
void adf7021n_setTxRx(paramTxRx txRx);
uint8_t adf7021n_getTxRx(void);
void adf7021n_setMuxout(paramMuxout muxout);
uint8_t adf7021n_getMuxout(void);

// Register 1 set / get functions
void adf7021n_setRCounter(uint8_t rCounter);
void adf7021n_setChargePumpCurrent (paramCpCurrent cpCurrent);
uint8_t adf7021n_getChargePumpCurrent(void);
void adf7021n_setVcoEnableOff(void);
void adf7021n_setVcoEnableOn(void);
void adf7021n_setVcoBias(uint8_t vcoBias);
uint8_t adf7021n_getVcoBias(void);
void adf7021n_setVcoAdjust(uint8_t vcoAdjust);
uint8_t adf7021n_getVcoAdjust(void);

// Register 2 set / get functions
void adf7021n_setModulationScheme(paramModulation modulationScheme);
void adf7021n_setPowerAmpOn(void);
void adf7021n_setPowerAmpOff(void);
void adf7021n_setPowerAmp(paramPaRamp paRamp, paramPaBias paBias, uint8_t paLevel);
uint8_t adf7021n_getPALevel(void);
void adf7021n_setTxFreqDeviation(uint16_t txFreqDev);

// Register 3 set / get functions
void adf7021n_setDemodDivider(uint8_t demodDiv);
void adf7021n_setCDRDivider(uint8_t cdrDiv);

// Register 4 set / get functions
void adf7021n_setDemodScheme(paramDemodScheme demodScheme);
void adf7021n_setDotProduct(paramDotProduct dotProduct);
void adf7021n_setRxInvert(paramRxInvert rxInvert);
void adf7021n_setDiscriminatorBW(uint16_t discrimBW);
void adf7021n_setPostDemodBW(uint16_t postDemodBW);
void adf7021n_setIFFilterBW(paramIfFiltBW ifFiltBW);

// Register 5 set / get functions
void adf7021n_setIFCalCoarseON(void);
void adf7021n_setIFCalCoarseOFF(void);
void adf7021n_setIFFliterAdj(uint8_t adj);
void adf7021n_setIRPhaseAdjMag(uint8_t mag);
void adf7021n_setIRPhasAdjDir_I(void);
void adf7021n_setIRPhasAdjDir_Q(void);
void adf7021n_setIRGainAdjMag(uint8_t mag);
void adf7021n_setIRGainAdj_I(void);
void adf7021n_setIRGainAdj_Q(void);
void adf7021n_setIRGainAdj_Gain(void);
void adf7021n_setIRGainAdj_Atten(void);

// Register 6 set / get functions
void adf7021n_setIFCalFineON(void);
void adf7021n_setIFCalFineOFF(void);
void adf7021n_setIRCalSrcDrvLevel(paramIRCalSrcDrvLevel level);
void adf7021n_setIRCalSrcDiv2ON(void);
void adf7021n_setIRCalSrcDiv2OFF(void);

// Register 7 set / get functions
void adf7021n_setADCMode(paramAdcMode adcMode);
void adf7021n_setReadbackMode(paramReadbackMode readbackMode);
void adf7021n_setReadbackSelectON(void);
void adf7021n_setReadbackSelectOFF(void);

// Register 9 set / get functions
void adf7021n_setAGCLowThreshold(uint8_t lowThreshold);
uint8_t adf7021n_getAGCLowThreshold(void);
void adf7021n_setAGCHighThreshold(uint8_t highThreshold);
uint8_t adf7021n_getAGCHighThreshold(void);
void adf7021n_setAGCMode(paramAgcMode agcMode);
void adf7021n_setLNAGain(paramLnaGain lnaGain);
void adf7021n_setFilterGain(paramFilterGain filtGain);
void adf7021n_setFilterCurrent_Low(void);
void adf7021n_setFilterCurrent_High(void);
void adf7021n_setLNAMode_Default(void);
void adf7021n_setLNAMode_ReducedGain(void);
void adf7021n_setMixerLinearity_Default(void);
void adf7021n_setMixerLinearity_High(void);


void adf7021n_sendStart(void);
void adf7021n_recvStart(void);

void adf7021n_tx(void);
void adf7021n_rx(void);

void adf7021n_txNotest();
void adf7021n_tx1010test(void);
void adf7021n_txCarriertest(void);
void adf7021n_txHightest(void);
void adf7021n_txLowtest(void);

void adf7021n_enable_data_interrupt(void);
unsigned char adf7021n_getMode(void);


#endif /* ADF7021N_H_ */
