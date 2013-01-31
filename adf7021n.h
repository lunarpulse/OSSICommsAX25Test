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

// Register 0
#define ADF7021N_TX_MODE	(0)
#define ADF7021N_RX_MODE	(1)
#define ADF7021N_UART_DISABLED (0)
#define ADF7021N_UART_ENABLED	(1)
#define ADF7021N_MUXOUT_REGULATOR_READY	(0)
#define ADF7021N_MUXOUT_FILTER_CAL_COMPLETE	(1)
#define ADF7021N_MUXOUT_DIGITAL_LOCK_DETECT	(2)
#define ADF7021N_MUXOUT_RSSI_READY	(3)
#define ADF7021N_MUXOUT_TX_RX	(4)
#define ADF7021N_MUXOUT_LOGIC_ZERO	(5)
#define ADF7021N_MUXOUT_TRISTATE	(6)
#define ADF7021N_MUXOUT_LOGIC_ONE	(7)

// Register 1
#define ADF7021N_XTAL_DOUBLER_DISABLE	(0)
#define ADF7021N_XTAL_DOUBLER_ENABLE	(1)
#define ADF7021N_XOSC_ENABLE_OFF		(0)
#define ADF7021N_XOSC_ENABLE_ON			(1) // for 0.8v p-p clipped sine-wave Oscillator
#define ADF7021N_XTAL_BIAS_20uA			(0)
#define ADF7021N_XTAL_BIAS_25uA			(1)
#define ADF7021N_XTAL_BIAS_30uA			(2)
#define ADF7021N_XTAL_BIAS_35uA			(3)
#define ADF7021N_CP_CURRENT_0_3mA		(0) // all when Rset = 3.6k Ohm
#define ADF7021N_CP_CURRENT_0_9mA		(1)
#define ADF7021N_CP_CURRENT_1_5mA		(2)
#define ADF7021N_CP_CURRENT_2_1mA		(3)
#define ADF7021N_VCO_ENABLE_OFF			(0)
#define ADF7021N_VCO_ENABLE_ON			(1)
#define ADF7021N_RF_DIVIDE_BY_2_OFF		(0)
#define ADF7021N_RF_DIVIDE_BY_2_ON		(1)
#define ADF7021N_VCO_INDUCTOR_INTERNAL	(0)
#define ADF7021N_VCO_INDUCTOR_EXTERNAL	(1)

// Register 2
#define ADF7021N_MODULATION_2FSK		(0)
#define ADF7021N_MODULATION_2GFSK		(1)
#define ADF7021N_MODULATION_3FSK		(2)
#define ADF7021N_MODULATION_4FSK		(3)
#define ADF7021N_MODULATION_OVER2FSK	(4)
#define ADF7021N_MODULATION_RS2FSK		(5)
#define ADF7021N_MODULATION_RS3FSK		(6)
#define ADF7021N_MODULATION_RS4FSK		(7)
#define ADF7021N_PA_ENABLE_OFF			(0)
#define ADF7021N_PA_ENABLE_ON			(1)
#define ADF7021N_PA_RAMP_NO_RAMP		(0)
#define ADF7021N_PA_RAMP_256			(1)
#define ADF7021N_PA_RAMP_128			(2)
#define ADF7021N_PA_RAMP_64				(3)
#define ADF7021N_PA_RAMP_32				(4)
#define ADF7021N_PA_RAMP_16				(5)
#define ADF7021N_PA_RAMP_8				(6)
#define ADF7021N_PA_RAMP_4				(7)
#define ADF7021N_PA_BIAS_5uA			(0)
#define ADF7021N_PA_BIAS_7uA			(1)
#define ADF7021N_PA_BIAS_9uA			(2)
#define ADF7021N_PA_BIAS_11uA			(3)
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


void adf7021n_init(void);
void adf7021n_sendStart(void);
void adf7021n_recvStart(void);

void adf7021n_tx(void);
void adf7021n_rx(void);

void adf7021n_tx1010test(void);
void adf7021n_txCarriertest(void);
void adf7021n_txHightest(void);
void adf7021n_txLowtest(void);
void adf7021n_enable_data_interrupt(void);
unsigned char adf7021n_get_mode(void);

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


#endif /* ADF7021N_H_ */
