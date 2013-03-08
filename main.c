#include <msp430.h>
#include "adf7021n.h"

int step = 10;

uint8_t sendData[18] = {'H','e','l','l','o',' ','T','h','i','s',' ','i','s',' ','O','S','S','I'};
char *dstAddr ="CQ    ";
char *srcAddr ="OSSI-1";

void configure_clock() {
	// DCO 8MHZ
	BCSCTL1 |= RSEL0 + RSEL1 + RSEL2;
	DCOCTL = DCO0 + DCO1 + DCO2;
	BCSCTL2 |= SELM_0 + DIVM_0;

//	volatile uint8_t i;
//	// XT2 (7.3782MHZ)
//	  BCSCTL1 &= ~XT2OFF;                       // XT2= HF XTAL
//
//	  do
//	  {
//	  IFG1 &= ~OFIFG;                           // Clear OSCFault flag
//	  for (i = 0xFF; i > 0; i--);               // Time for flag to set
//	  }
//	  while ((IFG1 & OFIFG));                   // OSCFault flag still set?
//
//	  BCSCTL2 |= SELM_2;                        // MCLK= XT2 (safe)

}

int main(void) {
	volatile unsigned int i;
	WDTCTL = WDTPW + WDTHOLD;		// Stop watchdog timer
	configure_clock();
	__delay_cycles(1000000);
	__delay_cycles(1000000);

	// Ready to Send
	//adf7021n_tx1010test();	// Ready to Receive
	// adf7021n_recvStart();
	//adf7021n_sendStart();
//	adf7021n_enable_data_interrupt();

	adf7021n_portSetup();



	// TX_XTAL = 19.2 MHz
	// fOUT = 437.525 MHz
	// fPFD = 3.84 MHz -> R = 5
	// fDev = 1000Hz
	// Data Rate = 1200bps


	// TX for RF Switch
//	P4DIR |= 0x04;
//	P4OUT |= 0x04;





//
//	// for Data Rate = 1200 bps
//	adf7021n_setDemodDivider(11);
//	adf7021n_setCDRDivider(45);




	// Power for PA ON
//	P4DIR |= 0x10;
//	P4OUT |= 0x10;
//

//	adf7021n_txCarriertest();










	while(1) {


//		volatile uint8_t i,j;
//
//		for (i = 0; i < 16; i++)
//		{
//
//			for (j= 0; j< 4 ; j++)
//			{
//				adf7021n_setVcoBias(i);
//						adf7021n_setVcoAdjust(j);
//					//	// 5dBm out
//					//	 adf7021n_setPowerAmp(ADF7021N_PA_RAMP_NO_RAMP, ADF7021N_PA_BIAS_9uA, 18);
//
//						// Best Power and Eff!!
//						adf7021n_setPowerAmp(ADF7021N_PA_RAMP_NO_RAMP, ADF7021N_PA_BIAS_9uA, 30);
//					//	adf7021n_setPowerAmp(ADF7021N_PA_RAMP_NO_RAMP, ADF7021N_PA_BIAS_9uA, 18);
//
//
//
//
//
//						P4DIR |= 0x10;
//						P4OUT |= 0x10;
//
//						adf7021n_setPowerAmpOn();
//
//
//					//	adf7021n_txEnable();
//					//	adf7021n_tx1010test();
//
//					//	_EINT();
//						adf7021n_txEnable();
//
//
//
//						adf7021n_txCarriertest();
//						__delay_cycles(1000000);
//							__delay_cycles(1000000);
//			}
//
//		}


//

		adf7021n_txInit();



		adf7021n_setRCounter(5);
		adf7021n_setIntegerN(227);
		adf7021n_setFracN(28772);
	// fDev = 1200 Hz at 300 bps
		adf7021n_setTxFreqDeviation(41);
		adf7021n_setDemodDivider(8);
		adf7021n_setCDRDivider(250);


	//  Best setting so far
		adf7021n_setVcoBias(7);
		adf7021n_setVcoAdjust(3);


		// Best Power and Eff!!
		// 5dBm out for current setting due to PA impedance mismatch
		adf7021n_setPowerAmp(ADF7021N_PA_RAMP_NO_RAMP, ADF7021N_PA_BIAS_9uA, 28); //Final tuning value!!!!

		adf7021n_setPowerAmpOn();
		adf7021n_txEnable();

		adf7021n_txCarriertest();
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);

		adf7021n_txHightest();
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);

		adf7021n_txLowtest();
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);
		__delay_cycles(1000000);

		_EINT();

	 	ax25_makePacket(dstAddr, srcAddr, sendData, sizeof(sendData));
	 	P2IFG &= ~BIT3; // P2.3 IFG cleared just in case
	 	P2IE |= BIT3; // interrupt enable
		adf7021n_sendStart();
		for(i=0;i <80;i++)
	    __delay_cycles(1000000);

		P2IFG &= ~BIT3; // P2.3 IFG cleared just in case
		P2IE &= ~BIT3; // interrupt disable

     }
}
