#include <msp430.h>
#include "adf7021n.h"

int step = 10;

uint8_t sendData[18] = {'H','e','l','l','o',' ','T','h','i','s',' ','i','s',' ','O','S','S','I'};
char *dstAddr ="CQ    ";
char *srcAddr ="OSSI-1";

void configure_clock() {
	// DCO 8MHZ
//	BCSCTL1 |= RSEL0 + RSEL1 + RSEL2;
//	DCOCTL = DCO0 + DCO1 + DCO2;
//	BCSCTL2 |= SELM_0 + DIVM_0;

	volatile uint8_t i;
	// XT2 (7.3782MHZ)
	  BCSCTL1 &= ~XT2OFF;                       // XT2= HF XTAL

	  do
	  {
	  IFG1 &= ~OFIFG;                           // Clear OSCFault flag
	  for (i = 0xFF; i > 0; i--);               // Time for flag to set
	  }
	  while ((IFG1 & OFIFG));                   // OSCFault flag still set?

	  BCSCTL2 |= SELM_2;                        // MCLK= XT2 (safe)

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
	adf7021n_txInit();

	adf7021n_setTxVcoBias(4);
	adf7021n_setTxVcoAdjust(2);
	adf7021n_setTxPowerAmp(ADF7021N_PA_RAMP_NO_RAMP, ADF7021N_PA_BIAS_5uA, 63);
	adf7021n_setTxPowerAmpOn();


	_EINT();
	while(1) {
		adf7021n_txEnable();

		//adf7021n_tx1010test();
//		adf7021n_sendStart();
		adf7021n_txCarriertest();
		for(i=0;i <20;i++)
	    __delay_cycles(1000000);
		adf7021n_txHightest();
		for(i=0;i <20;i++)
	    __delay_cycles(1000000);
		adf7021n_txLowtest();
		for(i=0;i <20;i++)
	    __delay_cycles(1000000);


	 	ax25_makePacket(dstAddr, srcAddr, sendData, sizeof(sendData));
	 	P2IFG &= ~BIT3; // P2.3 IFG cleared just in case
	 	P2IE |= BIT3; // interrupt enable
		adf7021n_sendStart();
		for(i=0;i <20;i++)
	    __delay_cycles(1000000);

		P2IFG &= ~BIT3; // P2.3 IFG cleared just in case
		P2IE &= ~BIT3; // interrupt enable

     }
}
