#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include "string.h"
#include "../header.h"
#include "boot.h"
#include "spi.h"

#define DEV_NAME "/dev/pcie-lat/00:05.0" 
/***************** main gyro control program below ***************/
Xint32 phi_drvout0 = 0, f_adj0 = 0, drv_gain0_fix = 47416438, drvout_hp = 0, drvs_hp = 0, sens_hp = 0;
Xint32 Wdrvout1 = 0, Wdrvout2 = 0, delayph_drv1 = 0, delayamp_drv1 = 0, ref_drvi_hp = 0, ref_drvq_hp = 0;
Xint32 Wdrvs1 = 0, Wdrvs2 = 0, Wsens1 = 0, Wsens2 = 0, delayf_drv1 = 0;
Xint32 adrslt1 = 0, adrslt2 = 0, adrslt3 = 0, adrslt4 = 0, phi_adj = 0;
Xuint8 a1 = 0,b1 = 0,c1 = 0,d1 = 0,a2 = 0,b2 = 0,c2 = 0,d2 = 0,a3 = 0,b3 = 0,c3 = 0,d3 = 0,a4 = 0,b4 = 0,c4 = 0,d4 = 0;
Xint32 drvsi_lpa1 = 0, drvsi_lpa2 = 0,drvsi_lpb1 = 0, drvsi_lpb2 = 0;
Xint32 drvsq_lpa1 = 0, drvsq_lpa2 = 0,drvsq_lpb1 = 0, drvsq_lpb2 = 0;
Xint32 sensi_lpa1 = 0, sensi_lpa2 = 0,sensi_lpb1 = 0, sensi_lpb2 = 0;
Xint32 sensq_lpa1 = 0, sensq_lpa2 = 0,sensq_lpb1 = 0, sensq_lpb2 = 0;
Xint32 drvouti_lpa1 = 0, drvouti_lpa2 = 0,drvouti_lpb1 = 0, drvouti_lpb2 = 0;
Xint32 drvoutq_lpa1 = 0, drvoutq_lpa2 = 0,drvoutq_lpb1 = 0, drvoutq_lpb2 = 0;

Xint32 ref_drvi_fix = 0,ref_drvq_fix = 1,ref_drvi_fix1 = 0, ref_drvq_fix1 = 1,ref_drvi_fix2 = 0, ref_drvq_fix2 = 1,daout = 0,drvout_fix = 0,temp1 = 0,temp2 = 0;


Xint32 y_drvout = 0, err_drvout = 0, miu_drvout_fix = 128849019, drvoutq = 0, drvouti = 0;
Xint32 delayph_drv2 = 0, phi_drvout = 0, kp_drvout_fix = 429496730, ki_drvout_fix = 420906795;

Xint32 drvs_fix = 0, y_drvs = 0, err_drvs = 0, miu_drvs_fix = 128849019, drvsq = 0, drvsi = 0;
Xint32 delayf_drv2 = 0, f_adj = 0, kp_drvs_fix = 42949673*4, ki_drvs_fix = 42863774*4, drv_freq = 0;


Xint32 drvamp_goal_fix = 94832874*0.1, delayamp_drv2 = 0, drv_gain_fix =47416438, kpamp_drvs_fix = 42949673*4, kiamp_drvs_fix = 42734925*4;
Xint32 sens_fix = 0, y_sens = 0, err_sens = 0, miu_sens_fix = 128849019, sensq = 0, sensi = 0;
Xint32 bals_fix = 0;

Xint32 darslt;
Xint32 deltaphi;


int process_time = 0;
void ad_int_handler()
{  
	TIMER0_CURR_CNT = 0xffff;
	TIMER_CTRL = 2<<2|1<<6;


// send out DA value
 DA_TX_DATA = (daout<<16); 

 ad_data0 = AD_RX_DATA;
 ad_data1 = AD_RX_DATA;
 ad_data2 = AD_RX_DATA;
 ad_data3 = AD_RX_DATA;
 
 
// printk("%08x %08x %08x %08x",ad_data0,ad_data1,ad_data2,ad_data3);

//Read drvout
 drvout_fix = ad_data3;

//Read bals
 bals_fix = ad_data2;

//Read drvs
 drvs_fix = -ad_data1;

//Read sens
 sens_fix = ad_data0;

 drvout_fix <<= 9;
 drvs_fix <<= 9;
 sens_fix <<= 11;
 
 SINCOS_DIN = phi_adj;		 
 ref_drvq_fix = SINCOS_DSINE;		 
 ref_drvi_fix = SINCOS_DCOSE;
 
 mulh(temp1,960767921,drv_freq);
 deltaphi = 2 * temp1;
 SINCOS_DIN = (phi_adj - deltaphi);		 				// fix phase shift for primary: 
 ref_drvq_fix1 = SINCOS_DSINE;		 
 ref_drvi_fix1 = SINCOS_DCOSE;
 
 mulh(temp2,960767921,drv_freq);
 deltaphi = 3 * temp2;
 SINCOS_DIN = (phi_adj - temp2 + 68130029*0);		 // fix phase shift for secondary: 9deg 107374182 131235111
 ref_drvq_fix2 = SINCOS_DSINE;		 
 ref_drvi_fix2 = SINCOS_DCOSE;

//HP filter
 drvout_fix = hp_filter1(drvout_fix);


 
//Demodulate drvout
 mulh(temp1,Wdrvout1,ref_drvi_fix);
 mulh(temp2,Wdrvout2,ref_drvq_fix);
 y_drvout = 2 * (temp1 + temp2);
 err_drvout = drvout_fix - y_drvout;
 mulh(temp1,miu_drvout_fix,err_drvout);
 mulh(temp2,temp1,ref_drvi_fix);
 Wdrvout1 = 2 * temp2 + Wdrvout1;
 mulh(temp2,temp1,ref_drvq_fix);
 Wdrvout2 = 2 * temp2 + Wdrvout2;
// drvouti = lp_filter5(Wdrvout1);
// drvoutq = lp_filter6(Wdrvout2);
 drvouti = Wdrvout1;
 drvoutq = Wdrvout2;




//DLL DA
// mulh(temp1,drvoutq,1056689149);
 temp2 = drvoutq << 4;
 mulh(temp1,temp2,1367130574);
 delayph_drv2 = -temp1;
 mulh(temp1,kp_drvout_fix,delayph_drv2);
 mulh(temp2,ki_drvout_fix,delayph_drv1);
 phi_drvout = phi_drvout0 + temp1 - temp2;
 delayph_drv1 = delayph_drv2;
 phi_drvout0 = phi_drvout;


//HP filter
 drvs_fix = hp_filter2(drvs_fix);
 

//Demodulate drvs
 mulh(temp1,Wdrvs1,ref_drvi_fix1);
 mulh(temp2,Wdrvs2,ref_drvq_fix1);
 y_drvs = 2 * (temp1 + temp2);
 err_drvs = drvs_fix - y_drvs;
 mulh(temp1,miu_drvs_fix,err_drvs);
 mulh(temp2,temp1,ref_drvi_fix1);
 Wdrvs1 = 2 * temp2 + Wdrvs1;
 mulh(temp2,temp1,ref_drvq_fix1);
 Wdrvs2 = 2 * temp2 + Wdrvs2;
// drvsi = lp_filter1(Wdrvs1);
// drvsq = lp_filter2(Wdrvs2);
 drvsi = Wdrvs1;
 drvsq = Wdrvs2;



//DLL GYRO
 mulh(temp1,drvsi,1527887453);
 delayf_drv2 = - 16 * temp1;
 mulh(temp1,kp_drvs_fix,delayf_drv2);
 mulh(temp2,ki_drvs_fix,delayf_drv1);
 f_adj = f_adj0 + temp1 - temp2;
 delayf_drv1 = delayf_drv2;
 f_adj0 = f_adj;
 drv_freq = 317000000 - f_adj;
 // drv_freq = 300000000;
// drv_freq = 545259520 - f_adj;

 mulh(temp1,drv_freq,1921535841);
// mulh(temp1,284100000,1921535841);
 phi_adj = phi_adj + 2 * temp1;

//LOCK AMP OF GYRO
 delayamp_drv2 = drvamp_goal_fix - abs(drvsq);
 mulh(temp1,kpamp_drvs_fix,delayamp_drv2);
 mulh(temp2,kiamp_drvs_fix,delayamp_drv1);
 drv_gain_fix = drv_gain0_fix + temp1 - temp2;
 delayamp_drv1 = delayamp_drv2;
 drv_gain0_fix = drv_gain_fix ;
 
 // drv_gain0_fix = 47416438;
 // drv_gain_fix = 47416438;


//HP filter
 sens_fix = hp_filter3(sens_fix);


	 
//Demodulate sens
 mulh(temp1,Wsens1,ref_drvi_fix2);
 mulh(temp2,Wsens2,ref_drvq_fix2);
 y_sens = 2 * (temp1 + temp2);
 err_sens = sens_fix - y_sens;
 mulh(temp1,miu_sens_fix,err_sens);
 mulh(temp2,temp1,ref_drvi_fix2);
 Wsens1 = 2 * temp2 + Wsens1;
 mulh(temp2,temp1,ref_drvq_fix2);
 Wsens2 = 2 * temp2 + Wsens2;
 sensi = lp_filter5(Wsens1);
 sensq = lp_filter6(Wsens2); 
// sensi =  Wsens1;
// sensq =  Wsens2;





 SINCOS_DIN = (phi_adj+phi_drvout0);
 temp1 = SINCOS_DSINE;		 
 temp1 = SINCOS_DCOSE;
 
 mulh(temp2,drv_gain0_fix,temp1);	// adjust the gain loop
 mulh(darslt,305835 * 4,temp2);

 daout = darslt + 32768;
 
 ii++;
 if(ii>=48000)
 {
   iflag = 1;
   ii = 0;
 }

	process_time = 65536-TIMER0_CURR_CNT;

}

void adda_init(void)
{
	AD_TIMING0 = 124|16<<16;
	AD_TIMING1 = 120|50<<16;
	DA_TIMING0 = 387|413<<16;
	DA_TIMING1 = 433|1<<12;
	INT_TIMING = 499;
	ADDA_CTRL = 0<<2|3;	

	INTC_EN |= (1<<28);
	INTC_MASK &= ~(1<<28);
}


int main(int argc, char *argv[])
{
	int i;
	u8 *p8;

	int fd=0;
	char buff[30];
	fd = open (DEV_NAME, O_RDWR);
	if(fd<0){
		perror("Open "DEV_NAME" Failed!\n");
		exit(1);
	}


	pmc_set_clk(3<<11|18<<2); //48M
	//pmc_set_clk(1<<11|23<<2);//100M

	PMC_CTRL = 0xffffffff;

	delay_1ms(48);
	
	init_uart(48000000,115200);
	
	//printk("hi jst110 app 48M\r\n");
	
	//init_uart(100000000,115200);
	printk("hi jst110 app 48M\r\n");
#if 0
	p8 = (u8*)0x8000;

	i = xmodem(p8);
	printk("xmodem recieve %08x bytes\r\n",i*128);
	write_spi_boot(p8,i*128);
	printk("write spi boot ok\r\n");

	
#endif

	gpio_set_adir(0,1);
	gpio_set_aout(0,0);
	delay_1ms(48);
	gpio_set_aout(1,1);
	pin_set_adda();
	adda_init();
	arm_enable_int();

	int DA[4];
	while(1)
	{
		
		if(iflag)
		{
			iflag = 0;
			mulh(adrslt1,drv_freq,42949673);
			mulh(adrslt2,drv_gain_fix,160000000);
			mulh(adrslt3,sensi,160000000);
			mulh(adrslt4,sensq,160000000);


			buff[0] = adrslt1;
			buff[2] = adrslt1;
			buff[3] = adrslt1;
			buff[4] = adrslt1;

			DA[0] = write(fd, &buff, 4);
			DA[1] = write(fd, &buff, 4);
			DA[2] = write(fd, &buff, 4);
			DA[3] = write(fd, &buff, 4);

			printk("%d %d %d %d %d\r\n", process_time, adrslt1, adrslt2,adrslt3,adrslt4);
		}
	
		
	/*	if(nj>=400){
			for(i=0;i<=63;i++){
				printk("i=%d,ad_data0=%08x,ad_data1=%08x,ad_data2=%08x,ad_data3=%08x\r\n",i,ad_data0[i],ad_data1[i],ad_data2[i],ad_data3[i]);
				//printk("i=%d,ad3=%08x i=%d,ad3=%08x i=%d,ad3=%08x i=%d,ad3=%08x\r\n",4*i,ad_data3[4*i],4*i+1,ad_data3[4*i+1],4*i+2,ad_data3[4*i+2],4*i+3,ad_data3[4*i+3]);
			}

			nj=0;
		} */
	
	
	}


}






{
	


	printf("read orig data from device\n");
	i = read(fd, &buff, 64);

	if (!i) {
		perror("read "DEV_NAME" Failed!\n");
		exit(1);
	}

	for (i=0; i<64; i++ ) {
		printf("0x%02x ", buff[i]);
	}



	printf("\n");
	printf("write data into device\n");

	for (i=0; i<64; i++ ) {
		buff[i] = 63 - i;
	}
	
	i = write(fd, &buff, 64);
	if (!i) {
		perror("write "DEV_NAME" Failed!\n");
		exit(1); 
	}



	printf("read new data from device\n"); 
	i = read(fd, &buff, 64);

	if (!i) {
		perror("read "DEV_NAME" Failed!\n");
		exit(1); 
	}

	for (i=0; i<64; i++ ) { 
		printf("0x%02x ", buff[i]);
	}
	printf("\n");
	
	close(fd);
	return 0;
}
