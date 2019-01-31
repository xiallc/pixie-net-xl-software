/*----------------------------------------------------------------------
 * Copyright (c) 2019 XIA LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, 
 * with or without modification, are permitted provided 
 * that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above 
 *     copyright notice, this list of conditions and the 
 *     following disclaimer.
 *   * Redistributions in binary form must reproduce the 
 *     above copyright notice, this list of conditions and the 
 *     following disclaimer in the documentation and/or other 
 *     materials provided with the distribution.
 *   * Neither the name of XIA LLC
 *     nor the names of its contributors may be used to endorse 
 *     or promote products derived from this software without 
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE.
 *----------------------------------------------------------------------*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>

#include "PixieNetDefs.h"
#include "PixieNetCommon.h"


void I2Cstart(volatile unsigned int *mapped)  {
   unsigned int mval;
   // I2C start
   mval = 7;   // SDA = 1; SCL = 1; CTRL = 1 
   mapped[AI2CREG] = mval;
   usleep(I2CWAIT);
   mval = 6;   // SDA = 0; SCL = 1; CTRL = 1 
   mapped[AI2CREG] = mval;
   usleep(I2CWAIT);
}

 
void I2Cstop(volatile unsigned int *mapped)  {
   unsigned int mval;
   // I2C stop
   mval = 4;   // SDA = 0; SCL = 0; CTRL = 1 
   mapped[AI2CREG] = mval;
   usleep(I2CWAIT);

   mval = 6;   // SDA = 0; SCL = 1; CTRL = 1 
   mapped[AI2CREG] = mval;
   usleep(I2CWAIT);
   mval = 7;   // SDA = 1; SCL = 1; CTRL = 1 
   mapped[AI2CREG] = mval;
   usleep(I2CWAIT);
}

void I2Cslaveack(volatile unsigned int *mapped) {
    unsigned int mval;
    // I2C acknowledge
    mval = 0x0000;   // clear SCL and CTRL to give slave control of SDA     
    mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
    mval = 2;   // set SCL     
    mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
    mval = 0x0000;   // clear SCL and CTRL to give slave control of SDA   
         // for PLL
         mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
    // now can read SDA bit for ACK
}

void I2Cmasterack(volatile unsigned int *mapped) {
    unsigned int mval;
    // I2C acknowledge
    mval = 0x0004;   // clear SCL and SDA but not CTRL to keep control of SDA     
    mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
    mval = 6;   // set SCL     
    mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
}

void I2Cmasternoack(volatile unsigned int *mapped) {
    unsigned int mval;
    // I2C acknowledge
    mval = 0x0004;   // clear SCL and SDA but not CTRL to keep control of SDA     
    mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
    mval = 3;   // set SCL  and SDA   
    mapped[AI2CREG] = mval; 
    usleep(I2CWAIT);
}



void I2Cbytesend(volatile unsigned int *mapped, unsigned int *data) {
    unsigned int mval, k;
 // I2C byte send
   // SDA is captured during the low to high transition of SCL
   mval = 4;   // SDA = 0; SCL = 0; CTRL = 1 
   for( k=0; k<8; k++ )
   {
   //  printf("Sending a bit\n");
      mval = mval & 0x0005;   // clear SCL      
      mapped[AI2CREG] = mval; 
      usleep(I2CWAIT);
      if(data[7-k])
         mval = 5;            // SDA = 1; SCL = 0; CTRL = 1 
      else 
         mval = 4;            // SDA = 0; SCL = 0; CTRL = 1 
      mapped[AI2CREG] = mval; 
      usleep(I2CWAIT);
      mval = mval | 0x0002;   // set SCL      
      mapped[AI2CREG] = mval; 
      usleep(I2CWAIT);
   }
   // for PLL
               mval = mval & 0x0005;   // clear SCL    
               mapped[AI2CREG] = mval; 
}

void I2Cbytereceive(volatile unsigned int *mapped, unsigned int *data) {
 // I2C byte send
   unsigned int mval, k;
   // SDA is captured during the low to high transition of SCL
   mval = 0;   // SDA = 0; SCL = 0; CTRL = 0 
   for( k=0; k<8; k++ )
   {
      mval = 0;   // SDA = 0; SCL = 0; CTRL = 0       
      mapped[AI2CREG] = mval; 
      usleep(I2CWAIT);
      mval = 2;   // set SCL      
      mapped[AI2CREG] = mval; 
      usleep(I2CWAIT);
   //   mapped[AOUTBLOCK] = OB_EVREG;          
      mval = mapped[AMZ_CSROUTL];
   //   printf("CSRout %x I2Cwait %d \n",mval,I2CWAIT);
      if(mval & 0x4)          // test for SDA out bit
         data[7-k] = 1;            
      else 
         data[7-k] = 0;            
   //   mapped[AI2CREG] = mval;   not for PLL
      usleep(I2CWAIT);
   }
}




 unsigned int setbit( unsigned int par, unsigned int bitc, unsigned int bitf)
 // returns 2^bitf if bit bitc of par is 1 
 { 
   unsigned int ret;
        ret = par & (1 << bitc);     // bitwise and or parameter with ccsra bit
        ret = ret >> bitc;                 // shift down to bit 0 
        ret = ret << bitf;                 // shift up to fippi bit
        return (ret);
  }


int hwinfo( volatile unsigned int *mapped )
// returns 32bit hwrev_sn, or 0 on error
{
   unsigned int  mval, i2cdata[8];
   unsigned int revsn, saveaux;
   unsigned int ctrl[8];
   int k;

  // ---------------- read EEPROM ---------------------------
    mapped[AOUTBLOCK] = CS_MZ;	  // read/write from/to MZ IO block
  saveaux = mapped[AAUXCTRL];	
  saveaux = saveaux & 0xFF8F;    // clear the I2C select bits
  mval = saveaux | I2C_SELMAIN;    // set bit 4-6 to select MZ I2C pins
  mapped[AAUXCTRL] = mval;

//   mapped[AOUTBLOCK] = CS_MZ;	  // read/write from/to MZ IO block
//  mval = mapped[AAUXCTRL];	
//  mval = mval | 0x0010;    // set bit 4 to select MZ I2C pins
//  mapped[AAUXCTRL] = mval;

 /*  mval = mapped[AMZ_BRDINFO];           TODO: ensure  AMZ_BRDINFO has the right address
   ctrl[7] = (mval & 0x800000) >> 23 ;    
   ctrl[6] = (mval & 0x400000) >> 22 ;    
   ctrl[5] = (mval & 0x200000) >> 21 ;    
   ctrl[4] = (mval & 0x100000) >> 20 ; 
   ctrl[3] = (mval & 0x080000) >> 19 ;    
   ctrl[2] = (mval & 0x040000) >> 18 ;   
   ctrl[1] = (mval & 0x020000) >> 17 ;    
   ctrl[0] = (mval & 0x010000) >> 16 ;      */

  ctrl[7] = 1;      // PN XL PROM  (TMP116)
  ctrl[6] = 0;
  ctrl[5] = 0;
  ctrl[4] = 1;  
  ctrl[3] = 0;
  ctrl[2] = 0;
  ctrl[1] = 0;
  ctrl[0] = 0;    


    // ------------- read serial number -------------------- 

     // 2 bytes: ctrl, addr  write
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*         // write starting addr to read from
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
    mval = 0x07;   // addr 7 = serial number
   i2cdata[7] = (mval & 0x0080) >> 7 ;    
   i2cdata[6] = (mval & 0x0040) >> 6 ;    
   i2cdata[5] = (mval & 0x0020) >> 5 ;    
   i2cdata[4] = (mval & 0x0010) >> 4 ;
   i2cdata[3] = (mval & 0x0008) >> 3 ;    
   i2cdata[2] = (mval & 0x0004) >> 2 ;   
   i2cdata[1] = (mval & 0x0002) >> 1 ;    
   i2cdata[0] = (mval & 0x0001)      ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);
     usleep(300);

   // read data bytes 
   mval = 0;
   ctrl[0] = 1;   // R/W*         // now read 
   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);      // device address
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+8));
   I2Cmasterack(mapped);

//   usleep(100);
//   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   //I2Cmasterack(mapped);
   I2Cmasternoack(mapped);
   I2Cstop(mapped);

//   printf("I2C read serial number %d\n",mval);

   mapped[ABVAL] = mval;
   revsn = (mval & 0xFFFF);
   //printf("Revision %04X, Serial Number %d \n",(revsn>>16), revsn&0xFFFF);

/*
   //printf("I2C read Revision 0x%04X\n",mval);
   if ( (mval == PN_BOARD_VERSION_12_250_A)     ||
        (mval == PN_BOARD_VERSION_12_250_B)     ||
        (mval == PN_BOARD_VERSION_12_250_B_PTP) ||
        (mval == 0                        )   )
        //printf("HW Revision 0x%04X\n",mval);
        revsn = mval << 16;
   else
   {
       printf("Unsupported HW Revision 0x%04X\n",mval);
       return(0);
   }

  */

   mapped[AAUXCTRL] = saveaux;

   return(revsn);

}


float board_temperature( volatile unsigned int *mapped, unsigned int I2Csel )
{
   unsigned int  mval, i2cdata[8], saveaux;
   unsigned int ctrl[8];
   int k;

  // ---------------- read EEPROM ---------------------------
  mapped[AOUTBLOCK] = CS_MZ;	  // read/write from/to MZ IO block
  saveaux = mapped[AAUXCTRL];	
  saveaux = saveaux & 0xFF8F;    // clear the I2C select bits
  mval = saveaux | I2Csel;    // set bit 4-6 to select MZ I2C pins
  mapped[AAUXCTRL] = mval;

 /*  mval = mapped[AMZ_BRDINFO];           TODO: ensure  AMZ_BRDINFO has the right address
   ctrl[7] = (mval & 0x800000) >> 23 ;    
   ctrl[6] = (mval & 0x400000) >> 22 ;    
   ctrl[5] = (mval & 0x200000) >> 21 ;    
   ctrl[4] = (mval & 0x100000) >> 20 ; 
   ctrl[3] = (mval & 0x080000) >> 19 ;    
   ctrl[2] = (mval & 0x040000) >> 18 ;   
   ctrl[1] = (mval & 0x020000) >> 17 ;    
   ctrl[0] = (mval & 0x010000) >> 16 ;      */

  ctrl[7] = 1;      // PN XL PROM  (TMP116)
  ctrl[6] = 0;
  ctrl[5] = 0;
  ctrl[4] = 1;  
  ctrl[3] = 0;
  ctrl[2] = 0;
  ctrl[1] = 0;
  ctrl[0] = 0;    
 
    // ------------- read serial number -------------------- 

     // 2 bytes: ctrl, addr  write
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*         // write starting addr to read from
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
    mval = 0x00;   // addr 7 = serial number
   i2cdata[7] = (mval & 0x0080) >> 7 ;    
   i2cdata[6] = (mval & 0x0040) >> 6 ;    
   i2cdata[5] = (mval & 0x0020) >> 5 ;    
   i2cdata[4] = (mval & 0x0010) >> 4 ;
   i2cdata[3] = (mval & 0x0008) >> 3 ;    
   i2cdata[2] = (mval & 0x0004) >> 2 ;   
   i2cdata[1] = (mval & 0x0002) >> 1 ;    
   i2cdata[0] = (mval & 0x0001)      ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);
     usleep(300);

   // read data bytes 
   mval = 0;
   ctrl[0] = 1;   // R/W*         // now read 
   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);      // device address
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+8));
   I2Cmasterack(mapped);

//   usleep(100);
//   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   //I2Cmasterack(mapped);
   I2Cmasternoack(mapped);
   I2Cstop(mapped);
  
//    printf("I2C read test 0x%04X\n",mval);
 //  printf("I2C (main) temperature (addr=0), temp (C) %f\n",mval*0.0078125);

   mapped[AAUXCTRL] = saveaux;	

  return(mval*0.0078125);
}//float board_temperature( volatile unsigned int *mapped )






float zynq_temperature()
{
  // try kernel <4 device file location
  float temperature = -999;
  char line[LINESZ];

  FILE *devfile = fopen( "/sys/devices/amba.0/f8007100.ps7-xadc/temp","r" );
  if( devfile )
  {
    fgets( line, LINESZ, devfile );    
    fclose(devfile);
    if( sscanf( line, "%f", &temperature ) != 1 )
    {
      //  printf( "got line '%s' trying to read ZYNQ temperature\n", line );
    }
  } else {

  // try kernel 4 device location
    // printf( "trying K4 location\n");
    // assume local shortcut exists to 
    // /sys/devices/soc0/amba/f8007100.adc/iio:device0/in_temp0_raw
    // which has trouble with fopen due to the :
    FILE *devfile1 = fopen( "/var/www/temp0_raw","r");
    if(!devfile1)
    { 
       // printf( "Could not open device file\n");
    } else {
       fgets( line, LINESZ, devfile1);
    //   printf( "%s\n", line);
       fclose(devfile1);
       if( sscanf( line, "%f", &temperature ) !=1 )
       {
         //  printf( "got line '%s' trying to read ZYNQ temperature\n", line );
       } else {
         temperature = (temperature - 2219)*123.04/1000;
         // constants 2219 and 123.04 are from .../in_temp0_offset and _scale
         // don't seem to change
       }
        
    }


  }

  return temperature;
}

int read_print_runstats(int mode, int dest, volatile unsigned int *mapped ) {
// mode 0: full print of all runstats, including raw values
// mode 1: only print times and rates
// dest 0: print to file
// dest 1: print to stdout      -- useful for cgi
// dest 2: print to both        -- currently fails if called by web client due to file write permissions

  int k, lastrs;
  FILE * fil;
  unsigned int m[N_PL_RS_PAR], c[NCHANNELS][N_PL_RS_PAR], csr, csrbit;
  double ma, ca[NCHANNELS], mb, cb[NCHANNELS], CT[NCHANNELS], val;   
  char N[7][32] = {      // names for the cgi array
    "ParameterM",
    "Module",
    "ParameterC",
    "Channel0",
    "Channel1",
    "Channel2",
    "Channel3" };


   // Run stats PL Parameter names applicable to a Pixie module 
char Module_PLRS_Names[N_PL_RS_PAR][MAX_PAR_NAME_LENGTH] = {
   "reserved",
   "CSROUT",		//0 
   "SYSTIME", 
   "RUNTIME", 
   "RUNTIME", 
   "TOTALTIME", 
   "TOTALTIME", 
   "NUMEVENTS", 
   "NUMEVENTS", 
   "BHL_EHL", 
   "CHL_FIFILENGTH", 
   "FW_VERSION", 	   //10
   "SNUM",
   "PPSTIME", 
   "T_ADC", 
   "T_ZYNQ", 
   //	"reserved", 
   "HW_VERSION", 
   "reserved", 
   "reserved",
   "reserved",
   "reserved",		    //20
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",	    //30
   "reserved"
};

 // Run stats PL Parameter names applicable to a Pixie channel 
char Channel_PLRS_Names[N_PL_RS_PAR][MAX_PAR_NAME_LENGTH] = {
   "reserved",
   "OOR*",		//0 
   "ICR", 
   "COUNTTIME", 
   "COUNTTIME", 
   "NTRIG", 
   "NTRIG", 
   "FTDT", 
   "FTDT", 
   "SFDT*", 
   "SFDT*", 
   "GCOUNT*", 	   //10
   "GCOUNT*", 
   "NOUT", 
   "NOUT", 
   "GDT*", 
   "GDT*", 
   "NPPI*", 
   "NPPI*", 
   //	"reserved",
   "reserved",
   "reserved",		    //20
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",	    //30
   "reserved"
};      

// return(0);

//}

  // ************** XIA code begins **************************
  // open the output file
  if(dest != 1)  {
          fil = fopen("RS.csv","w");
          fprintf(fil,"ParameterM,Module,ParameterC,Channel0,Channel1,Channel2,Channel3\n");
   }
      

  // read _used_ RS values (32bit) from FPGA 
  // at this point, raw binary values; later conversion into count rates etc

 //   mapped[AOUTBLOCK] = OB_RSREG;		// switch reads to run statistics block of addresses
 // must be done by calling function
   for( k = 0; k < N_USED_RS_PAR; k ++ )
   {
      m[k]  = mapped[ARS0_MOD+k];
      c[0][k] = mapped[ARS0_CH0+k];
      c[1][k] = mapped[ARS0_CH1+k];
      c[2][k] = mapped[ARS0_CH2+k];
      c[3][k] = mapped[ARS0_CH3+k];
   }
   csr = m[1];    // more memorable name for CSR

   // compute and print useful output values
   // run time = total time and Count time
   ma = ((double)m[3]+(double)m[4]*TWOTO32)*1.0e-9;
   if(dest != 1) fprintf(fil,"RUN_TIME,%4.6G,COUNT_TIME",ma); 
   if(dest != 0) printf("{%s:\"RUN_TIME\",%s:%4.6G,%s:\"COUNT_TIME\"",N[0], N[1],ma,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      CT[k] = ((double)c[k][3] + (double)c[k][4]*TWOTO32)*1.0e-9;
      if(dest != 1) fprintf(fil,",%4.6G",CT[k]);
      if(dest != 0) printf(",%s:%4.6G",N[3+k],CT[k]);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   
   // Total time and ICR
   if(dest != 1) fprintf(fil,"TOTAL_TIME,%4.6G,INPUT_COUNT_RATE",ma); 
   if(dest != 0) printf("{%s:\"TOTAL_TIME\",%s:%4.6G,%s:\"INPUT_COUNT_RATE\"",N[0], N[1],ma,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      ca[k] = (double)c[k][5] + (double)c[k][6]*TWOTO32;               //Ntrig
      cb[k] = ((double)c[k][7] + (double)c[k][8]*TWOTO32)*1.0e-9;      //FTDT
      if((CT[k]-cb[k])==0)
         val = 0;                 // avoid division by zero
      else
         val = ca[k]/(CT[k]-cb[k]);
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[3+k],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   // Event rate and OCR
   mb = (double)m[7]+(double)m[8]*TWOTO32;
   if(ma==0)
      val = 0;                 // avoid division by zero
   else
      val = mb/ma;
   if(dest != 1) fprintf(fil,"EVENT_RATE,%4.6G,OUTPUT_COUNT_RATE",val); 
   if(dest != 0) printf("{%s:\"EVENT_RATE\",%s:%4.6G,%s:\"OUTPUT_COUNT_RATE\"",N[0], N[1],val,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      ca[k] = (double)c[k][13] + (double)c[k][14]*TWOTO32;     // Nout
      if(CT[k]==0)
         val = 0;                 // avoid division by zero
      else
         val = ca[k]/CT[k];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[3+k],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   // FTDT
   if(dest != 1) fprintf(fil,"PS_CODE_VERSION,0x%X,FTDT",PS_CODE_VERSION); 
   if(dest != 0) printf("{%s:\"PS_CODE_VERSION\",%s:\"0x%X\",%s:\"FTDT\"",N[0], N[1],PS_CODE_VERSION,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      if(dest != 1) fprintf(fil,",%4.3E",cb[k]);
      if(dest != 0) printf(",%s:%4.3E",N[3+k],cb[k]);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   // Active bit, SFDT
   csrbit =  (csr & 0x00002000) >> 13;
   if(dest != 1) fprintf(fil,"ACTIVE,%d,SFDT*",csrbit ); 
   if(dest != 0) printf("{%s:\"ACTIVE\",%s:\"%d\",%s:\"SFDT*\"",N[0], N[1],csrbit,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      ca[k] = ((double)c[k][9] + (double)c[k][10]*TWOTO32)*1.0e-9;    // SFDT
      if(dest != 1) fprintf(fil,",%4.3E",ca[k]);
      if(dest != 0) printf(",%s:%4.3E",N[3+k],ca[k]);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   // PSA_LICENSED, PPR
   csrbit =  (csr & 0x00000400) >> 10;
   if(dest != 1) fprintf(fil,"PSA_LICENSED,%d,PASS_PILEUP_RATE*",csrbit); 
   if(dest != 0) printf("{%s:\"--\",%s:%d,%s:\"PASS_PILEUP_RATE*\"",N[0], N[1],csrbit,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      ca[k] = (double)c[k][17] + (double)c[k][18]*TWOTO32;     // NPPI
      if(CT[k]==0)
         val = 0;                 // avoid division by zero
      else
         val = ca[k]/CT[k];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[3+k],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");


   // PTP required, Gate rate
   csrbit =  (csr & 0x00000020) >> 5;
   if(dest != 1) fprintf(fil,"PTP_REQ,%d,GATE_RATE*",csrbit); 
   if(dest != 0) printf("{%s:\"PTP_REQ\",%s:%d,%s:\"GATE_RATE*\"",N[0], N[1],csrbit,N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      ca[k] = (double)c[k][11] + (double)c[k][12]*TWOTO32;     // GCOUNT
      if(CT[k]==0)
         val = 0;                 // avoid division by zero
      else
         val = ca[k]/CT[k];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[3+k],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");


   // Gate time
   if(dest != 1) fprintf(fil,"--,0,GDT*"); 
   if(dest != 0) printf("{%s:\"--\",%s:0,%s:\"GDT*\"",N[0], N[1],N[2]);
   for( k = 0; k < NCHANNELS; k ++ ) {
      ca[k] = ((double)c[k][15] + (double)c[k][16]*TWOTO32)*1.0e-9;    // GDT
      if(dest != 1) fprintf(fil,",%4.6G",ca[k]);
      if(dest != 0) printf(",%s:%4.6G",N[3+k],ca[k]);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   if(mode == 1) 
     lastrs = 3;
   else
   {
     lastrs = N_USED_RS_PAR;
     // temperatures
     m[14] = (int)board_temperature(mapped,I2C_SELMAIN);
     m[15] = (int)zynq_temperature();
     m[16] = (int)(0xFFFF & (hwinfo(mapped) >> 16));          // this is a pretty slow I2C I/O
   }





   // print raw values also
   for( k = 0; k < lastrs; k ++ )
   {
      if(k==16 || k==11 || k==1) {   // print bit patterns for some parameters
         if(dest != 1) fprintf(fil,"%s,0x%X,%s,%u,%u,%u,%u\n ",Module_PLRS_Names[k],m[k],Channel_PLRS_Names[k],c[0][k],c[1][k],c[2][k],c[3][k]);
         if(dest != 0) printf("{%s:\"%s\",%s:\"0x%X\",%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u},  \n",N[0],Module_PLRS_Names[k],N[1],m[k],N[2],Channel_PLRS_Names[k],N[3],c[0][k],N[4],c[1][k],N[5],c[2][k],N[6],c[3][k]);
      } else if(k==2) {    // ICR gets factor 15 to scale in cps
         if(dest != 1) fprintf(fil,"%s,0x%X,%s,%u,%u,%u,%u\n ",Module_PLRS_Names[k],m[k],Channel_PLRS_Names[k],ICRSCALE*c[0][k],ICRSCALE*c[1][k],ICRSCALE*c[2][k],ICRSCALE*c[3][k]);
         if(dest != 0) printf("{%s:\"%s\",%s:\"0x%X\",%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u},  \n",N[0],Module_PLRS_Names[k],N[1],m[k],N[2],Channel_PLRS_Names[k],N[3],ICRSCALE*c[0][k],N[4],ICRSCALE*c[1][k],N[5],ICRSCALE*c[2][k],N[6],ICRSCALE*c[3][k]);
      } else  {
         if(dest != 1) fprintf(fil,"%s,%u,%s,%u,%u,%u,%u\n ",Module_PLRS_Names[k],m[k],Channel_PLRS_Names[k],c[0][k],c[1][k],c[2][k],c[3][k]);
         if(dest != 0) printf("{%s:\"%s\",%s:%u,%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u},  \n",N[0],Module_PLRS_Names[k],N[1],m[k],N[2],Channel_PLRS_Names[k],N[3],c[0][k],N[4],c[1][k],N[5],c[2][k],N[6],c[3][k]);
      }
   }
      
       
 
 // clean up  
 if(dest != 1) fclose(fil);
 return 0;
}


int read_print_runstats_XL_2x4(int mode, int dest, volatile unsigned int *mapped ) {
// mode 0: full print of all runstats, including raw values
// mode 1: only print times and rates
// dest 0: print to file
// dest 1: print to stdout      -- useful for cgi
// dest 2: print to both        -- currently fails if called by web client due to file write permissions

  int k,q, lastrs;
  FILE * fil;
  unsigned int co[N_PL_RS_PAR] ={0};
  unsigned int sy[2][N_PL_RS_PAR]  ={{0}};  
  unsigned int ch[NCHANNELS_PRESENT][N_PL_RS_PAR]  ={{0}};  
  unsigned int csr, csrbit, revsn;
  double coa, sya, CT[NCHANNELS_PRESENT], val;
  char N[14][32] = {      // names for the cgi array
    "ParameterCo",
    "Controller",
    "ParameterSy",
    "System0",
    "System1",
    "ParameterCh",
    "Channel0",
    "Channel1",
    "Channel2",
    "Channel3",
    "Channel4",
    "Channel5",
    "Channel6",
    "Channel7" 
    
    
    };

  // Run stats PL Parameter names applicable to a Pixie module 
char Controller_PLRS_Names[N_PL_RS_PAR][MAX_PAR_NAME_LENGTH] = {
 //  "reserved",    // dummy read
   "CSROUT",		//0 
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved", 
   "SysTime", 
   "SysTime", 
   "FW_VERSION", 
   "HW_VERSION", 
   "TotalTime", 	   //10
   "TotalTime",
   "TotalTime", 
   "TotalTime", 
   "SNUM", 
   "T_BOARD", 
   "T_ZYNQ", 
   "PCB_SNUM",
   "PCB_VERSION",
   "reserved",		   
   "reserved",         //20
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",	    
   "reserved",       //30
    "reserved"
};


   // Run stats PL Parameter names applicable to a Pixie module 
char System_PLRS_Names[N_PL_RS_PAR][MAX_PAR_NAME_LENGTH] = {
//   "reserved",    // dummy read
   "CSROUT",		//0 
   "sysstatus", 
   "dpmstatus", 
   "dpmstatus", 
   "dpmstatus", 
   "dpmstatus", 
   "ADCframe", 
   "reserved", 
   "RunTime", 
   "RunTime", 
   "RunTime", 	   //10
   "reserved",
   "FW_VERSION", 
   "reserved", 
    "reserved",
   "T_ADC", 
   "T_WR", 
   "reserved",
   "reserved", 
   "reserved",
   "reserved",		    
   "reserved",        //20
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",	    
   "reserved"       //30
};

 // Run stats PL Parameter names applicable to a Pixie channel 
char Channel_PLRS_Names[N_PL_RS_PAR][MAX_PAR_NAME_LENGTH] = {
 //  "reserved",
   "COUNTTIME",		//0 
   "COUNTTIME", 
   "COUNTTIME", 
   "reserved", 
   "NTRIG", 
   "NTRIG", 
   "NTRIG", 
   "reserved", 
   "NOUT", 
   "NOUT", 
   "NOUT", 	   //10
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved", 
   "reserved",
   "reserved",		   
   "reserved",       //20
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",	   
   "reserved",       //30
    "reserved"
};      

// return(0);

//}

  // ************** XIA code begins **************************
  // open the output file
  if(dest != 1)  {
          fil = fopen("RS.csv","w");
          fprintf(fil,"ParameterCo,Controller,ParameterSy,System0,System1,ParameterCh,Channel0,Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7\n");
   }
      

  // ----------------- read _used_ RS values (16bit) ----------------------------
  // at this point, raw binary values; later conversion into count rates etc

  // read controller data
  mapped[AOUTBLOCK] = CS_MZ;
  for( k = 0; k < 16; k ++ )
  {
      co[k] =  mapped[AMZ_RS+k];
  }
     csr = co[0];    // more memorable name for CSR


  // read from K7 - 0 
  mapped[AOUTBLOCK] = CS_K0;

  // read system data
  mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
  mapped[AMZ_EXDWR]  = 0x000;        //                         0x000  = system     -> now addressing system page of K7-0

  for( k = 0; k < N_USED_RS_PAR; k ++ )
  {
       mapped[AMZ_EXAFRD] = AK7_SYS_RS+k;    // read from system output range
       sy[0][k] = mapped[AMZ_EXDRD];
       if(SLOWREAD) sy[0][k] = mapped[AMZ_EXDRD];         
  }

  // read channel data

  for( q = 0; q < NCHANNEL_PER_K7; q ++ )
  {
      mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
      mapped[AMZ_EXDWR]  = 0x100+q;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0

      for( k = 0; k < 3; k ++ )
      {
         mapped[AMZ_EXAFRD] = AK7_CHN_RS_CT+k;    // read from channel output range
         ch[q][k+0] = mapped[AMZ_EXDRD];
        if(SLOWREAD) ch[q][k+0] = mapped[AMZ_EXDRD];            

         mapped[AMZ_EXAFRD] = AK7_CHN_RS_NTRIG+k;    // read from channel output range
         ch[q][k+4] = mapped[AMZ_EXDRD];
         if(SLOWREAD) ch[q][k+4] = mapped[AMZ_EXDRD];            

         mapped[AMZ_EXAFRD] = AK7_CHN_RS_NOUT+k;    // read from channel output range
         ch[q][k+8] = mapped[AMZ_EXDRD];
         if(SLOWREAD) ch[q][k+8] = mapped[AMZ_EXDRD];           
      }
  }

  // read from K7 - 1 
  mapped[AOUTBLOCK] = CS_K1;

  // read system data
  mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
  mapped[AMZ_EXDWR]  = 0x000;        //                         0x000  = system     -> now addressing system page of K7-0

  for( k = 0; k < N_USED_RS_PAR; k ++ )
  {
       mapped[AMZ_EXAFRD] = AK7_SYS_RS+k;    // read from system output range
       sy[1][k] = mapped[AMZ_EXDRD];
       if(SLOWREAD) sy[1][k] = mapped[AMZ_EXDRD];
  }

  // read channel data
  for( q = NCHANNEL_PER_K7; q < NCHANNEL_PER_K7*2; q ++ )
  {
      mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
      mapped[AMZ_EXDWR]  = 0x100+q;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0

 
      for( k = 0; k < 3; k ++ )
      {
        mapped[AMZ_EXAFRD] = AK7_CHN_RS_CT+k;    // read from channel output range
         ch[q][k+0] = mapped[AMZ_EXDRD];
         if(SLOWREAD) ch[q][k+0] = mapped[AMZ_EXDRD];

         mapped[AMZ_EXAFRD] = AK7_CHN_RS_NTRIG+k;    // read from channel output range
         ch[q][k+4] = mapped[AMZ_EXDRD];
         if(SLOWREAD) ch[q][k+4] = mapped[AMZ_EXDRD];

         mapped[AMZ_EXAFRD] = AK7_CHN_RS_NOUT+k;    // read from channel output range
         ch[q][k+8] = mapped[AMZ_EXDRD];
         if(SLOWREAD) ch[q][k+8] = mapped[AMZ_EXDRD];
      }
  }

 
   // --------------- compute and print useful output values ----------------------- 
   // when printing to std out for cgi, N[i] provide the column titles (repeated for every row as in "name":value)
 
   // total time (MZ), run time (sys) and Count time (ch)
   coa = ( (double)co[10] + (double)co[11]*65536 + (double)co[12]*TWOTO32 + (double)co[13]*65536*TWOTO32 )*1.0e-9;
   if(dest != 1) fprintf(fil,"TOTAL_TIME,%4.6G",coa); 
   if(dest != 0) printf("{%s:\"TOTAL_TIME\",%s:%4.6G",N[0], N[1],coa);

   sya = ( (double)sy[0][8] + (double)sy[0][9]*65536 + (double)sy[0][10]*TWOTO32 )/SYSTEM_CLOCK_MHZ*1.0e-6;
   if(dest != 1) fprintf(fil,",RUN_TIME,%4.6G",sya); 
   if(dest != 0) printf(",%s:\"RUN_TIME\",%s:%4.6G",N[2], N[3],sya);
   sya = ( (double)sy[1][8] + (double)sy[1][9]*65536 + (double)sy[1][10]*TWOTO32 )/SYSTEM_CLOCK_MHZ*1.0e-6;
   if(dest != 1) fprintf(fil,",%4.6G",sya); 
   if(dest != 0) printf(",%s:%4.6G",N[4],sya);
  
   if(dest != 1) fprintf(fil,",COUNT_TIME"); 
   if(dest != 0) printf(",%s:\"COUNT_TIME\"",N[5]);
   for( k = 0; k < NCHANNELS_PRESENT; k ++ ) {
      CT[k] = ( (double)ch[k][0] + (double)ch[k][1]*65536 + (double)ch[k][2]*TWOTO32 )/FILTER_CLOCK_MHZ*1.0e-6;
      if(dest != 1) fprintf(fil,",%4.6G",CT[k]);
      if(dest != 0) printf(",%s:%4.6G",N[6+k],CT[k]);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   
   // PS_CODE_VERSION, --, ICR      
   if(dest != 1) fprintf(fil,"PS_CODE_VERSION,0x%X",PS_CODE_VERSION); 
   if(dest != 0) printf("{%s:\"PS_CODE_VERSION\",%s:\"0x%X\"",N[0], N[1],PS_CODE_VERSION);

   if(dest != 1) fprintf(fil,",--,0"); 
   if(dest != 0) printf(",%s:\"--\",%s:0",N[2], N[3]);
   if(dest != 1) fprintf(fil,",0"); 
   if(dest != 0) printf(",%s:0",N[4]);
  
   if(dest != 1) fprintf(fil,",INPUT_COUNT_RATE"); 
   if(dest != 0) printf(",%s:\"INPUT_COUNT_RATE\"",N[5]);
   for( k = 0; k < NCHANNELS_PRESENT; k ++ ) {   
      val = ( (double)ch[k][4] + (double)ch[k][5]*65536 + (double)ch[k][6]*TWOTO32 );    // fastpeaks, Nin
      if(CT[k]==0)
         val=0;
      else 
         val = val/CT[k];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[6+k],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");

   // Active bit, --, OCR   
   csrbit =  (csr & 0x00002000) >> 13;
   if(dest != 1) fprintf(fil,"ACTIVE,%d",csrbit); 
   if(dest != 0) printf("{%s:\"ACTIVE\",%s:\"%d\"",N[0], N[1],csrbit);

   if(dest != 1) fprintf(fil,",--,0"); 
   if(dest != 0) printf(",%s:\"--\",%s:0",N[2], N[3]);
   if(dest != 1) fprintf(fil,",0"); 
   if(dest != 0) printf(",%s:0",N[4]);
  
   if(dest != 1) fprintf(fil,",OUTPUT_COUNT_RATE"); 
   if(dest != 0) printf(",%s:\"OUTPUT_COUNT_RATE\"",N[5]);
   for( k = 0; k < NCHANNELS_PRESENT; k ++ ) {
      val = ( (double)ch[k][8] + (double)ch[k][9]*65536 + (double)ch[k][10]*TWOTO32 );    // Nout
      if(CT[k]==0)
         val=0;
      else 
         val = val/CT[k];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[6+k],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");



   if(mode == 1) 
      lastrs = 3;
   else
   {
      // ----------------- read I2C values (slow) to substitute some unused values


      // a dummy I2C operation required for the TMP116 I/O?
      co[15]    = (unsigned int)board_temperature(mapped,I2C_SELMAIN);
      sy[0][15] = (unsigned int)board_temperature(mapped,I2C_SELDB0);
      sy[1][15] = (unsigned int)board_temperature(mapped,I2C_SELDB1);
  //    printf("T_board %d, T_DB0 %d, T_DB1 %d\n", co[15], sy[0][15], sy[1][15]); 
      co[16]    = (int)zynq_temperature();
      revsn  = hwinfo(mapped);
      co[18] = (revsn>>16) & 0xFFFF;    // pcb rev from TMP116
      co[17] = revsn & 0xFFFF;          // s/n from TMP116
      lastrs = N_USED_RS_PAR;
   }




  
   // print raw values also
   for( k = 0; k < lastrs; k ++ )
   {
      if(k==14 || k==15 || k==16 || k==17) {   // print decimals for some parameters
         if(dest != 1) 
         {
            fprintf(fil,"%s,%u,",Controller_PLRS_Names[k],co[k]);
            fprintf(fil,"%s,%u,%u,",System_PLRS_Names[k], sy[0][k], sy[1][k]);
            fprintf(fil,"%s,%u,%u,%u,%u,%u,%u,%u,%u\n ", Channel_PLRS_Names[k],ch[0][k],ch[1][k],ch[2][k],ch[3][k],ch[4][k],ch[5][k],ch[6][k],ch[7][k]);
         }
         if(dest != 0) 
         {
            printf("{%s:\"%s\",%s:%u,",N[0],Controller_PLRS_Names[k],N[1],co[k]);
            printf("%s:\"%s\",%s:%u,%s:%u,",N[2],System_PLRS_Names[k], N[3],sy[0][k], N[4],sy[1][k]);
            printf("%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u},  \n", N[5],Channel_PLRS_Names[k],N[6],ch[0][k],N[7],ch[1][k],N[8],ch[2][k],N[9],ch[3][k],N[10],ch[4][k],N[11],ch[5][k],N[12],ch[6][k],N[13],ch[7][k]);
         }
      } else {                                // others are bit patterns
         if(dest != 1) 
         {
            fprintf(fil,"%s,0x%X,",Controller_PLRS_Names[k],co[k]);
            fprintf(fil,"%s,0x%X,0x%X,",System_PLRS_Names[k], sy[0][k], sy[1][k]);
            fprintf(fil,"%s,%u,%u,%u,%u,%u,%u,%u,%u\n ", Channel_PLRS_Names[k],ch[0][k],ch[1][k],ch[2][k],ch[3][k],ch[4][k],ch[5][k],ch[6][k],ch[7][k]);
         }
         if(dest != 0) 
         {
            printf("{%s:\"%s\",%s:\"0x%X\",",N[0],Controller_PLRS_Names[k],N[1],co[k]);
            printf("%s:\"%s\",%s:\"0x%X\",%s:\"0x%X\",",N[2],System_PLRS_Names[k], N[3],sy[0][k], N[4],sy[1][k]);
            printf("%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u},  \n", N[5],Channel_PLRS_Names[k],N[6],ch[0][k],N[7],ch[1][k],N[8],ch[2][k],N[9],ch[3][k],N[10],ch[4][k],N[11],ch[5][k],N[12],ch[6][k],N[13],ch[7][k]);
         }
   //      if(dest != 1) fprintf(fil,"%s,0x%X,%s,%u,%u,%u,%u\n ",Module_PLRS_Names[k],m[k],Channel_PLRS_Names[k],c[0][k],c[1][k],c[2][k],c[3][k]);
   //      if(dest != 0) printf("{%s:\"%s\",%s:\"0x%X\",%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u},  \n",N[0],Module_PLRS_Names[k],N[1],m[k],N[2],Channel_PLRS_Names[k],N[3],c[0][k],N[4],c[1][k],N[5],c[2][k],N[6],c[3][k]);
      }
   }  // end for

   
      
       
 
 // clean up  
 if(dest != 1) fclose(fil);
 return 0;
}




