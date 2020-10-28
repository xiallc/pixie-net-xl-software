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
   //   mapped[AMZ_DEVICESEL] = OB_EVREG;          
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
        ret = par & (1 << bitc);     // bitwise AND parameter with csr (= input) bit
        ret = ret >> bitc;           // shift down to bit 0 
        ret = ret << bitf;           // shift up to fippi (= output) bit
        return (ret);
  }


unsigned int hwinfo( volatile unsigned int *mapped, unsigned int I2Csel)
// returns 32bit hwrev_sn, or 0 on error
// upper 16 bit: revision              (TMP116 word 7)
// lower 16 bit serial number          (TMP116 word 6, valid only for I2C_SELMAIN)
{
   unsigned int  mval, i2cdata[8];
   unsigned int revsn, saveaux;
   unsigned int ctrl[8];
   int k;

  // ---------------- read EEPROM ---------------------------
  mapped[AMZ_DEVICESEL] = CS_MZ;	  // read/write from/to MZ IO block
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
    mval = 0x06;   // addr 7 = serial number
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


   // ------------- read hardware version  -------------------- 

     // 2 bytes: ctrl, addr  write
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*         // write starting addr to read from
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
    mval = 0x07;   // addr 8 =  hardware version
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

   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   //I2Cmasterack(mapped);
   I2Cmasternoack(mapped);
   I2Cstop(mapped);

   revsn = revsn | ((mval & 0xFFFF)<<16);


   // ------- When reading HW info for I2C_SELMAIN, include DB variant info from DB01 ----------------------

  if(I2Csel== I2C_SELMAIN) 
  {
     saveaux = saveaux & 0xFF8F;    // clear the I2C select bits
     mval = saveaux | I2C_SELDB1;    // set bit 4-6 to select MZ I2C pins
     mapped[AAUXCTRL] = mval;
   
   
      // 2 bytes: ctrl, addr  write
      I2Cstart(mapped);
      ctrl[0] = 0;   // R/W*         // write starting addr to read from
      I2Cbytesend(mapped, ctrl);
      I2Cslaveack(mapped);
       mval = 0x07;   // addr 8 =  hardware version
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
   
      I2Cbytereceive(mapped, i2cdata);
      for( k = 0; k < 8; k ++ )
         if(i2cdata[k])
            mval = mval + (1<<(k+0));
      //I2Cmasterack(mapped);
      I2Cmasternoack(mapped);
      I2Cstop(mapped);
   
      revsn = revsn | ((mval & 0x00F0)<<16);
   }

   // ---------------- finish up and return -----------------


/* // version check -- TODO not for now
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
  mapped[AMZ_DEVICESEL] = CS_MZ;	  // read/write from/to MZ IO block
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

 //   mapped[AMZ_DEVICESEL] = OB_RSREG;		// switch reads to run statistics block of addresses
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
     m[16] = (int)(0xFFFF & (hwinfo(mapped,I2C_SELMAIN) >> 16));          // this is a pretty slow I2C I/O
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

  int k, lastrs;
  FILE * fil;
  unsigned int co[N_PL_RS_PAR] ={0};
  unsigned int sy[N_K7_FPGAS][N_PL_RS_PAR]  ={{0}};  
  unsigned int chn[NCHANNELS][N_PL_RS_PAR]  ={{0}};  
  unsigned int csr, csrbit;
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
  unsigned int k7, ch, ch_k7;      // ch = abs ch. no; ch_k7 = ch. no in k7
  unsigned int revsn, NCHANNELS_PER_K7, NCHANNELS_PRESENT;
  unsigned int SYSTEM_CLOCK_MHZ, FILTER_CLOCK_MHZ;
  double coa, sya, CT[NCHANNELS], val;
  char N[22][MAX_PAR_NAME_LENGTH] = {      // names for the cgi array
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
    "Channel7",
    "Channel8",
    "Channel9",
    "Channel10",
    "Channel11",
    "Channel12",
    "Channel13",
    "Channel14",
    "Channel15" 
    
    
    };

  // Run stats PL Parameter names applicable to a Pixie module 
char Controller_PLRS_Names[N_PL_RS_PAR][MAX_PAR_NAME_LENGTH] = {
 //  "reserved",    // dummy read
   "CSROUT",		//0     (BEGIN HEX)
   "reserved", 
   "reserved", 
   "FPGA BOOTED", 
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
   "reserved", 
   "reserved", 
   "reserved", 
   "PCB_VERSION", 
   "PCB_SNUM",       // 18 (BEGIN DECIMAL)
   "SNUM",	
   "T_BOARD",         //20 
   "T_ZYNQ",            
   "reserved",        // 22 (BEGIN HEX)
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
   "CSROUT",		//0  (BEGIN HEX)
   "sysstatus", 
   "MEM_I_CNT", 
   "MEM_I_CNT", 
   "MEM_O_CNT", 
   "MEM_O_CNT", 
   "ADCframe", 
   "reserved", 
   "RunTime", 
   "RunTime", 
   "RunTime", 	   //10
   "reserved",
   "FW_VERSION", 
   "WR_TM_TAI", 
   "WR_TM_TAI",
   "WR_TM_TAI", 
   "WR_TM_CYC", 
   "WR_TM_CYC",
   "dpmstatus",      // 18 (BEGIN DECIMAL)
   "dpmstatus",
   "T_ADC",          //20 
   "T_WR",
   "reserved",		  // 22 (BEGIN HEX)   
   "reserved",      
   "reserved",        
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",
   "reserved",	      //30
   "reserved"      
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

  // ************** main code begins **************************

   // --------------------------- check HW version -------------------------------

   revsn =  mapped[AMZ_HWINFO]  <<16; //hwinfo(mapped,I2C_SELMAIN);    // some settings may depend on HW variants
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB02;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB02;
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB02;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB02;
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;        
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB02;       // DB1 125 operates at same filter, sys freq as DB02 (and all other DB's, probably)
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB02;
   } 
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;          
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB01;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB01;
   }


  // ---------------- open the output file -------------------------------------------
  if(dest != 1)  {
          fil = fopen("RS.csv","w");
          fprintf(fil,"ParameterCo,Controller,ParameterSy,System0,System1,ParameterCh,");
          for(ch=0;ch<NCHANNELS_PRESENT;ch++) fprintf(fil,"Channel%d,",ch);
          fprintf(fil,"\n");
   }
      

  // ----------------- read _used_ RS values (16bit) ----------------------------
  // at this point, raw binary values; later conversion into count rates etc

  // read controller data, 16 words
  mapped[AMZ_DEVICESEL] = CS_MZ;
  for( k = 0; k < 16; k ++ )
  {
      co[k] =  mapped[AMZ_RS+k];
  }
     csr = co[0];    // more memorable name for CSR



  // read from K7 loop
  for(k7=0;k7<N_K7_FPGAS;k7++)
 {
     mapped[AMZ_DEVICESEL] = cs[k7];
   
     // read system data
     mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
     mapped[AMZ_EXDWR]  = PAGE_SYS;        //                         0x000  = system     -> now addressing system page of K7-0   
     for( k = 0; k < N_USED_RS_PAR; k ++ )
     {
          mapped[AMZ_EXAFRD] = AK7_SYS_RS+k;    // read from system output range
          sy[k7][k] = mapped[AMZ_EXDRD];
          if(SLOWREAD) sy[k7][k] = mapped[AMZ_EXDRD];         
     }
   
     // read channel data
     for( ch_k7 = 0; ch_k7 < NCHANNELS_PER_K7; ch_k7++ )
     {
         ch = ch_k7+k7*NCHANNELS_PER_K7;
         mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
         mapped[AMZ_EXDWR]  = PAGE_CHN+ch_k7;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0
   
         for( k = 0; k < 3; k ++ )      // loop over number of time words
         {
            mapped[AMZ_EXAFRD] = AK7_CHN_RS_CT+k;    // read from channel output range
            chn[ch][k+0] = mapped[AMZ_EXDRD];
            if(SLOWREAD) chn[ch][k+0] = mapped[AMZ_EXDRD];            
   
            mapped[AMZ_EXAFRD] = AK7_CHN_RS_NTRIG+k;    // read from channel output range
            chn[ch][k+4] = mapped[AMZ_EXDRD];
            if(SLOWREAD) chn[ch][k+4] = mapped[AMZ_EXDRD];            
   
            mapped[AMZ_EXAFRD] = AK7_CHN_RS_NOUT+k;    // read from channel output range
            chn[ch][k+8] = mapped[AMZ_EXDRD];
            if(SLOWREAD) chn[ch][k+8] = mapped[AMZ_EXDRD]; 
          
            //printf("CT value %x   ", chn[ch+k7*NCHANNEL_PER_K7][k+0]);
            //printf("NTRIG value %x   ", chn[ch+k7*NCHANNEL_PER_K7][k+4]);
            //printf("NPPI value %x\n", chn[ch+k7*NCHANNEL_PER_K7][k+8]);
         }     //end for time words
     }    // end for channels in k7
  } // end for K7s
 
 
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
   for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ ) {
      CT[ch] = ( (double)chn[ch][0] + (double)chn[ch][1]*65536 + (double)chn[ch][2]*TWOTO32 )/FILTER_CLOCK_MHZ*1.0e-6;
      if(dest != 1) fprintf(fil,",%4.6G",CT[ch]);
      if(dest != 0) printf(",%s:%4.6G",N[6+ch],CT[ch]);
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
   for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ ) {   
      val = ( (double)chn[ch][4] + (double)chn[ch][5]*65536 + (double)chn[ch][6]*TWOTO32 );    // fastpeaks, Nin
      if(CT[ch]==0)
         val=0;
      else 
         val = val/CT[ch];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[6+ch],val);
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
   for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ ) {
      val = ( (double)chn[ch][8] + (double)chn[ch][9]*65536 + (double)chn[ch][10]*TWOTO32 );    // Nout
      if(CT[ch]==0)
         val=0;
      else 
         val = val/CT[ch];
      if(dest != 1) fprintf(fil,",%4.6G",val);
      if(dest != 0) printf(",%s:%4.6G",N[6+ch],val);
   }
   if(dest != 1) fprintf(fil,"\n ");
   if(dest != 0) printf("},  \n");



   if(mode == 1) 
      lastrs = 3;
   else
   {
      // ----------------- read I2C values (slow) to substitute some unused values

      revsn  = hwinfo(mapped,I2C_SELMAIN);
      co[3]  = (csr >> 14) & 0x1;
      co[17] = (revsn>>16) & 0xFFFF;    // pcb rev from TMP116
      co[18] = revsn & 0xFFFF;          // s/n from TMP116
      co[19]    = co[14];  // repeat s/n stored in MZ memory as item 19 (decimal print)
      co[20]    = (unsigned int)board_temperature(mapped,I2C_SELMAIN);
      co[21]    = (int)zynq_temperature(); 

      sy[0][20] = (unsigned int)board_temperature(mapped,I2C_SELDB0);
      sy[1][20] = (unsigned int)board_temperature(mapped,I2C_SELDB1);
  //    printf("T_board %d, T_DB0 %d, T_DB1 %d\n", co[15], sy[0][15], sy[1][15]); 
      
      lastrs = N_USED_RS_PAR;
   }

  
   // print raw values also
   for( k = 0; k < lastrs; k ++ )
   {
      if(k==18 || k==19 || k==20 || k==21) {   // print decimals for some parameters
         if(dest != 1) 
         {
            fprintf(fil,"%s,%u,",Controller_PLRS_Names[k],co[k]);
            fprintf(fil,"%s,%u,%u,%s",System_PLRS_Names[k], sy[0][k], sy[1][k],Channel_PLRS_Names[k]);
            for(ch=0;ch<NCHANNELS_PRESENT;ch++) fprintf(fil,",%u",chn[ch][k]);
            fprintf(fil,"\n ");
         }
         if(dest != 0) 
         {
            printf("{%s:\"%s\",%s:%u,",N[0],Controller_PLRS_Names[k],N[1],co[k]);
            printf("%s:\"%s\",%s:%u,%s:%u,%s:\"%s\"",N[2],System_PLRS_Names[k], N[3],sy[0][k], N[4],sy[1][k],N[5],Channel_PLRS_Names[k]);
            for(ch=0;ch<NCHANNELS_PRESENT;ch++) printf(",%s:%u",N[6+ch],chn[ch][k]);
            printf("},  \n");
           // printf("%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u},  \n", N[5],Channel_PLRS_Names[k],N[6],chn[0][k],N[7],chn[1][k],N[8],chn[2][k],N[9],chn[3][k],N[10],chn[4][k],N[11],chn[5][k],N[12],chn[6][k],N[13],chn[7][k]);
         }
      } else {                                // others are bit patterns
         if(dest != 1) 
         {
            fprintf(fil,"%s,0x%X,",Controller_PLRS_Names[k],co[k]);
            fprintf(fil,"%s,0x%X,0x%X,%s",System_PLRS_Names[k], sy[0][k], sy[1][k],Channel_PLRS_Names[k]);
            for(ch=0;ch<NCHANNELS_PRESENT;ch++) fprintf(fil,",%u",chn[ch][k]);
            fprintf(fil,"\n ");
            //fprintf(fil,"%s,%u,%u,%u,%u,%u,%u,%u,%u\n ", Channel_PLRS_Names[k],chn[0][k],chn[1][k],chn[2][k],chn[3][k],chn[4][k],chn[5][k],chn[6][k],chn[7][k]);
         }
         if(dest != 0) 
         {
            printf("{%s:\"%s\",%s:\"0x%X\",",N[0],Controller_PLRS_Names[k],N[1],co[k]);
            printf("%s:\"%s\",%s:\"0x%X\",%s:\"0x%X\",%s:\"%s\"",N[2],System_PLRS_Names[k], N[3],sy[0][k], N[4],sy[1][k],N[5],Channel_PLRS_Names[k]);
            for(ch=0;ch<NCHANNELS_PRESENT;ch++) printf(",%s:%u",N[6+ch],chn[ch][k]);
            printf("},  \n");
            //printf("%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u,%s:%u},  \n", N[5],Channel_PLRS_Names[k],N[6],chn[0][k],N[7],chn[1][k],N[8],chn[2][k],N[9],chn[3][k],N[10],chn[4][k],N[11],chn[5][k],N[12],chn[6][k],N[13],chn[7][k]);
         }
   //      if(dest != 1) fprintf(fil,"%s,0x%X,%s,%u,%u,%u,%u\n ",Module_PLRS_Names[k],m[k],Channel_PLRS_Names[k],c[0][k],c[1][k],c[2][k],c[3][k]);
   //      if(dest != 0) printf("{%s:\"%s\",%s:\"0x%X\",%s:\"%s\",%s:%u,%s:%u,%s:%u,%s:%u},  \n",N[0],Module_PLRS_Names[k],N[1],m[k],N[2],Channel_PLRS_Names[k],N[3],c[0][k],N[4],c[1][k],N[5],c[2][k],N[6],c[3][k]);
      }
   }  // end for

 //   printf("lastrs %d, k %d \n", lastrs, k); 

   
      
       
 
 // clean up  
 if(dest != 1) fclose(fil);
 return 0;
}


int ADCinit_DB01(volatile unsigned int *mapped ) {
 // adjusts bitslip and returns 0 if successful, -1 if not

   int ret=0;
   unsigned int mval = 0;
   unsigned int trys;
   unsigned int frame; 
   
   unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
   int k7; 
   unsigned int goodframe = 0x87;     // depends on FPGA compile?
   printf(" Target frame pattern is 0x%02x\n",goodframe); 
   
   for(k7=0;k7<N_K7_FPGAS;k7++)
      {
      trys = 0;
      frame= 0;
      
      do {
         // read frame 
         mapped[AMZ_DEVICESEL] = cs[k7];	            // select FPGA  
         mapped[AMZ_EXAFWR] = AK7_PAGE;         // write to  k7's addr        addr 3 = channel/system, select    
         mapped[AMZ_EXDWR] = PAGE_SYS;             //  0x000  = system page                
         mapped[AMZ_EXAFRD] = AK7_ADCFRAME;     // write register address to  K7
         usleep(1);
         frame = mapped[AMZ_EXDRD]; 
         printf( " K7 %d: frame pattern is 0x%x (try %d) \n", k7, frame, trys);
                   
         // turn testpattern off again
         mapped[AMZ_EXAFWR] = AK7_ADCSPI;       // write to  k7's addr     addr 5 = SPI
         mval = 0 << 15;                        // SPI write (high bit=0)
         mval = mval + (0x03 << 8);             // SPI reg address  (bit 13:8)
         mval = mval + 0;                       // test pattern off
         mapped[AMZ_EXDWR] = mval;              //  write to ADC SPI
         usleep(5);
                  
         if(frame!=goodframe) {
            // trigger a bitslip         
            mapped[AMZ_EXAFWR] = AK7_PAGE;         // write to  k7's addr        addr 3 = channel/system, select    
            mapped[AMZ_EXDWR] = PAGE_SYS;             //  0x000  = system page                           
            mapped[AMZ_EXAFWR] = AK7_ADCBITSLIP;   // write register address to  K7
            mapped[AMZ_EXDWR] = 0;             // any write will do
         }

         if(frame==0)
            trys = 16;  // break out of loop
         
         trys = trys+1;
      
      } while(frame!=goodframe && trys<16);
      
      if(frame==goodframe)  {
         printf( " K7 %d: ADC initialized ok \n", k7);
         // keep ret unchanged, default above 0 = success
      } else {
         printf( " K7 %d: ADC not initialized or missing, try again by calling adcinit? \n", k7);
         ret = -1;
      }
   
   } // end for K7s
   
   
   mapped[AMZ_DEVICESEL] = CS_MZ;	  // deselect FPGA 0  
   return (ret);
}

int PLLinit(volatile unsigned int *mapped ) {
 // programs the PLL registers for WR clock conditioning and returns 0 if successful, -1 if not
 // TODO: can be removed, now contained in FPGA

   int ret=0;
   //int nbytes = 68;
   int nbytes = 67;
   unsigned int mval[nbytes];
   unsigned int addr[nbytes];
   
   unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
   int k7, by; 

   // hardcoded address byte data to write PLL registers of AD9516
   // input 20 MHz
   // VCO = input /R * (P*B+A) = 1500 MHz
   // R=2
   // A=6
   // B=18
   // VCO divided by 3 for clock output channels (using 3,4 LVDS) = 500 MHz
   // channel output divided by 4 = 125 MHz
   // after programming, write 0x232[0] to activate
   // after changing settings, toggle 0 > 1 transition of reg 0x18[0] for calibration 
   //   (this bit powers up at as zero, so setting it to 1 in the first programming initialized calibration. afterwards, explicitely set to 0, then 1

   
   // 1. configure I/O
   //addr[0] = 0x0000;                         // reg 0x000:config
   //mval[0] = 0xBD;                           // SDO is output, long instructions

   addr[0] = 0x0000;                         // reg 0x000:config
   mval[0] = 0x99;                           // SDO is output, long instructions

   addr[1] = 0x0018;                         // reg 0x000:config
   mval[1] = 0x00;                           // SDO is output, long instructions
 
   addr[2] = 0x0232;                         // reg 0x232: update register
   mval[2] = 0x01;                           // set to 1 to update registers

   
   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
      
         // FPGA I/O select
         mapped[AMZ_DEVICESEL] = cs[k7];	      // select FPGA  
         mapped[AMZ_EXAFWR] = AK7_PAGE;         // write to  k7's addr        addr 3 = channel/system, select    
         mapped[AMZ_EXDWR] = PAGE_SYS;          //  0x000  = system page       
         
         // write registers
         for(by=0;by<3;by++)
         {
         // 
            mapped[AMZ_EXAFWR] = AK7_PLLSPIA;      // write to  k7's addr     addr 27 = PLL SPI addr
            mapped[AMZ_EXDWR] = addr[by];          //  write to ADC SPI
            mapped[AMZ_EXAFWR] = AK7_PLLSPID;      // write to  k7's addr     addr 28= PLL SPI data and start transfer
            mapped[AMZ_EXDWR] = mval[by];          //  write to ADC SPI
            usleep(10);
         }         
         printf( " K7 %d: PLL Reg 0 programmed \n", k7);

   
   } // end for K7s

    //2. read back key info  -- seems required to properly write data to PLL??
   addr[0] = 0x8003;                         // read reg 0x003:g
   mval[0] = 0x00;                           // SDO is output, long instructions
   addr[1] = 0x8004;                         // read reg 0x003:g
   mval[1] = 0x00;                           // SDO is output, long instructions
   addr[2] = 0x8005;                         // read reg 0x003:g
   mval[2] = 0x00;                           // SDO is output, long instructions
   addr[3] = 0x8006;                         // read reg 0x003:g
   mval[3] = 0x00;                           // SDO is output, long instructions


 
   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
      
         // FPGA I/O select
         mapped[AMZ_DEVICESEL] = cs[k7];	      // select FPGA  
         mapped[AMZ_EXAFWR] = AK7_PAGE;         // write to  k7's addr        addr 3 = channel/system, select    
         mapped[AMZ_EXDWR] = PAGE_SYS;          //  0x000  = system page       
         
         // write registers
         for(by=0;by<4;by++)
         {
         // 
            mapped[AMZ_EXAFWR] = AK7_PLLSPIA;      // write to  k7's addr     addr 27 = PLL SPI addr
            mapped[AMZ_EXDWR] = addr[by];          //  write to ADC SPI
            mapped[AMZ_EXAFWR] = AK7_PLLSPID;      // write to  k7's addr     addr 28= PLL SPI data and start transfer
            mapped[AMZ_EXDWR] = mval[by];          //  write to ADC SPI
            usleep(10);
            mapped[AMZ_EXAFRD] = 0x96;          // read PLL output reg
            mval[by] =  mapped[AMZ_EXDRD];
            printf( " K7 %d: PLL Reg 0x%x = 0x%x \n", k7,addr[by],mval[by]);
         }         
          printf( "  \n");

   
   } // end for K7s                        	


   addr[0] = 0x0001;                                                 
   mval[0] = 0x00;
   addr[1] = 0x0003;                                                 
   mval[1] = 0xC3;
   addr[2] = 0x0004;                                                 
   mval[2] = 0x00;
   addr[3] = 0x0010;                                                 
   mval[3] = 0x7C;
   addr[4] = 0x0011;                                                 
   mval[4] = 0x02;
   addr[5] = 0x0012;                                                 
   mval[5] = 0x00;
   addr[6] = 0x0013;                                                 
   mval[6] = 0x06;
   addr[7] = 0x0014;                                                 
   mval[7] = 0x12;
   addr[8] = 0x0015;                                                 
   mval[8] = 0x00;
   addr[9] = 0x0016;                                                 
   mval[9] = 0x04;
   addr[10] = 0x0017;                                                 
   mval[10] = 0x00;
   addr[11] = 0x0019;                                                 
   mval[11] = 0x00;
   addr[12] = 0x001A;                                                 
   mval[12] = 0x00;
   addr[13] = 0x001B;                                                 
   mval[13] = 0x00;
   addr[14] = 0x001C;                                                 
   mval[14] = 0x02;
   addr[15] = 0x001D;                                                 
   mval[15] = 0x00;
   addr[16] = 0x001E;                                                 
   mval[16] = 0x00;
   addr[17] = 0x001F;                                                 
   mval[17] = 0x0E;

   addr[18] = 0x00A0;                                                 
   mval[18] = 0x01;
   addr[19] = 0x00A1;                                                 
   mval[19] = 0x00;
   addr[20] = 0x00A2;                                                 
   mval[20] = 0x00;
   addr[21] = 0x00A3;                                                 
   mval[21] = 0x01;
   addr[22] = 0x00A4;                                                 
   mval[22] = 0x00;
   addr[23] = 0x00A5;                                                 
   mval[23] = 0x00;
   addr[24] = 0x00A6;                                                 
   mval[24] = 0x01;
   addr[25] = 0x00A7;                                                 
   mval[25] = 0x00;
   addr[26] = 0x00A8;                                                 
   mval[26] = 0x00;
   addr[27] = 0x00A9;                                                 
   mval[27] = 0x01;
   addr[28] = 0x00AA;                                                 
   mval[28] = 0x00;
   addr[29] = 0x00AB;                                                 
   mval[29] = 0x00;

   addr[30] = 0x00F0;                                                 
   mval[30] = 0x0B;
   addr[31] = 0x00F1;                                                 
   mval[31] = 0x0A;
   addr[32] = 0x00F2;                                                 
   mval[32] = 0x0B;
   addr[33] = 0x00F3;                                                 
   mval[33] = 0x0A;
   addr[34] = 0x00F4;                                                 
   mval[34] = 0x0B;
   addr[35] = 0x00F5;                                                 
   mval[35] = 0x0A;

   addr[36] = 0x0140;                                                 
   mval[36] = 0x42;
   addr[37] = 0x0141;                                                 
   mval[37] = 0x43;
   addr[38] = 0x0142;                                                 
   mval[38] = 0x42;
   addr[39] = 0x0143;                                                 
   mval[39] = 0x42;

   addr[40] = 0x0190;                                                 
   mval[40] = 0x00;
   addr[41] = 0x0191;                                                 
   mval[41] = 0x80;
   addr[42] = 0x0192;                                                 
   mval[42] = 0x00;
   addr[43] = 0x0193;                                                 
   mval[43] = 0xBB;
   addr[44] = 0x0194;                                                 
   mval[44] = 0x80;
   addr[45] = 0x0195;                                                 
   mval[45] = 0x00;
   addr[46] = 0x0196;                                                 
   mval[46] = 0x00;
   addr[47] = 0x0197;                                                 
   mval[47] = 0x00;
   addr[48] = 0x0198;                                                 
   mval[48] = 0x00;
   addr[49] = 0x0199;                                                 
   mval[49] = 0x00;
   addr[50] = 0x019A;                                                 
   mval[50] = 0x00;
   addr[51] = 0x019B;                                                 
   mval[51] = 0x00;
   addr[52] = 0x019C;                                                 
   mval[52] = 0x00;
   addr[53] = 0x019D;                                                 
   mval[53] = 0x00;
   addr[54] = 0x019E;                                                 
   mval[54] = 0x00;
   addr[55] = 0x019F;                                                 
   mval[55] = 0x00;

   addr[56] = 0x01A0;                                                 
   mval[56] = 0x00;
   addr[57] = 0x01A1;                                                 
   mval[57] = 0x00;
   addr[58] = 0x01A2;                                                 
   mval[58] = 0x00;
   addr[59] = 0x01A3;                                                 
   mval[59] = 0x00;

   addr[60] = 0x01E0;                                                 
   mval[60] = 0x01;
   addr[61] = 0x01E1;                                                 
   mval[61] = 0x02;

   addr[62] = 0x0230;                                                 
   mval[62] = 0x00;
   addr[63] = 0x0231;                                                 
   mval[63] = 0x00;
   addr[64] = 0x0232;                                                 
   mval[64] = 0x00;

   addr[65] = 0x0018;                                                 
   mval[65] = 0x01;
   addr[66] = 0x0232;                                                 
   mval[66] = 0x01;


/*
   addr[0] = 0x0010;                         // R/W*, 00 for write 1 byte, 13bit reg addr
   //mval[0] = 0x5c;                           // CP current 3.6mA, CP mode normal, PLL op mode normal
   mval[0] = 0x5C;

   addr[1] = 0x0011;                         // reg 0x11/12: input ref divider R
   mval[1] = 0x02;                           // R=2 

   addr[2] = 0x0013;                         // reg 0x13: A counter 
   mval[2] = 0x06;                           // A=6
 
   addr[3] = 0x0014;                         // reg 0x14/15: B counter 
   mval[3] = 0x12;                           // B=18
 
   addr[4] = 0x0016;                         // reg 0x16: resets and P
   mval[4] = 0x04;                           // no reset, P=8                        

   addr[5] = 0x001C;                         // reg 0x1C:ref input
   mval[5] = 0x02;                           // REF 1 power on, REF 1 selected ? 0x02
   
   addr[6] = 0x0018;                         // reg 0x18: calibration
   mval[6] = 0x06;                           // AG calibration disable       // VCO cal now > 1 **
  
   // channel output options 0x0140-143         
   addr[7] = 0x00140;                         //    channel 6 (test)       
   mval[7] = 0x42;                           //    LVDS, on, 3.5mA                       
   addr[8] = 0x00142;                         //    channel 8 (GTX)  
   mval[8] = 0x42;                           //    LVDS, on, 3.5mA                            
   addr[9] = 0x00143;                         //    channel 9 (MAIN/ADC/LMK)    
   mval[9] = 0x42;                           //    LVDS, on, 3.5mA [default is off]     

   // channel dividers in 0x0199-01A2        // defaults are NOT divide by 4 but 24 (DS disagrees in summary and detail page)
   addr[10] = 0x00199;                         //    channel div 3.1       
   //mval[10] = 0x11;                           //                          
   mval[10] = 0x00;
   addr[11] = 0x0019A;                         //    channel phase 3.1     
   mval[11] = 0x00;                           //                          
   addr[12] = 0x0019B;                         //    channel div 3.2       
   mval[12] = 0x00;                           // 
   //mval[12] = 0x00;
   addr[13] = 0x0019c;                         //    div 3.1 and 3.2 control    
   //mval[13] = 0x20;                           //	bypass 3.2, start low, no force
   mval[13] = 0x00;

   addr[14] = 0x0019E;                         //    channel div 4.1
   mval[14] = 0x11;       
   //mval[14] = 0x00;                           // 
   addr[15] = 0x0019F;                         //    channel phase 4.1
   mval[15] = 0x00;                           // 
   addr[16] = 0x001A0;                         //    channel div 4.2
   //mval[16] = 0x00;
   mval[16] = 0x11;                           // 
   addr[17] = 0x001A1;                         //      bypass channel 4
   //mval[17] = 0x20;				//	bypass 4.2, start low, no force
   mval[17] = 0x00;                           //      no bypass, use dividers
   
   addr[18] = 0x001E0;                         // reg 0x1E0:VCO divider
   mval[18] = 0x01;                           // VCO divider = 3

   addr[19] = 0x001E1;                         // reg 0x1E1:VCO divider source
   mval[19] = 0x02;                           // VCO divider source = VCO

   addr[20] = 0x00232;                        
   mval[20] = 0x81;  
*/

   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
      
         // FPGA I/O select
         mapped[AMZ_DEVICESEL] = cs[k7];	      // select FPGA  
         mapped[AMZ_EXAFWR] = AK7_PAGE;         // write to  k7's addr        addr 3 = channel/system, select    
         mapped[AMZ_EXDWR] = PAGE_SYS;          //  0x000  = system page       
         
         // write registers
         for(by=0;by<nbytes;by++)
      //     for(by=0;by<1;by++)
         {
         // 
            mapped[AMZ_EXAFWR] = AK7_PLLSPIA;      	// write to  k7's addr     addr 27 = PLL SPI addr
           mapped[AMZ_EXDWR] = addr[by];          //  write to ADC SPI
	    mapped[AMZ_EXAFWR] = AK7_PLLSPID;      // write to  k7's addr     addr 28= PLL SPI data and start transfer
            mapped[AMZ_EXDWR] = mval[by];          //  write to ADC SPI
            usleep(10);
	    printf( " reg %x: programmed \n", addr[by]);
         }         
         printf( " K7 %d: PLL programmed \n", k7);

   
   } // end for K7s
   
   
   mapped[AMZ_DEVICESEL] = CS_MZ;	  // deselect FPGA 0  
   return (ret);
}



int read_print_rates_XL_2x4(int dest, volatile unsigned int *mapped ) {
// only print times and rates   (mode 1)
// dest 0: print to file
// dest 1: print to stdout      -- useful for cgi
// dest 2: print to both        -- currently fails if called by web client due to file write permissions
// todo: add GOOD CHANNEL bit pattern to only read good channels
  int k;
  FILE * fil;
  unsigned int co[N_PL_RS_PAR] ={0};
  unsigned int sy[N_K7_FPGAS][N_PL_RS_PAR]  ={{0}};  
  unsigned int chn[NCHANNELS][N_PL_RS_PAR]  ={{0}};  
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
  unsigned int k7, ch, ch_k7;      // ch = abs ch. no; ch_k7 = ch. no in k7
  unsigned int revsn, NCHANNELS_PER_K7, NCHANNELS_PRESENT, SYSTEM_CLOCK_MHZ,FILTER_CLOCK_MHZ ;
  double coa, sya, CT[NCHANNELS], val1, val2;
  char N[4][MAX_PAR_NAME_LENGTH] = {      // names for the cgi array
    "Channel",
    "Time",
    "ICR",
    "OCR"
    };



//}

  // ************** main code begins **************************

   // --------------------------- check HW version -------------------------------

   revsn =  mapped[AMZ_HWINFO]  <<16; //hwinfo(mapped,I2C_SELMAIN);    // some settings may depend on HW variants
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB02;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB02;
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB02;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB02;
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;        
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB02;       // DB1 125 operates at same filter, sys freq as DB02 (and all other DB's, probably)
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB02;
   } 
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;          
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB01;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB01;
   }


  // ---------------- open the output file -------------------------------------------
  if(dest != 1)  {
          fil = fopen("RATES.csv","w");
          fprintf(fil,"CHANNEL,TIME,INPUT_COUNT_RATE,OUTPUT_COUNT_RATE,");
          fprintf(fil,"\n");
   }
      

  // ----------------- read _used_ RS values (16bit) ----------------------------
  // at this point, raw binary values; later conversion into count rates etc

  // read controller data, total time only
  mapped[AMZ_DEVICESEL] = CS_MZ;
  for( k = 0; k < 3; k ++ )
  {
      co[k] =  mapped[AMZ_RS_TT+k-1];
  }
  

  // read from K7 loop
  for(k7=0;k7<N_K7_FPGAS;k7++)
 {
     mapped[AMZ_DEVICESEL] = cs[k7];
   
     // read system data
     mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
     mapped[AMZ_EXDWR]  = PAGE_SYS;        //                         0x000  = system     -> now addressing system page of K7-0   
     for( k = 0; k < 3; k ++ )
     {
          mapped[AMZ_EXAFRD] = AK7_SYS_RS_RT+k;    // read from system output range for run time only
          sy[k7][k] = mapped[AMZ_EXDRD];
          if(SLOWREAD) sy[k7][k] = mapped[AMZ_EXDRD];         
     }
   
     // read channel data
     for( ch_k7 = 0; ch_k7 < NCHANNELS_PER_K7; ch_k7++ )
     {
         ch = ch_k7+k7*NCHANNELS_PER_K7;
         mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
         mapped[AMZ_EXDWR]  = PAGE_CHN+ch_k7;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0
   
         for( k = 0; k < 3; k ++ )      // loop over number of time words
         {
            mapped[AMZ_EXAFRD] = AK7_CHN_RS_CT+k;    // read from channel output range
            chn[ch][k+0] = mapped[AMZ_EXDRD];
            if(SLOWREAD) chn[ch][k+0] = mapped[AMZ_EXDRD];            
   
            mapped[AMZ_EXAFRD] = AK7_CHN_RS_NTRIG+k;    // read from channel output range
            chn[ch][k+4] = mapped[AMZ_EXDRD];
            if(SLOWREAD) chn[ch][k+4] = mapped[AMZ_EXDRD];            
   
            mapped[AMZ_EXAFRD] = AK7_CHN_RS_NOUT+k;    // read from channel output range
            chn[ch][k+8] = mapped[AMZ_EXDRD];
            if(SLOWREAD) chn[ch][k+8] = mapped[AMZ_EXDRD]; 
          
            //printf("CT value %x   ", chn[ch+k7*NCHANNEL_PER_K7][k+0]);
            //printf("NTRIG value %x   ", chn[ch+k7*NCHANNEL_PER_K7][k+4]);
            //printf("NPPI value %x\n", chn[ch+k7*NCHANNEL_PER_K7][k+8]);
         }     //end for time words
     }    // end for channels in k7
  } // end for K7s
 
 
   // --------------- compute and print useful output values ----------------------- 
   // when printing to std out for cgi, N[i] provide the column titles (repeated for every row as in "name":value)
 
// MZ controller
   coa = ( (double)co[0] + (double)co[1]*65536 + (double)co[2]*TWOTO32 )*1.0e-9;
   if(dest != 1) fprintf(fil,"Ctrl. TOTAL_TIME,%4.6G, --, --\n",coa); 
   if(dest != 0) printf("{%s:\"CTRL TOTAL_TIME\",%s:%4.6G,%s:\"--\",%s:\"--\"},    \n",N[0], N[1],coa,N[2], N[3]);

   // K7 System
   sya = ( (double)sy[0][0] + (double)sy[0][1]*65536 + (double)sy[0][2]*TWOTO32 )/SYSTEM_CLOCK_MHZ*1.0e-6;
   if(dest != 1) fprintf(fil,"Sys.0 RUN_TIME,%4.6G, --, --\n",sya); 
   if(dest != 0) printf("{%s:\"Sys.0 RUN_TIME\",%s:%4.6G,%s:\"--\",%s:\"--\"},    \n",N[0], N[1],sya,N[2], N[3]);
   sya = ( (double)sy[1][0] + (double)sy[1][1]*65536 + (double)sy[1][2]*TWOTO32 )/SYSTEM_CLOCK_MHZ*1.0e-6;
   if(dest != 1) fprintf(fil,"Sys.1 RUN_TIME,%4.6G, --, --\n",sya); 
   if(dest != 0) printf("{%s:\"Sys.1 RUN_TIME\",%s:%4.6G,%s:\"--\",%s:\"--\"},    \n",N[0], N[1],sya,N[2], N[3]);
  
   // Channels
   for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ ) {
      CT[ch] = ( (double)chn[ch][0] + (double)chn[ch][1]*65536 + (double)chn[ch][2]*TWOTO32 )/FILTER_CLOCK_MHZ*1.0e-6;
      val1 = ( (double)chn[ch][4] + (double)chn[ch][5]*65536 + (double)chn[ch][6]*TWOTO32 );    // fastpeaks, Nin
      val2 = ( (double)chn[ch][8] + (double)chn[ch][9]*65536 + (double)chn[ch][10]*TWOTO32 );    // Nout
      if(CT[ch]==0) {
         val1=0;
         val2=0;
      } else { 
         val1 = val1/CT[ch];
         val2 = val2/CT[ch];
      }
      if(dest != 1) fprintf(fil,"Ch %d COUNT_TIME,%4.6G,%4.6G,%4.6G\n",ch,CT[ch],val1,val2);
      if(dest != 0) printf("{%s:\"Ch %d COUNT_TIME\",%s:%4.6G,%s:%4.6G,%s:%4.6G},  \n",N[0], ch, N[1],CT[ch],N[2],val1,N[3],val2);
    }
   
 // clean up  
 if(dest != 1) fclose(fil);
 return 0;
}



