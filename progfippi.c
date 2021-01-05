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
#include <math.h>
#include <time.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/file.h>


// need to compile with -lm option

#include "PixieNetDefs.h"
#include "PixieNetCommon.h"
#include "PixieNetConfig.h"


int main(void) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;
  int k;


  // ******************* read ini file and fill struct with values ********************

  int verbose = 1;      // TODO: control with argument to function 
  // 0 print errors and minimal info only
  // 1 print errors and full info
  
  PixieNetFippiConfig fippiconfig;		// struct holding the input parameters
  const char *defaults_file = "defaults.ini";
  int rval = init_PixieNetFippiConfig_from_file( defaults_file, 0, &fippiconfig );   // first load defaults, do not allow missing parameters
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", defaults_file, rval );
    return rval;
  }
  const char *settings_file = "settings.ini";      
  rval = init_PixieNetFippiConfig_from_file( settings_file, 2, &fippiconfig );   // second override with user settings, do allow missing, don't print missing
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", settings_file, rval );
    return rval;
  }
  

  unsigned int mval, dac, reglo, reghi,traceena;
  unsigned int CW, SFR, FFR, SL[NCHANNELS], SG[NCHANNELS], FL[NCHANNELS], FG[NCHANNELS], TH[NCHANNELS];
  unsigned int PSAM, TL[NCHANNELS], TD[NCHANNELS], SAVER0[NCHANNELS];
  unsigned int i2cdata[8];
  unsigned int sw0bit01[NCHANNELS_PER_K7_DB01] = {6, 11, 4,  0};       // these arrays encode the mapping of gain bits to I2C signals for DB01
  unsigned int sw1bit01[NCHANNELS_PER_K7_DB01] = {8,  2, 5, 10};
  unsigned int gnbit01[NCHANNELS_PER_K7_DB01]  = {9, 12, 1,  3};
  unsigned int sw0bit06[NCHANNELS_PER_K7_DB01] = {0, 11, 2,  5};       // these arrays encode the mapping of gain bits to I2C signals for DB06
  unsigned int sw1bit06[NCHANNELS_PER_K7_DB01] = {8, 10, 4,  6};
  unsigned int gnbit06[NCHANNELS_PER_K7_DB01]  = {9, 12, 1,  3};
  unsigned int i2cgain[16] = {0}; 
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
  int ch, k7, ch_k7;    // loop counter better be signed int  . ch = abs ch. no; ch_k7 = ch. no in k7
  unsigned int revsn, NCHANNELS_PER_K7, NCHANNELS_PRESENT, NSAMPLES_PER_CYCLE;
  unsigned int ADC_CLK_MHZ, SYSTEM_CLOCK_MHZ, FILTER_CLOCK_MHZ;
  unsigned long long mac;
  unsigned int dip, sip;


  // *************** PS/PL IO initialization *********************
  // open the device for PD register I/O
  fd = open("/dev/uio0", O_RDWR);
  if (fd < 0) {
    perror("Failed to open devfile");
    return 1;
  }

  //Lock the PL address space so multiple programs cant step on eachother.
  if( flock( fd, LOCK_EX | LOCK_NB ) )
  {
    printf( "Failed to get file lock on /dev/uio0\n" );
    return 1;
  }
  
  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return 1;
  }

  mapped = (unsigned int *) map_addr;



  // *********************************************************
  // ******************* main code begins ********************
  // *********************************************************

   // ************************** check HW version ********************************


   revsn = hwinfo(mapped,I2C_SELMAIN);    // some settings may depend on HW variants
   SYSTEM_CLOCK_MHZ   =  SYSTEM_CLOCK_MHZ_MOST;     // defaults
   FILTER_CLOCK_MHZ   =  FILTER_CLOCK_MHZ_MOST; 


   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;
      ADC_CLK_MHZ       =  ADC_CLK_MHZ_DB01_75;             
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB01;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB01;
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;
      ADC_CLK_MHZ       =  ADC_CLK_MHZ_DB01_125;             
   } 
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
   {
      NCHANNELS_PRESENT  =  NCHANNELS_PRESENT_DB02;
      NCHANNELS_PER_K7   =  NCHANNELS_PER_K7_DB02;
      ADC_CLK_MHZ        =  ADC_CLK_MHZ_DB02;
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB04_14_250)
   {
      NCHANNELS_PRESENT  =  NCHANNELS_PRESENT_DB02;
      NCHANNELS_PER_K7   =  NCHANNELS_PER_K7_DB02;
      ADC_CLK_MHZ        =  ADC_CLK_MHZ_DB02;
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250)
   {
      NCHANNELS_PRESENT  =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7   =  NCHANNELS_PER_K7_DB01;
      ADC_CLK_MHZ        =  ADC_CLK_MHZ_DB06_250;   
   } 
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_14_500)
   {
      NCHANNELS_PRESENT  =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7   =  NCHANNELS_PER_K7_DB01;
      ADC_CLK_MHZ        =  ADC_CLK_MHZ_DB06_500;    
   } 
   if((revsn & PNXL_DB_VARIANT_MASK) == 0xF00000)      // no ADC DB: default to DB02
   {
      printf("HW Rev = 0x%04X, SN = %d, NO ADC DB! - assuming default DB02_12_250\n", revsn>>16, revsn&0xFFFF);
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB02;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB02;
      ADC_CLK_MHZ       =  ADC_CLK_MHZ_DB02;
   }

   NSAMPLES_PER_CYCLE =  (int)floor(ADC_CLK_MHZ/FILTER_CLOCK_MHZ); 

   // check if FPGA booted
   mval = mapped[AMZ_CSROUTL];
   if( (mval & 0x4000) ==0) {
       printf( "FPGA not booted, please run ./bootfpga first\n" );
       return -5;
   }

  // take struct values, convert to FPGA units, write to register, one by one
  // error/dependency check: returns -xxyy, 
  // with xx = parameter number (line) in file and yy = channel number (0 for module)

  // ********** SYSTEM PARAMETERS ******************
  //  if(fippiconfig.NUMBER_CHANNELS != NCHANNELS_PRESENT) {
  //    printf("Invalid NUMBER_CHANNELS = %d, should be %d\n",fippiconfig.NUMBER_CHANNELS,NCHANNELS_PRESENT);
  //    return -100;
  //  }

  if(fippiconfig.C_CONTROL > 65535) {
      printf("Invalid C_CONTROL = %d, and actually currently unused\n",fippiconfig.C_CONTROL);
      return -200;
    }
  if(fippiconfig.REQ_RUNTIME < 5.0) {
      printf("Invalid REQ_RUNTIME = %f, please increase\n",fippiconfig.REQ_RUNTIME);
      return -300;
    }
  
  if(fippiconfig.POLL_TIME < MIN_POLL_TIME) {
      printf("Invalid POLL_TIME = %d, please increase to more than %d\n",fippiconfig.POLL_TIME,MIN_POLL_TIME);
      return -400;
    }

   if(fippiconfig.CRATE_ID > MAX_CRATE_ID) {
      printf("Invalid CRATE_ID = %d, must be < %d\n",fippiconfig.CRATE_ID,MAX_CRATE_ID);
      return -500;
    }

   if(fippiconfig.SLOT_ID > MAX_SLOT_ID) {
      printf("Invalid SLOT_ID = %d, must be < %d\n",fippiconfig.SLOT_ID,MAX_SLOT_ID);
      return -600;
    }

   if(fippiconfig.MODULE_ID > MAX_MODULE_ID) {
       // todo: check for valid module type numbers
      printf("Invalid MODULE_ID = %d, must be < %d\n",fippiconfig.MODULE_ID,MAX_MODULE_ID);
      return -700;
    }

   // AUX CTRL:
    if(fippiconfig.AUX_CTRL > 65535) {
      printf("Invalid AUX_CTRL = 0x%x\n",fippiconfig.AUX_CTRL);
      return -800;
    }
    
    /* // CLK CTRL:
    if( (fippiconfig.CLK_CTRL == 3) | (fippiconfig.CLK_CTRL == 0) ) {
      // ok
    }  else {
      printf("Invalid CLK_CTRL = 0x%x, should be 0 or 3\n",fippiconfig.CLK_CTRL);
      return -800;
    }    */

   //  WR_RUNTIME_CTRL:
    if(fippiconfig.WR_RUNTIME_CTRL > 1) {
      printf("Invalid WR_RUNTIME_CTRL = 0x%x\n",fippiconfig.WR_RUNTIME_CTRL);
      return -900;
    }

    // UDP_PAUSE
    if(fippiconfig.UDP_PAUSE < 10) {
      printf("Invalid UDP_PAUSE = %d, must be > 10\n",fippiconfig.UDP_PAUSE);
      return -900;
    }
    if(fippiconfig.UDP_PAUSE > 65535) {
      printf("Invalid UDP_PAUSE = %d, must be < 64K\n",fippiconfig.UDP_PAUSE);
      return -900;
    }

    //  DATA_FLOW:
    if(fippiconfig.DATA_FLOW > 6) {
      printf("Invalid DATA_FLOW = 0x%x\n",fippiconfig.DATA_FLOW);
      return -900;
    }
    if( (fippiconfig.DATA_FLOW == 5) && (fippiconfig.RUN_TYPE != 0x301) ) {
      printf("Invalid DATA_FLOW = %d; can only be used with runtype 0x301 (no LM data out)\n",fippiconfig.DATA_FLOW);
      return -900;
    }
    if( (fippiconfig.DATA_FLOW <= 1) && (fippiconfig.RUN_TYPE == 0x301) ) {
      printf("Invalid DATA_FLOW = %d; can not be used with runtype 0x301\n",fippiconfig.DATA_FLOW);
      return -900;
    }
    if( ((fippiconfig.DATA_FLOW == 3) || (fippiconfig.DATA_FLOW == 4) || (fippiconfig.DATA_FLOW == 6) ) && (fippiconfig.RUN_TYPE != 0x100) ) {
      printf("Invalid DATA_FLOW = %d; WR UDP output currently only supported for runtype 0x100\n",fippiconfig.DATA_FLOW);
      return -900;
    }

    // WR Ethernet interface:
    // Not checking MAC and IP addresses (e.g. DEST_MAC0) for errors, but report for sanity check with hex numbers
    mac = fippiconfig.DEST_MAC1; 
    if(verbose) printf( " DEST_MAC1 equal to %02llX:%02llX:%02llX:%02llX:%02llX:%02llX\n", 
            (mac>>40) &0xFF,
            (mac>>32) &0xFF,
            (mac>>24) &0xFF,
            (mac>>16) &0xFF,
            (mac>> 8) &0xFF,
            (mac    ) &0xFF) ;
    mac = fippiconfig.SRC_MAC1; 
    if(verbose) printf( " SRC_MAC1 equal to %02llX:%02llX:%02llX:%02llX:%02llX:%02llX\n", 
            (mac>>40) &0xFF,
            (mac>>32) &0xFF,
            (mac>>24) &0xFF,
            (mac>>16) &0xFF,
            (mac>> 8) &0xFF,
            (mac    ) &0xFF) ;
    mac = fippiconfig.DEST_IP1; 
    if(verbose) printf( " DEST_IP1 0x%08llX equal to %lld.%lld.%lld.%lld\n", mac,
            (mac>>24) &0xFF,
            (mac>>16) &0xFF,
            (mac>> 8) &0xFF,
            (mac    ) &0xFF) ;
    mac = fippiconfig.SRC_IP0; 
    if(verbose) printf( " SRC_IP1 0x%08llX equal to %lld.%lld.%lld.%lld\n",mac, 
            (mac>>24) &0xFF,
            (mac>>16) &0xFF,
            (mac>> 8) &0xFF,
            (mac    ) &0xFF) ;

  
  // ********** MODULE PARAMETERS ******************

    //MODULE_CSRA     -- P16 trigger backplane functions: unused for now
    if(fippiconfig.MODULE_CSRA > 65535) {
      printf("Invalid MODULE_CSRA = 0x%x\n",fippiconfig.MODULE_CSRA);
      return -1000;
    }

    //MODULE_CSRB     -- P16 trigger backplane functions: unused for now
    if(fippiconfig.MODULE_CSRB > 65535) {
      printf("Invalid MODULE_CSRB = 0x%x\n",fippiconfig.MODULE_CSRB);
      return -1100;
    }

    // RUN_TYPE     -- not written to FPGA registers
    if( !( (fippiconfig.RUN_TYPE == 0x301)  ||
           (fippiconfig.RUN_TYPE == 0x100)  ||
           (fippiconfig.RUN_TYPE == 0x401)  ||
           (fippiconfig.RUN_TYPE == 0x400)   ) ) {
      printf("Invalid RUN_TYPE = 0x%x, please check manual for a list of supported run types\n",fippiconfig.RUN_TYPE);
      return -1200;
    }
    // remember to turn off trace capture for no-trace runtypes
    if( (fippiconfig.RUN_TYPE == 0x301)  || (fippiconfig.RUN_TYPE == 0x401) )
      traceena = 0;
    else
      traceena = 1;


    //MAX_EVENTS      -- not written to FPGA registers
    if(fippiconfig.MAX_EVENTS > 65535) {
      printf("Invalid MAX_EVENTS = 0x%x\n",fippiconfig.MAX_EVENTS);
      return -1300;
    }

    // COINCIDENCE_PATTERN -- unused P16
    if(fippiconfig.COINCIDENCE_PATTERN > 65535) {
      printf("Invalid COINCIDENCE_PATTERN = 0x%x\n",fippiconfig.COINCIDENCE_PATTERN);
      return -1400;
    }

    // COINCIDENCE_WINDOW   -- unused P16?
    CW = (int)floorf(fippiconfig.COINCIDENCE_WINDOW*SYSTEM_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
    if( (CW > MAX_CW) | (CW < MIN_CW) ) {
      printf("Invalid COINCIDENCE_WINDOW = %f, must be between %f and %f us\n",fippiconfig.COINCIDENCE_WINDOW, (double)MIN_CW/(double)SYSTEM_CLOCK_MHZ, (double)MAX_CW/(double)SYSTEM_CLOCK_MHZ);
      return -1500;
    }

     // SYNC_AT_START      --  written to FPGA registers XXX  ?
    if(fippiconfig.SYNC_AT_START >1) {
      printf("Invalid SYNC_AT_START = %d, can only be 0 and 1\n",fippiconfig.SYNC_AT_START);
      return -1600;
    }

     // RESUME        -- not written to FPGA registers
    if(fippiconfig.RESUME >1) {
      printf("Invalid RESUME = %d, can only be 0 and 1\n",fippiconfig.RESUME);
      return -1700;
    }
 

    // SLOW_FILTER_RANGE      --  written to FPGA registers below (with energy filter)
    SFR = fippiconfig.SLOW_FILTER_RANGE;
    if( (SFR > MAX_SFR) | (SFR < MIN_SFR) ) {
      printf("Invalid SLOW_FILTER_RANGE = %d, must be between %d and %d\n",SFR,MIN_SFR, MAX_SFR);
      return -1800;
    }

     // FAST_FILTER_RANGE   -- not implemented for now
    FFR = fippiconfig.FAST_FILTER_RANGE;
    if( (FFR > MAX_FFR) | (FFR < MIN_FFR) ) {
      printf("Invalid FAST_FILTER_RANGE = %d, must be between %d and %d\n",FFR,MIN_FFR, MAX_FFR);
      return -1900;
    }

     // FASTTRIG_BACKPLANEENA -- not implemented for now

     // TRIG_CONFIG# -- not implemented for now



  // ********** CHANNEL PARAMETERS ******************
   

  // ----- first, check limits -----------------
  
  for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ )
  {
      // CCSRA-C (B not used in P16)
      if(fippiconfig.CHANNEL_CSRA[ch] > 65535) {
         printf("Invalid CHANNEL_CSRA = 0x%x\n",fippiconfig.CHANNEL_CSRA[ch]);
         return -3000-ch;
      } 
      if(fippiconfig.CHANNEL_CSRC[ch] > 65535) {
         printf("Invalid CHANNEL_CSRC = 0x%x\n",fippiconfig.CHANNEL_CSRC[ch]);
         return -3100-ch;
      }  

      // gain and offset handled separately below

      // energy filter
      SL[ch] = (int)floorf(fippiconfig.ENERGY_RISETIME[ch] * FILTER_CLOCK_MHZ);
      SL[ch] = SL[ch] >> SFR;
      if(SL[ch] < MIN_SL) {
         printf("Invalid ENERGY_RISETIME = %f, minimum %f us at this filter range\n",fippiconfig.ENERGY_RISETIME[ch],(double)((MIN_SL<<SFR)/FILTER_CLOCK_MHZ));
         return -3200-ch;
      } 
      SG[ch] = (int)floorf(fippiconfig.ENERGY_FLATTOP[ch] * FILTER_CLOCK_MHZ);
      SG[ch] = SG[ch] >> SFR;
      if(SG[ch] < MIN_SG) {
         printf("Invalid ENERGY_FLATTOP = %f, minimum %f us at this filter range\n",fippiconfig.ENERGY_FLATTOP[ch],(double)((MIN_SG<<SFR)/FILTER_CLOCK_MHZ));
         return -3300-ch;
      } 
      if( (SL[ch]+SG[ch]) > MAX_SLSG) {
         printf("Invalid combined energy filter, maximum %f us at this filter range\n",(double)((MAX_SLSG<<SFR)/FILTER_CLOCK_MHZ));
         return -3400-ch;
      } 

  // trigger filter 
      FL[ch] = (int)floorf(fippiconfig.TRIGGER_RISETIME[ch] * FILTER_CLOCK_MHZ);
      FL[ch] = FL[ch] >> FFR;
      if(FL[ch] < MIN_FL) {
         printf("Invalid TRIGGER_RISETIME = %f, minimum %f us\n",fippiconfig.TRIGGER_RISETIME[ch],(double)(MIN_FL<<FFR/FILTER_CLOCK_MHZ));
         return -3500-ch;
      } 
      FG[ch] = (int)floorf(fippiconfig.TRIGGER_FLATTOP[ch] * FILTER_CLOCK_MHZ);
      FG[ch] = FG[ch] >> FFR;
      if(FG[ch] < MIN_FG) {
         printf("Invalid TRIGGER_FLATTOP = %f, minimum %f us\n",fippiconfig.TRIGGER_FLATTOP[ch],(double)(MIN_FG<<FFR/FILTER_CLOCK_MHZ));
         return -3600-ch;
      } 
      if( (FL[ch]+FG[ch]) > MAX_FLFG) {
         printf("Invalid combined trigger filter, maximum %f us\n",(double)(MAX_FLFG<FFR/FILTER_CLOCK_MHZ));
         return -3700-ch;
      } 
      
      TH[ch] = (int)floor(fippiconfig.TRIGGER_THRESHOLD[ch]*FL[ch]*NSAMPLES_PER_CYCLE);
      if(TH[ch] > MAX_TH)     {
         printf("Invalid TRIGGER_THRESHOLD = %f, maximum %f at this trigger filter rise time\n",fippiconfig.TRIGGER_THRESHOLD[ch],MAX_TH/(double)FL[ch]/NSAMPLES_PER_CYCLE);
         printf("(Note: pulses over %f always cause a trigger)\n",MAX_TH/(double)FL[ch]/NSAMPLES_PER_CYCLE);
         return -3800-ch;
      } 

      // THRESH_WIDTH   -- not implemented for a long time

      // waveforms
      TL[ch] = MULT_TL*(int)floor(fippiconfig.TRACE_LENGTH[ch]*ADC_CLK_MHZ/MULT_TL/(1<<FFR) );       // multiply time in us *  # ticks per us = time in ticks; multiple of MULT_TL
      if( (TL[ch] > MAX_TL) | (TL[ch] < MULT_TL) )  { 
         printf("Invalid TRACE_LENGTH = %f, must be between %f and %f us\n",fippiconfig.TRACE_LENGTH[ch],(double)MULT_TL/ADC_CLK_MHZ,(double)MAX_TL/ADC_CLK_MHZ);
         return -4000-ch;
         // Note: enfore TL >32 which is the minimum for 0x400 if traces are recorded. For no trace, use CCSRA bit to disable trace
      }
      // in UDP mode, all channels must have same TL  and CCSRA_TRACEENA
       if( (fippiconfig.DATA_FLOW==4) || (fippiconfig.DATA_FLOW==3) ) {
            if(TL[ch] != TL[0])   {
               printf("Invalid TRACE_LENGTH = %f (ch. %d), must be same as ch.0 in DATA_FLOW 3 or 4\n",fippiconfig.TRACE_LENGTH[ch],ch);
               return -4000-ch;
            }
            if( (fippiconfig.CHANNEL_CSRA[ch] &  (1<<CCSRA_TRACEENA)) != (fippiconfig.CHANNEL_CSRA[0] &  (1<<CCSRA_TRACEENA))   ) {
                printf("Invalid TRACEENA (ch. %d), must be same as ch.0 in DATA_FLOW 3 or 4\n",ch);
                return -4000-ch;
            }

       }

      if(TL[ch] <fippiconfig.TRACE_LENGTH[ch]*ADC_CLK_MHZ/(1<<FFR))  {
         printf("TRACE_LENGTH[%d] will be rounded off to = %f us, %d samples\n",ch,(double)TL[ch]/ADC_CLK_MHZ,TL[ch]);
      }
      TD[ch] = (int)floor(fippiconfig.TRACE_DELAY[ch]*ADC_CLK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
      if(TD[ch] > MAX_TL-TWEAK_UD)  {
         printf("Invalid TRACE_DELAY = %f, maximum %f us\n",fippiconfig.TRACE_DELAY[ch],(double)(MAX_TL-TWEAK_UD)/ADC_CLK_MHZ);
         return -4100-ch;
      }
      if(TD[ch] > TRACEDELAY_MAX)  {
         printf("Invalid TRACE_DELAY = %f, maximum %f us\n",fippiconfig.TRACE_DELAY[ch],(double)(TRACEDELAY_MAX)/ADC_CLK_MHZ);
         return -4200-ch;
      }

      // other parameters
      if(fippiconfig.BINFACTOR[ch] > MAX_BFACT)     {
         printf("Invalid BINFACTOR = %d, maximum %d\n",fippiconfig.BINFACTOR[ch],MAX_BFACT);
         return -4300-ch;
      } 

      if(fippiconfig.INTEGRATOR[ch] > 0)     {
         printf("Invalid INTEGRATOR = %d, currently not implemented\n",fippiconfig.INTEGRATOR[ch]);
         return -4400-ch;
      } 
     
      if(fippiconfig.BLCUT[ch] <4 || fippiconfig.BLCUT[ch] >1023 )  {
         printf("Invalid BLCUT = %d, must be between 4 and 1023\n",fippiconfig.BLCUT[ch]);
         return -4500-ch;
      }

      if(fippiconfig.BASELINE_PERCENT[ch] <1 || fippiconfig.BASELINE_PERCENT[ch] >99 )  {
         printf("Invalid BASELINE_PERCENT = %f, must be between 1 and 99\n",fippiconfig.BASELINE_PERCENT[ch]);
         return -4500-ch;
      }
                                               
      if(fippiconfig.BLAVG[ch] > 65535)     {
         printf("Invalid BLAVG = %d, maximum %d\n",fippiconfig.BLAVG[ch],65535);
         return -4600-ch;
      } 
      if( (fippiconfig.BLAVG[ch] >0) && (fippiconfig.BLAVG[ch] < 65535-MAX_BLAVG ) )     {
         printf("Invalid BLAVG = %d, minimum %d (or zero to turn off)\n",fippiconfig.BLAVG[ch],65535-MAX_BLAVG);
         return -4700-ch;
      } 

      if(fippiconfig.TAU[ch] <0)  {
         printf("Invalid TAU = %f, must be positive\n",fippiconfig.TAU[ch]);
         return -4800-ch;
      }

      if(fippiconfig.XDT[ch] < (MIN_XDT_MOST/ADC_CLK_MHZ) ) {
         printf("Invalid XDT = %f us, must be at least %f us\n",fippiconfig.XDT[ch],(double)MIN_XDT_MOST/ADC_CLK_MHZ );
         return -4900-ch;
      }
      mval = (int)floor( fippiconfig.XDT[ch]*ADC_CLK_MHZ/MIN_XDT_MOST) *MIN_XDT_MOST;
      if(fippiconfig.XDT[ch] > mval)     {
         printf("XDT = %f, will be rounded to %d \n",fippiconfig.XDT[ch],mval );
         //return -5000-ch;
      }

      // MULTIPLICITY_MASKL  -- not implemented for now


      if(fippiconfig.FASTTRIG_BACKLEN[ch] < FASTTRIGBACKLEN_MIN_100MHZFIPCLK/FILTER_CLOCK_MHZ)  {
         printf("Invalid FASTTRIG_BACKLEN = %f, must be at least %f us\n",fippiconfig.FASTTRIG_BACKLEN[ch], (double)FASTTRIGBACKLEN_MIN_100MHZFIPCLK/FILTER_CLOCK_MHZ);
         return -5100-ch;
      }
      if(fippiconfig.FASTTRIG_BACKLEN[ch] > FASTTRIGBACKLEN_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid FASTTRIG_BACKLEN = %f, must be less than %f us\n",fippiconfig.FASTTRIG_BACKLEN[ch], (double)FASTTRIGBACKLEN_MAX/FILTER_CLOCK_MHZ);
         return -5200-ch;
      }

      // CFD parameters specified in samples, not us as in P16!
      if(fippiconfig.CFD_THRESHOLD[ch] < CFDTHRESH_MIN)  {
         printf("Invalid CFD_THRESHOLD = %d, must be at least %d \n",fippiconfig.CFD_THRESHOLD[ch], CFDTHRESH_MIN);
         return -5300-ch;
      }
      if(fippiconfig.CFD_THRESHOLD[ch] > CFDTHRESH_MAX)  {
         printf("Invalid CFD_THRESHOLD = %d, must be less than %d \n",fippiconfig.CFD_THRESHOLD[ch], CFDTHRESH_MAX);
         return -5400-ch;
      }

      if(fippiconfig.CFD_DELAY[ch] < CFDDELAY_MIN)  {
         printf("Invalid CFD_DELAY = %d, must be at least %d samples\n",fippiconfig.CFD_DELAY[ch], CFDDELAY_MIN);
         return -5500-ch;
      }
      if(fippiconfig.CFD_DELAY[ch] > CFDDELAY_MAX)  {
         printf("Invalid CFD_DELAY = %d, must be less than %d samples\n",fippiconfig.CFD_DELAY[ch], CFDDELAY_MAX);
         return -5600-ch;
      }

      if(fippiconfig.CFD_SCALE[ch] < CFDSCALE_MIN)  {
         printf("Invalid CFD_SCALE = %d, must be at least %d samples\n",fippiconfig.CFD_SCALE[ch], CFDSCALE_MIN);
         return -5700-ch;
      }
      if(fippiconfig.CFD_SCALE[ch] > CFDSCALE_MAX)  {
         printf("Invalid CFD_SCALE = %d, must be less than %d samples\n",fippiconfig.CFD_SCALE[ch], CFDSCALE_MAX);
         return -5800-ch;
      }

      // stretches and delays ...
      if(fippiconfig.EXTTRIG_STRETCH[ch] < EXTTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid EXTTRIG_STRETCH = %f, must be at least %f us\n",fippiconfig.EXTTRIG_STRETCH[ch], (double)EXTTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ);
         return -5900-ch;
      }
      if(fippiconfig.EXTTRIG_STRETCH[ch] > EXTTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid EXTTRIG_STRETCH = %f, must be less than %f us\n",fippiconfig.EXTTRIG_STRETCH[ch], (double)EXTTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ);
         return -6000-ch;
      }

      if(fippiconfig.VETO_STRETCH[ch] < VETOSTRETCH_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid VETO_STRETCH = %f, must be at least %f us\n",fippiconfig.VETO_STRETCH[ch], (double)VETOSTRETCH_MIN/FILTER_CLOCK_MHZ);
         return -6100-ch;
      }
      if(fippiconfig.VETO_STRETCH[ch] > VETOSTRETCH_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid VETO_STRETCH = %f, must be less than %f us\n",fippiconfig.VETO_STRETCH[ch], (double)VETOSTRETCH_MAX/FILTER_CLOCK_MHZ);
         return -6200-ch;
      }

      if(fippiconfig.EXTERN_DELAYLEN[ch] < EXTDELAYLEN_MIN/FILTER_CLOCK_MHZ)  {                           
         printf("Invalid EXTERN_DELAYLEN = %f, must be at least %f us\n",fippiconfig.EXTERN_DELAYLEN[ch], (double)EXTDELAYLEN_MIN/FILTER_CLOCK_MHZ);
         return -6300-ch;
      }
      if(fippiconfig.EXTERN_DELAYLEN[ch] > EXTDELAYLEN_MAX_REVF/FILTER_CLOCK_MHZ)  {
         printf("Invalid EXTERN_DELAYLEN = %f, must be less than %f us\n",fippiconfig.EXTERN_DELAYLEN[ch], (double)EXTDELAYLEN_MAX_REVF/FILTER_CLOCK_MHZ);
         return -6400-ch;
      }  
    
      if(fippiconfig.FTRIGOUT_DELAY[ch] < FASTTRIGBACKDELAY_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid FTRIGOUT_DELAY = %f, must be at least %f us\n",fippiconfig.FTRIGOUT_DELAY[ch], (double)FASTTRIGBACKDELAY_MIN/FILTER_CLOCK_MHZ);
         return -6500-ch;
      }
      if(fippiconfig.FTRIGOUT_DELAY[ch] > FASTTRIGBACKDELAY_MAX_REVF/FILTER_CLOCK_MHZ)  {
         printf("Invalid FTRIGOUT_DELAY = %f, must be less than %f us\n",fippiconfig.FTRIGOUT_DELAY[ch], (double)FASTTRIGBACKDELAY_MAX_REVF/FILTER_CLOCK_MHZ);
         return -6600-ch;
      }      

      if(fippiconfig.CHANTRIG_STRETCH[ch] < CHANTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid CHANTRIG_STRETCH = %f, must be at least %f us\n",fippiconfig.CHANTRIG_STRETCH[ch], (double)CHANTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ);
         return -6700-ch;
      }
      if(fippiconfig.CHANTRIG_STRETCH[ch] > CHANTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid CHANTRIG_STRETCH = %f, must be less than %f us\n",fippiconfig.CHANTRIG_STRETCH[ch], (double)CHANTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ);
         return -6800-ch;
      } 


      // QDC parameters specified in samples, not as as in P16!
      if  ( (fippiconfig.CHANNEL_CSRA[ch] & (1<<CCSRA_QDCENA)) >0 )  {

         if( (fippiconfig.QDCLen0[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen0[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen0 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen0[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -6900-ch;
         }
         if( (fippiconfig.QDCLen1[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen1[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen1 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen1[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7000-ch;
         }
         if( (fippiconfig.QDCLen2[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen2[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen2 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen2[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7100-ch;
         }
         if( (fippiconfig.QDCLen3[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen3[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen3 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen3[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7200-ch;
         }
         if( (fippiconfig.QDCLen4[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen4[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen4 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen4[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7300-ch;
         }
         if( (fippiconfig.QDCLen5[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen5[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen5 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen5[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7400-ch;
         }
         if( (fippiconfig.QDCLen6[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen6[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen6 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen6[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7500-ch;
         }
         if( (fippiconfig.QDCLen7[ch] > QDCLEN_MAX) || (fippiconfig.QDCLen7[ch] < QDCLEN_MIN)  )  {
            printf("Invalid QDCLen7 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen7[ch], QDCLEN_MIN, QDCLEN_MAX);
            return -7600-ch;
         }

       } else {
         // check length limits
         if( (fippiconfig.QDCLen0[ch] > MAX_QDCL) || (fippiconfig.QDCLen0[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L0 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen0[ch], ch, MIN_QDCL, MAX_QDCL);
            return -6900-ch;
         }
         if( (fippiconfig.QDCLen1[ch] > MAX_QDCL) || (fippiconfig.QDCLen1[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L1 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen1[ch], ch, MIN_QDCL, MAX_QDCL);
            return -7000-ch;
         }
          if( (fippiconfig.QDCLen2[ch] > MAX_QDCL) || (fippiconfig.QDCLen0[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L2 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen2[ch], ch, MIN_QDCL, MAX_QDCL);
            return -6900-ch;
         }
         if( (fippiconfig.QDCLen3[ch] > MAX_QDCL) || (fippiconfig.QDCLen1[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L3 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen3[ch], ch, MIN_QDCL, MAX_QDCL);
            return -7000-ch;
         }
         if( (fippiconfig.QDCLen4[ch] > MAX_QDCL) || (fippiconfig.QDCLen0[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L4 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen4[ch], ch, MIN_QDCL, MAX_QDCL);
            return -6900-ch;
         }
         if( (fippiconfig.QDCLen5[ch] > MAX_QDCL) || (fippiconfig.QDCLen1[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L5 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen5[ch], ch, MIN_QDCL, MAX_QDCL);
            return -7000-ch;
         }
         if( (fippiconfig.QDCLen6[ch] > MAX_QDCL) || (fippiconfig.QDCLen0[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L6 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen6[ch], ch, MIN_QDCL, MAX_QDCL);
            return -6900-ch;
         }
         if( (fippiconfig.QDCLen7[ch] > MAX_QDCL) || (fippiconfig.QDCLen1[ch] < MIN_QDCL)  )  {
            printf("Invalid PSA L7 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen7[ch], ch, MIN_QDCL, MAX_QDCL);
            return -7000-ch;
         }

         // check delay+length max
         if( (fippiconfig.QDCLen0[ch]+fippiconfig.QDCDel0[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D0+L0 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen0[ch]+fippiconfig.QDCDel0[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7100-ch;
         }
         if( (fippiconfig.QDCLen1[ch]+fippiconfig.QDCDel1[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D1+L1 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen1[ch]+fippiconfig.QDCDel1[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7200-ch;
         }
         if( (fippiconfig.QDCLen2[ch]+fippiconfig.QDCDel2[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D2+L2 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen2[ch]+fippiconfig.QDCDel2[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7100-ch;
         }
         if( (fippiconfig.QDCLen3[ch]+fippiconfig.QDCDel3[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D3+L3 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen3[ch]+fippiconfig.QDCDel3[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7200-ch;
         }
         if( (fippiconfig.QDCLen4[ch]+fippiconfig.QDCDel4[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D4+L4 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen4[ch]+fippiconfig.QDCDel4[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7100-ch;
         }
         if( (fippiconfig.QDCLen5[ch]+fippiconfig.QDCDel5[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D5+L5 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen5[ch]+fippiconfig.QDCDel5[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7200-ch;
         }
         if( (fippiconfig.QDCLen6[ch]+fippiconfig.QDCDel6[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D6+L6 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen6[ch]+fippiconfig.QDCDel6[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7100-ch;
         }
         if( (fippiconfig.QDCLen7[ch]+fippiconfig.QDCDel7[ch] > MAX_QDCLD)   )  {
            printf("Invalid PSA D7+L7 = %d (ch. %d), must be between %d and %d samples\n",fippiconfig.QDCLen7[ch]+fippiconfig.QDCDel7[ch], ch, MIN_QDCL, MAX_QDCLD);
            return -7200-ch;                
         }

         // all sums must have same delay (in sub-sample group) as sum 0
         if( (fippiconfig.QDCDel1[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D1 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }
         if( (fippiconfig.QDCDel2[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D2 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }
         if( (fippiconfig.QDCDel3[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D3 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }
         if( (fippiconfig.QDCDel4[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D4 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }
         if( (fippiconfig.QDCDel5[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D5 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }
         if( (fippiconfig.QDCDel6[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D6 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }
         if( (fippiconfig.QDCDel7[ch] & 0x0001) != (fippiconfig.QDCDel0[ch] & 0x0001) ) {
            printf("Invalid PSA D7 = %d (ch. %d), lowest bit must be same as D0\n",fippiconfig.QDCLen1[ch], ch );
            return -7200-ch;  
         }


         // check scaling factor
         if( (fippiconfig.QDC_DIV[ch] == VAL0_QDC_DIV)  || (fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV))  {
            // ok
         } else {
            printf("Invalid PSA scale option = %d (ch. %d), must be %d  or %d \n",fippiconfig.QDCLen4[ch], ch, VAL0_QDC_DIV, VAL1_QDC_DIV);
            return -7300-ch;
         }

         // check threshold
         if( (fippiconfig.PSA_THRESHOLD[ch] > MAX_PSATH) || (fippiconfig.QDCLen5[ch] < 1)  )  {
            printf("Invalid PSA TH = %d (ch. %d), must be between %d and %d \n",fippiconfig.QDCLen5[ch], ch, 1, MAX_PSATH);
            return -7400-ch;
         }


       } // end QDC ena

      // EMIN can be used to cut outputs below a threshold of interest. TODO: use CSR bits to specify where cust occur      
      if( (fippiconfig.EMIN[ch] > 65535) || (fippiconfig.EMIN[ch] < 0)  )  {
         printf("Invalid EMIN = %d, must be between %d and %d \n",fippiconfig.EMIN[ch], 0, 65535);
         return -7700-ch;
      }



 
   }    // end for channels present


   // -------------- now package  and write -------------------

   // CONTROLLER REGISTERS IN MZ
   mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MicroZed Controller

  
  // first, set CSR run control options   
  mapped[AMZ_CSRIN] = 0x0000; // all off)
  mapped[AMZ_RUNCTRL] = 0x0000;    // MCA FIFO disabled

  mval =  fippiconfig.AUX_CTRL  & 0x00FF;  // upper bits reserved (yellow LED)
  mval = mval + 0x0100;     // set bit 8: yellow LED on
  mapped[AAUXCTRL] = mval;
  if(mapped[AAUXCTRL] != mval) printf("Error writing AUX_CTRL register\n");

 /*
    mval = fippiconfig.CLK_CTRL;              // low 2 bits set CLK SEL for PLL input (1=WRclkDB, 0 = FPGA/other)
    mapped[AMZ_PLLSTART] = mval;              // any write will start programming the LMK PLL for ADC and FPGA processing clock                                               
    if( (mval & 0x3) >0)
       printf(" initializing ADC PLL with clock from  WRclkDB\n");
    else
       printf(" initializing ADC PLL with clock from FPGA/other\n");
*/

  for(k7=0;k7<N_K7_FPGAS;k7++)
  {
      mapped[AMZ_DEVICESEL] =  cs[k7];	// select FPGA 


      // SYSTEM REGISTERS IN K7
      // Note: no "module" registers as this is confusing (2 K7 in this "desktop module", no system FPGA)
        
      mapped[AMZ_EXAFWR] = AK7_PAGE;      // specify   K7's addr:    PAGE register
      mapped[AMZ_EXDWR]  = PAGE_SYS;      //  PAGE 0: system, page 0x10n = channel n  
   
      // ................ WR runtime control > system CSR register .............................
      reglo = 0;                          // write 0 to reset
      mapped[AMZ_EXAFWR] = AK7_SCSRIN;    // write to  k7's addr to select register for write
      mapped[AMZ_EXDWR]  = reglo;         // write lower 16 bit
   
      reglo = reglo + setbit(fippiconfig.WR_RUNTIME_CTRL,WRC_RUNTIME, SCSR_WRRUNTIMECTRL   );      // check for bit enabling WR runtime control
      reglo = reglo + setbit(fippiconfig.MODULE_CSRA,MCSRA_P4ERUNSTATS, SCSR_P4ERUNSTATS   );      // check for bit enabling P4e convention for live time etc
      if(fippiconfig.DATA_FLOW==4) reglo = reglo + (1<<SCSR_AUTOUDP);                              // enabling LM UDP output without interaction with C code
      if(fippiconfig.DATA_FLOW==5) reglo = reglo + (1<<SCSR_AUTOQSPI);                             // enabling MCA output (only) to FIFO without interaction with C code
      if(fippiconfig.DATA_FLOW!=5) reglo = reglo + (1<<SCSR_HDRENA);                               // disable header memory in pure MCA runs where ARM only reads E from FIFO
      if(fippiconfig.DATA_FLOW==6) reglo = reglo + (1<<SCSR_DMCONTROL);                            // require DM approval to move data from SDRAM FIFO to WR (Eth out)
      //reglo = reglo + (1<<7);    // enable 10G test
      reglo = reglo + setbit(fippiconfig.MODULE_CSRA,MCSRA_FP_COUNT,  SCSR_FP_COUNT   );        // option to count FP pulses as ext_ts, else local clock (or WR) 
      reglo = reglo + setbit(fippiconfig.MODULE_CSRA,MCSRA_FP_VETO,   SCSR_FP_VETO   );         // option to use FP as VETO
      reglo = reglo + setbit(fippiconfig.MODULE_CSRA,MCSRA_FP_EXTCLR, SCSR_FP_EXTCLR   );       // option to use FP to clear ext_ts  
      reglo = reglo + setbit(fippiconfig.MODULE_CSRA,MCSRA_FP_PEDGE,  SCSR_FP_PEDGE   );        // option to select rising/falling edge for count or clear.  
      mapped[AMZ_EXAFWR] = AK7_SCSRIN;    // write to  k7's addr to select register for write
      mapped[AMZ_EXDWR]  = reglo;        // write lower 16 bit

      // ................ WR Ethernet output settings .............................

      // MAC addresses
      if(k7==0)
         mac = fippiconfig.DEST_MAC0;
      else 
         mac = fippiconfig.DEST_MAC1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_DEST_MAC;     // specify   K7's addr:    WR destination MAC
      mapped[AMZ_EXDWR]  =  mac      & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_DEST_MAC+1;   // specify   K7's addr:    
      mapped[AMZ_EXDWR]  =  (mac>>16) & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_DEST_MAC+2;   // specify   K7's addr:   
      mapped[AMZ_EXDWR]  =  (mac>>32) & 0x00000000FFFF;

      if(k7==0)
         mac = fippiconfig.SRC_MAC0;
      else 
         mac = fippiconfig.SRC_MAC1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_MAC;     // specify   K7's addr:    source MAC (WR uses EEPROM instead)
      mapped[AMZ_EXDWR]  =  mac      & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_MAC+1;   // specify   K7's addr:    
      mapped[AMZ_EXDWR]  =  (mac>>16) & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_MAC+2;   // specify   K7's addr:   
      mapped[AMZ_EXDWR]  =  (mac>>32) & 0x00000000FFFF;
 

      // UDP port numbers
      if(k7==0)
         mval = fippiconfig.DEST_PORT0;
      else 
         mval = fippiconfig.DEST_PORT1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_DEST_PORT;     // specify   K7's addr:    UDP dest port
      mapped[AMZ_EXDWR]  =  mval      & 0xFFFF;
  
      if(k7==0)
         mval = fippiconfig.SRC_PORT0;
      else 
         mval = fippiconfig.SRC_PORT1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_PORT;     // specify   K7's addr:    UDP dest port
      mapped[AMZ_EXDWR]  =  mval      & 0xFFFF;

      // IP addresses
      if(k7==0)
         dip = fippiconfig.DEST_IP0;
      else 
         dip = fippiconfig.DEST_IP1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_DEST_IP;     // specify   K7's addr:    WR destination IP
      mapped[AMZ_EXDWR]  =  dip      & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_DEST_IP+1;   // specify   K7's addr:    
      mapped[AMZ_EXDWR]  =  (dip>>16) & 0x00000000FFFF;
  
      if(k7==0)
         sip = fippiconfig.SRC_IP0;
      else 
         sip = fippiconfig.SRC_IP1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_IP;     // specify   K7's addr:    source IP (should match WR EPROM)
      mapped[AMZ_EXDWR]  =  sip      & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_IP+1;   // specify   K7's addr:    
      mapped[AMZ_EXDWR]  =  (sip>>16) & 0x00000000FFFF;


      // IPv4 checksum computation: SHORT (20 word header only)
      mval = 0;
      mval = mval + 0x4500;              // version etc
      mval = mval + 68;                  // 20 word data header (40bytes), 8bytes UDP header, 20 bytes IPv4 header
      mval = mval + 0;                   // identification
      mval = mval + 0;                   // flags, fragment offset
      mval = mval + 0x3F11;              // time to live (63), protocol UPD (17)
      mval = mval + 0;                   // checksum
      mval = mval + (sip>>16);           // source ip
      mval = mval + (sip & 0xFFFF);      // source ip
      mval = mval + (dip>>16);           // dest ip
      mval = mval + (dip & 0xFFFF);      // dest ip
      mval = (mval&0xFFFF) + (mval>>16); // add accumulated carrys 
      mval = mval + (mval>>16);                // add any more carrys
      reglo = ~mval;      
      mapped[AMZ_EXAFWR] =  AK7_ETH_CHECK_SHORT;     // specify   K7's addr:    checksum (SHORT)
      mapped[AMZ_EXDWR]  =  reglo;
      //printf("WR Ethernet data checksum FPGA %d (SHORT) = 0x%x\n",k7, reglo & 0xFFFF);

      // IPv4 checksum computation: LONG (20 word header plus trace)
      // Note: all channels must have same TL!
      mval = 0;
      mval = mval + 0x4500;              // version etc
      mval = mval + TL[0]*2+68;          // 20 word data header (40bytes), 8bytes UDP header, 20 bytes IPv4 header, 2*TL waveform bytes
      mval = mval + 0;                   // identification
      mval = mval + 0;                   // flags, fragment offset
      mval = mval + 0x3F11;              // time to live (63), protocol UPD (17)
      mval = mval + 0;                   // checksum
      mval = mval + (sip>>16);           // source ip
      mval = mval + (sip & 0xFFFF);      // source ip
      mval = mval + (dip>>16);           // dest ip
      mval = mval + (dip & 0xFFFF);      // dest ip
      mval = (mval&0xFFFF) + (mval>>16); // add accumulated carrys 
      mval = mval + (mval>>16);          // add any more carrys
      reglo = ~mval;      
      mapped[AMZ_EXAFWR] =  AK7_ETH_CHECK_LONG;     // specify   K7's addr:    checksum (LONG)
      mapped[AMZ_EXDWR]  =  reglo;
      //printf("WR Ethernet data checksum FPGA %d (LONG)  = 0x%x\n",k7, reglo & 0xFFFF);

      // event header info (for UDP) 
      mval = 0;
      mval = mval + (fippiconfig.SLOT_ID<<0);         // slot     // use for K7 0/1
      mval = mval + (fippiconfig.CRATE_ID<<4);        // crate
      mval = mval + (P16_HDR_LEN<<8);                 // header length (fixed to 10 32bit words)  
      mapped[AMZ_EXAFWR] =  AK7_HDR_IDS;              // specify   K7's addr:    HDR_IDS
      mapped[AMZ_EXDWR]  =  mval;        
 
      mval=fippiconfig.UDP_PAUSE; //UDP_PAUSE;
      mapped[AMZ_EXAFWR] =  AK7_UDP_PAUSE;              // specify   K7's addr:    AK7_UDP_PAUSE
      mapped[AMZ_EXDWR]  =  mval;      
      if(verbose) printf(" UDP_PAUSE, WR Ethernet minimum packet separation: %d (x 64ns cycles)\n",mval);
      

      // set the Ethernet control register with the trace length (for AutoUDP)
      mval =  ((fippiconfig.CHANNEL_CSRA[0] & (1<<CCSRA_TRACEENA)) >0); // check TraceEna bit   
      mapped[AMZ_EXAFWR] =  AK7_ETH_CTRL;    // specify   K7's addr:    Ethernet output control register
      mapped[AMZ_EXDWR]  =  ( (mval<<8) + (TL[0]>>5) );  // specify payload type with/without trace, TL blocks
      mapped[AMZ_EXAFWR] =  AK7_HOSTCLR;    // specify   K7's addr:    write to clear SDRAM, DEEPFIFO. must be after TL has been defined in AK7_ETH_CTRL
      mapped[AMZ_EXDWR]  =  mval;  // any write ok
   
      // CHANNEL REGISTERS IN K7
      for( ch_k7 = 0; ch_k7 < NCHANNELS_PER_K7 ; ch_k7 ++ )
      {
         ch = ch_k7+k7*NCHANNELS_PER_K7;            // pre-compute channel number for data source  
         mapped[AMZ_EXAFWR] = AK7_PAGE;         // specify   K7's addr:    PAGE register
         mapped[AMZ_EXDWR]  = PAGE_CHN + ch_k7;      // PAGE 0: system, page 0x10n = channel n
 
         
         // ......... P16 Reg 0  .......................            
         
         // package
         reglo = 1;     // halt bit =1
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_POLARITY,      FiPPI_INVRT   );    
         if(ch_k7==2)  {
            //if( ((revsn & PNXL_DB_VARIANT_MASK)==PNXL_DB01_14_125 ) | ((revsn & PNXL_DB_VARIANT_MASK)==PNXL_DB01_14_75) )  // if DB01, ch.2 is inverted   
            if( ((revsn & PNXL_DB_VARIANT_MASK)!=PNXL_DB02_12_250 ) | ((revsn & PNXL_DB_VARIANT_MASK)==PNXL_DB01_14_75) )
            {
               reglo = reglo ^ (1<<FiPPI_INVRT); 
               //printf("reglo_ch.2 0x%08x, revsn 0x%08x\n",reglo, revsn);
            }
         }
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_VETOENA,       FiPPI_VETOENA   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_EXTTRIGSEL,    FiPPI_EXTTRIGSEL   );     
         reglo = reglo + (SFR<<4);                        //  Store SlowFilterRange in bits [6:4] 
         mval = 129-SL[ch];
         //printf("SL: %d, 129-SL: %d,  ",SL[ch], mval);
         reglo = reglo + (mval<<7);                       //  SlowLength in bits [13:7]
         mval = 129-SL[ch]-SG[ch];
         // printf("SL+SG: %d, 129-SL-SG: %d \n",SL[ch]+SG[ch], mval);
         reglo = reglo + (mval<<14);                //  SlowLength + SlowGap in bits [20:14]
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_CHANTRIGSEL,   FiPPI_CHANTRIGSEL   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_SYNCDATAACQ,   FiPPI_SYNCDATAACQ   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRC[ch],CCSRC_GROUPTRIGSEL,  FiPPI_GROUPTRIGSEL   );    
         mval = 129-FL[ch];
         // printf("FL: %d, 129-FL: %d,  ",FL[ch], mval);
         reglo = reglo +( mval<<25) ;               // 128 - (FastLength - 1) in bits [31:25] 
         SAVER0[ch] = reglo;
          
         reghi = 0;
         reghi = 129 - FL[ch] - FG[ch];                          // 128 - (FastLength + FastGap - 1)
         // printf("FL+FG: %d, 129-SL-SG: %d \n",FL[ch]+FG[ch], reghi);
         reghi = reghi & 0x7F;                                 // Keep only bits [6:0]
         reghi = reghi + (TH[ch]<<7);                             // Threshold in [22:7]   
         reghi = reghi + ( (64 - fippiconfig.CFD_DELAY[ch]) <<23 );        //  CFDDelay in [28:23]       // in samples!
         reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC[ch],CCSRC_CHANVETOSEL,   FiPPI_CHANVETOSEL);     
         reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC[ch],CCSRC_MODVETOSEL,    FiPPI_MODVETOSEL );     
         reghi = reghi + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_ENARELAY,      FiPPI_ENARELAY   );     
         
         //printf("Reg 0 high 0x%08X, low 0x%08X \n",reghi, reglo);
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG00+0;                  // write to  k7's addr to select channel's register N
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                  // write lower 16 bit
         mapped[AMZ_EXAFWR] = AK7_P16REG00+1;                  // write to  k7's addr to select channel's register N+1
         mapped[AMZ_EXDWR]  = reglo >> 16;                     // write next 16 bit
         mapped[AMZ_EXAFWR] = AK7_P16REG00+2;                  // write to  k7's addr to select channel's register N+2
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;                  // write next 16 bit
         mapped[AMZ_EXAFWR] = AK7_P16REG00+3;                  // write to  k7's addr to select channel's register N+3
         mapped[AMZ_EXDWR]  = reghi >> 16;                     // write highest 16 bit
         
         
         // ......... P16 Reg 1  .......................    
         
         // package
         reglo = 129 - SG[ch];                                 //  SlowGap in bits [6:0]
         mval = (2*SL[ch]+SG[ch]+1);
         reglo = reglo + (mval<<7);                 // Store RBDEL_SF = (SlowLength + SlowGap + SlowLength + 1) in bits [18:7] of Fipreg1 lo
         mval =  8192 - ((SL[ch]+SG[ch])<<SFR); 
         reglo = reglo + (mval <<19); // Peaksep= SL+SG; store 8192 - PeakSep * 2^SlowFilterRange  in bits [31:19] of Fipreg1 lo
         
         if(SFR==1) PSAM = SL[ch]+SG[ch] -3;
         if(SFR==2) PSAM = SL[ch]+SG[ch] -2;
         if(SFR==3) PSAM = SL[ch]+SG[ch] -2;
         if(SFR==4) PSAM = SL[ch]+SG[ch] -1;
         if(SFR==5) PSAM = SL[ch]+SG[ch] -0;
         if(SFR==6) PSAM = SL[ch]+SG[ch] +1;
         reghi = 0;
         reghi = 8192 - (PSAM<<SFR);                             // Peaksample = SL+SG - a bit ; store 8192 - Peaksample * 2^SlowFilterRange  in bits [44:32] of Fipreg1 
         mval = 2*FL[ch]+FG[ch]+2;
         reghi = reghi + ( mval<<13 );                // Store RBDEL_TF = (FastLength + FastGap + FastLength + 2) in bits [56:45] of Fipreg1
         reghi = reghi + ( fippiconfig.CFD_SCALE[ch] <<25 );        //  Store CFDScale[3:0] in bits [60:57] of Fipreg1
         reghi = reghi + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_GOOD,            FiPPI_GOOD           );     
         reghi = reghi + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_PILEUPCTRL,      FiPPI_PILEUPCTRL     );     
         reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC[ch],CCSRC_INVERSEPILEUP,   FiPPI_INVERSEPILEUP  );     




         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG01+0;                 // write to  k7's addr to select channel's register N    
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                 // write lower 16 bit                                    
         mapped[AMZ_EXAFWR] = AK7_P16REG01+1;                 // write to  k7's addr to select channel's register N+1  
         mapped[AMZ_EXDWR]  = reglo >> 16;                    // write next 16 bit                                     
         mapped[AMZ_EXAFWR] = AK7_P16REG01+2;                 // write to  k7's addr to select channel's register N+2  
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;                 // write next 16 bit                                     
         mapped[AMZ_EXAFWR] = AK7_P16REG01+3;                 // write to  k7's addr to select channel's register N+3  
         mapped[AMZ_EXDWR]  = reghi >> 16;                    // write highest 16 bit                                  
         
         
         // ......... P16 Reg 2  .......................    
         
         // package
         reglo = 4096 - (int)(fippiconfig.CHANTRIG_STRETCH[ch]*FILTER_CLOCK_MHZ);               //  ChanTrigStretch goes into [11:0] of FipReg2   // in us
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_FTRIGSEL,   SelExtFastTrig   );    
         if(traceena) reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_TRACEENA,    FiPPI_TRACEENA   );   // add trace enable bit only in run types with trace capture  
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_QDCENA,      FiPPI_QDCENA   );  
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_CFDMODE,     FiPPI_CFDMODE   );   
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_GLOBTRIG,    FiPPI_GLOBTRIG   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_CHANTRIG,    FiPPI_CHANTRIG   );    
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRC[ch],CCSRC_RBADDIS,     FiPPI_RBADDIS   );     

         mval = 4096 - (int)(fippiconfig.VETO_STRETCH[ch]*FILTER_CLOCK_MHZ);
         reglo = reglo + (mval <<20);       //Store VetoStretch in bits [31:20] of Fipreg2   // in us
         
         reghi = (4096 - (int)(fippiconfig.FASTTRIG_BACKLEN[ch]*FILTER_CLOCK_MHZ)) <<8;            //  FastTrigBackLen goes into [19:8] ([51:40] in 64 bit)   // in us
         reghi = reghi + ( (4096 - (int)(fippiconfig.EXTTRIG_STRETCH[ch]*FILTER_CLOCK_MHZ)) <<20);   //  ExtTrigStretch goes into [31:20] ([63:52] in 64 bit)   // in us
         
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG02+0;                          // write to  k7's addr to select channel's register N    
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                          // write lower 16 bit                                    
         mapped[AMZ_EXAFWR] = AK7_P16REG02+1;                          // write to  k7's addr to select channel's register N+1  
         mapped[AMZ_EXDWR]  = reglo >> 16;                             // write next 16 bit                                     
         mapped[AMZ_EXAFWR] = AK7_P16REG02+2;                          // write to  k7's addr to select channel's register N+2  
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;                          // write next 16 bit                                     
         mapped[AMZ_EXAFWR] = AK7_P16REG02+3;                          // write to  k7's addr to select channel's register N+3  
         mapped[AMZ_EXDWR]  = reghi >> 16;                             // write highest 16 bit                                  
         
         // ......... P16 Reg 5  .......................    
         
         // package
         reglo = (int)(fippiconfig.FTRIGOUT_DELAY[ch]*FILTER_CLOCK_MHZ);        //  FtrigoutDelay goes into [8:0] of FipReg5             // in us                                                                                                    
         
         reghi = (int)(fippiconfig.EXTERN_DELAYLEN[ch]*FILTER_CLOCK_MHZ);       //Store EXTERN_DELAYLEN in bits [8:0] of FipReg5 hi      // in us      
         /*   bogus trace delay computation from C code?
         mval = SL[ch]+SG[ch];                                            // psep       TODO: check trace related delays. 
         mval = (mval-1) << SFR;                                        // trigger delay
         pafl = (mval>> SFR) + TD[ch];                                     // paf length   // check units!
         mval = pafl - mval;                                           //delay from DSP computation
         if( (fippiconfig.CHANNEL_CSRA[ch]  & 0x0400) >0 )  //(1<<CCSRA_CFDMODE) >0  )      
         mval = mval + FL[ch] + FG[ch];                                // add CFD delay if necessary
         */ 
         mval = TD[ch];
         reghi = reghi +  (mval<<9);                                      // trace delay (Capture_FIFOdelaylen )
         
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG05+0;                          // write to  k7's addr to select channel's register N        
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                          // write lower 16 bit                                        
         mapped[AMZ_EXAFWR] = AK7_P16REG05+1;                          // write to  k7's addr to select channel's register N+1      
         mapped[AMZ_EXDWR]  = reglo >> 16;                             // write next 16 bit                                         
         mapped[AMZ_EXAFWR] = AK7_P16REG05+2;                          // write to  k7's addr to select channel's register N+2      
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;                          // write next 16 bit                                         
         mapped[AMZ_EXAFWR] = AK7_P16REG05+3;                          // write to  k7's addr to select channel's register N+3      
         mapped[AMZ_EXDWR]  = reghi >> 16;                             // write highest 16 bit                                      
         
         
         // ......... P16 Reg 6  .......................    
         if(  (fippiconfig.CHANNEL_CSRA[ch] & (1<<CCSRA_QDCENA)) >0 ) {    
            // P16 style QDC
            // TODO: this will require variant switch for other ADC rates
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen0[ch]/2) + 1 ;       // FPGA expects 0x7FFF for a length of 1 x 2 samples per cycle
            reglo = mval;
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen1[ch]/2) + 1 ;
            reglo = reglo + (mval<<16);
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen2[ch]/2) + 1 ;
            reghi = mval;
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen3[ch]/2) + 1 ;
            reghi = reghi + (mval<<16);

         } else {

            if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250) |
                ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB04_14_250) |
                ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250)    )
            {
            // PN style PSA
               reglo = (fippiconfig.QDCLen0[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               mval  =  fippiconfig.QDCLen0[ch]*2 + 6 + fippiconfig.QDCDel0[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reglo = reglo + (mval<<5);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reglo = reglo + (1<<15);  // set "divide by extra 8" bit  

               mval  = (fippiconfig.QDCLen1[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               reglo = reglo + (mval<<16);
               mval  =  fippiconfig.QDCLen1[ch]*2 + 6 + fippiconfig.QDCDel1[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reglo = reglo + (mval<<21);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reglo = reglo + (1<<31);  // set "divide by extra 8" bit  

               reghi = (fippiconfig.QDCLen2[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               mval  =  fippiconfig.QDCLen2[ch]*2 + 6 + fippiconfig.QDCDel2[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reghi = reghi + (mval<<5);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reghi = reghi + (1<<15);  // set "divide by extra 8" bit  

               mval  = (fippiconfig.QDCLen3[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               reghi = reghi + (mval<<16);
               mval  =  fippiconfig.QDCLen3[ch]*2 + 6 + fippiconfig.QDCDel3[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reghi = reghi + (mval<<21);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reghi = reghi + (1<<31);  // set "divide by extra 8" bit     
            } // end revsn

            if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
            {
            // P4e style PSA
            // to be updated for Len+Del variables
               reglo = (fippiconfig.QDCLen0[ch]>>2)+1;                           //  L0 FPGA expects "len/4 + 1" for effective "len" 
               reghi = fippiconfig.QDCLen0[ch] + 4 + fippiconfig.QDCLen2[ch];    //  D0 FPGA expects total length + delay (=end)
               mval  = (fippiconfig.QDCLen1[ch]>>2)+1;
               reglo = reglo + (mval<<16);                                       //  L1 FPGA expects "len/4 + 1" for effective "len"                                                                                
               mval  = fippiconfig.QDCLen1[ch] +4 + fippiconfig.QDCLen3[ch];     //  D1 FPGA expects total length + delay (=end)
               reghi = reghi + (mval<<16);     
            } // end revsn
         }  // end QDC enable
         
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG06+0;              // write to  k7's addr to select channel's register N        
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;              // write lower 16 bit                                        
         mapped[AMZ_EXAFWR] = AK7_P16REG06+1;              // write to  k7's addr to select channel's register N+1      
         mapped[AMZ_EXDWR]  = reglo >> 16;                 // write next 16 bit                                         
         mapped[AMZ_EXAFWR] = AK7_P16REG06+2;              // write to  k7's addr to select channel's register N+2      
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;              // write next 16 bit                                         
         mapped[AMZ_EXAFWR] = AK7_P16REG06+3;              // write to  k7's addr to select channel's register N+3      
         mapped[AMZ_EXDWR]  = reghi >> 16;                 // write highest 16 bit                                      
         
         
         // ......... P16 Reg 7  .......................    
       


          if(  (fippiconfig.CHANNEL_CSRA[ch] & (1<<CCSRA_QDCENA)) >0 ) {    
            // P16 style QDC
            // TODO: this will require variant switch for other ADC rates
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen4[ch]/2) + 1 ;       // FPGA expects 0x7FFF for a length of 1 x 2 samples per cycle
            reglo = mval;
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen5[ch]/2) + 1 ;
            reglo = reglo + (mval<<16);
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen6[ch]/2) + 1 ;
            reghi = mval;
            mval = 0x7FFF - (int)floorf(fippiconfig.QDCLen7[ch]/2) + 1 ;
            reghi = reghi + (mval<<16);
          } else {

            if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250) |
                ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB04_14_250) |
                ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250)    )
            {
            // PN style PSA
               reglo = (fippiconfig.QDCLen4[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               mval  =  fippiconfig.QDCLen4[ch]*2 + 6 + fippiconfig.QDCDel4[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reglo = reglo + (mval<<5);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reglo = reglo + (1<<15);  // set "divide by extra 8" bit  

               mval  = (fippiconfig.QDCLen5[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               reglo = reglo + (mval<<16);
               mval  =  fippiconfig.QDCLen5[ch]*2 + 6 + fippiconfig.QDCDel5[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reglo = reglo + (mval<<21);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reglo = reglo + (1<<31);  // set "divide by extra 8" bit  

               reghi = (fippiconfig.QDCLen6[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               mval  =  fippiconfig.QDCLen6[ch]*2 + 6 + fippiconfig.QDCDel6[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reghi = reghi + (mval<<5);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reghi = reghi + (1<<15);  // set "divide by extra 8" bit  

               mval  = (fippiconfig.QDCLen7[ch]>>1)+1;                              //  FPGA expects "len/2 + 1" for effective "len" 
               reghi = reghi + (mval<<16);
               mval  =  fippiconfig.QDCLen7[ch]*2 + 6 + fippiconfig.QDCDel7[ch]*2;  //  FPGA expects total length + delay (=end), but in 2ns units
               reghi = reghi + (mval<<21);
               if( fippiconfig.QDC_DIV[ch] == VAL1_QDC_DIV)  reghi = reghi + (1<<31);  // set "divide by extra 8" bit     
            } // end revsn

         }  // end QDC enable
        
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG07+0;              // write to  k7's addr to select channel's register N        
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;              // write lower 16 bit                                        
         mapped[AMZ_EXAFWR] = AK7_P16REG07+1;              // write to  k7's addr to select channel's register N+1      
         mapped[AMZ_EXDWR]  = reglo >> 16;                 // write next 16 bit                                         
         mapped[AMZ_EXAFWR] = AK7_P16REG07+2;              // write to  k7's addr to select channel's register N+2      
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;              // write next 16 bit                                         
         mapped[AMZ_EXAFWR] = AK7_P16REG07+3;              // write to  k7's addr to select channel's register N+3      
         mapped[AMZ_EXDWR]  = reghi >> 16;                 // write highest 16 bit                                      
         
         // ......... P16 Reg 13  .......................    
         
         reglo = fippiconfig.CFD_THRESHOLD[ch];                //  CFD Thresh       // in steps
         reglo = reglo + (fippiconfig.PSA_THRESHOLD[ch]<<16);  //  PSA Thresh       // in steps
         
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG13+0;              // write to  k7's addr to select channel's register N
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;              // write lower 16 bit
         mapped[AMZ_EXAFWR] = AK7_P16REG13+1;              // write to  k7's addr to select channel's register N+1
         mapped[AMZ_EXDWR]  = reglo >> 16;                 // write next 16 bit
         
         
         // ......... P16 Reg 17 (Info)  .......................    
         
         reglo = ch;                                      // channel
         reglo = reglo + (fippiconfig.SLOT_ID<<4);           // slot     // use for K7 0/1
         reglo = reglo + (fippiconfig.CRATE_ID<<8);          // crate
         reglo = reglo + (fippiconfig.MODULE_ID<<12);        // module type 
                                                         // [15:20] reserved for mod address (always 0?)                                           
         reghi = TL[ch];                 
         
         // now write 
         mapped[AMZ_EXAFWR] = AK7_P16REG17+0;               // write to  k7's addr to select channel's register N      
         mapped[AMZ_EXDWR]  = reglo & 0xFFFF;               // write lower 16 bit                                      
         mapped[AMZ_EXAFWR] = AK7_P16REG17+1;               // write to  k7's addr to select channel's register N+1    
         mapped[AMZ_EXDWR]  = reglo >> 16;                  // write next 16 bit                                       
         mapped[AMZ_EXAFWR] = AK7_P16REG17+2;               // write to  k7's addr to select channel's register N+2    
         mapped[AMZ_EXDWR]  = reghi & 0xFFFF;               // write next 16 bit                                       
         mapped[AMZ_EXAFWR] = AK7_P16REG17+3;               // write to  k7's addr to select channel's register N+3    
         mapped[AMZ_EXDWR]  = reghi >> 16;                  // write highest 16 bit          
         
         
        // ......... Ecomp Regs  ....................... 

      // Compute Coefficients for E Computation  
      double C0, C1, Cg;
      double dt, elm, q;
      SL[ch] = (int)floorf(fippiconfig.ENERGY_RISETIME[ch] * FILTER_CLOCK_MHZ);
      dt  = 1.0/FILTER_CLOCK_MHZ;
      q   = exp(-1.0*dt/fippiconfig.TAU[ch]);
      elm = exp(-1.0*dt*SL[ch]/fippiconfig.TAU[ch]);
      C0  = (q-1.0)*elm/(1.0-elm);
      Cg  = 1.0-q;
      C1  = (1.0-q)/(1.0-elm);
  //    if(ch==10) printf("dt %f, q %f, elm %f\n", dt, q, elm);    
  //    if(ch==10) printf("E coefs ch %d: C0 %f Cg %f C1  %f\n", ch, C0, Cg, C1);    
      
      C0 = C0 * fippiconfig.DIG_GAIN[ch];   // multiply with digital gain 
      Cg = Cg * fippiconfig.DIG_GAIN[ch];
      C1 = C1 * fippiconfig.DIG_GAIN[ch];

      C0 = C0 * 262144 * (-1.0);    // upshift (*2^18) to have sufficient digits for integer representation [and make positive for FPGA]
      Cg = Cg * 262144; //67108864;
      C1 = C1 * 262144;

  //    if(ch==10) printf("E coefs scaled ch %d: C0  %u Cg %u C1  %u\n", ch, (int)floor(C0), (int)floor(Cg), (int)floor(C1) );    


      // now write 
      // C0 reg included BLavg
      if (fippiconfig.BLAVG[ch]==0)
         mval = 0;
      else
         mval = 65536-fippiconfig.BLAVG[ch];
      reglo = (int)floor(C0) & 0xFFFFFF;
      reglo = reglo + (mval << 24);
      mapped[AMZ_EXAFWR] = AK7_P16REG_C0+0;               // write to  k7's addr to select channel's register N      
      mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                // write lower 16 bit                                      
      mapped[AMZ_EXAFWR] = AK7_P16REG_C0+1;               // write to  k7's addr to select channel's register N+1    
      mapped[AMZ_EXDWR]  = reglo >> 16;                   // write next 16 bit                                       

      reglo = (int)floor(Cg);
      mapped[AMZ_EXAFWR] = AK7_P16REG_CG+0;               // write to  k7's addr to select channel's register N      
      mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                // write lower 16 bit                                      
      mapped[AMZ_EXAFWR] = AK7_P16REG_CG+1;               // write to  k7's addr to select channel's register N+1    
      mapped[AMZ_EXDWR]  = reglo >> 16;                   // write next 16 bit                                       

      // C0 reg includes BLcut
      mval = fippiconfig.BLCUT[ch]>>2;
      reglo = (int)floor(C1) & 0xFFFFFF;
      reglo = reglo + (mval << 24);
      mapped[AMZ_EXAFWR] = AK7_P16REG_C1+0;               // write to  k7's addr to select channel's register N      
      mapped[AMZ_EXDWR]  = reglo & 0xFFFF;                // write lower 16 bit                                      
      mapped[AMZ_EXAFWR] = AK7_P16REG_C1+1;               // write to  k7's addr to select channel's register N+1    
      mapped[AMZ_EXDWR]  = reglo >> 16;                   // write next 16 bit                                       

         
      }  // end for NCHANNEL_PER_K7

    } //end for K7s

  



   // --------------------------- DACs -----------------------------------
   // DACs are all controlled by MZ controller


  // if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)    // DB02 has no DACs
  // {   }

   if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)  |
       ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125) |
       ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250) |      
       ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_14_500)    )
   {
     mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller

     for( ch = 0; ch < NCHANNELS_PER_K7_DB01*N_K7_FPGAS ; ch ++ )    // only 4 DACs per K7 (DB01), not "NCHANNELS_PER_K7"
     {      
         dac = (int)floor( (1 - fippiconfig.VOFFSET[ch]/ V_OFFSET_MAX) * 32768);	
         if(dac > 65535)  {
            printf("Invalid VOFFSET = %f, must be between %f and -%f\n",fippiconfig.VOFFSET[ch], V_OFFSET_MAX-0.05, V_OFFSET_MAX-0.05);
            return -4300-ch;
         }
         mapped[AMZ_FIRSTDAC+ch] = dac;
         if(mapped[AMZ_FIRSTDAC+ch] != dac) printf("Error writing parameters to DAC register\n");
         usleep(DACWAIT);		// wait for programming
         mapped[AMZ_FIRSTDAC+ch] = dac;     // repeat, sometimes doesn't take?
         if(mapped[AMZ_FIRSTDAC+ch] != dac) printf("Error writing parameters to DAC register\n");
         usleep(DACWAIT);     
    //   printf("DAC %d, value 0x%x (%d), [%f V] \n",k, dac, dac,fippiconfig.VOFFSET[k]);
      }     // end for channels DAC
   }        // end most DBs

   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB04_14_250)    // DB04 has I2C DACs
   {
     printf("TODO: implement DB04 DAC programming\n");

     // programming the LTC2655 via DB-specific TWI interface
     // write address + 3 bytes
     // byte 0: command (4 bits) + address (4bits)
     // byte 1/2: 16bit DAC value, MSB first
    
     // I2C write for first 8 channels
     mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller

    for(k7=0;k7<N_K7_FPGAS;k7++)
    {
        if(k7==0)
          mapped[AAUXCTRL] = I2C_SELDB0;	  // select bit 5 -> DB0 I2C        // XXXXXX
        else 
          mapped[AAUXCTRL] = I2C_SELDB1;	  // select bit 6 -> DB1 I2C        // XXXXXX
    
        for( ch_k7 = 0; ch_k7 < NCHANNELS_PER_K7_DB02 ; ch_k7 ++ )    //  8 DACs per K7 (DB02/04), not "NCHANNELS_PER_K7"
        {      
            ch = ch_k7+k7*NCHANNELS_PER_K7_DB02;
            dac = (int)floor( (1 - fippiconfig.VOFFSET[ch]/ V_OFFSET_MAX) * 32768);	
            if(dac > 65535)  {
               printf("Invalid VOFFSET = %f, must be between %f and -%f\n",fippiconfig.VOFFSET[ch], V_OFFSET_MAX-0.05, V_OFFSET_MAX-0.05);
               return -4300-ch;
            }
                     
            I2Cstart(mapped);
      
            // I2C addr byte
            i2cdata[7] = 0;
            i2cdata[6] = 0;
            i2cdata[5] = 1;
            i2cdata[4] = 0;
            i2cdata[3] = 0;                  // A2
            i2cdata[2] = 0;                  // A1
            i2cdata[1] = (ch_k7<4);          // A0       // 2nd DAC chip for upper 4 channels
            i2cdata[0] = 0;                  // R/W*     // no reads possible
            I2Cbytesend(mapped, i2cdata);
            I2Cslaveack(mapped);
         
            // I2C control byte
            i2cdata[7] = 0;                  // C3
            i2cdata[6] = 0;                  // C2
            i2cdata[5] = 1;                  // C1
            i2cdata[4] = 1;                  // C0      // 0011: write to and update channel N (N set by A3-0)
            i2cdata[3] = 0;                  // A3
            i2cdata[2] = 0;                  // A2
            i2cdata[1] = (ch_k7>>1 & 0x01);  // A1
            i2cdata[0] = (ch_k7    & 0x01);  // A0
            I2Cbytesend(mapped, i2cdata);
            I2Cslaveack(mapped);
         
            // I2C data byte
            for( k = 0; k <8; k++ )     // fill i2cdata array with dac bits 
            {
               i2cdata[k] = (dac>>(k+8)) & 0x01;
            }
            I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
            I2Cslaveack(mapped);
      
            // I2C data byte
            for( k = 0; k <8; k++ )    // fill i2cdata array with dac bits 
            {
               i2cdata[k] = (dac>>(k)) & 0x01;
            }
            I2Cbytesend(mapped, i2cdata );      // send same bits again for enable?
            I2Cslaveack(mapped);
         
            I2Cstop(mapped);
   
        }   // end for (N channels)
      } // end for (K7s)

   } // end DB04



  if(1) {
   // --------------------------- Gains ----------------------------------

    // DB06 has 2 gains. Applied via I2C specific to each DB; 
    // Two opamps can be enabled with SW0 (gain 2) and SW1 (gain 5)
    // use 2.4 and 5.4 for easier compatibility to DB01

         /* gain bit map for PXdesk+DB06
        I2C bit   PXdesk DB signal      DB06 gain
         0        IO_DB_5                SW0_A (enable low)
         1        Gain_C                 unused
         2        IO_DB_2                SW0_C (enable low)
         3        Gain_D                 unused
         4        IO_DB_3                SW1_C (enable high)
         5        IO_DB_4                SW0_D (enable low)
         6        IO_DB_N                SW1_D (enable high)
         7        IO_DB_P                unused
         8        IO_DB_0                SW1_A (enable high)
         9        Gain_A                 unused
         10       IO_DB_6                SW1_B (enable high)
         11       IO_DB_1                SW0_B (enable low)
         12       Gain_B                 unused
         13       unused                 unused
         14       unused                 unused
         15       unused                 unused
         
         =>  unsigned int sw0bit[NCHANNELS_PER_K7_DB01] = {0, 11, 2, 5};       // these arrays encode the mapping of gain bits to I2C signals
             unsigned int sw1bit[NCHANNELS_PER_K7_DB01] = {8, 10, 4, 6};
             unsigned int gnbit[NCHANNELS_PER_K7_DB01]  = {9, 12, 1, 3};
       */

    if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250) | ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_14_500) )  {
      for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ )
      {
           if( !( (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)   ) ) {
           printf("ANALOG_GAIN = %f not matching available gains exactly, please choose from this list:\n",fippiconfig.ANALOG_GAIN[ch]);
           printf("    %f \n",DB01_GAIN1);
           printf("    %f \n",DB01_GAIN3);
           return -8000-ch;
         }  // end if
       }    // end for

          // ............. set the bits for 4 channels  ................. 
          for( ch = 0; ch < NCHANNELS_PER_K7_DB01; ch ++ )            // XXXXXX
         {
            ch_k7 = ch;                                         // XXXXXX
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  { i2cgain[sw1bit06[ch_k7]] = 0; i2cgain[sw0bit06[ch_k7]] = 1; i2cgain[gnbit06[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  { i2cgain[sw1bit06[ch_k7]] = 1; i2cgain[sw0bit06[ch_k7]] = 0; i2cgain[gnbit06[ch_k7]] = 0;  }
         }    // end for
    } //end DB

   // DB04 has no gains, just ignore

   // DB02 has no gains, just ignore

   // DB01 has 8 gains. Applied via I2C specific to each DB
   // no limits for DIG_GAIN
   // bit mapping
   
   if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125) | ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75) )  {
      for( ch = 0; ch < NCHANNELS_PRESENT; ch ++ )
      {
           if( !( (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN0)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN2)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN4)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN5)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN6)  ||
                  (fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN7)   ) ) {
           printf("ANALOG_GAIN = %f not matching available gains exactly, please choose from this list:\n",fippiconfig.ANALOG_GAIN[ch]);
           printf("    %f \n",DB01_GAIN0);
           printf("    %f \n",DB01_GAIN1);
           printf("    %f \n",DB01_GAIN2);
           printf("    %f \n",DB01_GAIN3);
           printf("    %f \n",DB01_GAIN4);
           printf("    %f \n",DB01_GAIN5);
           printf("    %f \n",DB01_GAIN6);
           printf("    %f \n",DB01_GAIN7);
           return -8000-ch;
         }  // end if
       }    // end for
   
    /*     (SGA = SW1/SW0/relay)	            gain
					    (0/0/0)                           1.6             // relay off = low gain    (matching P500e)
					    (0/1/0)                           2.4
					    (1/0/0)                           3.5
					    (1/1/0)                           5.4

					    (0/0/1)                           6.7              // relay on = high gain    (matching P500e)
					    (0/1/1)                           9.9
					    (1/0/1)                           14.7
					    (1/1/1)                           22.6	*/

           /* gain bit map for PXdesk+DB01
        I2C bit   PXdesk DB signal      DB01 gain
         0        IO_DB_5                SW0_D (CMOS)
         1        Gain_C                 Gain_C (relay)
         2        IO_DB_2                SW1_B (CMOS)
         3        Gain_D                 Gain_D (relay)
         4        IO_DB_3                SW0_C (CMOS)
         5        IO_DB_4                SW1_C (CMOS)
         6        IO_DB_N                SW0_A (CMOS)
         7        IO_DB_P                unused
         8        IO_DB_0                SW1_A (CMOS)
         9        Gain_A                 Gain_A (relay)
         10       IO_DB_6                SW1_D (CMOS)
         11       IO_DB_1                SW0_B (CMOS)
         12       Gain_B                 Gain_B (relay)
         13       unused                 unused
         14       unused                 unused
         15       unused                 unused
         
         =>  unsigned int sw0bit[NCHANNELS_PER_K7_DB01] = {6, 11, 4, 0};       // these arrays encode the mapping of gain bits to I2C signals
             unsigned int sw1bit[NCHANNELS_PER_K7_DB01] = {8, 2, 5, 10};
             unsigned int gnbit[NCHANNELS_PER_K7_DB01]  = {9, 12, 1, 3};
       */

         // ............. set the bits for the FIRST 4 channels  ................. 
          for( ch = 0; ch < NCHANNELS_PER_K7_DB01; ch ++ )            // XXXXXX
         {
            ch_k7 = ch;                                         // XXXXXX
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN0)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN2)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN4)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN5)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN6)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN7)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 0;  }
      
         }    // end for

       } //end DB

      // I2C write for 4 channels
       mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller
       mapped[AAUXCTRL] = I2C_SELDB0;	  // select bit 5 -> DB0 I2C        // XXXXXX

       // first 8 bits
       I2Cstart(mapped);

      // I2C addr byte
      i2cdata[7] = 0;
      i2cdata[6] = 1;
      i2cdata[5] = 0;
      i2cdata[4] = 0;
      i2cdata[3] = 0;   // A2
      i2cdata[2] = 1;   // A1
      i2cdata[1] = 0;   // A0
      i2cdata[0] = 0;   // R/W*
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      for( k = 0; k <8; k++ )     // NCHANNELS*2 gains, but 8 I2C bits
      {
         i2cdata[k] = i2cgain[k];
      }
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
      I2Cslaveack(mapped);
   
      I2Cstop(mapped);

       
      
      // second 8 bits
       I2Cstart(mapped);

      // I2C addr byte
      i2cdata[7] = 0;
      i2cdata[6] = 1;
      i2cdata[5] = 0;
      i2cdata[4] = 0;
      i2cdata[3] = 1;   // A2
      i2cdata[2] = 0;   // A1
      i2cdata[1] = 0;   // A0
      i2cdata[0] = 0;   // R/W*
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      for( k = 0; k <8; k++ )     // NCHANNELS*2 gains, but 8 I2C bits
      {
         i2cdata[k] = i2cgain[k+8];
      }
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
      I2Cslaveack(mapped);
   
      I2Cstop(mapped);

      // ............. set the bits for 4 MORE  channels  ................. 

      if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250) | ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_14_500) )  {
         for( ch = NCHANNELS_PER_K7_DB01; ch < 2*NCHANNELS_PER_K7_DB01; ch ++ )          // XXXXXX
         {
            ch_k7 = ch - NCHANNELS_PER_K7_DB01;                                         // XXXXXX
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  { i2cgain[sw1bit06[ch_k7]] = 0; i2cgain[sw0bit06[ch_k7]] = 1; i2cgain[gnbit06[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  { i2cgain[sw1bit06[ch_k7]] = 1; i2cgain[sw0bit06[ch_k7]] = 0; i2cgain[gnbit06[ch_k7]] = 0;  }
         }    // end for
      } //end DB

    // DB04 has no gains, just ignore

   // DB02 has no gains, just ignore

             
      if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125) | ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75) )  {
         for( ch = NCHANNELS_PER_K7_DB01; ch < 2*NCHANNELS_PER_K7_DB01; ch ++ )          // XXXXXX
         {
            ch_k7 = ch - NCHANNELS_PER_K7_DB01;                                         // XXXXXX
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN0)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN2)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 1;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN4)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN5)  { i2cgain[sw1bit01[ch_k7]] = 0; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN6)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 0; i2cgain[gnbit01[ch_k7]] = 0;  }
            if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN7)  { i2cgain[sw1bit01[ch_k7]] = 1; i2cgain[sw0bit01[ch_k7]] = 1; i2cgain[gnbit01[ch_k7]] = 0;  }  
         }    // end for
      } //end DB

      // I2C write for 4 channels
       mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller
       mapped[AAUXCTRL] = I2C_SELDB1;	  // select bit 6 -> DB1 I2C        // XXXXXX

       // first 8 bits
       I2Cstart(mapped);

      // I2C addr byte
      i2cdata[7] = 0;
      i2cdata[6] = 1;
      i2cdata[5] = 0;
      i2cdata[4] = 0;
      i2cdata[3] = 0;   // A2
      i2cdata[2] = 1;   // A1
      i2cdata[1] = 0;   // A0
      i2cdata[0] = 0;   // R/W*
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      for( k = 0; k <8; k++ )     // prepare I2C bits
      {
         i2cdata[k] = i2cgain[k];
      }
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
      I2Cslaveack(mapped);
   
      I2Cstop(mapped);

       
      
      // second 8 bits
       I2Cstart(mapped);

      // I2C addr byte
      i2cdata[7] = 0;
      i2cdata[6] = 1;
      i2cdata[5] = 0;
      i2cdata[4] = 0;
      i2cdata[3] = 1;   // A2
      i2cdata[2] = 0;   // A1
      i2cdata[1] = 0;   // A0
      i2cdata[0] = 0;   // R/W*
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      for( k = 0; k <8; k++ )     // prepare I2C bits 
      {
         i2cdata[k] = i2cgain[k+8];
      }
      I2Cbytesend(mapped, i2cdata);
      I2Cslaveack(mapped);
   
      // I2C data byte
      I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
      I2Cslaveack(mapped);
   
      I2Cstop(mapped);

  
     } // end gain turn off


     // --------------------------- finish up ----------------------------------


       
     // TODO
   // restart/initialize filters 
   usleep(100);      // wait for filter FIFOs to clear, really should be longest SL+SG
   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
      mapped[AMZ_DEVICESEL] =  cs[k7];	// select FPGA 

      for( ch_k7 = 0; ch_k7 < NCHANNELS_PER_K7 ; ch_k7 ++ )
      {
         ch = ch_k7+k7*NCHANNELS_PER_K7;            // pre-compute channel number for data source 
         mapped[AMZ_EXAFWR] = AK7_PAGE;         // specify   K7's addr:    PAGE register
         mapped[AMZ_EXDWR]  = PAGE_CHN + ch_k7;      // PAGE 0: system, page 0x10n = channel n

         reglo  = SAVER0[ch];
         mapped[AMZ_EXAFWR] = AK7_P16REG00+0;                  // write to  k7's addr to select channel's register N
         mapped[AMZ_EXDWR]  = reglo & 0xFFFE;                  // write lower 16 bit with bit 0 zerod (halt off)
      }
   }
   usleep(100);      // really should be longest SL+SG
   mapped[ADSP_CLR] = 1;
   mapped[ARTC_CLR] = 1;

      

  mapped[AMZ_DEVICESEL] = CS_MZ;	            // select MicroZed Controller

  mval =  fippiconfig.AUX_CTRL  & 0x00FF;    // upper bits reserved (yellow LED)
  mval = mval + 0x0000;                      // clr bit 8: yellow LED off
  mapped[AAUXCTRL] = mval;
  if(mapped[AAUXCTRL] != mval) printf("Error writing AUX_CTRL register\n");
 

  // toggle nLive to clear memories
   mapped[AMZ_DEVICESEL] = CS_MZ;	      // select MZ
   mapped[AMZ_CSRIN] = 0x0001;            // RunEnable=1 > nLive=0 (DAQ on)
   mapped[AMZ_DEVICESEL] = CS_MZ;	      // select MZ
   mapped[AMZ_CSRIN] = 0x0000;            // RunEnable=1 > nLive=0 (DAQ on)


     
    // --------------------------- HW info ----------------------------------

 
   // ADC board temperature
    printf(" PXdesk board temperature: %d C \n",(int)board_temperature(mapped, I2C_SELMAIN) );
    if(verbose) printf(" DB0 board temperature: %d C \n",(int)board_temperature(mapped, I2C_SELDB0) );
    if(verbose) printf(" DB1 board temperature: %d C \n",(int)board_temperature(mapped, I2C_SELDB1) );

   // ***** ZYNQ temperature
    if(verbose) printf(" MZ Zynq temperature: %d C \n",(int)zynq_temperature() );

   // ***** check HW info *********
   revsn = hwinfo(mapped,I2C_SELMAIN);
   printf(" Main board Revision 0x%04X, Serial Number %d \n",(revsn>>16) & 0xFFFF, revsn & 0xFFFF);
//   if(mval==0) printf("WARNING: HW may be incompatible with this SW/FW \n");

   revsn = hwinfo(mapped,I2C_SELDB0);
   if(verbose) printf(" DB0 Revision 0x%04X\n",(revsn>>16) & 0xFFFF);

   revsn = hwinfo(mapped,I2C_SELDB1);
   if(verbose) printf(" DB1 Revision 0x%04X\n",(revsn>>16) & 0xFFFF);

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










