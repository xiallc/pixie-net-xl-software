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


  
  PixieNetFippiConfig fippiconfig;		// struct holding the input parameters
  const char *defaults_file = "defaults.ini";
  int rval = init_PixieNetFippiConfig_from_file( defaults_file, 0, &fippiconfig );   // first load defaults, do not allow missing parameters
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", defaults_file, rval );
    return rval;
  }
  const char *settings_file = "settings.ini";      // TODO restore to settings.ini
  rval = init_PixieNetFippiConfig_from_file( settings_file, 1, &fippiconfig );   // second override with user settings, do allow missing
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", settings_file, rval );
    return rval;
  }
  

  unsigned int mval, dac, reglo, reghi;
  unsigned int CW, SFR, FFR, SL[NCHANNELS], SG[NCHANNELS], FL[NCHANNELS], FG[NCHANNELS], TH[NCHANNELS];
  unsigned int PSAM, TL[NCHANNELS], TD[NCHANNELS], SAVER0[NCHANNELS];
  unsigned int i2cdata[8];
  unsigned int sw0bit[NCHANNELS_PER_K7_DB01] = {6, 11, 4, 0};       // these arrays encode the mapping of gain bits to I2C signals
  unsigned int sw1bit[NCHANNELS_PER_K7_DB01] = {8, 2, 5, 10};
  unsigned int gnbit[NCHANNELS_PER_K7_DB01] = {9, 12, 1, 3};
  unsigned int i2cgain[16] = {0}; 
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
  int ch, k7, ch_k7;    // loop counter better be signed int  . ch = abs ch. no; ch_k7 = ch. no in k7
  unsigned int revsn, NCHANNELS_PER_K7, NCHANNELS_PRESENT;
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
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB02;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB02;
      ADC_CLK_MHZ       =  ADC_CLK_MHZ_DB02;
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB02;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB02;
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;
      ADC_CLK_MHZ       =  ADC_CLK_MHZ_DB01_125;             
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB02;       // DB1 125 operates at same filter, sys freq as DB02 (and all other DB's, probably)
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB02;
   } 
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)
   {
      NCHANNELS_PRESENT =  NCHANNELS_PRESENT_DB01;
      NCHANNELS_PER_K7  =  NCHANNELS_PER_K7_DB01;
      ADC_CLK_MHZ       =  ADC_CLK_MHZ_DB01_75;             
      SYSTEM_CLOCK_MHZ  =  SYSTEM_CLOCK_MHZ_DB01;
      FILTER_CLOCK_MHZ  =  FILTER_CLOCK_MHZ_DB01;
   }

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
    if(fippiconfig.NUMBER_CHANNELS != NCHANNELS_PRESENT) {
      printf("Invalid NUMBER_CHANNELS = %d, should be %d\n",fippiconfig.NUMBER_CHANNELS,NCHANNELS_PRESENT);
      return -100;
    }

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

   //  WR_RUNTIME_CTRL:
    if(fippiconfig.WR_RUNTIME_CTRL > 1) {
      printf("Invalid WR_RUNTIME_CTRL = 0x%x\n",fippiconfig.WR_RUNTIME_CTRL);
      return -900;
    }

    // WR Ethernet interface:
    // Not checking MAC and IP addresses (e.g. DEST_MAC0) for errors, but report for sanity check with hex numbers
    mac = fippiconfig.DEST_MAC1; 
    printf( " DEST_MAC1 equal to %02llX:%02llX:%02llX:%02llX:%02llX:%02llX\n", 
            (mac>>40) &0xFF,
            (mac>>32) &0xFF,
            (mac>>24) &0xFF,
            (mac>>16) &0xFF,
            (mac>> 8) &0xFF,
            (mac    ) &0xFF) ;
    mac = fippiconfig.DEST_IP1; 
    printf( " DEST_IP1 0x%08llX equal to %lld.%lld.%lld.%lld\n", mac,
            (mac>>24) &0xFF,
            (mac>>16) &0xFF,
            (mac>> 8) &0xFF,
            (mac    ) &0xFF) ;
    mac = fippiconfig.SRC_IP0; 
    printf( " SRC_IP1 0x%08llX equal to %lld.%lld.%lld.%lld\n",mac, 
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
           (fippiconfig.RUN_TYPE == 0x400)   ) ) {
      printf("Invalid RUN_TYPE = 0x%x, please check manual for a list of supported run types\n",fippiconfig.RUN_TYPE);
      return -1200;
    }

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
      if(FG[ch] < MIN_FL) {
         printf("Invalid TRIGGER_FLATTOP = %f, minimum %f us\n",fippiconfig.TRIGGER_FLATTOP[ch],(double)(MIN_FG<<FFR/FILTER_CLOCK_MHZ));
         return -3600-ch;
      } 
      if( (FL[ch]+FG[ch]) > MAX_FLFG) {
         printf("Invalid combined trigger filter, maximum %f us\n",(double)(MAX_FLFG<FFR/FILTER_CLOCK_MHZ));
         return -3700-ch;
      } 
      
      TH[ch] = (int)floor(fippiconfig.TRIGGER_THRESHOLD[ch]*FL[ch]);
      if(TH[ch] > MAX_TH)     {
         printf("Invalid TRIGGER_THRESHOLD = %f, maximum %f at this trigger filter rise time\n",fippiconfig.TRIGGER_THRESHOLD[ch],MAX_TH*8.0/(double)FL[ch]);
         return -3800-ch;
      } 

      // THRESH_WIDTH   -- not implemented for a long time

      // waveforms
      TL[ch] = MULT_TL*(int)floor(fippiconfig.TRACE_LENGTH[ch]*ADC_CLK_MHZ/MULT_TL/(1<<FFR) );       // multiply time in us *  # ticks per us = time in ticks; multiple of MULT_TL
      if( (TL[ch] > MAX_TL) | (TL[ch] < MULT_TL) )  { //|| TL[ch] < TRACELEN_MIN_250OR100MHZADC)  {
         printf("Invalid TRACE_LENGTH = %f, must be between %f and %f us\n",fippiconfig.TRACE_LENGTH[ch],(double)MULT_TL/ADC_CLK_MHZ,(double)MAX_TL/ADC_CLK_MHZ);
        // printf("Invalid TRACE_LENGTH = %f, must be between %f and %f us\n",fippiconfig.TRACE_LENGTH[ch],(double)TRACELEN_MIN_250OR100MHZADC/ADC_CLK_MHZ,(double)MAX_TL/ADC_CLK_MHZ);
         return -4000-ch;
         // Note: enfore TL >32 which is the minimum for 0x400 if traces are recorded. For no trace, use CCSRA bit to disable trace
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
           
      // no limit on BLCUT

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

  mval =  fippiconfig.AUX_CTRL  & 0x00FF;  // upper bits reserved (yellow LED)
  mval = mval + 0x0100;     // set bit 8: yellow LED on
  mapped[AAUXCTRL] = mval;
  if(mapped[AAUXCTRL] != mval) printf("Error writing AUX_CTRL register\n");


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
      mapped[AMZ_EXAFWR] = AK7_SCSRIN;    // write to  k7's addr to select register for write
      mapped[AMZ_EXDWR]  = reglo;        // write lower 16 bit
   

      // ................ WR Ethernet output settings .............................
      if(k7==0)
         mac = fippiconfig.DEST_MAC0;
      else 
         mac = fippiconfig.DEST_MAC1;
      mapped[AMZ_EXAFWR] =  AK7_ETH_MAC;     // specify   K7's addr:    WR destination MAC
      mapped[AMZ_EXDWR]  =  mac      & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_MAC+1;   // specify   K7's addr:    
      mapped[AMZ_EXDWR]  =  (mac>>16) & 0x00000000FFFF;
      mapped[AMZ_EXAFWR] =  AK7_ETH_MAC+2;   // specify   K7's addr:   
      mapped[AMZ_EXDWR]  =  (mac>>32) & 0x00000000FFFF;
 
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
      mapped[AMZ_EXAFWR] =  AK7_ETH_SRC_IP;     // specify   K7's addr:    WR source IP (matching EPROM)
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
      mapped[AMZ_EXDWR]  =  0xF7B9; //reglo;
      printf("WR Ethernet data checksum FPGA %d (SHORT) = 0x%x\n",k7, reglo & 0xFFFF);

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
      mapped[AMZ_EXDWR]  =  0xF7B9; //reglo;
      printf("WR Ethernet data checksum FPGA %d (LONG)  = 0x%x\n",k7, reglo & 0xFFFF);

 
   
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
         if( (ch_k7==2) && (revsn & PNXL_DB_VARIANT_MASK)!=PNXL_DB02_12_250 )  // if DB01, ch.2 is inverted   
         {
            reglo = reglo ^ (1<<FiPPI_INVRT); 
            //printf("reglo_ch.2 0x%08x, revsn 0x%08x\n",reglo, revsn);
         }
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_VETOENA,       FiPPI_VETOENA   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_EXTTRIGSEL,    FiPPI_EXTTRIGSEL   );     
         reglo = reglo + (SFR<<4);                        //  Store SlowFilterRange in bits [6:4] 
         mval = 129-SL[ch];
         reglo = reglo + (mval<<7);                       //  SlowLength in bits [13:7]
         mval = 129-SL[ch]-SG[ch];
         reglo = reglo + (mval<<14);                //  SlowLength + SlowGap in bits [20:14]
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_CHANTRIGSEL,   FiPPI_CHANTRIGSEL   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_SYNCDATAACQ,   FiPPI_SYNCDATAACQ   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRC[ch],CCSRC_GROUPTRIGSEL,  FiPPI_GROUPTRIGSEL   );    
         mval = 129-FL[ch]-FG[ch];
         reglo = reglo +( mval<<25) ;               // 128 - (FastLength - 1) in bits [31:25] 
         SAVER0[ch] = reglo;
          
         reghi = 0;
         reghi = 129 - FL[ch] - FG[ch];                          // 128 - (FastLength + FastGap - 1)
         reghi = reghi & 0x7F;                                 // Keep only bits [6:0]
         reghi = reghi + (TH[ch]<<7);                             // Threshold in [22:7]   
         reghi = reghi + (64 - (fippiconfig.CFD_DELAY[ch] <<23) );        //  CFDDelay in [28:23]       // in samples!
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
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_TRACEENA,    13   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_CFDMODE,     15   );   
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_QDCENA,      14   );  
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_GLOBTRIG,    16   );     
         reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[ch],CCSRA_CHANTRIG,    17   );     
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
         
         reglo = fippiconfig.QDCLen0[ch];                    //  QDC       // in samples
         reglo = reglo + (fippiconfig.QDCLen1[ch]<<16);        //  QDC       // in samples                                                                                 
         reghi = fippiconfig.QDCLen2[ch];                    //  QDC       // in samples
         reghi = reghi + (fippiconfig.QDCLen3[ch]<<16);        //  QDC       // in samples      
         
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
         
         reglo = fippiconfig.QDCLen4[ch];                    //  QDC       // in samples
         reglo = reglo + (fippiconfig.QDCLen5[ch]<<16);        //  QDC       // in samples                                                                               
         reghi = fippiconfig.QDCLen6[ch];                    //  QDC       // in samples
         reghi = reghi + (fippiconfig.QDCLen7[ch]<<16);        //  QDC       // in samples     
         
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
         
         reglo = fippiconfig.CFD_THRESHOLD[ch];             //  CFDThresh       // in steps
         
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
         
      }  // end for NCHANNEL_PER_K7

    } //end for K7s

  



   // --------------------------- DACs -----------------------------------
   // DACs are all controlled by MZ controller

    mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller

  // for( k = NCHANNELS_PRESENT-1; k >= 0 ; k ++ )         // DAC registers are in reverse order
   for( ch = 0; ch < NCHANNELS_PRESENT  ; ch ++ )         // DAC registers are in reverse order
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
 //     printf("DAC %d, value 0x%x (%d), [%f V] \n",k, dac, dac,fippiconfig.VOFFSET[k]);
   }           // end for channels DAC


  if(1) {
   // --------------------------- Gains ----------------------------------
   // DB01 has 4 gains. Applied via I2C specific to each DB
   // no limits for DIG_GAIN
   // bit mapping
   
  
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

       // ............. set the bits for 4 channels  ................. 
       for( ch = 0; ch < NCHANNELS_PER_K7_DB01; ch ++ )            // XXXXXX
      {
         ch_k7 = ch;                                         // XXXXXX
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN0)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN2)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN4)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN5)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN6)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN7)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 1;  }
   
      }    // end for

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

             // ............. set the bits for 4 more  channels  ................. 
       for( ch = NCHANNELS_PER_K7_DB01; ch < 2*NCHANNELS_PER_K7_DB01; ch ++ )          // XXXXXX
      {
         ch_k7 = ch - NCHANNELS_PER_K7_DB01;                                         // XXXXXX
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN0)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN1)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN2)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN3)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN4)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN5)  { i2cgain[sw1bit[ch_k7]] = 0; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN6)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 0; i2cgain[gnbit[ch_k7]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[ch] == DB01_GAIN7)  { i2cgain[sw1bit[ch_k7]] = 1; i2cgain[sw0bit[ch_k7]] = 1; i2cgain[gnbit[ch_k7]] = 1;  }
   
      }    // end for

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
    printf("PXdesk board temperature: %d C \n",(int)board_temperature(mapped, I2C_SELMAIN) );
    printf("DB0 board temperature: %d C \n",(int)board_temperature(mapped, I2C_SELDB0) );
    printf("DB1 board temperature: %d C \n",(int)board_temperature(mapped, I2C_SELDB1) );

   // ***** ZYNQ temperature
     printf("MZ Zynq temperature: %d C \n",(int)zynq_temperature() );

   // ***** check HW info *********
   revsn = hwinfo(mapped,I2C_SELMAIN);
   printf("Main board Revision 0x%04X, Serial Number %d \n",(revsn>>16) & 0xFFFF, revsn & 0xFFFF);
//   if(mval==0) printf("WARNING: HW may be incompatible with this SW/FW \n");

   revsn = hwinfo(mapped,I2C_SELDB0);
   printf("DB0 Revision 0x%04X\n",(revsn>>16) & 0xFFFF);

   revsn = hwinfo(mapped,I2C_SELDB1);
   printf("DB1 Revision 0x%04X\n",(revsn>>16) & 0xFFFF);

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










