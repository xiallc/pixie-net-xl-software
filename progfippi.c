/*----------------------------------------------------------------------
 * Copyright (c) 2018 XIA LLC
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
  int k, m;


  // ******************* read ini file and fill struct with values ********************


  
  PixieNetFippiConfig fippiconfig;		// struct holding the input parameters
  const char *defaults_file = "defaults.ini";
  int rval = init_PixieNetFippiConfig_from_file( defaults_file, 0, &fippiconfig );   // first load defaults, do not allow missing parameters
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", defaults_file, rval );
    return rval;
  }
  const char *settings_file = "defaults.ini";      // TODO restore to settings.ini
  rval = init_PixieNetFippiConfig_from_file( settings_file, 1, &fippiconfig );   // second override with user settings, do allow missing
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", settings_file, rval );
    return rval;
  }
  

  unsigned int  mval, dac, reglo, reghi, pafl;
  unsigned int CW, SFR, FFR, SL[NCHANNELS], SG[NCHANNELS], FL[NCHANNELS], FG[NCHANNELS], TH[NCHANNELS];
  unsigned int PSAM, TL[NCHANNELS], TD[NCHANNELS];
  unsigned int i2cdata[8];
  unsigned int sw0bit[NCHANNEL_PER_K7] = {6, 11, 4, 0};       // these arrays encode the mapping of gain bits to I2C signals
  unsigned int sw1bit[NCHANNEL_PER_K7] = {8, 2, 5, 10};
  unsigned int gnbit[NCHANNEL_PER_K7] = {9, 12, 1, 3};
  unsigned int i2cgain[16] = {0}; 


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




  // ******************* XIA code begins ********************



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
  //    return -400;
    }

   if(fippiconfig.CRATE_ID > MAX_CRATE_ID) {
      printf("Invalid CRATE_ID = %d, must be < %d\n",fippiconfig.CRATE_ID,MAX_CRATE_ID);
      return -400;
    }

       if(fippiconfig.SLOT_ID > MAX_SLOT_ID) {
      printf("Invalid SLOT_ID = %d, must be < %d\n",fippiconfig.SLOT_ID,MAX_SLOT_ID);
      return -400;
    }

       if(fippiconfig.MODULE_ID > MAX_MODULE_ID) {
       // todo: check for valid module type numbers
      printf("Invalid CRATE_ID = %d, must be < %d\n",fippiconfig.MODULE_ID,MAX_MODULE_ID);
      return -400;
    }

   // AUX CTRL:
    if(fippiconfig.AUX_CTRL > 65535) {
      printf("Invalid AUX_CTRL = 0x%x\n",fippiconfig.AUX_CTRL);
      return -2700;
    }

  
  // ********** MODULE PARAMETERS ******************

    //MODULE_CSRA     -- P16 trigger backplane functions: unused for now
    if(fippiconfig.MODULE_CSRA > 65535) {
      printf("Invalid MODULE_CSRA = 0x%x\n",fippiconfig.MODULE_CSRA);
      return -1700;
    }

    //MODULE_CSRB     -- P16 trigger backplane functions: unused for now
    if(fippiconfig.MODULE_CSRB > 65535) {
      printf("Invalid MODULE_CSRB = 0x%x\n",fippiconfig.MODULE_CSRB);
      return -1800;
    }

    // RUN_TYPE     -- not written to FPGA registers
    if( !( (fippiconfig.RUN_TYPE == 0x301)  ||
           (fippiconfig.RUN_TYPE == 0x100)  ||
           (fippiconfig.RUN_TYPE == 0x101)  ||
           (fippiconfig.RUN_TYPE == 0x102)  ||
           (fippiconfig.RUN_TYPE == 0x103)   ) ) {
      printf("Invalid RUN_TYPE = 0x%x, please check manual for a list of supported run types\n",fippiconfig.RUN_TYPE);
      return -2100;
    }

    //MAX_EVENTS      -- not written to FPGA registers
    if(fippiconfig.MAX_EVENTS > 65535) {
      printf("Invalid MAX_EVENTS = 0x%x\n",fippiconfig.MAX_EVENTS);
      return -1700;
    }

    // COINCIDENCE_PATTERN -- unused P16
    if(fippiconfig.COINCIDENCE_PATTERN > 65535) {
      printf("Invalid COINCIDENCE_PATTERN = 0x%x\n",fippiconfig.COINCIDENCE_PATTERN);
      return -1700;
    }

    // COINCIDENCE_WINDOW   -- unused P16?
    CW = (int)floorf(fippiconfig.COINCIDENCE_WINDOW*SYSTEM_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
    if( (CW > MAX_CW) | (CW < MIN_CW) ) {
      printf("Invalid COINCIDENCE_WINDOW = %f, must be between %f and %f us\n",fippiconfig.COINCIDENCE_WINDOW, (double)MIN_CW/(double)SYSTEM_CLOCK_MHZ, (double)MAX_CW/(double)SYSTEM_CLOCK_MHZ);
      return -2000;
    }

     // SYNC_AT_START      --  written to FPGA registers XXX  ?
    if(fippiconfig.SYNC_AT_START >1) {
      printf("Invalid SYNC_AT_START = %d, can only be 0 and 1\n",fippiconfig.SYNC_AT_START);
      return -2400;
    }

     // RESUME        -- not written to FPGA registers
    if(fippiconfig.RESUME >1) {
      printf("Invalid RESUME = %d, can only be 0 and 1\n",fippiconfig.RESUME);
      return -2400;
    }
 

    // SLOW_FILTER_RANGE      --  written to FPGA registers below (with energy filter)
    SFR = fippiconfig.SLOW_FILTER_RANGE;
    if( (SFR > MAX_SFR) | (SFR < MIN_SFR) ) {
      printf("Invalid SLOW_FILTER_RANGE = %d, must be between %d and %d\n",SFR,MIN_SFR, MAX_SFR);
      return -2200;
    }

     // FAST_FILTER_RANGE   -- not implemented for now
    FFR = fippiconfig.FAST_FILTER_RANGE;
    if( (FFR > MAX_FFR) | (FFR < MIN_FFR) ) {
      printf("Invalid FAST_FILTER_RANGE = %d, must be between %d and %d\n",FFR,MIN_FFR, MAX_FFR);
      return -2200;
    }

     // FASTTRIG_BACKPLANEENA -- not implemented for now

     // TRIG_CONFIG# -- not implemented for now



  // ********** CHANNEL PARAMETERS ******************
   

  // ----- first, check limits -----------------
  
  for( k = 0; k < NCHANNELS_PRESENT; k ++ )
  {
      // CCSRA-C (B not used in P16)
      if(fippiconfig.CHANNEL_CSRA[k] > 65535) {
         printf("Invalid CHANNEL_CSRA = 0x%x\n",fippiconfig.CHANNEL_CSRA[k]);
         return -3300-k;
      } 
      if(fippiconfig.CHANNEL_CSRC[k] > 65535) {
         printf("Invalid CHANNEL_CSRC = 0x%x\n",fippiconfig.CHANNEL_CSRC[k]);
         return -3500-k;
      }  

      // gain and offset handled separately below

      // energy filter
      SL[k] = (int)floorf(fippiconfig.ENERGY_RISETIME[k] * FILTER_CLOCK_MHZ);
      SL[k] = SL[k] >> SFR;
      if(SL[k] < MIN_SL) {
         printf("Invalid ENERGY_RISETIME = %f, minimum %f us at this filter range\n",fippiconfig.ENERGY_RISETIME[k],(double)((MIN_SL<<SFR)/FILTER_CLOCK_MHZ));
         return -3600-k;
      } 
      SG[k] = (int)floorf(fippiconfig.ENERGY_FLATTOP[k] * FILTER_CLOCK_MHZ);
      SG[k] = SG[k] >> SFR;
      if(SG[k] < MIN_SG) {
         printf("Invalid ENERGY_FLATTOP = %f, minimum %f us at this filter range\n",fippiconfig.ENERGY_FLATTOP[k],(double)((MIN_SG<<SFR)/FILTER_CLOCK_MHZ));
         return -3700-k;
      } 
      if( (SL[k]+SG[k]) > MAX_SLSG) {
         printf("Invalid combined energy filter, maximum %f us at this filter range\n",(double)((MAX_SLSG<<SFR)/FILTER_CLOCK_MHZ));
         return -3700-k;
      } 

  // trigger filter 
      FL[k] = (int)floorf(fippiconfig.TRIGGER_RISETIME[k] * FILTER_CLOCK_MHZ);
      FL[k] = FL[k] >> FFR;
      if(FL[k] < MIN_FL) {
         printf("Invalid TRIGGER_RISETIME = %f, minimum %f us\n",fippiconfig.TRIGGER_RISETIME[k],(double)(MIN_FL<<FFR/FILTER_CLOCK_MHZ));
         return -3800-k;
      } 
      FG[k] = (int)floorf(fippiconfig.TRIGGER_FLATTOP[k] * FILTER_CLOCK_MHZ);
      FG[k] = FG[k] >> FFR;
      if(FG[k] < MIN_FL) {
         printf("Invalid TRIGGER_FLATTOP = %f, minimum %f us\n",fippiconfig.TRIGGER_FLATTOP[k],(double)(MIN_FG<<FFR/FILTER_CLOCK_MHZ));
         return -3900-k;
      } 
      if( (FL[k]+FG[k]) > MAX_FLFG) {
         printf("Invalid combined trigger filter, maximum %f us\n",(double)(MAX_FLFG<FFR/FILTER_CLOCK_MHZ));
         return -3900-k;
      } 
      
      TH[k] = (int)floor(fippiconfig.TRIGGER_THRESHOLD[k]*FL[k]);
      if(TH[k] > MAX_TH)     {
         printf("Invalid TRIGGER_THRESHOLD = %f, maximum %f at this trigger filter rise time\n",fippiconfig.TRIGGER_THRESHOLD[k],MAX_TH*8.0/(double)FL[k]);
         return -4000-k;
      } 

      // THRESH_WIDTH   -- not implemented for a long time

      // waveforms
      TL[k] = MULT_TL*(int)floor(fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/MULT_TL/(1<<FFR) );       // multiply time in us *  # ticks per us = time in ticks; multiple of MULT_TL
      if(TL[k] > MAX_TL || TL[k] < TRACELEN_MIN_250OR100MHZADC)  {
         printf("Invalid TRACE_LENGTH = %f, must be between %f and %f us\n",fippiconfig.TRACE_LENGTH[k],(double)TRACELEN_MIN_250OR100MHZADC/ADC_CLK_MHZ,(double)MAX_TL/ADC_CLK_MHZ);
         return -4400-k;
      }

      if(TL[k] <fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/(1<<FFR))  {
         printf("TRACE_LENGTH[%d] will be rounded off to = %f us, %d samples\n",k,(double)TL[k]/ADC_CLK_MHZ,TL[k]);
      }
      TD[k] = (int)floor(fippiconfig.TRACE_DELAY[k]*ADC_CLK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
      if(TD[k] > MAX_TL-TWEAK_UD)  {
         printf("Invalid TRACE_DELAY = %f, maximum %f us\n",fippiconfig.TRACE_DELAY[k],(double)(MAX_TL-TWEAK_UD)/ADC_CLK_MHZ);
         return -4500-k;
      }
      if(TD[k] > TRACEDELAY_MAX)  {
         printf("Invalid TRACE_DELAY = %f, maximum %f us\n",fippiconfig.TRACE_DELAY[k],(double)(TRACEDELAY_MAX)/ADC_CLK_MHZ);
         return -4500-k;
      }

      // other parameters
      if(fippiconfig.BINFACTOR[k] > MAX_BFACT)     {
         printf("Invalid BINFACTOR = %d, maximum %d\n",fippiconfig.BINFACTOR[k],MAX_BFACT);
         return -4800-k;
      } 

      if(fippiconfig.INTEGRATOR[k] > 0)     {
         printf("Invalid INTEGRATOR = %d, currently not implemented\n",fippiconfig.INTEGRATOR[k]);
         return -5400-k;
      } 
           
      // no limit on BLCUT

      if(fippiconfig.BASELINE_PERCENT[k] <1 || fippiconfig.BASELINE_PERCENT[k] >99 )  {
         printf("Invalid BASELINE_PERCENT = %f, must be between 1 and 99\n",fippiconfig.BASELINE_PERCENT[k]);
         return -5100-k;
      }

      if(fippiconfig.BLAVG[k] > 65535)     {
         printf("Invalid BLAVG = %d, maximum %d\n",fippiconfig.BLAVG[k],65535);
         return -5800-k;
      } 
      if( (fippiconfig.BLAVG[k] >0) && (fippiconfig.BLAVG[k] < 65535-MAX_BLAVG ) )     {
         printf("Invalid BLAVG = %d, minimum %d (or zero to turn off)\n",fippiconfig.BLAVG[k],65535-MAX_BLAVG);
         return -5800-k;
      } 

      if(fippiconfig.TAU[k] <0)  {
         printf("Invalid TAU = %f, must be positive\n",fippiconfig.TAU[k]);
         return -4900-k;
      }

      if(fippiconfig.XDT[k] < (MIN_XDT_MOST/ADC_CLK_MHZ) ) {
         printf("Invalid XDT = %f us, must be at least %f us\n",fippiconfig.XDT[k],(double)MIN_XDT_MOST/ADC_CLK_MHZ );
         return -4900-k;
      }
      mval = (int)floor( fippiconfig.XDT[k]*ADC_CLK_MHZ/MIN_XDT_MOST) *MIN_XDT_MOST;
      if(fippiconfig.XDT[k] > mval)     {
         printf("XDT = %f, will be rounded to %d \n",fippiconfig.XDT[k],mval );
         //return -4900-k;
      }

      // MULTIPLICITY_MASKL  -- not implemented for now


      if(fippiconfig.FASTTRIG_BACKLEN[k] < FASTTRIGBACKLEN_MIN_100MHZFIPCLK/FILTER_CLOCK_MHZ)  {
         printf("Invalid FASTTRIG_BACKLEN = %f, must be at least %f us\n",fippiconfig.FASTTRIG_BACKLEN[k], (double)FASTTRIGBACKLEN_MIN_100MHZFIPCLK/FILTER_CLOCK_MHZ);
         return -4900-k;
      }
      if(fippiconfig.FASTTRIG_BACKLEN[k] > FASTTRIGBACKLEN_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid FASTTRIG_BACKLEN = %f, must be less than %f us\n",fippiconfig.FASTTRIG_BACKLEN[k], (double)FASTTRIGBACKLEN_MAX/FILTER_CLOCK_MHZ);
         return -4900-k;
      }

      // CFD parameters specified in samples, not us as in P16!
      if(fippiconfig.CFD_THRESHOLD[k] < CFDTHRESH_MIN)  {
         printf("Invalid CFD_THRESHOLD = %d, must be at least %d \n",fippiconfig.CFD_THRESHOLD[k], CFDTHRESH_MIN);
         return -4900-k;
      }
      if(fippiconfig.CFD_THRESHOLD[k] > CFDTHRESH_MAX)  {
         printf("Invalid CFD_THRESHOLD = %d, must be less than %d \n",fippiconfig.CFD_THRESHOLD[k], CFDTHRESH_MAX);
         return -4900-k;
      }

      if(fippiconfig.CFD_DELAY[k] < CFDDELAY_MIN)  {
         printf("Invalid CFD_DELAY = %d, must be at least %d samples\n",fippiconfig.CFD_DELAY[k], CFDDELAY_MIN);
         return -4900-k;
      }
      if(fippiconfig.CFD_DELAY[k] > CFDDELAY_MAX)  {
         printf("Invalid CFD_DELAY = %d, must be less than %d samples\n",fippiconfig.CFD_DELAY[k], CFDDELAY_MAX);
         return -4900-k;
      }

      if(fippiconfig.CFD_SCALE[k] < CFDSCALE_MIN)  {
         printf("Invalid CFD_SCALE = %d, must be at least %d samples\n",fippiconfig.CFD_SCALE[k], CFDSCALE_MIN);
         return -4900-k;
      }
      if(fippiconfig.CFD_SCALE[k] > CFDSCALE_MAX)  {
         printf("Invalid CFD_SCALE = %d, must be less than %d samples\n",fippiconfig.CFD_SCALE[k], CFDSCALE_MAX);
         return -4900-k;
      }

      // stretches and delays ...
      if(fippiconfig.EXTTRIG_STRETCH[k] < EXTTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid EXTTRIG_STRETCH = %f, must be at least %f us\n",fippiconfig.EXTTRIG_STRETCH[k], (double)EXTTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ);
         return -4900-k;
      }
      if(fippiconfig.EXTTRIG_STRETCH[k] > EXTTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid EXTTRIG_STRETCH = %f, must be less than %f us\n",fippiconfig.EXTTRIG_STRETCH[k], (double)EXTTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ);
         return -4900-k;
      }

      if(fippiconfig.VETO_STRETCH[k] < VETOSTRETCH_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid VETO_STRETCH = %f, must be at least %f us\n",fippiconfig.VETO_STRETCH[k], (double)VETOSTRETCH_MIN/FILTER_CLOCK_MHZ);
         return -4900-k;
      }
      if(fippiconfig.VETO_STRETCH[k] > VETOSTRETCH_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid VETO_STRETCH = %f, must be less than %f us\n",fippiconfig.VETO_STRETCH[k], (double)VETOSTRETCH_MAX/FILTER_CLOCK_MHZ);
         return -4900-k;
      }

      if(fippiconfig.EXTERN_DELAYLEN[k] < EXTDELAYLEN_MIN/FILTER_CLOCK_MHZ)  {                           
         printf("Invalid EXTERN_DELAYLEN = %f, must be at least %f us\n",fippiconfig.EXTERN_DELAYLEN[k], (double)EXTDELAYLEN_MIN/FILTER_CLOCK_MHZ);
         return -4900-k;
      }
      if(fippiconfig.EXTERN_DELAYLEN[k] > EXTDELAYLEN_MAX_REVF/FILTER_CLOCK_MHZ)  {
         printf("Invalid EXTERN_DELAYLEN = %f, must be less than %f us\n",fippiconfig.EXTERN_DELAYLEN[k], (double)EXTDELAYLEN_MAX_REVF/FILTER_CLOCK_MHZ);
         return -4900-k;
      }  
    
      if(fippiconfig.FTRIGOUT_DELAY[k] < FASTTRIGBACKDELAY_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid FTRIGOUT_DELAY = %f, must be at least %f us\n",fippiconfig.FTRIGOUT_DELAY[k], (double)FASTTRIGBACKDELAY_MIN/FILTER_CLOCK_MHZ);
         return -4900-k;
      }
      if(fippiconfig.FTRIGOUT_DELAY[k] > FASTTRIGBACKDELAY_MAX_REVF/FILTER_CLOCK_MHZ)  {
         printf("Invalid FTRIGOUT_DELAY = %f, must be less than %f us\n",fippiconfig.FTRIGOUT_DELAY[k], (double)FASTTRIGBACKDELAY_MAX_REVF/FILTER_CLOCK_MHZ);
         return -4900-k;
      }      

      if(fippiconfig.CHANTRIG_STRETCH[k] < CHANTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ)  {
         printf("Invalid CHANTRIG_STRETCH = %f, must be at least %f us\n",fippiconfig.CHANTRIG_STRETCH[k], (double)CHANTRIGSTRETCH_MIN/FILTER_CLOCK_MHZ);
         return -4900-k;
      }
      if(fippiconfig.CHANTRIG_STRETCH[k] > CHANTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ)  {
         printf("Invalid CHANTRIG_STRETCH = %f, must be less than %f us\n",fippiconfig.CHANTRIG_STRETCH[k], (double)CHANTRIGSTRETCH_MAX/FILTER_CLOCK_MHZ);
         return -4900-k;
      } 


      // QDC parameters specified in samples, not as as in P16!
      if( (fippiconfig.QDCLen0[k] > QDCLEN_MAX) || (fippiconfig.QDCLen0[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen0 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen0[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen1[k] > QDCLEN_MAX) || (fippiconfig.QDCLen1[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen1 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen1[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen2[k] > QDCLEN_MAX) || (fippiconfig.QDCLen2[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen2 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen2[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen3[k] > QDCLEN_MAX) || (fippiconfig.QDCLen3[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen3 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen3[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen4[k] > QDCLEN_MAX) || (fippiconfig.QDCLen4[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen4 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen4[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen5[k] > QDCLEN_MAX) || (fippiconfig.QDCLen5[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen5 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen5[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen6[k] > QDCLEN_MAX) || (fippiconfig.QDCLen6[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen6 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen6[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }
      if( (fippiconfig.QDCLen7[k] > QDCLEN_MAX) || (fippiconfig.QDCLen7[k] < QDCLEN_MIN)  )  {
         printf("Invalid QDCLen7 = %d, must be between %d and %d samples\n",fippiconfig.QDCLen7[k], QDCLEN_MIN, QDCLEN_MAX);
         return -4900-k;
      }


 
   }    // end for channels present

  

   // -------------- now package  and write -------------------

   mapped[AOUTBLOCK] = CS_MZ;	  // select MicroZed Controller

  
  // first, set CSR run control options   
  mapped[ACSRIN] = 0x0000; // all off

  mval =  fippiconfig.AUX_CTRL  & 0x00FF;  // upper bits reserved for FPGA boot, do not toggle 
  mval = mval + 0x0200;     // set bit 9  to keep FPGA configurured  // TODO: move this to another register
  mval = mval + 0x0000;     // clear bit 8: yellow LED on
  mapped[AAUXCTRL] = mval;
  if(mapped[AAUXCTRL] != mval) printf("Error writing AUX_CTRL register\n");



   /*
    
   mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 

    for( k = 0; k < NCHANNEL_PER_K7 ; k ++ )
    {

       mapped[AMZ_EXAFWR] = AK7_CHANNEL;     // write to  k7's addr to select CHANNEL register
       mapped[AMZ_EXDWR]  = k;               // write selected channel      


     // ......... P16 Reg 0  .......................            

      // package
      reglo = 0;     // halt bit 0
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_POLARITY,      FiPPI_INVRT   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_VETOENA,       FiPPI_VETOENA   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_EXTTRIGSEL,    FiPPI_EXTTRIGSEL   );     
      reglo = reglo + (SFR<<4);                               //  Store SlowFilterRange in bits [6:4] 
      mval = 129-SL[k];
      reglo = reglo + (mval<<7);                       //  SlowLength in bits [13:7]
      mval = 129-SL[k]-SG[k];
      reglo = reglo + (mval<<14);                //  SlowLength + SlowGap in bits [20:14]
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_CHANTRIGSEL,   FiPPI_CHANTRIGSEL   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_SYNCDATAACQ,   FiPPI_SYNCDATAACQ   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRC[k],CCSRC_GROUPTRIGSEL,  FiPPI_GROUPTRIGSEL   );    
      mval = 129-FL[k]-FG[k];
      reglo = reglo +( mval<<25) ;               // 128 - (FastLength - 1) in bits [31:25] 
          
      reghi = 0;
      reghi = 129 - FL[k] - FG[k];                          // 128 - (FastLength + FastGap - 1)
      reghi = reghi & 0x7F;                                 // Keep only bits [6:0]
      reghi = reghi + (TH[k]<<7);                             // Threshold in [22:7]   
      reghi = reghi + (fippiconfig.CFD_DELAY[k] <<23);        //  CFDDelay in [28:23]       // in samples!
      reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC[k],CCSRC_CHANVETOSEL,   FiPPI_CHANVETOSEL);     
      reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC[k],CCSRC_MODVETOSEL,    FiPPI_MODVETOSEL );     
      reghi = reghi + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_ENARELAY,      FiPPI_ENARELAY   );     

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
      reglo = 129 - SG[k];                                 //  SlowGap in bits [6:0]
      mval = (2*SL[k]+SG[k]+1);
      reglo = reglo + (mval<<7);                 // Store RBDEL_SF = (SlowLength + SlowGap + SlowLength + 1) in bits [18:7] of Fipreg1 lo
      mval =  8192 - ((SL[k]+SG[k])<<SFR); 
      reglo = reglo + (mval <<19); // Peaksep= SL+SG; store 8192 - PeakSep * 2^SlowFilterRange  in bits [31:19] of Fipreg1 lo
         
      if(SFR==1) PSAM = SL[k]+SG[k] -3;
      if(SFR==2) PSAM = SL[k]+SG[k] -2;
      if(SFR==3) PSAM = SL[k]+SG[k] -2;
      if(SFR==4) PSAM = SL[k]+SG[k] -1;
      if(SFR==5) PSAM = SL[k]+SG[k] -0;
      if(SFR==6) PSAM = SL[k]+SG[k] +1;
      reghi = 0;
      reghi = 8192 - (PSAM<<SFR);                             // Peaksample = SL+SG - a bit ; store 8192 - Peaksample * 2^SlowFilterRange  in bits [44:32] of Fipreg1 
      mval = 2*FL[k]+FG[k]+2;
      reghi = reghi + ( mval<<13 );                // Store RBDEL_TF = (FastLength + FastGap + FastLength + 2) in bits [56:45] of Fipreg1
      reghi = reghi + ( fippiconfig.CFD_SCALE[k] <<25 );        //  Store CFDScale[3:0] in bits [60:57] of Fipreg1
  
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
      reglo = 4096 - (int)(fippiconfig.CHANTRIG_STRETCH[k]*FILTER_CLOCK_MHZ);               //  ChanTrigStretch goes into [11:0] of FipReg2   // in us
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_FTRIGSEL,   SelExtFastTrig   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_TRACEENA,    13   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_CFDMODE,     14   );   
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_QDCENA,      15   );  
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_GLOBTRIG,    16   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA[k],CCSRA_CHANTRIG,    17   );     
      mval = 4096 - (int)(fippiconfig.VETO_STRETCH[k]*FILTER_CLOCK_MHZ);
      reglo = reglo + (mval <<20);       //Store VetoStretch in bits [31:20] of Fipreg2   // in us
      
      reghi = (4096 - (int)(fippiconfig.FASTTRIG_BACKLEN[k]*FILTER_CLOCK_MHZ)) <<8;            //  FastTrigBackLen goes into [19:8] ([51:40] in 64 bit)   // in us
      reghi = reghi + ( (4096 - (int)(fippiconfig.EXTTRIG_STRETCH[k]*FILTER_CLOCK_MHZ)) <<20);   //  ExtTrigStretch goes into [31:20] ([63:52] in 64 bit)   // in us
 
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
      reglo = (int)(fippiconfig.FTRIGOUT_DELAY[k]*FILTER_CLOCK_MHZ);        //  FtrigoutDelay goes into [8:0] of FipReg5             // in us
                                                                                                 
      reghi = (int)(fippiconfig.EXTERN_DELAYLEN[k]*FILTER_CLOCK_MHZ);       //Store EXTERN_DELAYLEN in bits [8:0] of FipReg5 hi      // in us
     
      mval = SL[k]+SG[k];                                            // psep       TODO: check trace related delays. 
      mval = (mval-1) << SFR;                                        // trigger delay
      pafl = (mval>> SFR) + TD[k];                                     // paf length   // check units!
      mval = pafl - mval;                                           //delay from DSP computation
      if( (fippiconfig.CHANNEL_CSRA[k]  & 0x0400) >0 )  //(1<<CCSRA_CFDMODE) >0  )      
         mval = mval + FL[k] + FG[k];                                // add CFD delay if necessary
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
        
      reglo = fippiconfig.QDCLen0[k];                    //  QDC       // in samples
      reglo = reglo + (fippiconfig.QDCLen1[k]<<16);        //  QDC       // in samples                                                                                 
      reghi = fippiconfig.QDCLen2[k];                    //  QDC       // in samples
      reghi = reghi + (fippiconfig.QDCLen3[k]<<16);        //  QDC       // in samples      
      
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
        
       reglo = fippiconfig.QDCLen4[k];                    //  QDC       // in samples
       reglo = reglo + (fippiconfig.QDCLen5[k]<<16);        //  QDC       // in samples                                                                               
       reghi = fippiconfig.QDCLen6[k];                    //  QDC       // in samples
       reghi = reghi + (fippiconfig.QDCLen7[k]<<16);        //  QDC       // in samples     
      
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
        
       reglo = fippiconfig.CFD_THRESHOLD[k];             //  CFDThresh       // in steps
        
         // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG13+0;              // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;              // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG13+1;              // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;                 // write next 16 bit


       // ......... P16 Reg 17 (Info)  .......................    
        
       reglo = k;                                        // channel
       reglo = reglo + (fippiconfig.SLOT_ID<<4);           // slot     // use for K7 0/1
       reglo = reglo + (fippiconfig.CRATE_ID<<8);          // crate
       reglo = reglo + (fippiconfig.MODULE_ID<<12);        // module type 
                                                         // [15:20] reserved for mod address (always 0?)                                           
       reghi = TL[k];                 

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

  */



   // --------------------------- DACs -----------------------------------

    mapped[AOUTBLOCK] = CS_MZ;	  // select MZ controller

   for( k = 0; k < NCHANNELS_PRESENT; k ++ )
   {
      dac = (int)floor( (1 - fippiconfig.VOFFSET[k]/ V_OFFSET_MAX) * 32768);	
      if(dac > 65535)  {
         printf("Invalid VOFFSET = %f, must be between %f and -%f\n",fippiconfig.VOFFSET[k], V_OFFSET_MAX-0.05, V_OFFSET_MAX-0.05);
         return -4300-k;
      }
      mapped[AMZ_FIRSTDAC+k] = dac;
      if(mapped[AMZ_FIRSTDAC+k] != dac) printf("Error writing parameters to DAC register\n");
      usleep(DACWAIT);		// wait for programming
      mapped[AMZ_FIRSTDAC+k] = dac;     // repeat, sometimes doesn't take?
      if(mapped[AMZ_FIRSTDAC+k] != dac) printf("Error writing parameters to DAC register\n");
      usleep(DACWAIT);     
 //     printf("DAC %d, value 0x%x (%d), [%f V] \n",k, dac, dac,fippiconfig.VOFFSET[k]);
   }           // end for channels DAC


  if(0) {
   // --------------------------- Gains ----------------------------------
   // DB01 has 4 gains. Applied via I2C specific to each DB
   // no limits for DIG_GAIN
   // bit mapping
   
  
   for( k = 0; k < NCHANNELS_PRESENT; k ++ )
   {
        if( !( (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN0)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN1)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN2)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN3)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN4)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN5)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN6)  ||
               (fippiconfig.ANALOG_GAIN[k] == DB01_GAIN7)   ) ) {
        printf("ANALOG_GAIN = %f not matching available gains exactly, please choose from this list:\n",fippiconfig.ANALOG_GAIN[k]);
        printf("    %f \n",DB01_GAIN0);
        printf("    %f \n",DB01_GAIN1);
        printf("    %f \n",DB01_GAIN2);
        printf("    %f \n",DB01_GAIN3);
        printf("    %f \n",DB01_GAIN4);
        printf("    %f \n",DB01_GAIN5);
        printf("    %f \n",DB01_GAIN6);
        printf("    %f \n",DB01_GAIN7);
        return -4300-k;
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
         
         =>  unsigned int sw0bit[NCHANNEL_PER_K7] = {6, 11, 4, 0};       // these arrays encode the mapping of gain bits to I2C signals
             unsigned int sw1bit[NCHANNEL_PER_K7] = {8, 2, 5, 10};
             unsigned int gnbit[NCHANNEL_PER_K7] = {9, 12, 1, 3};
       */

       // ............. set the bits for 4 channels  ................. 
       for( k = 0; k < NCHANNEL_PER_K7; k ++ )            // XXXXXX
      {
         m = k;                                         // XXXXXX
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN0)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN1)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN2)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN3)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN4)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN5)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN6)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN7)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 1;  }
   
      }    // end for

      // I2C write for 4 channels
       mapped[AOUTBLOCK] = CS_MZ;	  // select MZ controller
       mapped[AAUXCTRL] = 0x0020;	  // select bit 5 -> DB0 I2C        // XXXXXX

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
       for( k = NCHANNEL_PER_K7; k < 2*NCHANNEL_PER_K7; k ++ )          // XXXXXX
      {
         m = k - NCHANNEL_PER_K7;                                         // XXXXXX
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN0)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN1)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN2)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN3)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 0;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN4)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN5)  { i2cgain[sw1bit[m]] = 0; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN6)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 0; i2cgain[gnbit[m]] = 1;  }
         if(fippiconfig.ANALOG_GAIN[k] == DB01_GAIN7)  { i2cgain[sw1bit[m]] = 1; i2cgain[sw0bit[m]] = 1; i2cgain[gnbit[m]] = 1;  }
   
      }    // end for

      // I2C write for 4 channels
       mapped[AOUTBLOCK] = CS_MZ;	  // select MZ controller
       mapped[AAUXCTRL] = 0x0040;	  // select bit 6 -> DB1 I2C        // XXXXXX

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


       /*
     // TODO
   // restart/initialize filters 
   usleep(100);      // wait for filter FIFOs to clear, really should be longest SL+SG
   for( k = 0; k < NCHANNELS; k ++ )
   {
        addr = 16+k*16;
        mapped[addr+2] = saveR2[k];       // restart filters with the halt bit in R2 set to zero
   }
   usleep(100);      // really should be longest SL+SG
   mapped[ADSP_CLR] = 1;
   mapped[ARTC_CLR] = 1;

     */ 

  mapped[AOUTBLOCK] = CS_MZ;	  // select MicroZed Controller

  mval =  fippiconfig.AUX_CTRL  & 0x00FF;  // upper bits reserved for FPGA boot, do not toggle 
  mval = mval + 0x0200;     // set bit 9  to keep FPGA configurured  // TODO: move this to another register
  mval = mval + 0x0100;     // set bit 8: yellow LED off
  mapped[AAUXCTRL] = mval;
  if(mapped[AAUXCTRL] != mval) printf("Error writing AUX_CTRL register\n");
 



     
    // --------------------------- HW info ----------------------------------

 
   // ADC board temperature
 //   printf("ADC board temperature: %d C \n",(int)board_temperature(mapped) );

   // ***** ZYNQ temperature
 //    printf("Zynq temperature: %d C \n",(int)zynq_temperature() );

   // ***** check HW info *********
//   k = hwinfo(mapped);
//   printf("Revision %04X, Serial Number %d \n",(k>>16) & 0xFFFF, k & 0xFFFF);
//   if(k==0) printf("WARNING: HW may be incompatible with this SW/FW \n");

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










