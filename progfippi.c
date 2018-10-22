/*----------------------------------------------------------------------
 * Copyright (c) 2017 XIA LLC
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
  int k, addr;


  // ******************* read ini file and fill struct with values ********************


  
  PixieNetFippiConfig fippiconfig;		// struct holding the input parameters
  const char *defaults_file = "defaults.ini";
  int rval = init_PixieNetFippiConfig_from_file( defaults_file, 0, &fippiconfig );   // first load defaults, do not allow missing parameters
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", defaults_file, rval );
    return rval;
  }
  const char *settings_file = "settings.ini";
  rval = init_PixieNetFippiConfig_from_file( settings_file, 1, &fippiconfig );   // second override with user settings, do allow missing
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", settings_file, rval );
    return rval;
  }

  unsigned int  mval, dac, reglo, reghi;
  unsigned int CW, SFR, FFR, SL[NCHANNELS], SG[NCHANNELS], FL[NCHANNELS], FG[NCHANNELS], TH[NCHANNELS];
  unsigned int PSAM, PSEP, TL[NCHANNELS], TD[NCHANNELS];
  unsigned int gain[NCHANNELS*2], i2cdata[8];


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
  // first, set CSR run control options   
  mapped[ACSRIN] = 0x0000; // all off
  mapped[AOUTBLOCK] = OB_IOREG;	  // read from IO block


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
      return -400;
    }

       if(fippiconfig.SLOT_ID > MAX_SLOT_ID) {
      printf("Invalid SLOT_ID = %d, must be < %d\n",fippiconfig.SLOT_ID,MAX_SLOT_ID);
      return -400;
    }

       if(fippiconfig.MODULE_ID > MAX_MODULE_ID) {
      printf("Invalid CRATE_ID = %d, must be < %d\n",fippiconfig.MODULE_ID,MAX_MODULE_ID);
      return -400;
    }

   // AUX CTRL:
    if(fippiconfig.AUX_CTRL > 65535) {
      printf("Invalid AUX_CTRL = 0x%x\n",fippiconfig.AUX_CTRL);
      return -2700;
    }
    mapped[AAUXCTRL] = fippiconfig.AUX_CTRL;
    if(mapped[AAUXCTRL] != fippiconfig.AUX_CTRL) printf("Error writing AUX_CTRL register\n");

  
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

    // RUN_TYPE
    if( !( (fippiconfig.RUN_TYPE == 0x301)  ||
           (fippiconfig.RUN_TYPE == 0x100)  ||
           (fippiconfig.RUN_TYPE == 0x101)  ||
           (fippiconfig.RUN_TYPE == 0x102)  ||
           (fippiconfig.RUN_TYPE == 0x103)  ||  ) ) {
      printf("Invalid RUN_TYPE = 0x%x, please check manual for a list of supported run types\n",fippiconfig.RUN_TYPE);
      return -2100;
    }

    //MAX_EVENTS     
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
      printf("Invalid COINCIDENCE_WINDOW = %f, must be between %f and %f us\n",fippiconfig.COINCIDENCE_WINDOW, (double)MIN_CW/SYSTEM_CLOCK_MHZ, (double)MAX_CW/SYSTEM_CLOCK_MHZ);
      return -2000;
    }


     // SYNC_AT_START
    if(fippiconfig.SYNC_AT_START >1) {
      printf("Invalid SYNC_AT_START = %d, can only be 0 and 1\n",fippiconfig.SYNC_AT_START);
      return -2400;
    }

     // RESUME
    if(fippiconfig.RESUME >1) {
      printf("Invalid RESUME = %d, can only be 0 and 1\n",fippiconfig.RESUME);
      return -2400;
    }
 

    // SLOW_FILTER_RANGE
    SFR = fippiconfig.SLOW_FILTER_RANGE;
    if( (FR > MAX_SFR) | (FR < MIN_SFR) ) {
      printf("Invalid SLOW_FILTER_RANGE = %d, must be between %d and %d\n",FR,MIN_SFR, MAX_SFR);
      return -2200;
    }

     // FAST_FILTER_RANGE
    FFR = fippiconfig.FAST_FILTER_RANGE;
    if( (FR > MAX_FFR) | (FR < MIN_FFR) ) {
      printf("Invalid FAST_FILTER_RANGE = %d, must be between %d and %d\n",FR,MIN_FFR, MAX_FFR);
      return -2200;
    }

     // FASTTRIG_BACKPLANEENA -- not implemented for now



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

      // energy filter
      SL[k] = (int)floorf(fippiconfig.ENERGY_RISETIME[k] * FILTER_CLOCK_MHZ);
      SL[k] = SL[k] >> SFR;
      if(SL[k] <MIN_SL) {
         printf("Invalid ENERGY_RISETIME = %f, minimum %f us at this filter range\n",fippiconfig.ENERGY_RISETIME[k],(double)((MIN_SL<<SFR)/FILTER_CLOCK_MHZ));
         return -3600-k;
      } 
      SG[k] = (int)floorf(fippiconfig.ENERGY_FLATTOP[k] * FILTER_CLOCK_MHZ);
      SG[k] = SG[k] >> SFR;
      if(SG[k] <MIN_SG) {
         printf("Invalid ENERGY_FLATTOP = %f, minimum %f us at this filter range\n",fippiconfig.ENERGY_FLATTOP[k],(double)((MIN_SG<<SFR)/FILTER_CLOCK_MHZ));
         return -3700-k;
      } 
      if( (SL[k]+SG[k]) >MAX_SLSG) {
         printf("Invalid combined energy filter, maximum %f us at this filter range\n",(double)((MAX_SLSG<<SFR)/FILTER_CLOCK_MHZ));
         return -3700-k;
      } 

  // trigger filter 
      FL[k] = (int)floorf(fippiconfig.TRIGGER_RISETIME[k] * FILTER_CLOCK_MHZ);
      FL[k] = FL[k] >> FFR;
      if(FL[k] <MIN_FL) {
         printf("Invalid TRIGGER_RISETIME = %f, minimum %f us\n",fippiconfig.TRIGGER_RISETIME[k],(double)(MIN_FL<<FFR/FILTER_CLOCK_MHZ));
         return -3800-k;
      } 
      FG[k] = (int)floorf(fippiconfig.TRIGGER_FLATTOP[k] * FILTER_CLOCK_MHZ);
      FG[k] = FG[k] >> FFR;
      if(FG[k] <MIN_FL) {
         printf("Invalid TRIGGER_FLATTOP = %f, minimum %f us\n",fippiconfig.TRIGGER_FLATTOP[k],(double)(MIN_FG/<<FFRFILTER_CLOCK_MHZ));
         return -3900-k;
      } 
      if( (FL[k]+FG[k]) >MAX_FLFG) {
         printf("Invalid combined trigger filter, maximum %f us\n",(double)(MAX_FLFG<FFR/FILTER_CLOCK_MHZ));
         return -3900-k;
      } 
      
      TH[k] = (int)floor(fippiconfig.TRIGGER_THRESHOLD[k]*FL[k]);
      if(TH[k] > MAX_TH)     {
         printf("Invalid TRIGGER_THRESHOLD = %f, maximum %f at this trigger filter rise time\n",fippiconfig.TRIGGER_THRESHOLD[k],MAX_TH*8.0/(double)FL[k]);
         return -4000-k;
      } 

      // waveforms
      TL[k] = 2*(int)floor(fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/2/(1<<FFR) );       // multiply time in us *  # ticks per us = time in ticks; multiple of 2
      if(TL[k] > MAX_TL || TL[k] < TRACELEN_MIN_250OR100MHZADC)  {
         printf("Invalid TRACE_LENGTH = %f, must be between %f and %f us\n",fippiconfig.TRACE_LENGTH[k],(double)TRACELEN_MIN_250OR100MHZADC/ADC_CLK_MHZ,(double)MAX_TL/ADC_CLK_MHZ);
         return -4400-k;
      }

      if(TL[k] <fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ(1<<FFR))  {
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
      if(fippiconfig.XDT[k] <0)  {
         printf("Invalid XDT = %f, must be positive\n",fippiconfig.XDT[k]);
         return -5000-k;
      }
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

      if(fippiconfig.XDT[k] < (MIN_XDT_MOST/ADC_CLK_MHZ)  {
         printf("Invalid XDT = %f, must be at least %f \n",fippiconfig.XDT[k],MIN_XDT_MOST/ADC_CLK_MHZ );
         return -4900-k;
      }
      mval = (int)floor( fippiconfig.XDT[k]*ADC_CLK_MHZ/MIN_XDT_MOST) *MIN_XDT_MOST;
      if(fippiconfig.XDT[k] > mval)     {
         printf("XDT = %f, will be rounded to %f \n",fippiconfig.XDT[k],mval );
         //return -4900-k;
      }
 
   }    // end for channels present

 


   // -------------- now package  and write -------------------

    
   mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 

    for( k = 0; k < NCHANNEL_PER_K7 ; k ++ )
    {

       mapped[AMZ_EXAFWR] = AK7_CHANNEL;     // write to  k7's addr to select CHANNEL register
       mapped[AMZ_EXDWR]  = k;               // write selected channel      


     // ......... P16 Reg 0  .......................            

      // package
      reglo = 0;     // halt bit 0
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_POLARITY,   FiPPI_INVRT   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_VETOENA,    FiPPI_VETOENA   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_EXTTRIGSEL,    FiPPI_EXTTRIGSEL   );     
      reglo = reglo + SFR<<4;                  //Store SlowFilterRange in bits [6:4] 
      reglo = reglo + (129-SL[k])<<7;          //  SlowLength in bits [13:7]
      reglo = reglo + (129-SL[k]-SG[k])<<14;   // SlowLength + SlowGap in bits [20:14]
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_CHANTRIGSEL,   FiPPI_CHANTRIGSEL   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_SYNCDATAACQ,    FiPPI_SYNCDATAACQ   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRC,CCSRC_GROUPTRIGSEL,    FiPPI_GROUPTRIGSEL   );     
      reglo = reglo + (129-FL[k]-FG[k])<<25 ;      // 128 - (FastLength - 1) in bits [31:25] 
          
      reghi = 0;
      reghi = 129 - FL[k] - FG[k];      // 128 - (FastLength + FastGap - 1)
      reghi = reghi & 0x7F;            // Keep only bits [6:0]
      reghi = reghi + TH[k]<<7;        // Threshold in [22:7]   
      reghi = reghi + fippiconfig.CFD_DELAY[k] <<23;  //  CFDDelay in [28:23]       // check units!
      reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC,CCSRC_CHANVETOSEL,FiPPI_CHANVETOSEL);     
      reghi = reghi + setbit(fippiconfig.CHANNEL_CSRC,CCSRC_MODVETOSEL, FiPPI_MODVETOSEL );     
      reghi = reghi + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_ENARELAY,   FiPPI_ENARELAY   );     

      // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG00+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG00+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG00+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG00+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit


       // ......... P16 Reg 1  .......................    
        
      // package
      reglo = 129 - SG[k]);          //  SlowGap in bits [6:0]
      reglo = reglo + (2*SL[k]+SG[k]+1)<<7;   // Store RBDEL_SF = (SlowLength + SlowGap + SlowLength + 1) in bits [18:7] of Fipreg1 lo
      reglo = reglo + ( 8192 - ((SL[k]+SG[k])<<SFR) ) <<19;   // Peaksep= SL+SG; store 8192 - PeakSep * 2^SlowFilterRange  in bits [31:19] of Fipreg1 lo
         
      if(SFR==1) PSAM = SL[k]+SG[k] -3;
      if(SFR==2) PSAM = SL[k]+SG[k] -2;
      if(SFR==3) PSAM = SL[k]+SG[k] -2;
      if(SFR==4) PSAM = SL[k]+SG[k] -1;
      if(SFR==5) PSAM = SL[k]+SG[k] -0;
      if(SFR==6) PSAM = SL[k]+SG[k] +1;
      reghi = 0;
      reghi = 8192 - PSAM<<SFR;      // Peaksample = SL+SG - a bit ; store 8192 - Peaksample * 2^SlowFilterRange  in bits [44:32] of Fipreg1 
      reghi = reghi + (2*FL[k]+FG[k]+2)<<13;   // Store RBDEL_TF = (FastLength + FastGap + FastLength + 2) in bits [56:45] of Fipreg1
      reghi = reghi + fippiconfig.CFD_SCALE[k] <<25;  //  Store CFDScale[3:0] in bits [60:57] of Fipreg1
  
      // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG01+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG01+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG01+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG01+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit


       // ......... P16 Reg 2  .......................    
        
      // package
      reglo = 4096 - fippiconfig.CHANTRIG_STRETCH[k]);          //  ChanTrigStretch goes into [11:0] of FipReg2   // check units!
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_FTRIGSEL,   SelExtFastTrig   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_TRACEENA,    13   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_CFDMODE,     14   );   
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_QDCENA,      15   );  
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_GLOBTRIG,    16   );     
      reglo = reglo + setbit(fippiconfig.CHANNEL_CSRA,CCSRA_CHANTRIG,    17   );     
      reglo = reglo + (4096-fippiconfig.VETO_STRETCH[k]) <<20;             //Store VetoStretch in bits [31:20] of Fipreg2   // check units!
      
      reghi = 4096 - fippiconfig.FASTTRIG_BACKLEN[k] <<8;  //  FastTrigBackLen goes into [19:8] ([51:40] in 64 bit)               // check units!
      reghi = reghi +  4096 - fippiconfig.EXTTRIG_STRETCH[k] <<20;  //  ExtTrigStretch goes into [31:20] ([63:52] in 64 bit)      // check units!
 
      // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG02+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG02+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG02+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG02+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit

         // ......... P16 Reg 5  .......................    
        
      // package
      reglo = fippiconfig.FTRIGOUT_DELAY[k];          //  FtrigoutDelay goes into [8:0] of FipReg5                // check units!
                                                                                                 
      reghi = fippiconfig.EXTERN_DELAYLEN[k];             //Store EXTERN_DELAYLEN in bits [8:0] of FipReg5 hi      // check units!
     
      mval = SL[k]+SG[k];     // psep
      mval = (mval-1) << SFR;     // trigger delay
      pafl = mval>> SFR + TD[k]; // paf length   // check units!
      mavl = pafl - mval);   //delay from DSP computation
      if( fippiconfig.CHANNEL_CSRA  & (1<<CCSRA_CFDMODE) )
         mval = mval + FL[k] + FG[k];     // add CFD delay if necessary
      reghi = reghi +  mval<<9;                 // trace delay (Capture_FIFOdelaylen )
   
      // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG05+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG05+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG05+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG05+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit


      // ......... P16 Reg 6  .......................    
        
      reglo = fippiconfig.QDCLen0[k];                    //  QDC       // check units! 
      reglo = reglo + fippiconfig.QDCLen1[k]<<16;        //  QDC       // check units!                                                                                 
      reghi = fippiconfig.QDCLen2[k];                    //  QDC       // check units! 
      reghi = reghi + fippiconfig.QDCLen3[k]<<16;        //  QDC       // check units!      
      
         // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG06+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG06+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG06+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG06+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit

    
       // ......... P16 Reg 7  .......................    
        
       reglo = fippiconfig.QDCLen4[k];                    //  QDC       // check units! 
       reglo = reglo + fippiconfig.QDCLen5[k]<<16;        //  QDC       // check units!                                                                                 
       reghi = fippiconfig.QDCLen6[k];                    //  QDC       // check units! 
       reghi = reghi + fippiconfig.QDCLen7[k]<<16;        //  QDC       // check units!      
      
         // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG07+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG07+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG07+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG07+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit

       // ......... P16 Reg 13  .......................    
        
       reglo = fippiconfig.CFD_THRESHOLD[k];                    //  CFDThresh       // check units! 
        
         // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG13+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG13+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit


       // ......... P16 Reg 17 (Info)  .......................    
        
       reglo = k;                                        // channel
       reglo = reglo + fippiconfig.SLOT_ID<<4;           // slot     // use for K7 0/1
       reglo = reglo + fippiconfig.CRATE_ID<<8;          // crate
       reglo = reglo + fippiconfig.MODULE_ID<<12;        // module type 
                                                         // reserved for mod address (always 0?)                                           
       reghi = TL[k];                 

       // now write 
       mapped[AMZ_EXAFWR] = AK7_P16REG17+0;     // write to  k7's addr to select channel's register N
       mapped[AMZ_EXDWR]  = reglo & 0xFFFF;     // write lower 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG17+1;     // write to  k7's addr to select channel's register N+1
       mapped[AMZ_EXDWR]  = reglo >> 16;        // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG17+2;     // write to  k7's addr to select channel's register N+2
       mapped[AMZ_EXDWR]  = reghi & 0xFFFF;     // write next 16 bit
       mapped[AMZ_EXAFWR] = AK7_P16REG17+3;     // write to  k7's addr to select channel's register N+3
       mapped[AMZ_EXDWR]  = reghi >> 16;        // write highest 16 bit



                                                    

      /*
      setbit (par,bitc, bitf) {
      unsigned int ret;
        ret = par & (1 << bitc);
        ret >> bitc;
        ret << bitf;
        return ret 
        }
        */



    }  // end for NCHANNEL_PER_K7




   // gain, Efilter2: R3
   // current version only has 2 gains: 2 and 5. applied via I2C below, only save bit pattern here
   for( k = 0; k < NCHANNELS; k ++ )
   {
        if( !( (fippiconfig.ANALOG_GAIN[k] == GAIN_HIGH)  ||
               (fippiconfig.ANALOG_GAIN[k] == GAIN_LOW)   ) ) {
        printf("ANALOG_GAIN = %f not matching available gains exactly, rounding to nearest\n",fippiconfig.ANALOG_GAIN[k]);
    }

      if(fippiconfig.ANALOG_GAIN[k] > (GAIN_HIGH+GAIN_LOW)/2 ) {
         gain[2*k+1] = 1;      // 2'b10 = gain 5
         gain[2*k]   = 0;   
      }
      else  {
        gain[2*k+1] = 0;      
        gain[2*k]   = 1;      // 2'b01 = gain 2
      }
        // no limits for DIG_GAIN
   }

   // DAC : R4 
   for( k = 0; k < NCHANNELS; k ++ )
   {
      dac = (int)floor( (1 - fippiconfig.VOFFSET[k]/ V_OFFSET_MAX) * 32768);	
      if(dac > 65535)  {
         printf("Invalid VOFFSET = %f, must be between %f and -%f\n",fippiconfig.VOFFSET[k], V_OFFSET_MAX-0.05, V_OFFSET_MAX-0.05);
         return -4300-k;
      }
      mval=dac;
      addr = N_PL_IN_PAR+k*N_PL_IN_PAR;   // channel registers begin after NPLPAR system registers, NPLPAR each
      mapped[addr+4] = mval;
      if(mapped[addr+4] != mval) printf("Error writing parameters to DAC register\n");
      usleep(DACWAIT);		// wait for programming
      mapped[addr+4] = mval;     // repeat, sometimes doesn't take?
      if(mapped[addr+4] != mval) printf("Error writing parameters to DAC register\n");
      usleep(DACWAIT);     
 //     printf("DAC %d, value 0x%x (%d), [%f V] \n",k, dac, dac,fippiconfig.VOFFSET[k]);
   }



 


   // coincdelay: R9
   for( k = 0; k < NCHANNELS; k ++ )
   {
      mval = (int)floor( fippiconfig.COINC_DELAY[k] * ADC_CLK_MHZ);    
      if(mval > MAX_CD) {
         printf("Invalid COINC_DELAY = %f, maximum %d us\n",fippiconfig.COINC_DELAY[k],MAX_CD/ADC_CLK_MHZ);
         return -5700-k;
      }

      addr = N_PL_IN_PAR+k*N_PL_IN_PAR;   // channel registers begin after NPLPAR system registers, NPLPAR each     
      mapped[addr+9] = mval;
      if(mapped[addr+9] != mval) printf("Error writing parameters to COINC_DELAY register");
   }



   // PSA: R10
   for( k = 0; k < NCHANNELS; k ++ )
   { 
      if(fippiconfig.QDC0_LENGTH[k] > MAX_QDCL)    {
         printf("Invalid QDC0_LENGTH = %d, maximum %d samples \n",fippiconfig.QDC0_LENGTH[k],MAX_QDCL);
         return -5900-k;
      } 
      if(fippiconfig.QDC0_LENGTH[k]+fippiconfig.QDC0_DELAY[k] > MAX_QDCLD)    {
         printf("Invalid QDC0_DELAY = %d, maximum length plus delay %d samples \n",fippiconfig.QDC0_DELAY[k],MAX_QDCLD);
         return -6000-k;
      } 
      if(fippiconfig.QDC1_LENGTH[k] > MAX_QDCL)    {
         printf("Invalid QDC1_LENGTH = %d, maximum %d samples \n",fippiconfig.QDC1_LENGTH[k],MAX_QDCL);
         return -6100-k;
      } 
      if(fippiconfig.QDC1_LENGTH[k]+fippiconfig.QDC1_DELAY[k] > MAX_QDCLD)    {
         printf("Invalid QDC1_DELAY = %d, maximum length plus delay %d samples \n",fippiconfig.QDC1_DELAY[k],MAX_QDCLD);
         return -6200-k;
      } 
      if(fippiconfig.QDC0_LENGTH[k]+fippiconfig.QDC0_DELAY[k] > fippiconfig.QDC1_LENGTH[k]+fippiconfig.QDC1_DELAY[k])    {
         printf("Invalid QDC1_DELAY/_LENGTH; must finish later than QDC0 \n");
         return -6300-k;
      } 
      if( (fippiconfig.QDC0_LENGTH[k] & 0x0001) == 1)    {
         printf("NOTE: QDC0_LENGTH is an odd number, rounding down to nearest even number \n");
      } 
      if( (fippiconfig.QDC1_LENGTH[k] & 0x0001) == 1)    {
         printf("NOTE: QDC1_LENGTH is an odd number, rounding down to nearest even number \n");
      } 
      if( ( (fippiconfig.QDC0_DELAY[k] - fippiconfig.QDC1_DELAY[k]) & 0x0001) == 1)    {
         printf("NOTE: QDC_DELAYs are NOT both even or both odd, using last bit of QDC0_DELAY \n");
      }                                            

      // 250 MHz implementation works on 2 samples at a time, so divide by 2
      QDCL0[k] = (int)floorf( fippiconfig.QDC0_LENGTH[k]/2.0)+1;  
      QDCD0[k] = fippiconfig.QDC0_DELAY[k] + QDCL0[k]*2 +2;
      QDCL1[k] = (int)floorf( fippiconfig.QDC1_LENGTH[k]/2.0)+1;
      QDCD1[k] = fippiconfig.QDC1_DELAY[k] + QDCL1[k]*2 +2;
      mval = QDCL0[k];
      mval = mval + (QDCD0[k] <<  7);
      mval = mval + (QDCL1[k] <<  16);
      mval = mval + (QDCD1[k] <<  23);
      mval = mval + (1<<15);     // set bit 15 for 2x correction for QDC0 (always)
      mval = mval + (1<<31);     // set bit 31 for 2x correction for QDC1 (always)
      if( fippiconfig.QDC_DIV8[k])  {
         mval = mval | (1<<5);      // set bits to divide result by 8
         mval = mval | (1<<21);
      }

      // optional division by 8 of output sums not implemented, controlled by MCSRB bits
      addr = N_PL_IN_PAR+k*N_PL_IN_PAR;   // channel registers begin after NPLPAR system registers, NPLPAR each     
      mapped[addr+10] = mval;
      if(mapped[addr+10] != mval) printf("Error writing parameters to QDC register");
   }

       // MCA2D_SCALEX/Y, PSA_NG_THRESHOLD: check only
   for( k = 0; k < NCHANNELS; k ++ )
   {
      if( (fippiconfig.MCA2D_SCALEX[k] > (65535/MCA2D_BINS)) || (fippiconfig.MCA2D_SCALEX[k] <0) )    {
         // must be positive, and at most its a 64K number spread over MCA2D_BINS
         printf("Invalid MCA2D_SCALEX = %f, maximum %d\n",fippiconfig.MCA2D_SCALEX[k],(65535/MCA2D_BINS));
         return -6400-k;
      } 

      if( (fippiconfig.MCA2D_SCALEY[k] > (65535/MCA2D_BINS)) || (fippiconfig.MCA2D_SCALEY[k] <0) )    {
         // must be positive, and at most its a 64K number spread over MCA2D_BINS
         printf("Invalid MCA2D_SCALEY = %f, maximum %d\n",fippiconfig.MCA2D_SCALEY[k],(65535/MCA2D_BINS));
         return -6500-k;
      } 
       
      if( fippiconfig.PSA_NG_THRESHOLD[k] <0) {
         // must be positive
         printf("Invalid PSA_NG_THRESHOLD = %f, must be positive\n",fippiconfig.PSA_NG_THRESHOLD[k]);
         return -6600-k;
      } 
   }
   

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


   // ************************ I2C programming *********************************
   // gain and termination applied across all channels via FPGA's I2C
   // TODO
   // I2C connects to gain enables, termination relays, thermometer, PROM (with s/n etc), optional external

    // ---------------------- program gains -----------------------

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
      i2cdata[k] = gain[k];
   }
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);

   // I2C data byte
   I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
   I2Cslaveack(mapped);

   I2Cstop(mapped);

      // ---------------------- program termination -----------------------

   I2Cstart(mapped);

   // I2C addr byte
   i2cdata[7] = 0;
   i2cdata[6] = 1;
   i2cdata[5] = 0;
   i2cdata[4] = 0;
   i2cdata[3] = 0;   // A2
   i2cdata[2] = 0;   // A1
   i2cdata[1] = 1;   // A0
   i2cdata[0] = 0;   // R/W*
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);

   // I2C data byte
   // settings taken from MCSRB
   i2cdata[7] = (fippiconfig.MODULE_CSRB & 0x0080) >> 7 ;    // power down ADC driver D, NYI
   i2cdata[6] = (fippiconfig.MODULE_CSRB & 0x0040) >> 6 ;    // power down ADC driver C, NYI
   i2cdata[5] = (fippiconfig.MODULE_CSRB & 0x0020) >> 5 ;    // power down ADC driver B, NYI
   i2cdata[4] = (fippiconfig.MODULE_CSRB & 0x0010) >> 4 ;    // power down ADC driver A, NYI
   i2cdata[3] = (fippiconfig.MODULE_CSRB & 0x0008) >> 3 ;    //unused
   i2cdata[2] = (fippiconfig.MODULE_CSRB & 0x0004) >> 2 ;    // term. CD
   i2cdata[1] = (fippiconfig.MODULE_CSRB & 0x0002) >> 1 ;    // term. AB
   i2cdata[0] = (fippiconfig.MODULE_CSRB & 0x0001)      ;    //unused
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);

   // I2C data byte
   I2Cbytesend(mapped, i2cdata);      // send same bits again for enable?
   I2Cslaveack(mapped);

   I2Cstop(mapped);

  
   // ************************ end I2C *****************************************

   // ADC board temperature
    printf("ADC board temperature: %d C \n",(int)board_temperature(mapped) );

   // ***** ZYNQ temperature
     printf("Zynq temperature: %d C \n",(int)zynq_temperature() );

   // ***** check HW info *********
   k = hwinfo(mapped);
   printf("Revision %04X, Serial Number %d \n",(k>>16) & 0xFFFF, k & 0xFFFF);
   if(k==0) printf("WARNING: HW may be incompatible with this SW/FW \n");

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










