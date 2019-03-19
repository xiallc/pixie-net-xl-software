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
  int k, ch;
  FILE * filmca;
  FILE * fil;

  unsigned int RunType, SyncT, ReqRunTime, PollTime, WR_RTCtrl;
  unsigned int SL[NCHANNELS], CCSRA[NCHANNELS], PILEUPCTRL[NCHANNELS];
  //unsigned int SG[NCHANNELS];
  float Tau[NCHANNELS], Dgain[NCHANNELS];
  unsigned int BLavg[NCHANNELS], BLcut[NCHANNELS], Binfactor[NCHANNELS], TL[NCHANNELS];
  double C0[NCHANNELS], C1[NCHANNELS], Cg[NCHANNELS];
  double baseline[NCHANNELS] = {0};
  double dt, ph;
  double elm, q;
  //double cfdlev, tmpD, bscale;
  time_t starttime, currenttime;
  unsigned int w0, w1, revsn;
  //unsigned int startTS, m, c0, c1, c2, c3, w0, w1, tmpI, revsn;
  unsigned int tmp0, tmp1, tmp2; //, tmp3;
  unsigned long long WR_tm_tai, WR_tm_tai_start, WR_tm_tai_stop, WR_tm_tai_next;
  unsigned int hdr[32];
  unsigned int out0, out1, out2, out3, trace_staddr, tracewrite, pileup, exttsL, exttsH;
  unsigned int evstats, R1, timeL, timeH, hit;
  //unsigned int evstats, R1, hit, timeL, timeH, psa0, psa1, cfd0;
  //unsigned int psa_base, psa_Q0, psa_Q1, psa_ampl, psa_R;
  //unsigned int cfdout, cfdlow, cfdhigh, cfdticks, cfdfrac, ts_max;
  unsigned int lsum, tsum, gsum, energy, bin; //, binx, biny;
  unsigned int mca[NCHANNELS][MAX_MCA_BINS] ={{0}};    // full MCA for end of run
  unsigned int wmca[NCHANNELS][WEB_MCA_BINS] ={{0}};    // smaller MCA during run
  //unsigned int mca2D[NCHANNELS][MCA2D_BINS*MCA2D_BINS] ={{0}};    // 2D MCA for end of run
  unsigned int onlinebin;
  unsigned int wf[MAX_TL/2];    // two 16bit values per word
  unsigned int loopcount, eventcount, NumPrevTraceBlks, TraceBlks;
  unsigned short buffer1[FILE_HEAD_LENGTH_400] = {0};
  unsigned char buffer2[CHAN_HEAD_LENGTH_400*2] = {0};
  unsigned int wm = WATERMARK;
  unsigned int BLbad[NCHANNELS];
  onlinebin=MAX_MCA_BINS/WEB_MCA_BINS;
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};
  int k7, ch_k7;


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

  // assign to local variables, including any rounding/discretization
  //Accept       = fippiconfig.ACCEPT_PATTERN;
  RunType      = fippiconfig.RUN_TYPE;
  WR_RTCtrl    = fippiconfig.WR_RUNTIME_CTRL;
  SyncT        = fippiconfig.SYNC_AT_START;
  ReqRunTime   = fippiconfig.REQ_RUNTIME;
  PollTime     = fippiconfig.POLL_TIME;
  //CW           = (int)floor(fippiconfig.COINCIDENCE_WINDOW*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks

  if( (RunType==0x100) || (RunType==0x400) || (RunType==0x301)  ) {      // check run type
   // 0x100, 0x400, 0x301 are ok
  } else {
      printf( "This function only support runtypes 0x100 (P16) or 0x400 or 0x301 \n");
      return(-1);
  }

  for( k = 0; k < NCHANNELS; k ++ )
  {
      SL[k]          = (int)floor(fippiconfig.ENERGY_RISETIME[k]*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
//    SG[k]          = (int)floor(fippiconfig.ENERGY_FLATTOP[k]*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
      Dgain[k]       = fippiconfig.DIG_GAIN[k];
 //   TL[k]          = BLOCKSIZE_100*(int)floor(fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/BLOCKSIZE_100);       // multiply time in us *  # ticks per us = time in ticks, multiple of 4
      TL[k]          = MULT_TL*(int)floor(fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/MULT_TL);
      Binfactor[k]   = fippiconfig.BINFACTOR[k];
      Tau[k]         = fippiconfig.TAU[k];
      BLcut[k]       = fippiconfig.BLCUT[k];
      BLavg[k]       = 65536 - fippiconfig.BLAVG[k];
      if(BLavg[k]<0)          BLavg[k] = 0;
      if(BLavg[k]==65536)     BLavg[k] = 0;
      if(BLavg[k]>MAX_BLAVG)  BLavg[k] = MAX_BLAVG;
      BLbad[k] = MAX_BADBL;   // initialize to indicate no good BL found yet
      CCSRA[k]       =  fippiconfig.CHANNEL_CSRA[k]; 
      PILEUPCTRL[k] =  ( CCSRA[k] & (1<<CCSRA_PILEUPCTRL) ) >0;   // if bit set, only allow "single" non-piledup events
   }



  // *************** PS/PL IO initialization *********************
  // open the device for PD register I/O
  fd = open("/dev/uio0", O_RDWR);
  if (fd < 0) {
    perror("Failed to open devfile");
    return -2;
  }

  //Lock the PL address space so multiple programs cant step on eachother.
  if( flock( fd, LOCK_EX | LOCK_NB ) )
  {
    printf( "Failed to get file lock on /dev/uio0\n" );
    return -3;
  }

  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return -4;
  }

  mapped = (unsigned int *) map_addr;

  // --------------------------------------------------------
  // ------------------- Main code begins --------------------
  // --------------------------------------------------------


   // **********************  Compute Coefficients for E Computation  ********************** 
   dt = 1.0/FILTER_CLOCK_MHZ;
   for( k = 0; k < NCHANNELS; k ++ )
   { 
      q = exp(-1.0*dt/Tau[k]);
      elm = exp(-1.0*dt*SL[k]/Tau[k]);
      C0[k] = (q-1.0)*elm/(1.0-elm);
      Cg[k] = 1.0-q;
      C1[k] = (1.0-q)/(1.0-elm);
      // printf("%f  %f   %f\n", C0[k], Cg[k], C1[k]);    
      
      C0[k] = C0[k] * Dgain[k];
      Cg[k] = Cg[k] * Dgain[k];
      C1[k] = C1[k] * Dgain[k];
   }

   revsn = hwinfo(mapped,I2C_SELMAIN);

    // ********************** Run Start **********************

   NumPrevTraceBlks = 0;
   loopcount =  0;
   eventcount = 0;
   starttime = time(NULL);                         // capture OS start time
   if( (RunType==0x100) ||  (RunType==0x400) )  {    // list mode runtypes    
   
      if(RunType==0x100){
        // write a 0x100 header  -- actually there is no header, just events
        fil = fopen("LMdata.bin","wb");
      }  
        
      if(RunType==0x400){
        // write a 0x400 header
        // this is limited to the first 4 channels for now
        // fwrite is slow so we will write to a buffer, and then to the file.
        fil = fopen("LMdata.b00","wb");
        buffer1[0] = BLOCKSIZE_400;
        buffer1[1] = 0;                                       // module number (get from settings file?)
        buffer1[2] = RunType;
        buffer1[3] = CHAN_HEAD_LENGTH_400;
        buffer1[4] = fippiconfig.COINCIDENCE_PATTERN;
        buffer1[5] = fippiconfig.COINCIDENCE_WINDOW;
        buffer1[7] = revsn>>16;               // HW revision from EEPROM
        buffer1[12] = revsn & 0xFFFF;         // serial number from EEPROM
        for( ch = 0; ch < NCHANNEL_MAX400; ch++) {
            buffer1[6]   +=(int)floor((TL[ch] + CHAN_HEAD_LENGTH_400) / BLOCKSIZE_400);         // combined event length, in blocks
            buffer1[8+ch] =(int)floor((TL[ch] + CHAN_HEAD_LENGTH_400) / BLOCKSIZE_400);			// each channel's event length, in blocks
        }
        fwrite( buffer1, 2, FILE_HEAD_LENGTH_400, fil );     // write to file
      }           


    }

    // Run Start Control
   mapped[AMZ_DEVICESEL] = CS_MZ;	 // select MZ
   if(SyncT==1)
      mapped[ARTC_CLR] = 0x0001; // any write will create a pulse to clear timers



   if(WR_RTCtrl==1)     // RunEnable/Live set via WR time comparison  (if startT < WR time < stopT => RunEnable=1) 
   {
      mapped[AMZ_DEVICESEL] = CS_K0;	      // specify which K7 
      mapped[AMZ_EXAFWR] = AK7_PAGE;   // specify   K7's addr:    PAGE register
      mapped[AMZ_EXDWR]  = PAGE_SYS;      //  PAGE 0: system, page 0x10n = channel n

      // get current WR time
      mapped[AMZ_EXAFRD] = AK7_WR_TM_TAI+0;   
      tmp0 =  mapped[AMZ_EXDRD];
      if(SLOWREAD)      tmp0 =  mapped[AMZ_EXDRD];
      mapped[AMZ_EXAFRD] = AK7_WR_TM_TAI+1;   
      tmp1 =  mapped[AMZ_EXDRD];
      if(SLOWREAD)      tmp1 =  mapped[AMZ_EXDRD];
      mapped[AMZ_EXAFRD] = AK7_WR_TM_TAI+2;   
      tmp2 =  mapped[AMZ_EXDRD];
      if(SLOWREAD)      tmp2 =  mapped[AMZ_EXDRD];
      WR_tm_tai = tmp0 +  65536*tmp1 + TWOTO32*tmp2;

      //find next "round" time point 
      WR_tm_tai_next = WR_TAI_STEP*(unsigned long long)floor(WR_tm_tai/WR_TAI_STEP)+ WR_TAI_STEP;   // next coarse time step
     // if( WR_tm_tai_next - WR_tm_tai < WR_TAI_MARGIN)                                          // if too close, 
     //       WR_tm_tai_next = WR_tm_tai_next + WR_TAI_STEP;                                     // one more step   
     // probably bogus. a proper scheme to ensure multiple modules start at the same time should be implemented on the DAQ network master 
    
      WR_tm_tai_start =  WR_tm_tai_next;
      WR_tm_tai_stop  =  WR_tm_tai_next + ReqRunTime;
      ReqRunTime = ReqRunTime + WR_TAI_STEP;    // increase time for local DAQ counter accordingly

      printf( "Current WR time %llu\n",WR_tm_tai );
      printf( "Start time %llu\n",WR_tm_tai_start );
      printf( "Stop time %llu\n",WR_tm_tai_stop );

      // write start/stop to both K7
      // todo: this requires both K7s to be a WR slave with valid time from master
      for(k7=0;k7<N_K7_FPGAS;k7++)
      {     
         mapped[AMZ_DEVICESEL] =  cs[k7];	   // select FPGA 
         mapped[AMZ_EXAFWR] = AK7_PAGE;      // specify   K7's addr:    PAGE register
         mapped[AMZ_EXDWR]  = PAGE_SYS;      //  PAGE 0: system, page 0x10n = channel n

         mapped[AMZ_EXAFWR] =  AK7_WR_TM_TAI_START+0;   // specify   K7's addr:    WR start time register
         mapped[AMZ_EXDWR]  =  WR_tm_tai_start      & 0x00000000FFFF;
         mapped[AMZ_EXAFWR] =  AK7_WR_TM_TAI_START+1;   // specify   K7's addr:    WR start time register
         mapped[AMZ_EXDWR]  =  (WR_tm_tai_start>>16) & 0x00000000FFFF;
         mapped[AMZ_EXAFWR] =  AK7_WR_TM_TAI_START+2;   // specify   K7's addr:    WR start time register
         mapped[AMZ_EXDWR]  =  (WR_tm_tai_start>>32) & 0x00000000FFFF;
   
         mapped[AMZ_EXAFWR] =  AK7_WR_TM_TAI_STOP+0;   // specify   K7's addr:    WR stop time register
         mapped[AMZ_EXDWR]  =  WR_tm_tai_stop      & 0x00000000FFFF;
         mapped[AMZ_EXAFWR] =  AK7_WR_TM_TAI_STOP+1;   // specify   K7's addr:    WR stop time register
         mapped[AMZ_EXDWR]  =  (WR_tm_tai_stop>>16) & 0x00000000FFFF;
         mapped[AMZ_EXAFWR] =  AK7_WR_TM_TAI_STOP+2;   // specify   K7's addr:    WR stop time register
         mapped[AMZ_EXDWR]  =  (WR_tm_tai_stop>>32) & 0x00000000FFFF; 
      } // end K7s
   }  
   
   mapped[AMZ_DEVICESEL] = CS_MZ;	 // select MZ
   mapped[AMZ_CSRIN] = 0x0001; // RunEnable=1 > nLive=0 (DAQ on)
   // this is a bit in a MZ register tied to a line to both FPGAs
   // falling edge of nLive clears counters and memory address pointers
   // line ignored for WR_RTCtrl in K7, but stull usefule for TotalTime in MZ  
   

    // ********************** Run Loop **********************
   do {

    
      //----------- Periodically read BL and update average -----------
      // this will be moved into the FPGA soon
      if(loopcount % BLREADPERIOD == 0) {  //|| (loopcount ==0) ) {     // sometimes 0 mod N not zero and first few events have wrong E? watch
         for(k7=0;k7<N_K7_FPGAS;k7++)
         {
            mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA 
  
            for( ch=0; ch < NCHANNEL_PER_K7; ch++) {

               ch_k7 = ch+k7*NCHANNEL_PER_K7;
               
               // read raw BL sums 
               mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
               mapped[AMZ_EXDWR]  = PAGE_CHN+ch;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0
   
               mapped[AMZ_EXAFRD] = AK7_BLLOCK;    // read from 0xD4 to lock BL registers (no data)
               tmp0 = mapped[AMZ_EXDRD];
   
               mapped[AMZ_EXAFRD] = AK7_BLSTART+0;        // lsum low 16 bit
               tmp0 =  mapped[AMZ_EXDRD];
               if(SLOWREAD)  tmp0 =  mapped[AMZ_EXDRD];
               mapped[AMZ_EXAFRD] = AK7_BLSTART+1;        // lsum high 16 bit
               tmp1 =  mapped[AMZ_EXDRD];
               if(SLOWREAD) tmp1 =  mapped[AMZ_EXDRD];
               lsum = tmp0 + (tmp1<<16); 
   
               mapped[AMZ_EXAFRD] = AK7_BLSTART+2;        // tsum low 16 bit
               tmp0 =  mapped[AMZ_EXDRD];
               if(SLOWREAD)  tmp0 =  mapped[AMZ_EXDRD];
               mapped[AMZ_EXAFRD] = AK7_BLSTART+3;        // tsum high 16 bit
               tmp1 =  mapped[AMZ_EXDRD];
               if(SLOWREAD) tmp1 =  mapped[AMZ_EXDRD];
               tsum = tmp0 + (tmp1<<16); 
   
               mapped[AMZ_EXAFRD] = AK7_BLSTART+6;        // gsum low 16 bit
               tmp0 =  mapped[AMZ_EXDRD];
               if(SLOWREAD)  tmp0 =  mapped[AMZ_EXDRD];
               mapped[AMZ_EXAFRD] = AK7_BLSTART+7;        // gsum high 16 bit, unlock
               tmp1 =  mapped[AMZ_EXDRD];
               if(SLOWREAD) tmp1 =  mapped[AMZ_EXDRD];
               gsum = tmp0 + (tmp1<<16); 
   
               if (tsum>0)		// tum=0 indicates bad baseline
               {
                 ph = C1[ch_k7]*lsum+Cg[ch_k7]*gsum+C0[ch_k7]*tsum;
                 //if (ch==0) printf("ph %f, BLcut %d, BLavg %d, baseline %f\n",ph,BLcut[ch],BLavg[ch],baseline[ch] );
                 if( (BLcut[ch_k7]==0) || (abs(ph-baseline[ch_k7])<BLcut[ch_k7]) || (BLbad[ch_k7] >=MAX_BADBL) )       // only accept "good" baselines < BLcut, or if too many bad in a row (to start over)
                 {
                     if( (BLavg[ch_k7]==0) || (BLbad[ch_k7] >=MAX_BADBL) )
                     {
                         baseline[ch_k7] = ph;
                         BLbad[ch_k7] = 0;
                     } else {
                         // BL average: // avg = old avg + (new meas - old avg)/2^BLavg
                         baseline[ch_k7] = baseline[ch_k7] + (ph-baseline[ch_k7])/(1<<BLavg[ch_k7]);
                         BLbad[ch_k7] = 0;
                     } // end BL avg
                  } else {
                     BLbad[ch_k7] = BLbad[ch_k7]+1;
                 }     // end BLcut check
               }       // end tsum >0 check
            }          // end for channels
         }             // end for K7s
      }             // end periodicity check
         
      
      // -----------poll for events -----------
      // if data ready. read out, compute E, increment MCA *********

      for(k7=0;k7<N_K7_FPGAS;k7++)
      {
         mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA 
         mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
         mapped[AMZ_EXDWR]  = PAGE_SYS;     //                         0x0  = system  page
    
         // Read Header DPM status
         mapped[AMZ_EXAFRD] = AK7_SYSSYTATUS;     // write to  k7's addr for read -> reading from 0x85 system status register
         evstats = mapped[AMZ_EXDRD];   // bits set for every channel that has data in header memory
         if(SLOWREAD)  evstats = mapped[AMZ_EXDRD];   
   
         // event readout compatible with P16 DSP code
         // very slow and inefficient; can improve or better bypass completely in final WR data out implementation
         if(evstats) {					  // if there are events in any channel
       //   printf( "K7 0 read from 0x85: 0x%X\n", evstats );
            for( ch=0; ch < NCHANNEL_PER_K7; ch++)
            {

               ch_k7 = ch+k7*NCHANNEL_PER_K7;     // total channel count
               R1 = 1 << ch;
               if(evstats & R1)	{	 //  if there is an event in the header memory for this channel
      
                     mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
                     mapped[AMZ_EXDWR]  = PAGE_CHN+ch;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0
                    
                    if(  eventcount==0) {
                     // dummy reads
                        mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                        hdr[0] = mapped[AMZ_EXDRD];      // read 16 bits
                     }            
   
                     // read 8 64bit words from header
                     for( k=0; k < 8; k++)
                     {
                        mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                        hdr[4*k+3] = mapped[AMZ_EXDRD];      // read 16 bits
                         if(SLOWREAD)  hdr[4*k+3] = mapped[AMZ_EXDRD];      // read 16 bits
                        mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                        hdr[4*k+2] = mapped[AMZ_EXDRD];      // read 16 bits
                         if(SLOWREAD)  hdr[4*k+2] = mapped[AMZ_EXDRD];      // read 16 bits
                        mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                        hdr[4*k+1] = mapped[AMZ_EXDRD];      // read 16 bits
                         if(SLOWREAD)  hdr[4*k+1] = mapped[AMZ_EXDRD];      // read 16 bits
                        mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                        hdr[4*k+0] = mapped[AMZ_EXDRD];      // read 16 bits
                         if(SLOWREAD)   hdr[4*k+0] = mapped[AMZ_EXDRD];      // read 16 bits
                        // the next 8 words only need to be read if QDCs are enabled
                     }
   
                   /*     printf( "Event count %d\n",eventcount );
                     printf( "Read 0 H-L: 0x %X %X %X %X\n",hdr[ 3], hdr[ 2], hdr[ 1], hdr[ 0] );
                     printf( "Read 1 H-L: 0x %X %X %X %X\n",hdr[ 7], hdr[ 6], hdr[ 5], hdr[ 4] );
                     printf( "Read 2 H-L: 0x %X %X %X %X\n",hdr[11], hdr[10], hdr[ 9], hdr[ 8] );
                     printf( "Read 3 H-L: 0x %X %X %X %X\n",hdr[15], hdr[14], hdr[13], hdr[12] );
                     printf( "Read 4 H-L: 0x %X %X %X %X\n",hdr[19], hdr[18], hdr[17], hdr[16] );
                     printf( "Read 5 H-L: 0x %X %X %X %X\n",hdr[23], hdr[22], hdr[21], hdr[20] );
                     printf( "Read 6 H-L: 0x %X %X %X %X\n",hdr[27], hdr[26], hdr[25], hdr[24] );
                     printf( "Read 7 H-L: 0x %X %X %X %X\n",hdr[31], hdr[30], hdr[29], hdr[28] );
                   */
                   
                     out0   = hdr[0]+(hdr[1]<<16);  // preliminary, more bits to be filled in
                     timeL  = hdr[4]+(hdr[5]<<16); 
                     timeH  = hdr[8];  
                     TL[ch] = hdr[9]; 
                     tsum = hdr[12]+(hdr[13]<<16);
                     lsum = hdr[16]+(hdr[17]<<16);
                     gsum = hdr[20]+(hdr[21]<<16);
                     trace_staddr = hdr[25]>>3;     // tmp2,3 + ext TS. tmp1[15:3] = trace start. rest = cfdout 1
                 //    printf( "time Low: 0x%08X = %0f ms \n",timeL,timeL*13.333/1000000 );
                     pileup = (out0 & 0x8000000)>>31;   // extract pileup bit
                     exttsL = hdr[26]+(hdr[27]<<16);
                     exttsH = hdr[30];

                    // printf( "channel %d, pileup %d, TL %d, exttsL %d \n",ch, pileup, TL[ch],exttsL); 
   
       
                 if( (PILEUPCTRL[k]==0)     || (PILEUPCTRL[k]==1 && !pileup )    )
                 {    // either don't care  OR pilup test required and  pileup bit not set

                 //printf( "pileup test passed\n"); 
   
                     // waveform read (if accepted)
                 //    if(0) {
                     if( (TL[ch_k7] >0) && ( CCSRA[ch_k7] & (1<<CCSRA_TRACEENA)) )  {   // check if TL >0 and traces are recorded (bit 8 of CCSRA)
                       tracewrite = 1;
                    //   printf( "N samples %d, start addr 0x%X \n", TL[ch], trace_staddr);
                       mapped[AMZ_EXAFWR] = AK7_MEMADDR+ch;     // specify   K7's addr     addr 4 = memory address
                       mapped[AMZ_EXDWR]  = trace_staddr;      //  take data from location recorded in trace memory
   
                       // dummy read
                     //   if(  eventcount==0) {
                          mapped[AMZ_EXAFRD] = AK7_TRCMEM_A;     // write to  k7's addr for read -> reading from AK7_TRCMEM_A channel header memory, next 16bit
                          w0 = mapped[AMZ_EXDRD];      // read 16 bits
                          if(SLOWREAD)  w0 = mapped[AMZ_EXDRD];
                          mapped[AMZ_EXAFRD] = AK7_TRCMEM_B;     // write to  k7's addr for read -> reading from AK7_TRCMEM_B channel header memory, high 16bit and addr increase
                          w1 = mapped[AMZ_EXDRD];      // read 16 bits  , increments trace memory address
                          if(SLOWREAD) w1 = mapped[AMZ_EXDRD]; 
                     //  }
   
   
                       for( k=0; k < (TL[ch_k7]/2); k++)
                       {
                           mapped[AMZ_EXAFRD] = AK7_TRCMEM_A;     // write to  k7's addr for read -> reading from AK7_TRCMEM_A channel header memory, next 16bit
                           w0 = mapped[AMZ_EXDRD];      // read 16 bits
                           if(SLOWREAD)  w0 = mapped[AMZ_EXDRD];
                           mapped[AMZ_EXAFRD] = AK7_TRCMEM_B;     // write to  k7's addr for read -> reading from AK7_TRCMEM_B channel header memory, high 16bit and addr increase
                           w1 = mapped[AMZ_EXDRD];      // read 16 bits  , increments trace memory address
                           if(SLOWREAD) w1 = mapped[AMZ_EXDRD]; 
                   
                          // re-order 2 sample words from 32bit FIFO
                          wf[k] = w0+(w1<<16);
                       }
                     }  else {
                        tracewrite = 0;
                     }
   
                            
                     //printf( "start computing E\n"); 
                     // compute and histogram E
                     ph = C1[ch_k7]*(double)lsum+Cg[ch_k7]*(double)gsum+C0[ch_k7]*(double)tsum;
                    //  printf("ph %f, BLavg %f, E %f\n",ph,baseline[ch], ph-baseline[ch]);
                     ph = ph-baseline[ch_k7];
                     if ((ph<0.0)|| (ph>65536.0))	ph =0.0;	   // out of range energies -> 0
                     energy = (int)floor(ph);
                     //if ((hit & (1<< HIT_LOCALHIT))==0)	  	energy =0;	   // not a local hit -> 0
   
                     //   printf( "now incrementing MCA, E = %d\n", energy); 
                     //  histogramming if E< max mcabin
                     bin = energy >> Binfactor[ch_k7];
                     if( (bin<MAX_MCA_BINS) && (bin>0) ) {
                        mca[ch_k7][bin] =  mca[ch_k7][bin] + 1;	// increment mca
                        bin = bin >> WEB_LOGEBIN;
                        if(bin>0) wmca[ch_k7][bin] = wmca[ch_k7][bin] + 1;	// increment wmca
                     }
                  
                     // cfd and psa need some recomputation, not fully implemented yet
   
                     // fill in some constants. For now, report 10 32bit word headers (all except QDC)
                     tmp0 = CHAN_HEAD_LENGTH_100;  // header length
                     tmp1 = tmp0 + TL[ch_k7];
                     out1 = out0 & 0x80000FFF;     // keep pileup and crate/slot/channel #
                     out1 = out1 + (tmp0<<12);     // add header length
                     out1 = out1 + (tmp1<<17);     // add event length
                     out2 = energy;
                     out2 = out2 + (TL[ch_k7]<<16);
                     if(out0& 0x40000000) // test OOR
                        out2 = out2 + (1<<31);
                     out3 = 0;      // baseline placeholder, float actually
          
                     // now store list mode data
   
                     if(RunType==0x100)   {                         
                          memcpy( buffer2 + 0, &(out1), 4 );
                          memcpy( buffer2 + 4, &(timeL), 4 );
                          memcpy( buffer2 + 8, &(timeH), 4 );
                          memcpy( buffer2 + 12, &(out2), 4 );
   
                          memcpy( buffer2 + 16, &(tsum), 4 );
                          memcpy( buffer2 + 20, &(lsum), 4 );   
                          memcpy( buffer2 + 24, &(gsum), 4 );
                          memcpy( buffer2 + 28, &(out3), 4 );      // BL
   
                          memcpy( buffer2 + 32, &(exttsL), 4 );      // ext TS
                          memcpy( buffer2 + 36, &(exttsH), 4 );      // ext TS
                          fwrite( buffer2, 1, CHAN_HEAD_LENGTH_100*4, fil );
             
                          if( tracewrite )  {   // previously checked if TL >0 and traces are recorded (bit 8 of CCSRA)                          
                             fwrite( wf, TL[ch_k7]/2, 4, fil );                        
                          }   // end trace write                   
                     }      // 0x100     
   
                      if(RunType==0x400 && ch_k7<NCHANNEL_MAX400)   {
                   //     printf( "writing 0x400 record\n"); 
                          hit = (1<<ch) + 0x20 + (0x100<<ch);
                          if(tracewrite==1)
                              TraceBlks = (int)floor(TL[ch]/BLOCKSIZE_400);
                          else 
                              TraceBlks = 0;
                          memcpy( buffer2 + 0, &(hit), 4 );
                          memcpy( buffer2 + 4, &(TraceBlks), 2 );
                          memcpy( buffer2 + 6, &(NumPrevTraceBlks), 2 ); 
                          memcpy( buffer2 + 8, &(timeL), 4 );
                          memcpy( buffer2 + 12, &(timeH), 4 );
                          memcpy( buffer2 + 16, &(energy), 2 );
                          memcpy( buffer2 + 18, &(ch), 2 );
                          memcpy( buffer2 + 20, &(out3), 2 );
                          memcpy( buffer2 + 22, &(out3), 2 );   // actually cfd time
                          memcpy( buffer2 + 24, &(out3), 2 );
                          memcpy( buffer2 + 26, &(out3), 2 );
                          memcpy( buffer2 + 28, &(out3), 2 );
                          memcpy( buffer2 + 30, &(out3), 2 );  
                          memcpy( buffer2 + 32, &(out3), 2 );      // debug
                          memcpy( buffer2 + 34, &(out3), 2 );   
                          memcpy( buffer2 + 36, &(exttsL), 4 );      // debug
                          memcpy( buffer2 + 40, &(exttsH), 4 );   
                          // no checksum  for now
                          memcpy( buffer2 + 60, &(wm), 4 );
                          fwrite( buffer2, 1, CHAN_HEAD_LENGTH_400*2, fil );
                          NumPrevTraceBlks = TraceBlks;
   
                          if( tracewrite )  {   // previously checked if TL >0 and traces are recorded (bit 8 of CCSRA)     
                            fwrite( wf, TL[ch_k7]/2, 4, fil );
                          }   // end trace write
                     }      // 0x400
                     
                     eventcount++;             
                  }
                  else { // event not acceptable (piled up 
                       // header memory already advanced, now also advance trace memory address
                       mapped[AMZ_EXAFWR] = AK7_MEMADDR+ch;             // specify   K7's trace memory address
                       mapped[AMZ_EXDWR]  = trace_staddr+TL[ch_k7]/2 ;     //  advance to end of this event's trace
                       // no write out
                  }
               }     // end event in this channel
            }        //end for ch
         }           // end event in any channel
      }              //end for K7s



        // ----------- Periodically save MCA, PSA, and Run Statistics  -----------
       
        if(loopcount % PollTime == 0) 
        {
       
            // 1) Run Statistics 

            // for debug purposes, print to std out so we see what's going on
            mapped[AMZ_DEVICESEL] = CS_MZ;
            tmp0 = mapped[AMZ_RS_TT+0];   // address offset by 1?
            tmp1 = mapped[AMZ_RS_TT+1];
            printf("%s %4.5G \n","Total_Time",((double)tmp0*65536+(double)tmp1*TWOTO32)*1e-9);    
           //  printf("%s %d %d \n","Total_Time",tmp0,tmp1);    

            // print (small) set of RS to file, visible to web
            read_print_runstats_XL_2x4(1, 0, mapped);
      

            // 2) MCA
            filmca = fopen("MCA.csv","w");
            fprintf(filmca,"bin,MCAch0,MCAch1,MCAch2,MCAch3,MCAch4,MCAch5,MCAch6,MCAch7\n");
            for( k=0; k <WEB_MCA_BINS; k++)       // report the 4K spectra during the run (faster web update)
            {
            //   fprintf(filmca,"%d,%u,%u,%u,%u\n ", k*onlinebin,wmca[0][k],wmca[1][k],wmca[2][k],wmca[3][k]);

               fprintf(filmca,"%d",k*onlinebin);                  // bin number
               for(k7=0;k7<N_K7_FPGAS;k7++)
                  for(ch=0;ch<NCHANNEL_PER_K7;ch++) 
                     fprintf(filmca,",%d",wmca[ch+k7*NCHANNEL_PER_K7][k]);    // print channel data
               fprintf(filmca,"\n");

            }
            fclose(filmca);    

        }

        
        
        // ----------- loop housekeeping -----------

         loopcount ++;
         currenttime = time(NULL);
      } while (currenttime <= starttime+ReqRunTime); // run for a fixed time   
    //  } while (eventcount <= 10); // run for a fixed number of events   



   // ********************** Run Stop **********************

   /* debug */

      mapped[AMZ_DEVICESEL] = CS_K0;	      // specify which K7 
      mapped[AMZ_EXAFWR] = AK7_PAGE;   // specify   K7's addr:    PAGE register
      mapped[AMZ_EXDWR]  = PAGE_SYS;      //  PAGE 0: system, page 0x10n = channel n

         // get current time
      mapped[AMZ_EXAFRD] = AK7_WR_TM_TAI+0;   
      tmp0 =  mapped[AMZ_EXDRD];
      if(SLOWREAD)      tmp0 =  mapped[AMZ_EXDRD];
      mapped[AMZ_EXAFRD] = AK7_WR_TM_TAI+1;   
      tmp1 =  mapped[AMZ_EXDRD];
      if(SLOWREAD)      tmp1 =  mapped[AMZ_EXDRD];
      mapped[AMZ_EXAFRD] = AK7_WR_TM_TAI+2;   
      tmp2 =  mapped[AMZ_EXDRD];
      if(SLOWREAD)      tmp2 =  mapped[AMZ_EXDRD];
      WR_tm_tai = tmp0 +  65536*tmp1 + TWOTO32*tmp2;

      printf( "Current WR time %llu\n",WR_tm_tai );
      /* end debug */

   // set nLive bit to stop run
    mapped[AMZ_DEVICESEL] = CS_MZ;	 // select MZ
    mapped[AMZ_CSRIN] = 0x0000; // all off       
   // todo: there may be events left in the buffers. need to stop, then keep reading until nothing left
                      
   // final save MCA and RS
   filmca = fopen("MCA.csv","w");
   fprintf(filmca,"bin,MCAch0,MCAch1,MCAch2,MCAch3,MCAch4,MCAch5,MCAch6,MCAch7\n");
   for( k=0; k <MAX_MCA_BINS; k++)
   {
    //  fprintf(filmca,"%d,%u,%u,%u,%u\n ", k,mca[0][k],mca[1][k],mca[2][k],mca[3][k] );
       fprintf(filmca,"%d",k);                  // bin number
       for(k7=0;k7<N_K7_FPGAS;k7++)
         for(ch=0;ch<NCHANNEL_PER_K7;ch++) 
            fprintf(filmca,",%d",mca[ch+k7*NCHANNEL_PER_K7][k]);    // print channel data
       fprintf(filmca,"\n");
   }
   fclose(filmca);

   mapped[AMZ_DEVICESEL] = CS_MZ;
   read_print_runstats_XL_2x4(0, 0, mapped);
   mapped[AMZ_DEVICESEL] = CS_MZ;

 
 // clean up  
 if( (RunType==0x100) ||  (RunType==0x400) )  { 
   fclose(fil);
 }
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}
