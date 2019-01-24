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

  unsigned int RunType, SyncT, ReqRunTime, PollTime;
  unsigned int SL[NCHANNELS], CCSRA[NCHANNELS];
  //unsigned int SG[NCHANNELS];
  float Tau[NCHANNELS], Dgain[NCHANNELS];
  unsigned int BLavg[NCHANNELS], BLcut[NCHANNELS], Binfactor[NCHANNELS], TL[NCHANNELS];
  double C0[NCHANNELS], C1[NCHANNELS], Cg[NCHANNELS];
  double baseline[NCHANNELS] = {0};
  double dt, ph;
  double elm, q;
  //double cfdlev, tmpD, bscale;
  time_t starttime, currenttime;
  unsigned int startTS, w0, w1, revsn;
  //unsigned int startTS, m, c0, c1, c2, c3, w0, w1, tmpI, revsn;
  unsigned int tmp0, tmp1, tmp2, tmp3;
  unsigned int hdr[32];
  unsigned int out0, out1, out2, out3, trace_staddr;
  unsigned int evstats, R1, timeL, timeH;
  //unsigned int evstats, R1, hit, timeL, timeH, psa0, psa1, cfd0;
  //unsigned int psa_base, psa_Q0, psa_Q1, psa_ampl, psa_R;
  //unsigned int cfdout, cfdlow, cfdhigh, cfdticks, cfdfrac, ts_max;
  unsigned int lsum, tsum, gsum, energy, bin; //, binx, biny;
  unsigned int mca[NCHANNELS][MAX_MCA_BINS] ={{0}};    // full MCA for end of run
  unsigned int wmca[NCHANNELS][WEB_MCA_BINS] ={{0}};    // smaller MCA during run
  //unsigned int mca2D[NCHANNELS][MCA2D_BINS*MCA2D_BINS] ={{0}};    // 2D MCA for end of run
  unsigned int onlinebin;
  unsigned int wf[MAX_TL/2];    // two 16bit values per word
  unsigned int loopcount, eventcount; //, NumPrevTraceBlks, TraceBlks;
  //unsigned short buffer1[FILE_HEAD_LENGTH_400] = {0};
  unsigned char buffer2[CHAN_HEAD_LENGTH_400*2] = {0};
  unsigned int HMaddr[NCHANNELS] = {0};
  //unsigned int TMaddr[NCHANNELS] = {0};
 // unsigned int wm = WATERMARK;
  unsigned int BLbad[NCHANNELS];
  onlinebin=MAX_MCA_BINS/WEB_MCA_BINS;


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
  SyncT        = fippiconfig.SYNC_AT_START;
  ReqRunTime   = fippiconfig.REQ_RUNTIME;
  PollTime     = fippiconfig.POLL_TIME;
  //CW           = (int)floor(fippiconfig.COINCIDENCE_WINDOW*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
    printf( "REQ_RUNTIME %d  \n", ReqRunTime);

  if( (RunType!=0x100) ) {      // grouped list mode run (equiv 0x402)
      printf( "This function only support runtype 0x100 (P16) \n");
      return(-1);
  }

  for( k = 0; k < NCHANNELS; k ++ )
  {
      SL[k]          = (int)floor(fippiconfig.ENERGY_RISETIME[k]*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
//    SG[k]          = (int)floor(fippiconfig.ENERGY_FLATTOP[k]*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
      Dgain[k]       = fippiconfig.DIG_GAIN[k];
      TL[k]          = BLOCKSIZE_100*(int)floor(fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/BLOCKSIZE_100);       // multiply time in us *  # ticks per us = time in ticks, multiple of 4
      Binfactor[k]   = fippiconfig.BINFACTOR[k];
      Tau[k]         = fippiconfig.TAU[k];
      BLcut[k]       = fippiconfig.BLCUT[k];
      BLavg[k]       = 65536 - fippiconfig.BLAVG[k];
      if(BLavg[k]<0)          BLavg[k] = 0;
      if(BLavg[k]==65536)     BLavg[k] = 0;
      if(BLavg[k]>MAX_BLAVG)  BLavg[k] = MAX_BLAVG;
      BLbad[k] = MAX_BADBL;   // initialize to indicate no good BL found yet
      CCSRA[k]       =  fippiconfig.CHANNEL_CSRA[k]; 
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

   revsn = hwinfo(mapped);

    // ********************** Run Start **********************


   loopcount =  0;
   eventcount = 0;
   starttime = time(NULL);                         // capture OS start time
   if( (RunType==0x100)  )  {    // list mode runtypes    
      if(SyncT) mapped[ARTC_CLR] = 1;              // write to reset time counter
      mapped[AOUTBLOCK] = 2;
      startTS = mapped[AREALTIME];

      if(RunType==0x100){
        // write a 0x100 header  -- actually there is no header, just events
        fil = fopen("LMdata.bin","wb");
        }           
    }

    // Run Start Control
   mapped[AOUTBLOCK] = CS_MZ;	 // select MZ
   mapped[ACSRIN] = 0x0001; // RunEnable=1 > nLive=0 (DAQ on)
   // this is a bit in a MZ register tied to a line to both FPGAs
   // falling edge of nLive clears counters and memory address pointers
   
   mapped[AOUTBLOCK] = CS_K0;	 // select FPGA 0 

    // ********************** Run Loop **********************
   do {

   /*  no baselines for now
      //----------- Periodically read BL and update average -----------
      // this will be moved into the FPGA soon
      if(loopcount % BLREADPERIOD == 0) {  //|| (loopcount ==0) ) {     // sometimes 0 mod N not zero and first few events have wrong E? watch
         for( ch=0; ch < NCHANNELS; ch++) {
            // read raw BL sums 
            chaddr = ch*16+16;
            lsum  = mapped[chaddr+CA_LSUMB];         
            tsum  = mapped[chaddr+CA_TSUMB];
            gsum  = mapped[chaddr+CA_GSUMB];
            if (tsum>0)		// tum=0 indicates bad baseline
            {
              ph = C1[ch]*lsum+Cg[ch]*gsum+C0[ch]*tsum;
              //if (ch==0) printf("ph %f, BLcut %d, BLavg %d, baseline %f\n",ph,BLcut[ch],BLavg[ch],baseline[ch] );
              if( (BLcut[ch]==0) || (abs(ph-baseline[ch])<BLcut[ch]) || (BLbad[ch] >=MAX_BADBL) )       // only accept "good" baselines < BLcut, or if too many bad in a row (to start over)
              {
                  if( (BLavg[ch]==0) || (BLbad[ch] >=MAX_BADBL) )
                  {
                      baseline[ch] = ph;
                      BLbad[ch] = 0;
                  } else {
                      // BL average: // avg = old avg + (new meas - old avg)/2^BLavg
                      baseline[ch] = baseline[ch] + (ph-baseline[ch])/(1<<BLavg[ch]);
                      BLbad[ch] = 0;
                  } // end BL avg
               } else {
                  BLbad[ch] = BLbad[ch]+1;
              }     // end BLcut check
            }       // end tsum >0 check
         }          // end for loop
      }             // end periodicity check
     */ 
      
      // -----------poll for events -----------
      // if data ready. read out, compute E, increment MCA *********

      mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
      mapped[AMZ_EXDWR]  = 0x0;          //                         0x0  = system  page
 

      mapped[AMZ_EXAFRD] = 0x81;     // write to  k7's addr for read -> reading from 0x85 system status register
  //      usleep(1);
      evstats = mapped[AMZ_EXDRD];   // bits set for every channel that has data in header memory
    //  printf( "K7 0 read from 0x81: 0x%X\n", evstats );


      // Read Header DPM status
      mapped[AMZ_EXAFRD] = AK7_SYSSYTATUS;     // write to  k7's addr for read -> reading from 0x85 system status register
        usleep(1);      // required?
      evstats = mapped[AMZ_EXDRD];   // bits set for every channel that has data in header memory

      // event readout compatible with P16 DSP code
      // very slow and inefficient; can improve or better bypass completely in final WR data out implementation
      if(evstats) {					  // if there are events in any channel
    //   printf( "K7 0 read from 0x85: 0x%X\n", evstats );
         for( ch=0; ch < NCHANNEL_PER_K7; ch++)
         {
            R1 = 1 << ch;
            if(evstats & R1)	{	 //  if there is an event in the header memory for this channel
                  mapped[AMZ_EXAFWR] = AK7_MEMADDR;     // specify   K7's addr     addr 4 = memory address
                  mapped[AMZ_EXDWR]  = 0x123; //HMaddr[ch];      //  take data from top of memory as remembered in C variable

                  HMaddr[ch] += 8;              // increment remembered header address, roll over if necessary
                  if(HMaddr[ch] >= 512)
                     HMaddr[ch] = HMaddr[ch] - 512;

                  mapped[AMZ_EXAFWR] = AK7_PAGE;     // specify   K7's addr     addr 3 = channel/system
                  mapped[AMZ_EXDWR]  = 0x100+ch;      //                         0x10n  = channel n     -> now addressing channel ch page of K7-0
                 
                 if(  eventcount==0) {
                  // dummy reads
                     mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                     hdr[0] = mapped[AMZ_EXDRD];      // read 16 bits
                     mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                     hdr[0] = mapped[AMZ_EXDRD];      // read 16 bits
              //       mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
              //       hdr[0] = mapped[AMZ_EXDRD];      // read 16 bits
              //       mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
              //       hdr[0] = mapped[AMZ_EXDRD];      // read 16 bits
                     }
            

                  // read 8 64bit words from header
                  for( k=0; k < 8; k++)
                  {
                     mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                     hdr[4*k+3] = mapped[AMZ_EXDRD];      // read 16 bits
                     mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                     hdr[4*k+2] = mapped[AMZ_EXDRD];      // read 16 bits
                     mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                     hdr[4*k+1] = mapped[AMZ_EXDRD];      // read 16 bits
                     mapped[AMZ_EXAFRD] = AK7_HDRMEM_D;     // write to  k7's addr for read -> reading from AK7_HDRMEM_A channel header fifo, low 16bit
                     hdr[4*k+0] = mapped[AMZ_EXDRD];      // read 16 bits
                     // the next 8 words only need to be read if QDCs are enabled
                  }

             /*     printf( "Read 0 H-L: 0x %X %X %X %X\n",hdr[ 3], hdr[ 2], hdr[ 1], hdr[ 0] );
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
                  printf( "time Low: 0x%08X = %0f ms \n",timeL,timeL*13.333/1000000 );

    
                  // TODO: add pileup (and other) acceptance check

                             
                  // compute and histogram E
                  ph = C1[ch]*(double)lsum+Cg[ch]*(double)gsum+C0[ch]*(double)tsum;
                 //  printf("ph %f, BLavg %f, E %f\n",ph,baseline[ch], ph-baseline[ch]);
                  ph = ph-baseline[ch];
                  if ((ph<0.0)|| (ph>65536.0))	ph =0.0;	   // out of range energies -> 0
                  energy = (int)floor(ph);
                  //if ((hit & (1<< HIT_LOCALHIT))==0)	  	energy =0;	   // not a local hit -> 0

                  //  histogramming if E< max mcabin
                  bin = energy >> Binfactor[ch];
                  if( (bin<MAX_MCA_BINS) && (bin>0) ) {
                     mca[ch][bin] =  mca[ch][bin] + 1;	// increment mca
                     bin = bin >> WEB_LOGEBIN;
                     if(bin>0) wmca[ch][bin] = wmca[ch][bin] + 1;	// increment wmca
                  }
               
                  // cfd and psa need some recomputation, not fully implemented yet

                  // fill in some constants. For now, report 10 32bit word headers (all except QDC)
                  tmp0 = CHAN_HEAD_LENGTH_100;  // header length
                  tmp1 = tmp0 + TL[ch];
                  out1 = out0 & 0x80000FFF;     // keep pileup and crate/slot/channel #
                  out1 = out1 + (tmp0<<12);     // add header length
                  out1 = out1 + (tmp1<<17);     // add event length
                  out2 = energy;
                  out2 = out2 + (TL[ch]<<16);
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

                          memcpy( buffer2 + 32, &(out3), 4 );      // ext TS
                          memcpy( buffer2 + 36, &(out3), 4 );      // ext TS
                          fwrite( buffer2, 1, CHAN_HEAD_LENGTH_100*4, fil );
             
                 //          if(0) {
                           if( (TL[ch] >0) && ( CCSRA[ch] & (1<<CCSRA_TRACEENA)) )  {   // check if TL >0 and traces are recorded (bit 8 of CCSRA)
                             printf( "N samples %d, start addr 0x%X \n", TL[ch], trace_staddr);
                             mapped[AMZ_EXAFWR] = AK7_MEMADDR+ch;     // specify   K7's addr     addr 4 = memory address
                             mapped[AMZ_EXDWR]  = trace_staddr;      //  take data from location recorded in headers
                           //  w0 = mapped[AWF0+ch];  // dummy read?
                             for( k=0; k < (TL[ch]/2); k++)
                             {
                                 mapped[AMZ_EXAFRD] = AK7_TRCMEM_A;     // write to  k7's addr for read -> reading from AK7_TRCMEM_A channel header memory, next 16bit
                                 w0 = mapped[AMZ_EXDRD];      // read 16 bits
                                 mapped[AMZ_EXAFRD] = AK7_TRCMEM_B;     // write to  k7's addr for read -> reading from AK7_TRCMEM_B channel header memory, high 16bit and addr increase
                                 w1 = mapped[AMZ_EXDRD];      // read 16 bits
                         
                                // re-order 2 sample words from 32bit FIFO
                                wf[k] = w0+(w1<<16);
                               // wf[2*k+0] = w0;
                             }
   
                             fwrite( wf, TL[ch]/2, 4, fil );   
                              
                             /*
                             //printf( "Trace 0-3 %d %d %d %d\n", wf[0], wf[1], wf[2], wf[3] );
                             //printf( "Trace 4-7 %d %d %d %d\n", wf[4], wf[5], wf[6], wf[7] );
                              for( k=0; k < (TL[ch]/2); k++)
                             {
                               printf( "Trace %d :  %d \n", 2*k+0, wf[k]&0xFFFF  );
                               printf( "Trace %d :  %d \n", 2*k+1, wf[k]>>16 );
                             }     */

                           }   // end trace read and save
                   
                  }      // 0x400     
                  
                  eventcount++;             
     //          }
     //          else { // event not acceptable (piled up 
     //             R1 = mapped[chaddr+CA_REJECT];		// read this register to advance event FIFOs without incrementing Nout etc
     //          }
            }     // end event in this channel
         }        //end for ch
      }           // end event in any channel



        // ----------- Periodically save MCA, PSA, and Run Statistics  -----------
       
        if(loopcount % PollTime == 0) 
        {
        /*
            // 1) Run Statistics 
            mapped[AOUTBLOCK] = OB_RSREG;

            // for debug purposes, print to std out so we see what's going on
            k = 3;    // no loop for now
            {
              m  = mapped[ARS0_MOD+k];
              c0 = mapped[ARS0_CH0+k];
              c1 = mapped[ARS0_CH1+k];
              c2 = mapped[ARS0_CH2+k];
              c3 = mapped[ARS0_CH3+k];
              printf("%s,%u,%s,%u,%u,%u,%u\n ","RunTime",m,"COUNTTIME",c0,c1,c2,c3);        
            }

            // print (small) set of RS to file, visible to web
            read_print_runstats(1, 0, mapped);

            mapped[AOUTBLOCK] = OB_EVREG;     // read from event registers
         */   

            // 2) MCA
            filmca = fopen("MCA.csv","w");
            fprintf(filmca,"bin,MCAch0,MCAch1,MCAch2,MCAch3\n");
            for( k=0; k <WEB_MCA_BINS; k++)       // report the 4K spectra during the run (faster web update)
            {
               fprintf(filmca,"%d,%u,%u,%u,%u\n ", k*onlinebin,wmca[0][k],wmca[1][k],wmca[2][k],wmca[3][k]);
            }
            fclose(filmca);    
         /*
           // 3) 2D MCA or PSA
            if(RunType==0x502)   {
               filmca = fopen("psa2D.csv","w");

               // title row (x index)
               for( ch=0; ch <NCHANNELS; ch++)       
               {
                  for( binx=0;binx<MCA2D_BINS;binx++)
                  {
                     fprintf(filmca,",%d",binx+MCA2D_BINS*ch);
                  }
               }    // channel loop
               fprintf(filmca,"\n");
              
               for( biny=0;biny<MCA2D_BINS;biny++)
               {
                  fprintf(filmca, "%d",biny);        // beginning of line 
                  for( ch=0; ch <NCHANNELS; ch++)       
                  {
                     for( binx=0;binx<MCA2D_BINS;binx++)
                     {
                        fprintf(filmca,",%d",mca2D[ch][biny+MCA2D_BINS*binx]);
                     }  // binx loop
                  }    // channel loop
                  fprintf(filmca,"\n");            // end of line
         
               }  // biny loop

               fclose(filmca); 
            }  // runtype 0x502    */   
        }

          // ----------- loop housekeeping -----------




         loopcount ++;
         currenttime = time(NULL);
    //           usleep(100);
    //           printf( "currenttime: %d\n", currenttime );
   //   } while (currenttime <= starttime+ReqRunTime); // run for a fixed time   
      } while (eventcount <= 2); // run for a fixed number of events   



   // ********************** Run Stop **********************

   // set nLive bit to stop run
    mapped[ACSRIN] = 0x0000; // all off       
   // todo: there may be events left in the buffers. need to stop, then keep reading until nothing left
                      
   // final save MCA and RS
   filmca = fopen("MCA.csv","w");
   fprintf(filmca,"bin,MCAch0,MCAch1,MCAch2,MCAch3\n");
   for( k=0; k <MAX_MCA_BINS; k++)
   {
      fprintf(filmca,"%d,%u,%u,%u,%u\n ", k,mca[0][k],mca[1][k],mca[2][k],mca[3][k] );
   }
   fclose(filmca);

   /*
   mapped[AOUTBLOCK] = OB_RSREG;
   read_print_runstats(0, 0, mapped);
   mapped[AOUTBLOCK] = OB_IOREG;

   // 3) 2D MCA
   if(RunType==0x502)   {
      filmca = fopen("psa2D.csv","w");

      // title row (x index)
      for( ch=0; ch <NCHANNELS; ch++)       
      {
         for( binx=0;binx<MCA2D_BINS;binx++)
         {
            fprintf(filmca,",%d",binx+MCA2D_BINS*ch);
         }
      }    // channel loop
      fprintf(filmca,"\n");
     
      for( biny=0;biny<MCA2D_BINS;biny++)
      {
         fprintf(filmca, "%d",biny);        // beginning of line 
         for( ch=0; ch <NCHANNELS; ch++)       
         {
            for( binx=0;binx<MCA2D_BINS;binx++)
            {
               fprintf(filmca,",%d",mca2D[ch][biny+MCA2D_BINS*binx]);
            }  // binx loop
         }    // channel loop
         fprintf(filmca,"\n");            // end of line

      }  // biny loop

      fclose(filmca); 
   }  // runtype 0x502

   */
 
 // clean up  
 if( (RunType==0x100) )  { 
   fclose(fil);
 }
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}
