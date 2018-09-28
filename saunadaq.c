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

 /* This code manages a data acquisition specifically for the SAUNA system. 
    It is modified from the standard code as follows
    - runtype 0x301, but collects data in MCA mode and coincidence mode without waveforms
    - saves 1D and 2D singles and coincidence spectra in binary format

*/

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
// use command line:  make -f makefile_SAUNA

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

  #define mca1Dsize     8192
  #define web1Dsize     256
  #define mca2Dgbsize   256
  #define mcapossize    257
  #define mca2Dbbsize   128
  #define SAUNA_WEB_LOGEBIN   5  // log2 ( mca1Dsize/ web1Dsize)
  #define SAUNAgammabinfactor 8  // binning factors (2^N) for 2D spectrum
  #define SAUNAbetabinfactor  8


  unsigned int Accept, SyncT, ReqRunTime, PollTime;
  unsigned int SL[NCHANNELS];
  //unsigned int SG[NCHANNELS];
  float Tau[NCHANNELS], Dgain[NCHANNELS];
  unsigned int BLavg[NCHANNELS], BLcut[NCHANNELS], Binfactor[NCHANNELS]; //, TL[NCHANNELS];
  double C0[NCHANNELS], C1[NCHANNELS], Cg[NCHANNELS];
  double baseline[NCHANNELS] = {0};
  double dt, ph; // bscale;
  double elm, q;
  time_t starttime, currenttime;
  unsigned int m, c0, c1, c2, c3;
  unsigned int evstats, bin; 
  unsigned int RunType;//, startTS;
  unsigned int hit[NCHANNELS], energy [NCHANNELS];
  // unsigned int hit[NCHANNELS], timeL[NCHANNELS], energy [NCHANNELS], ev_timeH, ev_timeL;
  unsigned int lsum, tsum, gsum;
  unsigned int mcag[mca1Dsize] ={0};    // singles MCAs for end of run
  unsigned int mcabs[mca1Dsize] ={0}; 
  unsigned int mcab1[mca1Dsize] ={0}; 
  unsigned int mcab2[mca1Dsize] ={0}; 
  unsigned int mcapos[mca1Dsize] ={0};
  unsigned int wmca[NCHANNELS][web1Dsize] ={{0}};    // smaller sigles MCAs for live update during run
  unsigned int mcagb[mca2Dgbsize*mca2Dgbsize] ={0};  // 2D coinc spectrum: gamma-beta
  unsigned int mcabb[mca2Dbbsize*mca2Dbbsize] ={0};  // 2D coinc spectrum: beta-beta
  unsigned int mca2D[MCA2D_BINS*MCA2D_BINS] ={0};    // 2D MCA for end of run
  unsigned int Eg, Ebs, Eb1, Eb2, binx, biny;
  double pos; 
    
  unsigned int onlinebin;
  unsigned int chaddr, loopcount, eventcount;
  unsigned int Nchok; // unsigned int PPStime, Nchok;
  unsigned int BLbad[NCHANNELS];
  unsigned int mval;
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
  Accept       = fippiconfig.ACCEPT_PATTERN;
  RunType      = fippiconfig.RUN_TYPE;
  SyncT        = fippiconfig.SYNC_AT_START;
  ReqRunTime   = fippiconfig.REQ_RUNTIME;
  PollTime     = fippiconfig.POLL_TIME;
//
    if( (fippiconfig.RUN_TYPE == 0x400) ||
        (fippiconfig.RUN_TYPE == 0x402) ||
        (fippiconfig.RUN_TYPE == 0x500) ||
        (fippiconfig.RUN_TYPE == 0x501) ||
        (fippiconfig.RUN_TYPE == 0x502) ||
        (fippiconfig.RUN_TYPE == 0x301)   )
    {
     printf( "This funtion only supports runtype 0x503.\n");
     return -1;
  }


  for( k = 0; k < NCHANNELS; k ++ )
  {
      SL[k]          = (int)floor(fippiconfig.ENERGY_RISETIME[k]*FILTER_CLOCK_MHZ);      // multiply time in us *  # ticks per us = time in ticks
//    SG[k]          = (int)floor(fippiconfig.ENERGY_FLATTOP[k]*FILTER_CLOCK_MHZ);       // multiply time in us *  # ticks per us = time in ticks
      Dgain[k]       = fippiconfig.DIG_GAIN[k];
//    TL[k]          = BLOCKSIZE_400*(int)floor(fippiconfig.TRACE_LENGTH[k]*ADC_CLK_MHZ/BLOCKSIZE_400);       // multiply time in us *  # ticks per us = time in ticks, multiple of 4
      Binfactor[k]   = fippiconfig.BINFACTOR[k];
      Tau[k]         = fippiconfig.TAU[k];
      BLcut[k]       = fippiconfig.BLCUT[k];
      BLavg[k]       = fippiconfig.BLAVG[k];
      if(BLavg[k]<0)          BLavg[k] = 0;
      if(BLavg[k]==65536)     BLavg[k] = 0;
      if(BLavg[k]>MAX_BLAVG)  BLavg[k] = MAX_BLAVG;
      BLbad[k] = MAX_BADBL;   // initialize to indicate no good BL found yet
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

    // ********************** Run Start **********************

  // set a few parameters specific for this run
//   mval = mapped[ACOINCPATTERN];
//   mval = mval | 0x70000;                 // set LM402, CWGROUP, MCA/notrace bits
//   mapped[ACOINCPATTERN] = mval;


 //  NumPrevTraceBlks = 0;
   loopcount =  0;
   eventcount = 0;
   starttime = time(NULL);                         // capture OS start time
   if(SyncT) mapped[ARTC_CLR] = 1;              // write to reset time counter
   mapped[AOUTBLOCK] = 2;
   //startTS = mapped[AREALTIME];   
 
   mapped[ADSP_CLR] = 1;             // write to reset DAQ buffers
   mapped[ACOUNTER_CLR] = 1;         // write to reset RS counters
   mapped[ACSRIN] = 1;               // set RunEnable bit to start run
   mapped[AOUTBLOCK] = OB_EVREG;     // read from event registers
    

    // ********************** Run Loop **********************
   do {

      //----------- Periodically read BL and update average -----------
      // this will be moved into the FPGA soon
      if(loopcount % BLREADPERIOD == 0) {
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

      // -----------poll for events -----------
      // if data ready. read out, compute E, increment MCA *********
      
      evstats = mapped[AEVSTATS];
      if( (evstats & 0xF)==0xF)  { // LM402:  if there are events in _every_ channel

         Nchok = 0;
         for( ch=0; ch < NCHANNELS; ch++)          // first loop: check if ALL acceptable
         {
            // read hit pattern and status info
               chaddr = ch*16+16;
               hit[ch]   = mapped[chaddr+CA_HIT];
            // printf("channel %d, hit 0x%x\n",ch,hit[ch]);
               if(hit[ch] & Accept) Nchok = Nchok+1;
         }

         if(Nchok !=4)        // event not acceptable (e.g. piled up )
         { 
            for( ch=0; ch < NCHANNELS; ch++)          // second loop: clear ALL
            {
               chaddr = ch*16+16;
               hit[ch] = mapped[chaddr+CA_REJECT];		// read this register to advance event FIFOs without incrementing Nout etc
            }
         } 
         else
         {

  //          ev_timeL = mapped[AEVTSL];      // event time low
  //          ev_timeH = mapped[AEVTSH];      // event time high
  //          PPStime =  mapped[AEVPPS];      // PPS time latched by event

            for( ch=0; ch < NCHANNELS; ch++)        // third loop: read and store
            {
               // read hit pattern and status info
               chaddr = ch*16+16;
               if(1) {           
                  // read data not needed for pure MCA runs
   //               timeL[ch]  = mapped[chaddr+CA_TSL] >> 8;   // local time (lower 24 bits)
                                  
                  // read raw energy sums 
                  lsum  = mapped[chaddr+CA_LSUM];        // leading, larger, "S1", past rising edge
                  tsum  = mapped[chaddr+CA_TSUM];        // trailing, smaller, "S0" before rising edge
                  gsum  = mapped[chaddr+CA_GSUM];		   // gap sum, "Sg", during rising edge; also advances FIFO and increments Nout etc
                  
                  // compute E
                  ph = C1[ch]*(double)lsum+Cg[ch]*(double)gsum+C0[ch]*(double)tsum;
                  ph = ph-baseline[ch];
                  if ((ph<0.0)|| (ph>65536.0))	ph =0.0;	   // out of range energies -> 0
                  energy[ch] = (int)floor(ph);
                  if ((hit[ch] & (1 << HIT_LOCALHIT) )==0)	  	energy[ch] =0;	   // not a local hit -> 0

 /*                 //  histogramming if E< max mcabin
                  bin = energy[ch] >> Binfactor[ch];
                  if( (bin<MAX_MCA_BINS) && (bin>0) ) {
                     mca[ch][bin] =  mca[ch][bin] + 1;	// increment mca
                     bin = bin >> WEB_LOGEBIN;
                     if(bin>0) wmca[ch][bin] = wmca[ch][bin] + 1;	// increment wmca


                    // TODO: sum spectrum 
                  } // end MCA
  */              }    // end unconditional if
            }       // end for ch 
 
            // SAUNA/Xe histogramming
            Eg  = energy[1];
            Eb1 = energy[2];
            Eb2 = energy[3];
            Ebs = Eb1+Eb2;

            // 1D spectra
            bin = Eg >> Binfactor[1];   /* in SAUNA, binfactor1 is for gamma singles */
            if( (bin<mca1Dsize) && (bin>0) ) {
                 mcag[bin] =  mcag[bin] + 1;	// increment mca
                 bin = bin >> SAUNA_WEB_LOGEBIN;     // downshift for smaller web mca
                 if(bin>0) wmca[0][bin] = wmca[0][bin] + 1;	// increment wmca  (ch.0 -> gamma)
            } // end MCA

            bin = Ebs >> Binfactor[0];   /* in SAUNA, binfactor0 is for beta sum*/
            if( (bin<mca1Dsize) && (bin>0) ) {
                 mcabs[bin] =  mcabs[bin] + 1;	// increment mca
                 bin = bin >> SAUNA_WEB_LOGEBIN;     // downshift for smaller web mca
                 if(bin>0) wmca[1][bin] = wmca[1][bin] + 1;	// increment wmca  (ch.1 -> beta sum)
            } // end MCA

            bin = Eb1 >> Binfactor[2];   /* in SAUNA, binfactor2 is for beta 1*/
            if( (bin<mca1Dsize) && (bin>0) ) {
                 mcab1[bin] =  mcab1[bin] + 1;	// increment mca
                 bin = bin >> SAUNA_WEB_LOGEBIN;     // downshift for smaller web mca
                 if(bin>0) wmca[2][bin] = wmca[2][bin] + 1;	// increment wmca  (ch.2 -> beta 1)
            } // end MCA

            bin = Eb2 >> Binfactor[3];   /* in SAUNA, binfactor3 is for beta 2*/
            if( (bin<mca1Dsize) && (bin>0) ) {
                 mcab2[bin] =  mcab2[bin] + 1;	// increment mca
                 bin = bin >> SAUNA_WEB_LOGEBIN;     // downshift for smaller web mca
                 if(bin>0) wmca[3][bin] = wmca[3][bin] + 1;	// increment wmca  (ch.3 -> beta 2)
            } // end MCA

            // POS spectrum
            pos = (double)Eb1 / ((double)(Eb1+Eb2)) * 256.0;
            bin = (int)floor(pos);
            if( (bin<mcapossize) && (bin>=0) ) {
                 mcapos[bin] =  mcapos[bin] + 1;	// increment mca
            } // end MCA

            // 2D beta-beta spectrum
            binx =  Eb1 >> (SAUNAbetabinfactor+1);     // downshift for available spectrum bins
            biny =  Eb2 >> (SAUNAbetabinfactor+1); 
            if( (binx<mca2Dbbsize) && (binx>0) && (biny<mca2Dbbsize) && (biny>0) ) {
                 bin =  binx+mca2Dbbsize*biny;
                 mcabb[bin] = mcabb[bin] +1;
            } // end MCA

            // 2D gamma-beta spectrum
            binx =  Ebs >> (SAUNAbetabinfactor);     // downshift for available spectrum bins
            biny =  Eg  >> (SAUNAgammabinfactor); 
            if( (binx<mca2Dgbsize) && (binx>0) && (biny<mca2Dgbsize) && (biny>0) ) {
                 bin = binx+mca2Dgbsize*biny;
                 mcagb[bin] = mcagb[bin] +1;
           //      bin = 100+256*55;
           //      mcagb[bin+5] = mcagb[bin+5] +1;     // debug
            } // end MCA

            // smaller gamma-beta 2D MCA for web server, compatible with PSA page
            binx = (int)floor((double)Ebs/fippiconfig.MCA2D_SCALEX[0]);
            biny = (int)floor((double)Eg /fippiconfig.MCA2D_SCALEY[0]);
            if( (binx<MCA2D_BINS) && (biny<MCA2D_BINS) && (binx>0) && (biny>0) ) 
            {
              mca2D[binx+MCA2D_BINS*biny] =   mca2D[binx+MCA2D_BINS*biny] +1; // increment 2D MCA
            }
      
            eventcount++;
         
 /*           // now store list mode data            
            if(RunType==0x503)   {
               R1 = hit[0] | hit[1];   // or hits together and clear channel specific bits
               R1 = R1 | hit[2];
               R1 = R1 | hit[3];
               R1 = R1 & 0x00FFFFFF;
               // fprintf(fil,"Event_No,Hit_Pattern,Event_Time_H,Event_Time_L,PPStime,Time0,Time1,Time2,Time3,Energy0,Energy1,Energy2,Energy3 \n");
               fprintf(fil,"%u,0x%X,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",eventcount,R1,ev_timeH,ev_timeL,PPStime,timeL[0],timeL[1],timeL[2],timeL[3],energy[0],energy[1],energy[2],energy[3]);
            }    // 0x503
 */
         }          // end acceptable in all 4 channels       
      }           // end event in 4 channels



        // ----------- Periodically save MCA, PSA, and Run Statistics  -----------
       
        if(loopcount % PollTime == 0) 
        {
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
            
            // 2) MCA singles for web server (csv)
            filmca = fopen("MCA.csv","w");
            fprintf(filmca,"bin,gamma,betasum,beta1,beta2\n");
            for( k=0; k <web1Dsize; k++)       // report the small spectra as csv during the run (faster web update)
            {
               fprintf(filmca,"%d,%u,%u,%u,%u\n ", k,wmca[0][k],wmca[1][k],wmca[2][k],wmca[3][k]);
            }
            fclose(filmca);   
 
            // 3) MCA 2D for web server (csv)
            filmca = fopen("psa2D.csv","w");

            // title row (x index)
            for( binx=0;binx<MCA2D_BINS;binx++) 
               fprintf(filmca,",%d",binx);
            fprintf(filmca,"\n");
           
            for( biny=0;biny<MCA2D_BINS;biny++)
            {
               fprintf(filmca, "%d",biny);        // beginning of line 

               for( binx=0;binx<MCA2D_BINS;binx++)
                  fprintf(filmca,",%d",mca2D[biny+MCA2D_BINS*binx]);

               fprintf(filmca,"\n");            // end of line      
            }  // biny loop

            fclose(filmca); 
   
            // 4) binary MCA array as in Pixie-4(e)
            fil = fopen("spectra.mca","wb");     
            fwrite( mcag,   4, mca1Dsize, fil );
            fwrite( mcabs,  4, mca1Dsize, fil );
            fwrite( mcab1,  4, mca1Dsize, fil );
            fwrite( mcab2,  4, mca1Dsize, fil );
            fwrite( mcapos, 4, mca1Dsize, fil );
            fwrite( mcapos, 4, mca1Dsize, fil ); // duplicate: dummy fill
            fwrite( mcabb,  4, (mca2Dbbsize*mca2Dbbsize), fil );
            fwrite( mcagb,  4, (mca2Dgbsize*mca2Dgbsize), fil );
            fclose(fil);


        }

          // ----------- loop housekeeping -----------

         loopcount ++;
         currenttime = time(NULL);
     } while (currenttime <= starttime+ReqRunTime); // run for a fixed time   
 //       } while (eventcount <= 35); // run for a fixed number of events   



   // ********************** Run Stop **********************

   // clear RunEnable bit to stop run
   mapped[ACSRIN] = 0;               
   // todo: there may be events left in the buffers. need to stop, then keep reading until nothing left
                      
   // final save MCA and RS

   // 1) final (full) runstats
   mapped[AOUTBLOCK] = OB_RSREG;
   read_print_runstats(0, 0, mapped);
   mapped[AOUTBLOCK] = OB_IOREG;

   // 2) MCA singles for web server (csv)
   filmca = fopen("MCA.csv","w");
   //fprintf(filmca,"bin,MCAch0,MCAch1,MCAch2,MCAch3\n");
   fprintf(filmca,"bin,gamma,betasum,beta1,beta2\n");
   for( k=0; k <web1Dsize; k++)       // report the small spectra as csv during the run (faster web update)
   {
      fprintf(filmca,"%d,%u,%u,%u,%u\n ", k,wmca[0][k],wmca[1][k],wmca[2][k],wmca[3][k]);
   }
   fclose(filmca);   

   // 3) MCA 2D for web server (csv)
   filmca = fopen("psa2D.csv","w");
   
   // title row (x index)
   for( binx=0;binx<MCA2D_BINS;binx++) 
      fprintf(filmca,",%d",binx);
   fprintf(filmca,"\n");
  
   for( biny=0;biny<MCA2D_BINS;biny++)
   {
      fprintf(filmca, "%d",biny);        // beginning of line 

      for( binx=0;binx<MCA2D_BINS;binx++)
         fprintf(filmca,",%d",mca2D[biny+MCA2D_BINS*binx]);

      fprintf(filmca,"\n");            // end of line      
   }  // biny loop

   fclose(filmca); 

   // 4) binary MCA array as in Pixie-4(e)
   fil = fopen("spectra.mca","wb");
   fwrite( mcag,   4, mca1Dsize, fil );
   fwrite( mcabs,  4, mca1Dsize, fil );
   fwrite( mcab1,  4, mca1Dsize, fil );
   fwrite( mcab2,  4, mca1Dsize, fil );
   fwrite( mcapos, 4, mca1Dsize, fil );
   fwrite( mcapos, 4, mca1Dsize, fil ); // duplicate: dummy fill
   fwrite( mcabb,  4, (mca2Dbbsize*mca2Dbbsize), fil );
   fwrite( mcagb,  4, (mca2Dgbsize*mca2Dgbsize), fil );
   fclose(fil);
 /*  filmca = fopen("MCA.csv","w");
   fprintf(filmca,"bin,MCAch0,MCAch1,MCAch2,MCAch3\n");
   for( k=0; k <MAX_MCA_BINS; k++)
   {
      fprintf(filmca,"%d,%u,%u,%u,%u\n ", k,mca[0][k],mca[1][k],mca[2][k],mca[3][k] );
   }
   fclose(filmca);
*/
   
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}
