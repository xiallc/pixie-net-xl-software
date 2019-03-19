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
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <math.h>
// need to compile with -lm option

#include "PixieNetDefs.h"
#include "PixieNetCommon.h"


int main(void) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;
  int k, addr, ch, adc, dac, tmp;
  unsigned int mval, bit;

  //unsigned int mins[NCHANNELS_PRESENT] = {4192,4192,4192,4192};
  //unsigned int mint[NCHANNELS_PRESENT] = {4192,4192,4192,4192};
  unsigned int mins[NCHANNELS_PRESENT] = {16384,16384,16384,16384};     //TODO  adjust max ADC per HW variant
  unsigned int mint[NCHANNELS_PRESENT] = {16384,16384,16384,16384};     //TODO  adjust max ADC per HW variant
 // unsigned int readdr[NCHANNELS_PRESENT] = {AADC0,AADC1,AADC2,AADC3};
  unsigned int targetdac[NCHANNELS_PRESENT] = {0,0,0,0};
  // unsigned int targetBL[NCHANNELS_PRESENT] = {400,400,400,400};     // TODO: BL% read from ini file, compute 
  unsigned int targetBL[NCHANNELS_PRESENT] = {1600,1600,1600,1600};     // TODO: BL% read from ini file, compute 
  double dacadj;
  unsigned int oldadc, adcchanged, saveaux, revsn;
  int k7, ch_k7;
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};

    unsigned int trys;
  unsigned int frame; // regno;
  unsigned int goodframe = 0x87;     // depends on FPGA compile?

  unsigned int ADCmax, DACstep, DACstart; 





  // *************** PS/PL IO initialization *********************
  // open the device for PD register I/O
  fd = open("/dev/uio0", O_RDWR);
  if (fd < 0) {
    perror("Failed to open devfile");
    return 1;
  }

  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return 1;
  }

  mapped = (unsigned int *) map_addr;


  // ******************* Main code begins ********************

   // ***** check HW info *********
   revsn = hwinfo(mapped,I2C_SELMAIN);   // assuming all DBs are the same!

   //TODO  adjust max ADC per HW variant
   ADCmax = 16384;      // adjust 
   DACstep = 256;
   DACstart = 27648;

  mapped[AMZ_DEVICESEL] =  CS_MZ;      // select MZ controller	
  saveaux = mapped[AAUXCTRL];
  mapped[AAUXCTRL] = 0;    // turn off pulser, other stuff
  mapped[AMZ_CSRIN] = 0x0000; // all off

  // ----------- swap channels 0<>1 and 2<>3 if necessary  -------------
  if( (revsn & PNXL_DB_VARIANT_MASK)==PNXL_DB02_12_250)  // if DB02, need to check for swapped channels from ADC
  {
     printf("Checking for swapped channels ...\n");
     for(k7=0;k7<N_K7_FPGAS;k7++)
     {
     
        for( ch = 0; ch < NCHANNEL_PER_K7; ch ++ )
        {

            ch_k7 = ch_k7;
           // find if change of DAC changes ADC
           adc = 0;
           oldadc = 0;
           adcchanged = 0;
           dac = 0; 
           k=0;
      
           // scan through DAC settings
           do {

               mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller
               mapped[AMZ_FIRSTDAC+ch_k7] = dac;
               usleep(DACWAIT);
               mapped[AMZ_FIRSTDAC+ch_k7] = dac; //;     //TODO: double write required?
               if(mapped[AMZ_FIRSTDAC+ch_k7] != dac) printf("Error writing parameters to DAC register\n");
               usleep(DACSETTLE);		// wait for DAC's RC filter

               mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA
               mapped[AMZ_EXAFWR] = AK7_PAGE;     // write to  k7's addr        addr 3 = channel/syste, select    
               mapped[AMZ_EXDWR]  = PAGE_CHN+ch;                                //  0x100  =channel 0                  
               mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
               usleep(1);
               adc = mapped[AMZ_EXDRD];          // read K7 data from MZ
 


     /*        // Pixie-Net 
               mapped[AOUTBLOCK] = OB_IOREG;	  // read from IO block
               mapped[addr] = dac;
               usleep(DACWAIT);
               mapped[addr] = dac; //;     //TODO: double write required?
               if(mapped[addr] != dac) printf("Error writing parameters to DAC register\n");
               usleep(DACSETTLE);		// wait for DAC's RC filter
      
               mapped[AOUTBLOCK] = OB_EVREG;		// switch reads to event data block of addresses
               adc = (mapped[readdr[ch]] & 0xFFFF);  // dummy read to refresh read register
               adc = (mapped[readdr[ch]] & 0xFFFF);

               */
               if (k==0)   {
                  oldadc = adc;
               } else {
                  if ( abs(oldadc-adc)>200)  adcchanged = 1;  // look for a change > 200 steps. Not foolproof with pulses!
               }
               k=k+1;
               dac = dac+4096;
            //   printf("Channel %u: DAC value %u, adc %u, adcdiff %d\n",ch,dac,adc,abs(oldadc-adc));

           } while ( (adcchanged==0) & (k<16) );//  dac loop
      
           // check if there was a change, if not, swap channels
           if (k==16)  {
                  bit = 0x0001 << (ch/2);          // compute bit to toggle per ADC channel pair
                  mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA
                  mapped[AMZ_EXAFWR] = AK7_PAGE;     // write to  k7's addr        addr 3 = channel/syste, select    
                  mapped[AMZ_EXDWR]  = PAGE_SYS;                                              
                  mapped[AMZ_EXAFRD] = AK7_ADCCTRL;     // write to  k7's addr
                  usleep(1);
                  mval = mapped[AMZ_EXDRD];          // read K7 dta from MZ

                  mval = mval ^ bit;
                  mapped[AMZ_EXAFWR] = AK7_ADCCTRL;     // write to  k7's addr        addr 3 = channel/syste, select    
                  mapped[AMZ_EXDWR]  = mval;              // swap 0/1                                 

                  // PN mval =  mapped[AADCCTRL];
                  // mval = mval ^ 0x0001;
                  // mapped[AADCCTRL] = mval;	  // swap 0/1
            //   } else {
                //  mval =  mapped[AADCCTRL];
                //  mval = mval ^ 0x0002;
                //  mapped[AADCCTRL] = mval;	  // swap 2/3
           //    }
               printf("Channel %u: ADC values does not change with DAC. Swapped channel inputs\n",ch);
           }
      
        } // endfor  channels
     } // endfor K7s
  }    // end version check
  
  
  // ----------- need to have correct polarity  -------------

  // TODO!

  // ----------- calibrate the ADC bit slip   -------------
  if(1)  // TODO: figure out version ID and do some things only if necessary
  {
    printf("Checking for ADC data frame alignment ...\n");
    printf("Target frame pattern is 0x%02x\n",goodframe); 

    for(k7=0;k7<N_K7_FPGAS;k7++)
    {
       trys = 0;
       frame= 0;
       mapped[AMZ_DEVICESEL] = cs[k7];	            // select FPGA  

       do {
              // read frame 
            mapped[AMZ_EXAFWR] = AK7_PAGE;         // write to  k7's addr        addr 3 = channel/system, select    
            mapped[AMZ_EXDWR] = PAGE_SYS;             //  0x000  = system page                
            mapped[AMZ_EXAFRD] = AK7_ADCFRAME;     // write register address to  K7
            usleep(1);
            frame = mapped[AMZ_EXDRD]; 
            printf( "K7 %d: frame pattern is 0x%x (try %d) \n", k7, frame, trys);
      
            if(frame!=goodframe) {
               // trigger a bitslip         
               mapped[AMZ_EXAFWR] = AK7_ADCBITSLIP;   // write register address to  K7
               mapped[AMZ_EXDWR] = 0;             // any write will do
            }
   
          trys = trys+1;
   
       } while(frame!=goodframe && trys<7);

       if(trys==7) {
         printf("ADC data frame alignment can not be resolved, exiting\n");
         return(-1);
       }

    } // end for K7s
  }   //  end version check 


   

  // ----------- adjust offset: search for two DAC settings with valid ADC response, then extrapolate  -------
  printf("Adjusting DC offsets (correct polarity required) ...\n");
  printf(" still preliminary, not precise, slow ...\n");
   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
     for( ch = 0; ch < NCHANNEL_PER_K7; ch ++ )
     {
        ch_k7 = ch+k7*NCHANNEL_PER_K7;
        dac = DACstart;
        adc = 0;
        k=0;
   
        // 1. find first DAC value with valid response
        do  {
            // set DAC
            mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller
            mapped[AMZ_FIRSTDAC+ch_k7] = dac; //dac;
            usleep(DACWAIT);
            mapped[AMZ_FIRSTDAC+ch_k7] = dac; //;     //TODO: double write required?
            if(mapped[AMZ_FIRSTDAC+ch_k7] != dac) printf("Error writing parameters to DAC register\n");
            usleep(DACSETTLE);		// wait for DAC's RC filter
   
            // read ADC
            mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA
            mapped[AMZ_EXAFWR] = AK7_PAGE;     // write to  k7's addr        addr 3 = channel/syste, select    
            mapped[AMZ_EXDWR] = PAGE_CHN+ch;                                //  0x100  =channel 0                  
            mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
            usleep(1);
            adc = mapped[AMZ_EXDRD];
           
         //   printf("Channel %u: DAC value %u, adc %u\n",ch_k7,dac,adc);
            dac = dac + DACstep;
            k=k+1;
       } while ( ((adc>(unsigned int)floor(0.8*ADCmax)) | (adc<(unsigned int)floor(0.2*ADCmax))) & (dac < 65536)  );    //& (k<33)
     //    } while ( k<33*4  );    //& (k<33)
        dac = dac - DACstep;               // dac is now the lowest valid DAC value
      //  printf("Channel %u: DAC value %u, adc %u\n",ch_k7,dac,adc);
       
     

   
        // 2. get min/max of many samples

         mins[ch_k7] = adc;
        for( k = 0; k < NTRACE_SAMPLES; k ++ )   {

            mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
              usleep(1);
            adc = mapped[AMZ_EXDRD];
            
           // mins[ch] = mins[ch]+ adc/NTRACE_SAMPLES;   // find average
            //avg[ch] = avg[ch]+ adc/NTRACE_SAMPLES;   // find average
           if ( (adc < mins[ch_k7]) && (adc>200 ))  mins[ch_k7] = adc;    // find min but exclude zeros
            //if (adc > maxs[ch])  maxs[ch] = adc;    // find max
        }   
        printf("Channel %u: DAC value %u, min adc read %u\n",ch_k7,dac,mins[ch_k7]);
   

        // 3. change DAC settings
         dac = dac + DACstep;               // new, second dac
         mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller
         mapped[AMZ_FIRSTDAC+ch_k7] = dac; //dac;
         usleep(DACWAIT);
         mapped[AMZ_FIRSTDAC+ch_k7] = dac; //;     //TODO: double write required?
         if(mapped[AMZ_FIRSTDAC+ch_k7] != dac) printf("Error writing parameters to DAC register\n");
         usleep(DACSETTLE);		// wait for DAC's RC filter
   

      // 4. get min/max of many samples
         mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA
         mapped[AMZ_EXAFWR] = AK7_PAGE;     // write to  k7's addr        addr 3 = channel/syste, select    
         mapped[AMZ_EXDWR] = PAGE_CHN+ch;                                //  0x100  =channel 0                  
  //      adc = (mapped[readdr[ch]] & 0xFFFF);  // dummy read to refresh read register
         mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
         usleep(1);
         adc = mapped[AMZ_EXDRD];
         mint[ch_k7] = adc;     
           for( k = 0; k < NTRACE_SAMPLES; k ++ )     
         {
            mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
              usleep(1);
            adc = mapped[AMZ_EXDRD];
           // adc = (mapped[readdr[ch]] & 0xFFFF);
         //   mint[ch] = mint[ch]+ adc/NTRACE_SAMPLES;   // find average
           // avg[ch] = avg[ch]+ adc/NTRACE_SAMPLES;   // find average
            if ((adc < mint[ch_k7])  && (adc>200 ))  mint[ch_k7] = adc;    // find min  but exclude zeros
           // if (adc > maxs[ch])  maxs[ch] = adc;    // find max
        }    

        printf("Channel %u: DAC value %u, min adc read %u\n",ch_k7,dac,mint[ch_k7]);
   

        // 5. compute target dac from 2 points
        dacadj =  DACstep * ((double)targetBL[ch_k7]-(double)mint[ch_k7]) / ((double)mint[ch_k7]-(double)mins[ch_k7]);
      
        tmp = dac + (int)floor(dacadj);
        if( (tmp>0) & (tmp<65536) )
        {
           printf("Channel %u: DAC adjustment %f\n",ch_k7,dacadj  );
           targetdac[ch_k7]  = dac + (int)floor(dacadj*0.8);      // bogus factor 0.8 to not overshoot 
        } else {
             printf("Channel %u: could not find target DAC value\n",ch_k7);
             targetdac[ch_k7]  = dac;
        }
         
      }    // endfor channels
   } //end for K7s




   // 6. set all channels to target and report voltages
    printf("\n");
   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
      for( ch = 0; ch < NCHANNEL_PER_K7; ch ++ )
      {
         ch_k7 = ch+k7*NCHANNEL_PER_K7;;
         addr = AMZ_FIRSTDAC+ch_k7;   

         mapped[AMZ_DEVICESEL] = CS_MZ;	  // select MZ controller
         dac = targetdac[ch_k7];
         mapped[addr] = dac; //dac;
         usleep(DACWAIT);
         mapped[addr] = dac; //;     //TODO: double write required?
         if(mapped[addr] != dac) printf("Error writing parameters to DAC register\n");
         usleep(DACSETTLE);		// wait for DAC's RC filter
   
         mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA
         mapped[AMZ_EXAFWR] = AK7_PAGE;     // write to  k7's addr        addr 3 = channel/syste, select    
         mapped[AMZ_EXDWR] = PAGE_CHN+ch;                                //  0x100  =channel 0    
         
         mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
         adc = mapped[AMZ_EXDRD];
         printf("Channel %u: DAC value %u, offset %fV, ADC %u\n",ch_k7,dac,V_OFFSET_MAX*(1.0-(double)dac/32678.0), adc);
      }  // end channels
   } //end for K7s

   mapped[AMZ_DEVICESEL] = CS_MZ;	  // read from IO block
   mapped[AAUXCTRL] = saveaux;    // turn on pulser TODO: read from file!
    
 
 // clean up  
 munmap(map_addr, size);
 close(fd);
 return 0;
}







