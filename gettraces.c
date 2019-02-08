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

int main(void) {		 

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;
  int k,ch, k7;
  FILE * fil;
  unsigned int adc[NCHANNEL_PER_K7*N_K7_FPGAS][NTRACE_SAMPLES];
  unsigned int cs[N_K7_FPGAS] = {CS_K0,CS_K1};


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


   // **************** XIA code begins **********************
   
   // read 8K samples from ADC register 
   // at this point, no guarantee that sampling is truly periodic
   for(k7=0;k7<N_K7_FPGAS;k7++)
   {
      mapped[AMZ_DEVICESEL] =  cs[k7];	            // select FPGA 
      
      for(ch=0;ch<NCHANNEL_PER_K7;ch++) {
      
         mapped[AMZ_EXAFWR] = AK7_PAGE;     // write to  k7's addr        addr 3 = channel/syste, select    
         mapped[AMZ_EXDWR] = PAGE_CHN+ch;                                //  0x100  =channel 0                  
         
         for(k=0;k<NTRACE_SAMPLES;k++) {
            mapped[AMZ_EXAFRD] = AK7_ADC;     // write to  k7's addr
            //      usleep(1);
            adc[ch+k7*NCHANNEL_PER_K7][k] = mapped[AMZ_EXDRD]; 
         }       //    end for NTRACE_SAMPLES 
      } // end for channels
   } //end for K7s

  // open the output file
  fil = fopen("ADC.csv","w");
  fprintf(fil,"sample,adc0,adc1,adc2,adc3,adc4,adc5,adc6,adc7\n");

  //  write to file
  for( k = 0; k < NTRACE_SAMPLES; k ++ )
  {
      fprintf(fil,"%d",k);                  // sample number
      for(k7=0;k7<N_K7_FPGAS;k7++)
         for(ch=0;ch<NCHANNEL_PER_K7;ch++) 
             fprintf(fil,",%d",adc[ch+k7*NCHANNEL_PER_K7][k]);    // print channel data
      fprintf(fil,"\n");
   }
 
 
 // clean up  
 fclose(fil);
 munmap(map_addr, size);
 close(fd);
 return 0;
}
