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

// gcc -Wall adcinit.c -o adcinit


#include "PixieNetDefs.h"
#include "PixieNetCommon.h"

int main( int argc, char *argv[] ) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;

  unsigned int mval = 0;
  unsigned int k, trys;
  unsigned int upper, lower;
  unsigned int chsel, regno, frame;

 unsigned int adc[4][14];
 int ch;


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

  

  // ======================= ADC SPI programming =======================

/*  // LVDS drive strength

  mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 

   mapped[AMZ_EXAFWR] = 5;     // write to  k7's addr     addr 5 = SPI
   mval = 0 << 15;                  // SPI write (high bit=0)
   mval = mval + (0x02 << 8);       // SPI reg address  (bit 13:8)
   mval = mval + 0x40;              // test pattern on, pattern bits [13:8]  = 0A
   mapped[AMZ_EXDWR] = mval;                                 //  write to ADC SPI
         usleep(5);
 */

 trys = 0;
 frame= 0;

 do {
      // read frame 
      mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 
      chsel = 0x000;               // sys range
      regno = 6 ;                  // register 6  = ADC frame
      mapped[AMZ_EXAFWR] = 3;     // write to  k7's addr        addr 3 = channel/system, select    
      mapped[AMZ_EXDWR] = chsel;                                //  0x100  =channel 0                  
      mapped[AMZ_EXAFRD] = regno+0x80;     // write register address to  K7
      usleep(1);
      frame = mapped[AMZ_EXDRD]; 
      printf( "frame pattern is 0x%x (try %d) \n", frame, trys);
      
      
      
 
      for(k=0;k<14;k++) {
      
         // set up test pattern    
         mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 
         
         if(k<8) {
            upper = 0x0;
            lower = (1<<k);
         } else {
            upper = (1<<(k-8));
            lower = 0x0;
         }         
         
         mapped[AMZ_EXAFWR] = 5;     // write to  k7's addr     addr 5 = SPI
         mval = 0 << 15;                  // SPI write (high bit=0)
         mval = mval + (0x03 << 8);       // SPI reg address  (bit 13:8)
         mval = mval + 0x80 +upper;              // test pattern on, pattern bits [13:8]  = 0A
         mapped[AMZ_EXDWR] = mval;                                 //  write to ADC SPI
         usleep(5);
         
         mapped[AMZ_EXAFWR] = 5;     // write to  k7's addr     addr 5 = SPI
         mval = 0 << 15;                  // SPI write (high bit=0)
         mval = mval + (0x04 << 8);       // SPI reg address  (bit 14:8)
         mval = mval + lower;              // test pattern on, pattern bits [7:0]  = BC
         mapped[AMZ_EXDWR] = mval;                                 //  write to ADC SPI
         
         //  printf( "test pattern is 0x%x \n", (addr<<8));
         usleep(5);
         

         // read 1 sample from ADC register          
         mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 
         chsel = 0x100;               // start with channel 0
         regno = 4 ;                  // register 4  = ADC
         
         for(ch=0;ch<4;ch++) {    
            mapped[AMZ_EXAFWR] = 3;     // write to  k7's addr        addr 3 = channel/syste, select    
            mapped[AMZ_EXDWR] = chsel+ch;                                //  0x100  =channel 0                         
            mapped[AMZ_EXAFRD] = regno+0xC0;     // write register address to  K7
            usleep(1);
            adc[ch][k] = mapped[AMZ_EXDRD];      
         } // end for channels
         
         printf( "test pattern 0x%04x: adc0 0x%04x, adc1 0x%04x, adc2 0x%04x, adc3 0x%04x \n", (upper<<8)+lower, adc[0][k],adc[1][k],adc[2][k],adc[3][k] );
      }    // end for tespatterns
      
      
      // turn testpattern off again
      mapped[AMZ_EXAFWR] = 5;     // write to  k7's addr     addr 5 = SPI
      mval = 0 << 15;                  // SPI write (high bit=0)
      mval = mval + (0x03 << 8);       // SPI reg address  (bit 13:8)
      mval = mval + 0;              // test pattern off
      mapped[AMZ_EXDWR] = mval;                                 //  write to ADC SPI
      usleep(5);


      if(frame!=0xe1) {
         // trigger a bitslip
         mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 
         
         chsel = 0x000;               // sys range
         regno = 6 ;                  // register 6  = ADC frame
         
         mapped[AMZ_EXAFWR] = 3;     // write to  k7's addr        addr 3 = channel/system, select    
         mapped[AMZ_EXDWR] = chsel;                                //  0x000  = system range                  
         
         mapped[AMZ_EXAFWR] = 0x6;     // write register address to  K7
         mapped[AMZ_EXDWR] = chsel;    // any write will do
      }

       trys = trys+1;

    } while(frame!=0xe1 && trys<7);
         





 
  mapped[AOUTBLOCK] = CS_MZ;	  // deselect FPGA 0  
 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}









