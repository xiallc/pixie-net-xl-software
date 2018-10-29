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

// gcc -Wall testreads.c PixieNetCommon.o -o testreads


#include "PixieNetDefs.h"
#include "PixieNetCommon.h"

int main( int argc, char *argv[] ) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;

//  unsigned int val = 0;
//  unsigned int addr = 0;
  unsigned int mval = 0;
  unsigned int k;
  unsigned int chsel, regno;


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

   // ************************ parse arguments *********************************
 /*
   if( argc!=3)  {
     printf( "please give arguments addr (0x###) for test read/write and value (decimal)  \n" );
     return 2;
   }

   addr = strtol(argv[1], NULL, 16);
   val = strtol(argv[2], NULL, 10);

   */

   // ************************ set up controller registers for external R/W *********************************


 
  mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 


   mapped[AMZ_EXAFWR] = 5;     // write to  k7's addr     addr 5 = SPI
   mval = 0 << 15;                  // SPI write (high bit=0)
   mval = mval + (0x03 << 8);       // SPI reg address  (bit 14:8)
   mval = mval + 0x00;              // test pattern on, pattern bits [13:8]  = 0A
   mapped[AMZ_EXDWR] = mval;                                 //  write to ADC SPI
         usleep(5);


  mapped[AMZ_EXAFWR] = 5;     // write to  k7's addr     addr 5 = SPI
   mval = 0 << 15;                  // SPI write (high bit=0)
   mval = mval + (0x04 << 8);       // SPI reg address  (bit 14:8)
   mval = mval + 0x01;              // test pattern on, pattern bits [7:0]  = BC
   mapped[AMZ_EXDWR] = mval;                                 //  write to ADC SPI

  
  /*chsel = 0;
  regno = 14 ;

  // select sys registers
  mapped[AMZ_EXAFWR] = 3;     // write to  k7's addr     addr 3 = channel/syste, select
  mapped[AMZ_EXDWR] = chsel;                                 //  0x000  =system 
 

   
  // read from ext sys O regs
  // sys time counter
  for(k=0;k<16;k++) {
      mapped[AMZ_EXAFRD] = regno+0x80;     // write to  k7's addr
        usleep(1);
      mval = mapped[AMZ_EXDRD]; 
     printf( "K7 0 read from 0x%x (chsel=0x%x): %d\n", regno+0x80, chsel,mval );
    //    usleep(5);
  }
    */


   /*
   // select ch registers
    chsel = 0x101;
    regno = 3 ;
   mapped[AMZ_EXAFWR] = 3;     // write to  k7's addr        addr 3 = channel/syste, select    
   mapped[AMZ_EXDWR] = chsel;                                //  0x100  =channel 0                  
 
   // read from ext ch O regs reg 4 = ADC
  for(k=0;k<16;k++) {
      mapped[AMZ_EXAFRD] = regno+0xC0;     // write to  k7's addr
        usleep(1);
      mval = mapped[AMZ_EXDRD]; 
     printf( "K7 0 read from 0x%x (chsel=0x%x): %d\n", regno+0xC0, chsel,mval );
    //    usleep(5);
  }

    */

 
  mapped[AOUTBLOCK] = CS_MZ;	  // deselect FPGA 0  
 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










