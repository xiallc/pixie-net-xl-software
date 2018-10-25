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

// gcc -Wall testcsrK7.c PixieNetCommon.o -o testcsrK7


#include "PixieNetDefs.h"
#include "PixieNetCommon.h"

int main( int argc, char *argv[] ) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;

  unsigned int val = 0;
  unsigned int addr = 0;
  unsigned int mval = 0;
  unsigned int k;


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

   if( argc!=3)  {
     printf( "please give arguments addr (0x###) for test read/write and value (decimal)  \n" );
     return 2;
   }

   addr = strtol(argv[1], NULL, 16);
   val = strtol(argv[2], NULL, 10);

   // ************************ set up controller registers for external R/W *********************************

/*  mapped[AOUTBLOCK] = CS_MZ;	  // read/write from/to MZ IO block
  mval = mapped[ACOINCPATTERN];	     
  printf( "MZ CP read: 0x%x\n", mval );

  mval = mval+15;
  mapped[ACOINCPATTERN] = mval;	  // change value
  printf( "MZ CP write: 0x%x\n", mval );
  mval = 0;
  mval = mapped[ACOINCPATTERN];	     //read back
  printf( "MZ CP read: 0x%x\n", mval );
*/
  mapped[AOUTBLOCK] = CS_MZ;	  // read/write from/to MZ IO block
  mval = mapped[AMZ_SYSREV];	    
  printf( "MZ SYSREV low read: 0x%x\n", mval );
   mval = mapped[AMZ_CSROUTH];	     
  printf( "MZ CSRout high read: 0x%x\n\n", mval );
 //    mval = mapped[0x0010];	     
 // printf( "MZ sysrevision read: 0x%x\n", mval );


 
  mapped[AOUTBLOCK] = CS_K0;	  // select FPGA 0 

  // select sys registers
        mapped[AMZ_EXAFWR] = 3;     // write to  k7's addr
      mapped[AMZ_EXDWR] = 0;
 
  // write into ext sys I/O regs
  for(k=0;k<16;k++) {
     if(k!=3) {
      mapped[AMZ_EXAFWR] = k;     // write to  k7's addr
      mapped[AMZ_EXDWR] = k+1;
     //   usleep(5);
        }
 //        printf( "K7 0 write to 0x%x: %d\n", k, k*val );

  }

   // read from ext sys I/O regs
  for(k=0;k<16;k++) {
      mapped[AMZ_EXAFRD] = k;     // write to  k7's addr
        usleep(1);
      mval = mapped[AMZ_EXDRD]; 
     printf( "K7 0 read from 0x%x: %d\n", k, mval );
     //   usleep(5);
  }

     // read from ext sys O regs
  for(k=0;k<16;k++) {
      mapped[AMZ_EXAFRD] = k+0x80;     // write to  k7's addr
        usleep(1);
      mval = mapped[AMZ_EXDRD]; 
     printf( "K7 0 read from 0x%x: %d\n", k+0x80, mval );
    //    usleep(5);
  }


  mapped[AMZ_EXAFWR] = addr;     // write to  k7's addr
  mapped[AMZ_EXDWR] = val;
//  usleep(2);

// write somewhere else to make sure we don't read a stuck bus
//    mapped[AMZ_EXAFWR] = addr+1;     // write to  k7's addr
//  mapped[AMZ_EXDWR] = val+1;
//  usleep(2);

mapped[AMZ_EXAFRD] = addr;     // read from k7's addr
  usleep(2);
 mval = mapped[AMZ_EXDRD];      // read value
//printf( "K7 0 CP read: %d\n", mval );


 // mapped[AMZ_EXAFWR] = addr;     // write to K7's CP
 // mapped[AMZ_EXDWR] = val;    // write value
 // mapped[AMZ_EXAFRD] = 0x01;     // read from k7's CP
//mval = mapped[AMZ_EXDRD];      // read value
  printf( "K7 0 read from 0x%x: %d\n\n", addr, mval );










 
  mapped[AOUTBLOCK] = CS_MZ;	  // deselect FPGA 0  
 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










