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

// gcc -Wall writeI2C.c PixieNetCommon.o -o writeI2C


#include "PixieNetDefs.h"
#include "PixieNetCommon.h"

int main( int argc, char *argv[] ) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;
  int k;

  unsigned int sn = 0xDEAD;
  unsigned int rev = 0xBEEF;
  unsigned int zero[8] = {0};
  unsigned int i2cdata[8] = {0};
  unsigned int mval = 0;
  unsigned int ctrl[8];
  ctrl[7] = 1;
  ctrl[6] = 0;
  ctrl[5] = 1;
  ctrl[4] = 0;  
  ctrl[3] = 0;
  ctrl[2] = 0;
  ctrl[1] = 0;
  ctrl[0] = 0;


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
     printf( "please give arguments revision (0x####) and serial number (decimal)  \n" );
     return 2;
   }

   rev = strtol(argv[1], NULL, 16);
   sn = strtol(argv[2], NULL, 10);


   // ************************ I2C programming EEPROM *********************************
   
   /*       bugs: sequential reads seem to need a restart each time, data sheet says only the first
     
   byte  content
       0.. 1     revision low
       2.. 3     s/n
       4.. 5     gain/nyquist ch0
       6.. 7     gain/nyquist ch1
       8.. 9     gain/nyquist ch2
      10..11     gain/nyquist ch3
      12..23     0 for now

    */

   // 3 bytes: ctrl, addr L/H
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*
   I2Cbytesend(mapped, ctrl);     // I2C control byte: write
   I2Cslaveack(mapped);  
   I2Cbytesend(mapped, zero);      // memory addr byte high
   I2Cslaveack(mapped);  
   I2Cbytesend(mapped, zero);      // memory addr byte low    
   I2Cslaveack(mapped);

   // I2C data byte   0..1 
   i2cdata[7] = (rev & 0x0080) >> 7 ;    
   i2cdata[6] = (rev & 0x0040) >> 6 ;    
   i2cdata[5] = (rev & 0x0020) >> 5 ;    
   i2cdata[4] = (rev & 0x0010) >> 4 ;
   i2cdata[3] = (rev & 0x0008) >> 3 ;    
   i2cdata[2] = (rev & 0x0004) >> 2 ;   
   i2cdata[1] = (rev & 0x0002) >> 1 ;    
   i2cdata[0] = (rev & 0x0001)      ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);

   i2cdata[7] = (rev & 0x8000) >> 15 ;    
   i2cdata[6] = (rev & 0x4000) >> 14 ;    
   i2cdata[5] = (rev & 0x2000) >> 13 ;    
   i2cdata[4] = (rev & 0x1000) >> 12 ; 
   i2cdata[3] = (rev & 0x0800) >> 11 ;    
   i2cdata[2] = (rev & 0x0400) >> 10 ;   
   i2cdata[1] = (rev & 0x0200) >> 9 ;    
   i2cdata[0] = (rev & 0x0100) >> 8 ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);


   // I2C data byte   2..3 
   i2cdata[7] = (sn & 0x0080) >> 7 ;    
   i2cdata[6] = (sn & 0x0040) >> 6 ;    
   i2cdata[5] = (sn & 0x0020) >> 5 ;    
   i2cdata[4] = (sn & 0x0010) >> 4 ;
   i2cdata[3] = (sn & 0x0008) >> 3 ;    
   i2cdata[2] = (sn & 0x0004) >> 2 ;   
   i2cdata[1] = (sn & 0x0002) >> 1 ;    
   i2cdata[0] = (sn & 0x0001)      ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);

   i2cdata[7] = (sn & 0x8000) >> 15 ;    
   i2cdata[6] = (sn & 0x4000) >> 14 ;    
   i2cdata[5] = (sn & 0x2000) >> 13 ;    
   i2cdata[4] = (sn & 0x1000) >> 12 ; 
   i2cdata[3] = (sn & 0x0800) >> 11 ;    
   i2cdata[2] = (sn & 0x0400) >> 10 ;   
   i2cdata[1] = (sn & 0x0200) >> 9 ;    
   i2cdata[0] = (sn & 0x0100) >> 8 ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);
   
   // others
    for(k=4;k<32;k++) 
    {  
      I2Cbytesend(mapped, zero);
      I2Cslaveack(mapped);
    }
   
   I2Cstop(mapped);

   usleep(1000);
   // ----------- read back for verification --------------
   

   // 3 bytes: ctrl, addr L/H
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*         // write starting addr to read from
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   I2Cbytesend(mapped, zero);     // memory addr byte high
   I2Cslaveack(mapped);
   I2Cbytesend(mapped, zero);      // memory addr byte low   
   I2Cslaveack(mapped);


    // read data byte 0..1
   mval = 0;
   ctrl[0] = 1;   // R/W*         // now read
  
   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   I2Cmasterack(mapped);

   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+8));
   I2Cmasterack(mapped);

   printf("I2C read Revision 0x%04X\n",mval);


   // read data byte 1..2
   mval = 0;
   ctrl[0] = 1;   // R/W*         // now read
  
   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   I2Cmasterack(mapped);

   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+8));
   I2Cmasterack(mapped);

   printf("I2C read Serial number %d \n",mval);
 
 //  I2Cmasternoack(mapped);
   I2Cstop(mapped);

   
 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










