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
  unsigned int addr = 0xBEEF;
  unsigned int rev = 0x0000;
  unsigned int dev = 0;
  unsigned int zero[8] = {0};
  unsigned int i2cdata[8] = {0};
  unsigned int mval = 0;
  unsigned int ctrl[8];
  unsigned int addr_of_sn  = 6;
  unsigned int addr_of_rev = 7;

 /* ctrl[7] = 1;     // PN PROM
  ctrl[6] = 0;
  ctrl[5] = 1;
  ctrl[4] = 0;  
  ctrl[3] = 0;
  ctrl[2] = 0;
  ctrl[1] = 0;
  ctrl[0] = 0;     */

  ctrl[7] = 1;      // PN XL PROM  (TMP116)
  ctrl[6] = 0;
  ctrl[5] = 0;
  ctrl[4] = 1;  
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

   if( argc!=5)  {
     printf( "Please give arguments\n 1) addr (0x##) for test read\n 2) serial number (decimal) for write\n" );
     printf( " 3) device (0=board, 1=DB1, 2=DB2)\n 4) revision (0x####) for write\n" );
     printf( "  if addr > 0x0F, skip write and only read sn and from addr & 0xF\n" );
     return 2;
   }

   addr = strtol(argv[1], NULL, 16);      // address from test read
   sn   = strtol(argv[2], NULL, 10);      // serial number
   dev  = strtol(argv[3], NULL, 10);      // device select board or DB#
   rev  = strtol(argv[4], NULL, 16);      // revision

   // ************************ prepare to write *********************************

   if(dev>2) {
     printf( "invalid device number \n" );
     return 3;
   }
   

  mapped[AMZ_DEVICESEL] = CS_MZ;	  // read/write from/to MZ IO block
  if(dev==0) mval = I2C_SELMAIN ;          // set bit 4 to select MZ I2C pins
  if(dev==1) mval = I2C_SELDB0 ;
  if(dev==2) mval = I2C_SELDB1 ;
  mapped[AAUXCTRL] = mval;

   // ************************ I2C programming EEPROM *********************************
   
   /*       bugs: sequential reads seem to need a restart each time, data sheet says only the first
     
   byte  content
       0,1    temperature
       2,3    configuration
       4,5    temp high   addrision low   ?
       6,7    temp low     s/n           ?
       8,9     unlock
       10-17   unique ID
       30,31   device ID 
    */

    if (addr<16) {  
 
       // ============== write the serial number to addr 6 ======================
       
       if(dev==0) {     // only main board is serialized
      
        // ----------- write unlock reg --------------
        
         // 4 bytes: ctrl, addr, data, data  : 
         I2Cstart(mapped);
         ctrl[0] = 0;   // R/W*
         I2Cbytesend(mapped, ctrl);     // I2C control byte: write
         I2Cslaveack(mapped);  
      
         mval = 0x04;   // register address  : 4 = unlock reg
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         mval = 0x8000;  //  register data  : set unlock bit
         i2cdata[7] = (mval & 0x8000) >> 15 ;    
         i2cdata[6] = (mval & 0x4000) >> 14 ;    
         i2cdata[5] = (mval & 0x2000) >> 13 ;    
         i2cdata[4] = (mval & 0x1000) >> 12 ; 
         i2cdata[3] = (mval & 0x0800) >> 11 ;    
         i2cdata[2] = (mval & 0x0400) >> 10 ;   
         i2cdata[1] = (mval & 0x0200) >> 9 ;    
         i2cdata[0] = (mval & 0x0100) >> 8 ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         I2Cstop(mapped);
         usleep(1000);
      
         // ----------- write serial number  --------------
      
         // 4 bytes: ctrl, addr, data, data  : 
          I2Cstart(mapped);
         ctrl[0] = 0;   // R/W*
         I2Cbytesend(mapped, ctrl);     // I2C control byte: write
         I2Cslaveack(mapped);  
      
         mval = addr_of_sn;   // register address  : 7 = unique id all zeros
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
           //  register data  : serial number
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
         I2Cstop(mapped);
      
         usleep(10000);    // wait 7ms or more. should check and wait more if necessary, really, 
         usleep(10000);  
          
         // -----------general call reset --------------
      
         // 2 bytes: ctrl, reset  : 
         I2Cstart(mapped);
         I2Cbytesend(mapped, zero);     // I2C control byte (general address): write
         I2Cslaveack(mapped);  
      
         mval = 0x06;   // reset code word
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         I2Cslaveack(mapped);
         I2Cstop(mapped);
      
         usleep(10000);
         usleep(10000);   
      }  // end if main board


      // ================== write versin/revision/variant to reg 7 =======================

         // ----------- write unlock reg --------------
        
         // 4 bytes: ctrl, addr, data, data  : 
         I2Cstart(mapped);
         ctrl[0] = 0;   // R/W*
         I2Cbytesend(mapped, ctrl);     // I2C control byte: write
         I2Cslaveack(mapped);  
      
         mval = 0x04;   // register address  : 4 = unlock reg
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         mval = 0x8000;  //  register data  : set unlock bit
         i2cdata[7] = (mval & 0x8000) >> 15 ;    
         i2cdata[6] = (mval & 0x4000) >> 14 ;    
         i2cdata[5] = (mval & 0x2000) >> 13 ;    
         i2cdata[4] = (mval & 0x1000) >> 12 ; 
         i2cdata[3] = (mval & 0x0800) >> 11 ;    
         i2cdata[2] = (mval & 0x0400) >> 10 ;   
         i2cdata[1] = (mval & 0x0200) >> 9 ;    
         i2cdata[0] = (mval & 0x0100) >> 8 ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         I2Cstop(mapped);
         usleep(1000);
      
         // ----------- write serial number  --------------
      
         // 4 bytes: ctrl, addr, data, data  : 
          I2Cstart(mapped);
         ctrl[0] = 0;   // R/W*
         I2Cbytesend(mapped, ctrl);     // I2C control byte: write
         I2Cslaveack(mapped);  
      
         mval = addr_of_rev;   // register address  : 7 = unique id all zeros
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
           //  register data  : serial number
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
         I2Cstop(mapped);
      
         usleep(10000);    // wait 7ms or more. should check and wait more if necessary, really, 
         usleep(10000);  
          
         // -----------general call reset --------------
      
         // 2 bytes: ctrl, reset  : 
         I2Cstart(mapped);
         I2Cbytesend(mapped, zero);     // I2C control byte (general address): write
         I2Cslaveack(mapped);  
      
         mval = 0x06;   // reset code word
         i2cdata[7] = (mval & 0x0080) >> 7 ;    
         i2cdata[6] = (mval & 0x0040) >> 6 ;    
         i2cdata[5] = (mval & 0x0020) >> 5 ;    
         i2cdata[4] = (mval & 0x0010) >> 4 ;
         i2cdata[3] = (mval & 0x0008) >> 3 ;    
         i2cdata[2] = (mval & 0x0004) >> 2 ;   
         i2cdata[1] = (mval & 0x0002) >> 1 ;    
         i2cdata[0] = (mval & 0x0001)      ;   
         I2Cbytesend(mapped, i2cdata);
         I2Cslaveack(mapped);
      
         I2Cslaveack(mapped);
         I2Cstop(mapped);
      
         usleep(10000);
         usleep(10000);   






    }   // end <16

      
    // ************* read back the serial number from addr 6 ***********************

     // 2 bytes: ctrl, addr  write
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*         // write starting addr to read from
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   mval = addr_of_sn;   // addr 6 = serial number
   i2cdata[7] = (mval & 0x0080) >> 7 ;    
   i2cdata[6] = (mval & 0x0040) >> 6 ;    
   i2cdata[5] = (mval & 0x0020) >> 5 ;    
   i2cdata[4] = (mval & 0x0010) >> 4 ;
   i2cdata[3] = (mval & 0x0008) >> 3 ;    
   i2cdata[2] = (mval & 0x0004) >> 2 ;   
   i2cdata[1] = (mval & 0x0002) >> 1 ;    
   i2cdata[0] = (mval & 0x0001)      ;   
   I2Cbytesend(mapped, i2cdata);
   I2Cslaveack(mapped);
   usleep(300);

    // read data bytes 
   mval = 0;
   ctrl[0] = 1;   // R/W*         // now read 
   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);      // device address
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+8));
   I2Cmasterack(mapped);

   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   //I2Cmasterack(mapped);
   I2Cmasternoack(mapped);
   I2Cstop(mapped);

   printf("I2C read serial number: %d\n",mval);
   usleep(10000); 
   

   
   // ************* read requested addr (-16 if > 16) ***********************
   
   mval = addr & 0xF;   // register address  : 7 = unique id all zeros
   i2cdata[7] = (mval & 0x0080) >> 7 ;    
   i2cdata[6] = (mval & 0x0040) >> 6 ;    
   i2cdata[5] = (mval & 0x0020) >> 5 ;    
   i2cdata[4] = (mval & 0x0010) >> 4 ;
   i2cdata[3] = (mval & 0x0008) >> 3 ;    
   i2cdata[2] = (mval & 0x0004) >> 2 ;   
   i2cdata[1] = (mval & 0x0002) >> 1 ;    
   i2cdata[0] = (mval & 0x0001)      ;   

   // 2 bytes: ctrl, addr  write
   I2Cstart(mapped);
   ctrl[0] = 0;   // R/W*         // write starting addr to read from
   I2Cbytesend(mapped, ctrl);
   I2Cslaveack(mapped);
   I2Cbytesend(mapped, i2cdata);     // address 0  = temp value
   I2Cslaveack(mapped);
   usleep(300);
  
   // read data bytes
   mval = 0;
   ctrl[0] = 1;   // R/W*         // now read 
   usleep(100);
   I2Cstart(mapped);               //restart
   I2Cbytesend(mapped, ctrl);      // device address
   I2Cslaveack(mapped);
   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+8));
   I2Cmasterack(mapped);

   I2Cbytereceive(mapped, i2cdata);
   for( k = 0; k < 8; k ++ )
      if(i2cdata[k])
         mval = mval + (1<<(k+0));
   //I2Cmasterack(mapped);
   I2Cmasternoack(mapped);
   I2Cstop(mapped);

   printf("I2C read from 0x%02X:  0x%04X\n",addr & 0xF,mval);
   printf("if temperature (addr=0x00), temp (C) %f\n",mval*0.0078125);



   
 
 // clean up  
 mapped[AAUXCTRL] = 0;

 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










