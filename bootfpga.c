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

// gcc -Wall bootfpga.c -o bootfpga

#include "PixieNetDefs.h"
#include "PixieNetCommon.h"


unsigned short confdata[N_FPGA_BYTES_B/2];
int main( void ) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;

  unsigned int mval = 0;
  FILE * fil;
  unsigned int j, revsn;
  unsigned int counter1, nWords;
  unsigned int N_FPGA_BYTES;


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

 printf("Configuring FPGAs:\n");


    // ************************ read data  *********************************

   revsn = hwinfo(mapped,I2C_SELMAIN);    // some settings may depend on HW variants
   mapped[AMZ_HWINFO] = revsn >> 16;      // store PROM revsion info in MZ register, so it can be read without slow I2C
   mapped[AMZ_PLLSTART] = 1;              // any write will start programming the LMK PLL for ADC and FPGA processing clock  

   N_FPGA_BYTES =10;
   if((revsn & PNXL_MB_REV_MASK) == PNXL_MB_REVB)
   {
      if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
      {
         fil = fopen("PNXLK7B_DB02_12_250.bin","rb");
         printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7B_DB02_12_250.bin\n", revsn>>16, revsn&0xFFFF);
         N_FPGA_BYTES = N_FPGA_BYTES_B;
      }
      if((revsn & PNXL_DB_VARIANT_MASK) == 0xF00000)      // no ADC DB: default to DB02
      {
         fil = fopen("PNXLK7B_DB02_12_250.bin","rb");
         printf(" HW Rev = 0x%04X, SN = %d, NO ADC DB! - loading default PNXLK7B_DB02_12_250.bin\n", revsn>>16, revsn&0xFFFF);
         N_FPGA_BYTES = N_FPGA_BYTES_B;
      }
   } else {
      if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
      {
         fil = fopen("PNXLK7_DB02_12_250.bin","rb");
         printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7_DB02_12_250.bin\n", revsn>>16, revsn&0xFFFF);
         N_FPGA_BYTES = N_FPGA_BYTES_A;
      }
      if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
      {
         fil = fopen("PNXLK7_DB01_14_125.bin","rb");  
         printf(" HW Rev = 0x%04X, SN = %d, loading PNXLK7_DB01_14_125.bin\n", revsn>>16, revsn&0xFFFF);
         N_FPGA_BYTES = N_FPGA_BYTES_A;
      }
      if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)
      {
         fil = fopen("PNXLK7_DB01_14_75.bin","rb");  
         printf(" HW Rev = 0x%04X, SN = %d, loading PNXLK7_DB01_14_75.bin\n", revsn>>16, revsn&0xFFFF);
         N_FPGA_BYTES = N_FPGA_BYTES_A;
      }
      if((revsn & PNXL_DB_VARIANT_MASK) == 0xF00000)
      {
         fil = fopen("PNXLK7_DB02_12_250.bin","rb");
         printf(" HW Rev = 0x%04X, SN = %d, NO ADC DB! - loading default PNXLK7_DB02_12_250.bin\n", revsn>>16, revsn&0xFFFF);
         N_FPGA_BYTES = N_FPGA_BYTES_A;
      }
   }
   if(N_FPGA_BYTES==10) {
      printf("ERROR: invalid HW configuration 0x%x \n", revsn);
      printf(" Rev test 0x%x =?= 0x%x\n",revsn & PNXL_MB_REV_MASK, PNXL_MB_REVB); 
      printf(" DB test 0x%x =?= 0x%x\n",revsn & PNXL_DB_VARIANT_MASK, PNXL_DB02_12_250); 
      flock( fd, LOCK_UN );
      munmap(map_addr, size);
      close(fd);
      return(-1);
   }


       
   
   nWords = fread(confdata, 2, (N_FPGA_BYTES/2), fil);

  // return 0;
   if(((N_FPGA_BYTES/2) - nWords) > 1) {
      // ndat differing from nWords by 1 is OK if N_COMFPGA_BYTES is an odd number 
      printf("ERROR: reading FPGA configuration incomplete %d of %d\n",nWords,N_FPGA_BYTES/2);
      flock( fd, LOCK_UN );
      munmap(map_addr, size);
      close(fd);
      fclose(fil);
      return(-1);
   } 
   fclose(fil);
 
   // ************************ FPGA programming  *********************************

  mapped[AMZ_DEVICESEL] = CS_MZ;	  // read/write from/to MZ 



  // progb toggle
  mval = mapped[AFPGAPROG];	
  mval = 0x0000;
  mapped[AFPGAPROG] = mval;
  usleep(I2CWAIT);
  mval = 0x0001;
  mapped[AFPGAPROG] = mval;
  usleep(I2CWAIT);
  mval = mapped[AFPGAPROG];	

  // check INIT, continue when high
  // Initialize counter1 to 0. If mval.15==0, finished clearing communication FPGA 
  mval = mapped[AMZ_CSROUTL];	
  counter1 = 0;
  while ((mval& 0x8000) == 0x0000 && counter1 < 100) {
      usleep(I2CWAIT);
      mval = mapped[AMZ_CSROUTL];
      counter1++;
  }
  if (counter1 == 100)
  {
         printf("ERROR: Clearing FPGA timed out\n");
          flock( fd, LOCK_UN );
          munmap(map_addr, size);      
          close(fd);
         return(-2);
  }

  
  // download configuration data
  printf(" Starting FPGA download\n Percent done: ");

   for( j=0; j < N_FPGA_BYTES/2; j++)      
   {
      mapped[AFPGACONF] = confdata[j];
      mval = mapped[AAUXCTRL];	    // read for delay
      mval = mapped[AAUXCTRL];	
      mval = mapped[AAUXCTRL];	
      if( j % (N_FPGA_BYTES/10) ==0)  { printf  ("%d",200*j/N_FPGA_BYTES); fflush(stdout); }
      if( j % (N_FPGA_BYTES/90) ==0)  { printf  ("_"); fflush(stdout); }

   } 
    printf(" done\n");

   
  // check DONE, ok when high
  // If mval.14==0, configuration ok
  mval = mapped[AMZ_CSROUTL];	
  if( (mval& 0x4000) != 0x4000) {
      printf("ERROR: Programming FPGA not successful.\n");
      flock( fd, LOCK_UN );
      munmap(map_addr, size);      
      close(fd);
      return(-3);
  } else {
    printf(" Programming FPGA successful !\n");
  }


   // ************************ ADC  initialization  *********************************

    if((revsn & PNXL_DB_VARIANT_MASK) != PNXL_DB02_12_250)
    {
      printf("Initializing ADCs:\n");  
      ADCinit_DB01(mapped);
      // TODO: check return value for success
    }

    // ************************ WR PLL  initialization  *********************************
    if((revsn & PNXL_MB_REV_MASK) == PNXL_MB_REVB)
    {
      printf("Initializing PLLs:\n");  
      PLLinit(mapped);
      // TODO: check return value for success
    }

   // ************************ clean up  *********************************

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










