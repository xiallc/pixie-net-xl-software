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

int main( void ) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;

  unsigned int mval = 0;
  FILE * fil;
  unsigned int j, revsn;
  unsigned int counter1, nWords;
  unsigned short confdata[N_FPGA_BYTES/2];


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


   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
   {
      fil = fopen("PNXLK7_DB02_12_250.bin","rb");
      printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7_DB02_12_250.bin\n", revsn>>16, revsn&0xFFFF);
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
   {
      fil = fopen("PNXLK7_DB01_14_125.bin","rb");  
      printf(" HW Rev = 0x%04X, SN = %d, loading PNXLK7_DB01_14_125.bin\n", revsn>>16, revsn&0xFFFF);
   }
   if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75)
   {
      fil = fopen("PNXLK7_DB01_14_75.bin","rb");  
      printf(" HW Rev = 0x%04X, SN = %d, loading PNXLK7_DB01_14_75.bin\n", revsn>>16, revsn&0xFFFF);
   }
   nWords = fread(confdata, 2, (N_FPGA_BYTES/2), fil);
   if(((N_FPGA_BYTES/2) - nWords) > 1) {
      // ndat differing from nWords by 1 is OK if N_COMFPGA_BYTES is an odd number 
      printf("ERROR: reading FPGA configuration incomplete\n");
      flock( fd, LOCK_UN );
      munmap(map_addr, size);
      close(fd);
      fclose(fil);
      return(-1);
   } else {
      printf(" FPGA file loaded (%d words).\n", nWords);
   }
   fclose(fil);
 
   // ************************ FPGA programming  *********************************

  mapped[AMZ_DEVICESEL] = CS_MZ;	  // read/write from/to MZ 

 /* // test read write
  mval = mapped[0x1];	
  printf("0x1 read: 0x%x\n",mval);
  mval = 0x0;
  mapped[0x6] = mval;	
  mval = mapped[AMZ_CSROUTH];	
  printf("0x1 read: 0x%x\n\n",mval);
  */

  // progb toggle
  mval = mapped[AFPGAPROG];	
//  printf("AFPGAPROG read: 0x%x\n",mval);
  mval = 0x0000;
  mapped[AFPGAPROG] = mval;
//  printf("AFPGAPROG write: 0x%x\n",mval);
  usleep(I2CWAIT);
//  mval = mval | 0x0200;    // Set  FPGA Progb = 1 to start configuration
  mval = 0x0001;
  mapped[AFPGAPROG] = mval;
//  printf("AFPGAPROG write: 0x%x\n",mval);
  usleep(I2CWAIT);
  mval = mapped[AFPGAPROG];	
//  printf("AFPGAPROG read: 0x%x\n",mval);


  // check INIT, continue when high
  // Initialize counter1 to 0. If mval.15==0, finished clearing communication FPGA 
  mval = mapped[AMZ_CSROUTL];	
//  printf("ACSROUT read: 0x%x\n",mval);
  counter1 = 0;
  while ((mval& 0x8000) == 0x0000 && counter1 < 100) {
      usleep(I2CWAIT);
      mval = mapped[AMZ_CSROUTL];
   //   printf("ACSROUT read: 0x%x\n",mval);
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
  else
  {
      printf(" FPGA cleared (CSR = 0x%x)\n",mval);
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
      if( j % 65536 ==0)  { printf  ("_"); fflush(stdout); }

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


   // ************************ clean up  *********************************

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










