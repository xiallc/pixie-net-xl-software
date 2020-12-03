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
#include "PixieNetConfig.h"


unsigned short confdata[N_FPGA_BYTES_B/2];
int main( int argc, const char **argv) {

  int fd;
  void *map_addr;
  int size = 4096;
  volatile unsigned int *mapped;

  unsigned int mval = 0;
  FILE * fil;
  unsigned int j, revsn, tenG;
  unsigned int counter1, nWords;
  unsigned int N_FPGA_BYTES;

  // for now, any argument switches to 10G bootfiles
    if (argc > 1) {
        tenG = 1;
     //    printf("na: %d, argument: %s\n", argc, argv[1]);
    } else {
        tenG = 0;
    //     printf("na: %d, argument: %s\n", argc, argv[1]);
    }


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
//   revsn = 0xA1110008;
   mapped[AMZ_HWINFO] = revsn >> 16;      // store PROM revsion info in MZ register, so it can be read without slow I2C
//   mapped[AMZ_PLLSTART] = 4;              // low 2 bits set CLK SEL for PLL input (1=WRclkDB, 0 = FPGA/other)
                                          // any write will start programming the LMK PLL for ADC and FPGA processing clock  
   N_FPGA_BYTES =10;
   // Rev B DB options
   if((revsn & PNXL_MB_REV_MASK) == PNXL_MB_REVB)
   {
      if(tenG==0) {
      // 1G files
         if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
         {
            fil = fopen("PNXLK7B_DB01_14_125_1G.bin","rb");  
            printf(" HW Rev = 0x%04X, SN = %d, loading PNXLK7B_DB01_14_125_1G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
         if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
         {
            fil = fopen("PNXLK7B_DB02_12_250_1G.bin","rb");
            printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7B_DB02_12_250_1G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
         if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250)
         {
            fil = fopen("PNXLK7B_DB06_16_250_1G.bin","rb");
            printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7B_DB06_16_250_1G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
         if((revsn & PNXL_DB_VARIANT_MASK) == 0xF00000)      // no ADC DB: default to DB02
         {
            fil = fopen("PNXLK7B_DB06_16_250_1G.bin","rb");
            printf(" HW Rev = 0x%04X, SN = %d, NO ADC DB! - loading default PNXLK7B_DB06_16_250_1G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
      } else {
       // 10G files
         if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125)
         {
            fil = fopen("PNXLK7B_DB01_14_125_10G.bin","rb");  
            printf(" HW Rev = 0x%04X, SN = %d, loading PNXLK7B_DB01_14_125_10G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
         if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB02_12_250)
         {
            fil = fopen("PNXLK7B_DB02_12_250_10G.bin","rb");
            printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7B_DB02_12_250_10G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
         if((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB06_16_250)
         {
            fil = fopen("PNXLK7B_DB06_16_250_10G.bin","rb");
            printf(" HW Rev = 0x%04X, SN = %d,  loading PNXLK7B_DB06_16_250_10G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }
         if((revsn & PNXL_DB_VARIANT_MASK) == 0xF00000)      // no ADC DB: default to DB02
         {
            fil = fopen("PNXLK7B_DB06_16_250_10G.bin","rb");
            printf(" HW Rev = 0x%04X, SN = %d, NO ADC DB! - loading default PNXLK7B_DB06_16_250_10G.bin\n", revsn>>16, revsn&0xFFFF);
            N_FPGA_BYTES = N_FPGA_BYTES_B;
         }

      }  // end 10G switch
   } else {
   // Rev A DB options
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


    // ******************* read ini file and fill struct with values ********************

  //int verbose = 1;      // TODO: control with argument to function 
  // 0 print errors and minimal info only
  // 1 print errors and full info
  
  PixieNetFippiConfig fippiconfig;		// struct holding the input parameters
  const char *defaults_file = "defaults.ini";
  int rval = init_PixieNetFippiConfig_from_file( defaults_file, 0, &fippiconfig );   // first load defaults, do not allow missing parameters
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", defaults_file, rval );
    return rval;
  }
  const char *settings_file = "settings.ini";      
  rval = init_PixieNetFippiConfig_from_file( settings_file, 2, &fippiconfig );   // second override with user settings, do allow missing, don't print missing
  if( rval != 0 )
  {
    printf( "Failed to parse FPGA settings from %s, rval=%d\n", settings_file, rval );
    return rval;
  }


    // ************************ LMK PLL  initialization  *********************************

    printf(" Waiting for clock initialization (0x%x)\n",fippiconfig.CLK_CTRL);
    usleep(100000);



    // CLK CTRL:
    if( (fippiconfig.CLK_CTRL == 3) | (fippiconfig.CLK_CTRL == 0) ) {
      // ok
    }  else {
      printf("Invalid CLK_CTRL = 0x%x, should be 0 or 3\n",fippiconfig.CLK_CTRL);
      return -800;
    }

    
    mval = fippiconfig.CLK_CTRL;              // low 2 bits set CLK SEL for PLL input (1=WRclkDB, 0 = FPGA/other)
    mapped[AMZ_PLLSTART] = mval;              // any write will start programming the LMK PLL for ADC and FPGA processing clock                                               
    if( (mval & 0x3) >0)
       printf(" initializing ADC PLL with clock from  WRclkDB\n");
    else
       printf(" initializing ADC PLL with clock from FPGA/other\n");

    printf(" Waiting for clock initialization (0x%x)...",fippiconfig.CLK_CTRL);
    usleep(100000);  
    printf(" ... done\n");

    
   // ************************ ADC  initialization  *********************************

    if( ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_125) | ((revsn & PNXL_DB_VARIANT_MASK) == PNXL_DB01_14_75) )
    {
      printf("Initializing ADCs:\n");  
      ADCinit_DB01(mapped);
      // TODO: check return value for success
    }
      
    // ************************ WR PLL  initialization  *********************************
    /* no longer needed, now inside FPGA
    if((revsn & PNXL_MB_REV_MASK) == PNXL_MB_REVB)
    {
      printf("Initializing PLLs:\n");  
      PLLinit(mapped);
      // TODO: check return value for success
    }

    */
   // ************************ clean up  *********************************

 
 // clean up  
 flock( fd, LOCK_UN );
 munmap(map_addr, size);
 close(fd);
 return 0;
}










