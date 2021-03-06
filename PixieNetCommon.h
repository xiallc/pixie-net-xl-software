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

// refs for shared functions

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

 #include "PixieNetConfig.h"

  int read_print_runstats(int mode, int dest, volatile unsigned int *mapped );
  int read_print_runstats_XL_2x4(int mode, int dest, volatile unsigned int *mapped );
  int read_print_rates_XL_2x4(int dest, volatile unsigned int *mapped );
  void I2Cstart(volatile unsigned int *mapped);
  void I2Cstop(volatile unsigned int *mapped);
  void I2Cslaveack(volatile unsigned int *mapped);
  void I2Cmasterack(volatile unsigned int *mapped);
  void I2Cmasternoack(volatile unsigned int *mapped);
  void I2Cbytesend(volatile unsigned int *mapped, unsigned int *data);
  void I2Cbytereceive(volatile unsigned int *mapped, unsigned int *data);
  unsigned int setbit( unsigned int par, unsigned int bitc, unsigned int bitf);
  unsigned int hwinfo( volatile unsigned int *mapped,unsigned int I2Csel );
  float board_temperature( volatile unsigned int *mapped, unsigned int I2Csel  );
  float zynq_temperature();
  int ADCinit_DB01(volatile unsigned int *mapped );
  int PLLinit(volatile unsigned int *mapped );

struct PixieNet_File;
typedef struct PixieNet_File PixieNet_File;

typedef int (*PixieNet_FileOpen)(PixieNet_File* pf,
                                 const char* name,
                                 const char* mode);
typedef int (*PixieNet_FileClose)(PixieNet_File* pf);
typedef ssize_t (*PixieNet_FileWrite)(const void* ptr,
                                      size_t size,
                                      size_t memb,
                                      PixieNet_File* pf);
typedef int (*PixieNet_FilePrintf)(PixieNet_File* pf,
                                   const char * format,
                                   ...) __attribute__ ((format (printf, 2, 3)));

struct PixieNet_File
{
  PixieNet_FileOpen open;
  PixieNet_FileClose close;
  PixieNet_FileWrite write;
  PixieNet_FilePrintf printf;
  void* data;
};

int program_fippi(int verbose,
                  PixieNetFippiConfig *fippiconfig,
                  volatile unsigned int *mapped);

int daq_start(int verbose, PixieNet_File* fil,
              PixieNetFippiConfig *fippiconfig, volatile unsigned int *mapped);
// mode = 0 : timed
// mode = 1 : number of loops
// mode = 2:  number of events
int daq_run(int mode, size_t count, int verbose, int maxmsg,
            PixieNet_File* fil, PixieNet_File* filmca,
            PixieNetFippiConfig *fippiconfig, volatile unsigned int *mapped);
int daq_stop(int verbose, PixieNet_File* fil, PixieNet_File* filmca,
             PixieNetFippiConfig *fippiconfig, volatile unsigned int *mapped);

#ifdef __cplusplus
}
#endif
