/*
 * Copyright (c) 2020 XIA LLC
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
 */

#if !defined(HW_H)
#define HW_H

#include <mutex>
#include <string>

#include "commands.h"
#include "memfile.h"

#include <PixieNetConfig.h>
#include <PixieNetCommon.h>

namespace xia
{
namespace pixie
{
namespace net
{
namespace hw
{
  struct element
  {
    const std::string label;
    const bool        single;
    const std::string type;
    std::string       setting;

    element(const char* label_, const bool single_, const char* type_);

    void set(const std::string& new_setting);

    bool operator< (const element& right) const {
      return label.length() < right.label.length();
    }
  };

  typedef std::vector<element> elements;

  struct hal
  {
    const std::string dev;

    hw::elements elements;
    struct PixieNetFippiConfig config;

    hal(const char* dev, const char* defaults);
    ~hal();

    /*
     * Set a config element
     */
    int set(element& el, const std::string& setting);

    /*
     * Program the FIPPI. Loads the settings from the string table
     * delimited with line feeds.
     */
    int program(int verbose);

    /*
     * Data acquisition.
     */
    int daq_start(bool verbose, memfile::files& data);
    int daq_run(int mode, size_t count, size_t maxmsg, bool verbose,
                memfile::files& data, memfile::files& mca);
    int daq_stop(bool verbose, memfile::files& data, memfile::files& mca);

    /*
     * Print run stats
     */
    int print_runstats(int mode);

    /*
     * IO interfaces.
     */
    volatile unsigned int* addr() const {
      return static_cast<volatile unsigned int*>(base);
    }
    unsigned int read(const size_t reg) const {
      return addr()[reg];
    }
    void write(const size_t reg, const unsigned int value) {
      addr()[reg] = value;
    }

  private:
    int fd;
    void* base;

    std::mutex lock;
  };

  /*
   * Externally generated from the PixieNet sources.
   */
  void load(elements& e);
}
}
}
}

#endif
