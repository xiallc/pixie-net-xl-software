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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "hw.h"

#include <PixieNetCommon.h>

namespace xia
{
namespace pixie
{
namespace net
{
namespace hw
{
  const size_t io_size = 4096;

  [[ noreturn ]] static void
  throw_errno(const char* dev, const char* what)
  {
      std::string s = what;
      s += ": ";
      s += dev;
      s += ": ";
      s += ::strerror(errno);
      throw std::runtime_error(s);
  }

  element::element(const char* label_, const bool single_, const char* type_)
    : label(label_),
      single(single_),
      type(type_) {
  }

  void
  element::set(const std::string& new_setting)
  {
    /*
     * Silently ignore settings not for this element.
     */
    if (!new_setting.empty() && new_setting[0] != '#') {
      std::istringstream iss(new_setting);
      std::string token;
      if (iss >> token) {
        if (token == label) {
          setting.clear();
          while (iss >> token) {
            if (!setting.empty())
              setting += ' ';
            setting += token;
          }
        }
      }
    }
  }

  hal::hal(const char* dev_, const char* defaults_)
    : dev(dev_),
      fd(-1),
      base(nullptr)
  {
    /*
     * Load the defaults
     */
    load(elements);
    int r = init_PixieNetFippiConfig_from_file(defaults_,
                                               0,
                                               &config);
    if (r != 0)
      throw std::runtime_error("cannot load fippi defaults");

    /*
     * Load the settings for reporting. We need the defaults.
     */
    std::ifstream input(defaults_, std::ios::in | std::ios::binary);
    if (!input)
      throw std::runtime_error("cannot load fippi defaults");

    for (std::string line; std::getline(input, line); ) {
      if (line.empty() || line[0] == '#')
        continue;
      std::istringstream iss(line);
      std::string token;
      if (iss >> token) {
        auto ei = std::find_if(elements.begin(),
                               elements.end(),
                               [token](const element& e) {
                                 return e.label == token;
                               });
        if (ei != elements.end()) {
          element& element = *ei;
          element.setting.clear();
          while (iss >> token) {
            if (!element.setting.empty())
              element.setting += ' ';
            element.setting += token;
          }
        }
      }
    }

    input.close();

    /*
     * Map the AXI bus port into the address space.
     */
    fd = ::open(dev_, O_RDWR);
    if (fd < 0) {
      throw_errno(dev_, "hw io open");
    }
    base = ::mmap(nullptr, io_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == nullptr) {
      throw_errno(dev_, "hw io mmap");
    }
  }

  hal::~hal()
  {
    if (base != nullptr)
      ::munmap(base, io_size);
    if (fd >= 0)
      ::close(fd);
  }

  int
  hal::set(element& e, const std::string& setting)
  {
    std::string text = e.label + ' ' + setting;
    int r = ::init_PixieNetFippiConfig_from_buffer(&text[0],
                                                   2,
                                                   &config);
    if (r == 0) {
      e.setting = setting;
    }
    return r;
  }

  int
  hal::program(int verbose)
  {
    return program_fippi(verbose, &config, addr());
  }

  int
  hal::print_runstats(int mode)
  {
    return ::read_print_runstats_XL_2x4(mode, 1, addr());
  }
}
}
}
}
