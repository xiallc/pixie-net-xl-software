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

#include "fippi.h"

namespace xia
{
namespace pixie
{
namespace net
{
  namespace control
  {
    fippi::fippi(const char* defaults)
    {
      config::load(elements);
      int r = init_PixieNetFippiConfig_from_file(defaults,
                                                 0,
                                                 &config);
      if (r != 0)
        throw std::runtime_error("cannot load fippi defaults");

      /*
       * Load the settings for reporting. We need the defaults.
       */
      std::ifstream input(defaults, std::ios::in | std::ios::binary);
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
                                 [token](const config::element& e) {
                                   return e.label == token;
                                 });
          if (ei != elements.end()) {
            config::element& element = *ei;
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
    }

    int
    fippi::set(const std::string& text)
    {
      return init_PixieNetFippiConfig_from_buffer(&text[0],
                                                  2,
                                                  &config);
    }
  }
}
}
}
