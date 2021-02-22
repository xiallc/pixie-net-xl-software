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
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>

#include "get.h"

namespace xia
{
namespace pixie
{
namespace net
{
namespace control
{
  get::get(hw::hal& hal_)
    : hal(hal_),
      command("get", "Get a configuration value, try 'get -h'",
              *this, &get::handler)
  {
  }

  int
  get::handler(const util::commands::argv& args)
  {
    for (auto f : args.flags) {
      if (f == "-h" || f == "-?") {
        std::cout << "usage: get [option] [label] [index]:" << std::endl
                  << " Options:" << std::endl
                  << "   -h   : help (also '-?')" << std::endl
                  << "   -l   : list the labels" << std::endl
                  << " Get a setting. If no label is provide all setting are" << std::endl
                  << " returned. You can optionally specify a value index. If no" << std::endl
                  << " index is provided all values are returned." << std::endl;
        return 0;
      }
      else       if (f == "-l") {
        auto maxi = std::max_element(hal.elements.begin(),
                                     hal.elements.end());
        const auto max = (*maxi).label.length() + 1;
        auto cstate(std::cout.flags());
        std::cout << std::setfill(' ');
        for (auto& e : hal.elements) {
          std::cout << std::setw(static_cast<int>(max)) << e.label
                    << " " << static_cast<const char*>(e.single ? "single" : "multi ")
                    << " " << e.type
                    << std::endl;
        }
        std::cout.flags(cstate);
        return 0;
      } else {
        std::cerr << "warning: invalid option: " << f << std::endl;
      }
    }

    std::string label;
    ssize_t index = -1;

    if (args.options.size() > 0) {
      label = args.options[0];
      if (args.options.size() > 1) {
        std::istringstream iss(args.options[1]);
        iss >> index;
      }
    }

    for (auto& e : hal.elements) {
      if (label.empty() || e.label == label) {
        if (e.single && index > 0) {
          std::cerr << "error: element has a single value: " << label
                    << std::endl;
          return 1;
        }
        std::string result(e.setting);
        if (index >= 0) {
          std::istringstream iss(e.setting);
          for (auto issi = std::istream_iterator<std::string>(iss);
               issi != std::istream_iterator<std::string>();
               ++issi, --index) {
            if (index == 0) {
              result = *issi;
              index = -1;
              break;
            }
          }
          if (index >= 0) {
          std::cerr << "error: element index not found: " << label
                    << std::endl;
          return 1;
          }
        }

        std::cout << e.label << " = " << result << std::endl;

        if (!label.empty()) {
          std::cout << "ok" << std::endl;
          return 0;
        }
      }
    }

    if (!label.empty()) {
      std::cerr << "error: not found: " << label
                << std::endl;
    }

    return 1;
  }
}
}
}
}
