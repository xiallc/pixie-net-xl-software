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

#include "set.h"

namespace xia
{
namespace pixie
{
namespace net
{
namespace control
{
  set::set(hw::hal& hal_)
    : hal(hal_),
      command("set", "Set a configuration value, try 'set -h'",
              *this, &set::handler)
  {
  }

  int
  set::handler(const util::commands::argv& args)
  {
    bool verbose = false;
    for (auto f : args.flags) {
      if (f == "-h" || f == "-?") {
        std::cout << "usage: set [option] label [value(s)]:" << std::endl
                  << " Options:" << std::endl
                  << "   -h   : help (also '-?')" << std::endl
                  << "   -l   : list the labels" << std::endl
                  << " Set a setting to the value(s) provided. This does not" << std::endl
                  << " program the value in to the hardware." << std::endl
                  << " Note, setting are not persistent." << std::endl;
        return 0;
      }
      else if (f == "-l") {
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
      } else if (f == "-v") {
        verbose = true;
      } else {
        std::cerr << "error: invalid option: " << f << std::endl;
        return 1;
      }
    }

    if (args.options.size() < 2) {
      std::cerr << "error: no setting/value pair" << std::endl;
      return 1;
    }

    for (auto& e : hal.elements) {
      if (e.label == args.options[0]) {
        std::string s;
        for (auto si = args.options.begin() + 1;
             si != args.options.end();
             si++) {
          if (!s.empty())
            s += ' ';
          s += *si;
        }
        if (verbose)
          std::cout << args.options[0] << " <= " << s << std::endl;

        int r = hal.set(e, s);
        if (r == 0) {
          std::cout << "ok" << std::endl;
        } else {
          std::cerr << "error: setting " << args.options[0]
                    << ": " << r << std::endl;
        }
        return 0;
      }
    }

    std::cerr << "error: not found: " << args.options[0]
              << std::endl;
    return 1;
  }
}
}
}
}
