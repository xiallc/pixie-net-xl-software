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

#include "report.h"

namespace xia
{
namespace pixie
{
namespace net
{
namespace control
{
  report::report(hw::hal& hal_)
    : hal(hal_),
      command("report", "Report the configuration (-q for quiet)",
              *this, &report::handler)
  {
  }

  int
  report::handler(const util::commands::argv& args)
  {
    bool quiet = false;
    for (auto f : args.flags) {
      if (f == "-q") {
        quiet = true;
      } else {
        std::cerr << "error: invalid option: " << f << std::endl;
        return 1;
      }
    }

    auto maxi = std::max_element(hal.elements.begin(),
                                 hal.elements.end());
    const auto max = (*maxi).label.length() + 1;

    auto cstate(std::cout.flags());
    std::cout << std::setfill(' ');

    for (auto& e : hal.elements) {
      std::cout << std::setw(static_cast<int>(max)) << std::left << e.label
                << ' ' << e.setting
                << std::endl;
    }

    std::cout.flags(cstate);

    if (!quiet)
      std::cout << "ok" << std::endl;

    return 0;
  }
}
}
}
}
