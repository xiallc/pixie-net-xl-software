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

#include <iostream>

#include "program.h"

namespace xia
{
namespace pixie
{
namespace net
{
namespace control
{
  program::program(hw::hal& hal_)
    : hal(hal_),
      command("program", "Program the FIPPI. Use -v for verbose",
              *this, &program::handler)
  {
  }

  int
  program::handler(const util::commands::argv& args)
  {
    int verbose = 0;
    if (args.options.size() > 1) {
      for (auto fi = args.options.begin() + 1;
           fi != args.options.end();
           ++fi) {
        const std::string& f = *fi;
        if (f == "-v" || f == "--verbose") {
          verbose = 1;
        } else {
          std::cerr << "warning: unknown option: " << f << std::endl;
        }
      }
    }
    int r = hal.program(verbose);
    if (r == 0) {
      std::cout << "ok" << std::endl;
    } else {
      std::cout << "error: code: " << r << std::endl;
    }
    return 0;
  }

}
}
}
}
