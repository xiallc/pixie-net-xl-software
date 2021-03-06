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

#include <chrono>
#include <iostream>

#include "run.h"

/*
 * For ease of milliseconds
 */
using namespace std::chrono_literals;

namespace xia
{
namespace pixie
{
namespace net
{
namespace control
{
  run::run(hw::hal& hal_)
    : hal(hal_),
      command("run", "Run control, try 'run -h'",
              *this, &run::handler),
      state(run_idle),
      trace(true),
      data(5 * 1024 * 1024),
      mca(5 * 1024 * 1024)
  {
  }

  run::~run()
  {
    if (is_running()) {
      state = run_finishing;
      while (!is_idle())
        std::this_thread::sleep_for(10ms);
    }
  }

  int
  run::handler(const util::commands::argv& args)
  {
    if (args.options.empty()) {
      for (auto f : args.flags) {
        if (f == "-h" || f == "-?") {
          std::cout << "usage: run [option] [cmd ..]:" << std::endl
                    << " Options:" << std::endl
                    << "   -h   : help (also '-?')" << std::endl
                    << " Commands:" << std::endl
                    << "   start     : Start a run" << std::endl
                    << "   stop      : Stop a run" << std::endl;
          return 0;
        }
      }
      std::cout << "run: ";
      switch (state.load()) {
      case run_idle:
        std::cout << "IDLE" << std::endl;
        break;
      case run_starting:
        std::cout << "STARTING" << std::endl;
        break;
      case run_running:
        {
          auto now = std::chrono::system_clock::now();
          std::chrono::duration<double> period = now - start;
          std::cout << "RUNNING period:" << period.count() << "s files:(data:c:"
                    << data.count() << ",i:" << data.total_bytes_in() << ",s:" << data.size()
                    << " mca:"
                    << mca.count() << ",i:" << mca.total_bytes_in() << ",s:" << mca.size()
                    << ')'
                  << std::endl;
        }
        break;
      case run_finishing:
        std::cout << "FINISHING" << std::endl;
        break;
      }
      return 0;
    }

    bool verbose = false;
    if (args.options.size() > 1) {
      for (auto fi = args.options.begin() + 1;
           fi != args.options.end();
           ++fi) {
        const std::string& f = *fi;
        if (f == "-v" || f == "--verbose") {
          verbose = true;
        } else {
          std::cerr << "warning: unknown option: " << f << std::endl;
        }
      }
    }

    if (args.options[0] == "start") {
      if (!is_idle()) {
        std::cerr << "error: already running" << std::endl;
        return 1;
      }
      state = run_starting;
      start = std::chrono::system_clock::now();
      int r = hal.daq_start(verbose, data);
      if (r == 0) {
        std::promise<int> result;
        runner_answer = result.get_future();
        runner = std::thread(&run::monitor, this, std::move(result), 0, verbose);
        runner.detach();
      }
      if (r == 0) {
        std::cout << "ok" << std::endl;
      } else {
        std::cout << "error: code: " << r << std::endl;
      }
    } else if (args.options[0] == "stop") {
      if (!is_running()) {
        std::cerr << "error: not running" << std::endl;
        return 1;
      }
      state = run_finishing;
      int r = hal.daq_stop(verbose, data, mca);
      auto now = std::chrono::system_clock::now();
      std::chrono::duration<double> period = now - start;
      std::cout << "Run duration: " << period.count() << "s" << std::endl;
      if (r == 0) {
        std::cout << "ok" << std::endl;
      } else {
        std::cout << "error: code: " << r << std::endl;
      }
      while (!is_idle())
        std::this_thread::sleep_for(10ms);
    } else {
      std::cerr << "error: invalid command" << std::endl;
      return 1;
    }

    return 0;
  }

  void
  run::monitor(std::promise<int> result, size_t maxmsg, bool verbose)
  {
    int r = 0;
    state = run_running;
    while (is_running()) {
      r = hal.daq_run(1, 1, maxmsg, verbose, data, mca);
      if (r != 0)
        break;
      if (/* DISABLES CODE */ (false)) {
        std::this_thread::sleep_for(1ms);
      }
    }
    result.set_value(r);
    state = run_idle;
  }

  bool
  run::is_idle() const
  {
    return state.load() == run_idle;
  }

  bool
  run::is_starting() const
  {
    return state.load() == run_starting;
  }

  bool
  run::is_running() const
  {
    run_state s = state.load();
    return s == run_starting || s == run_running;
  }
}
}
}
}
