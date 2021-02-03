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

#if !defined(RUN_H)
#define RUN_H

#include <atomic>
#include <future>
#include <thread>

#include "commands.h"
#include "hw.h"
#include "memfile.h"

namespace xia
{
namespace pixie
{
namespace net
{
namespace control
{
  struct run
  {
    hw::hal& hal;
    util::commands::command command;

    run(hw::hal& hal);
    ~run();

    int handler(const util::commands::argv& args);

  private:

    enum run_state {
      run_idle,
      run_starting,
      run_running,
      run_finishing
    };

    enum run_mode {
      run_manual,
      run_period,
      run_event_count
    };

    void monitor(std::promise<int> result, size_t maxmsg, bool verbose);

    bool is_idle() const;
    bool is_starting() const;
    bool is_running() const;

    std::thread runner;
    std::atomic<run_state> state;
    std::atomic<bool> trace;

    memfile::file data;
    memfile::file mca;

    std::future<int> runner_answer;
  };
}
}
}
}

#endif
