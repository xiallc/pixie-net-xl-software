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
#include <iomanip>
#include <list>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "commands.h"

namespace xia
{
namespace util
{
namespace commands
{
  static std::mutex                      handlers_lock;
  static std::list<command::handler_ptr> handlers;

  bool
  iequals(const std::string& a, const std::string& b)
  {
    size_t sz = a.size();
    if (b.size() != sz)
      return false;
    for (size_t i = 0; i < sz; ++i)
      if (std::tolower(a[i]) != std::tolower(b[i]))
        return false;
    return true;
  }

  argv::argv(const char* command_)
  {
    /*
     * Keep it simple for how. Add quoting if we need it.
     */
    std::istringstream iss(command_);
    std::string token;
    while (iss >> token) {
      args.push_back(token);
    }
    if (!args.empty()) {
      for (auto ai = args.begin() + 1; ai != args.end(); ++ai) {
        const std::string& a = *ai;
        if (a[0] != '-')
          break;
        flags.push_back(a);
      }
      bool add = false;
      for (auto ai = args.begin() + 1; ai != args.end(); ++ai) {
        const std::string& a = *ai;
        if (a[0] == '-' && !add) {
          continue;
        }
        add = true;
        options.push_back(a);
      }
    }
  }

  command::handler_base::handler_base(const char* name_,
                                      const char* description_)
    : name(name_),
      description(description_)
  {
  }

  command::handler_base::~handler_base() = default;

  command::command()
  {
  }

  command::command(command&& command_)
  {
    std::lock_guard<std::mutex> guard(handlers_lock);
    std::swap(handler_, command_.handler_);
  }

  command&
  command::operator=(command&& command_)
  {
    std::lock_guard<std::mutex> guard(handlers_lock);
    std::swap(handler_, command_.handler_);
    return *this;
  }

  void
  command::add()
  {
    std::lock_guard<std::mutex> guard(handlers_lock);
    handlers.push_back(handler_);
    handlers.sort();
  }

  void
  command::remove()
  {
    handlers.remove(handler_);
  }

  int
  execute(const char* command_)
  {
    argv args(command_);
    if (args.args.empty())
      return 0;
    std::lock_guard<std::mutex> guard(handlers_lock);
    for (auto& h : handlers)
      if (iequals(args.args[0], h->name))
        return h->dispatch(args);
    std::cout << "error: invalid command: " << args.args[0] << std::endl;
    return 1;
  }

  help::help()
    : command("help", "Print the help", *this, &help::handler)
  {
  }

  int
  help::handler(const util::commands::argv& )
  {
    std::lock_guard<std::mutex> guard(handlers_lock);
    size_t max_length = 0;

    for (auto& h : handlers)
      if (h->name.length() > max_length)
        max_length = h->name.length();
    max_length++;

    std::ios_base::fmtflags cstate(std::cout.flags());

    std::cout << std::setfill(' ');

    for (auto& h : handlers)
      std::cout << std::setw(static_cast<int>(max_length)) << h->name
                << " "
                << h->description
                << std::endl;

    std::cout.flags(cstate);

    return 0;
  }
}
}
}
