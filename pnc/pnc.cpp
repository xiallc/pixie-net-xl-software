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

#include <getopt.h>
#include <signal.h>
#include <unistd.h>

#include "util/crossline.h"

#include "commands.h"

#include "pnc.h"

static const char* const fippi_defaults = "defaults.ini";
static const char* const io_device = "/dev/uio0";

namespace xia
{
namespace pixie
{
namespace net
{
  static int verbose_;

  const char*
  version()
  {
    return "1.0";
  }

  void
  verbose_inc()
  {
    verbose_++;
  }

  bool
  verbose(int level)
  {
    return verbose_ >= level;
  }

  namespace control
  {
    session::session(const char* defaults,
                     const char* uio)
      : io(uio),
        fippi_(defaults),
        run_(io, fippi_),
        set_(fippi_),
        status_(io),
        report_(fippi_)
    {
    }

    session::~session()
    {
    }

    void
    session::shell(bool editing, bool echo)
    {
      while (true) {
        if (editing) {
          char buf[512];
          if (::crossline_readline("pnet # ", buf, sizeof(buf)) == nullptr)
            break;
          xia::util::commands::execute(buf);
        }
        else {
          if (std::cin.eof())
            break;
          std::string line;
          std::cout << "pnet # " << std::flush;
          std::getline(std::cin, line);
          if (echo)
            std::cout << line << std::endl << std::flush;
          xia::util::commands::execute(line.c_str());
        }
        std::cout << std::flush;
        std::cerr << std::flush;
      }
      std::cout << "Finished" << std::endl;
    }
  }
}
}
}

static void
fatal_signal (int signum)
{
  signal (signum, SIG_DFL);

  /*
   * Get the same signal again, this time not handled, so its normal effect
   * occurs.
   */
  kill (::getpid (), signum);
}

static void
setup_signals(void)
{
  if (signal(SIGINT, SIG_IGN) != SIG_IGN)
    signal(SIGINT, fatal_signal);
#ifdef SIGHUP
  if (signal(SIGHUP, SIG_IGN) != SIG_IGN)
    signal(SIGHUP, fatal_signal);
#endif
  if (signal(SIGTERM, SIG_IGN) != SIG_IGN)
    signal(SIGTERM, fatal_signal);
#ifdef SIGPIPE
  if (signal(SIGPIPE, SIG_IGN) != SIG_IGN)
    signal(SIGPIPE, fatal_signal);
#endif
#ifdef SIGCHLD
  signal(SIGCHLD, SIG_DFL);
#endif
}

[[ noreturn ]] static void
usage (int exit_code)
{
  std::cout << "pnc [options]" << std::endl
            << "Options and arguments:" << std::endl
            << " -h          : help (also --help)" << std::endl
            << " -V          : print version and exit (also --version)" << std::endl
            << " -v          : verbose, can supply multiple times" << std::endl
            << "               to increase verbosity (also --verbose)" << std::endl
            << " -c file     : fippi default config (default: "
            << fippi_defaults << ")" << std::endl
            << " -u path     : IO device (default: "
            << io_device << ")" << std::endl;
  ::exit (exit_code);
}

/*
 * Command line options
 */
static struct option pnc_opts[] = {
  { "help",        no_argument,            nullptr,           'h' },
  { "version",     no_argument,            nullptr,           'V' },
  { "verbose",     no_argument,            nullptr,           'v' },
  { "defaults",    no_argument,            nullptr,           'c' },
  { "user-io",     no_argument,            nullptr,           'u' },
  { NULL,          0,                      nullptr,            0 }
};

int
main (int argc, char* argv[])
{
  bool editing = ::isatty(STDIN_FILENO);
  const char* defaults = fippi_defaults;
  const char* uio = io_device;
  int ec = 0;

  setup_signals ();

  try {
    while (true) {
      int opt = ::getopt_long (argc, argv, "hVvc:u:", pnc_opts, NULL);
      if (opt < 0)
        break;

      switch (opt) {
      case 'V':
        std::cout << "pncontrol (Pixie Net Control) "
                  << xia::pixie::net::version ()
                  << std::endl;
        ::exit (0);

      case 'v':
        xia::pixie::net::verbose_inc();
        break;

      case '?':
        usage(3);

      case 'h':
        usage (0);

      case 'n':
        editing = false;
        break;

      case 'c':
        defaults = optarg;
        break;

      case 'u':
        uio = optarg;
        break;
      }
    }

    argc -= optind;
    argv += optind;

    std::cout.setf(std::ios::boolalpha);
    std::cerr.setf(std::ios::boolalpha);

    xia::pixie::net::control::session session(defaults, uio);

    if (xia::pixie::net::verbose()) {
      std::cout << "Pixie Net Control "
                << xia::pixie::net::version () << std::endl
                << " defaults: " << defaults << std::endl
                << " user-io:  " << uio << std::endl
                << " editing:  " << editing << std::endl;
    }

    session.shell(editing, false);
  }
  catch (xia::exit& e) {
    if (e.exit_code == 0)
      std::cout << e.what () << std::endl;
    else
      std::cerr << "error: " << e.what () << std::endl;
    ec = e.exit_code;
  }
  catch (std::exception& e) {
    std::cerr << "error: " << e.what () << std::endl;
    ec = 10;
  }
  catch (...) {
    /*
     * Helps to know if this happens.
     */
    std::cout << "error: unhandled exception" << std::endl;
    ec = 12;
  }

  return ec;
}
