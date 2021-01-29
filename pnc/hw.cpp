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
#include <stdexcept>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "hw.h"

namespace xia
{
namespace pixie
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

  io::io(const char* dev_)
    : dev(dev_),
      fd(-1),
      base(nullptr)
  {
    fd = ::open(dev_, O_RDWR);
    if (fd < 0) {
      throw_errno(dev_, "hw io open");
    }
    base = ::mmap(nullptr, io_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == nullptr) {
      throw_errno(dev_, "hw io mmap");
    }
  }

  io::~io()
  {
    if (base != nullptr)
      ::munmap(base, io_size);
    if (fd >= 0)
      ::close(fd);
  }
}
}
}
