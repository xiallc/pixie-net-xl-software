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

#include <cstring>
#include <iostream>

#include <stdarg.h>

#include "memfile.h"

namespace xia
{
namespace pixie
{
namespace net
{
namespace memfile
{
  static int
  file_open(::PixieNet_File* pf,
            const char* name,
            const char* mode)
  {
    file* f = static_cast<file*>(pf->data);
    return f->open(name, mode);
  }

  static int
  file_close(::PixieNet_File* pf)
  {
    file* f = static_cast<file*>(pf->data);
    return f->close();
  }

  static ssize_t
  file_write(const void* ptr,
             size_t size,
             size_t memb,
             ::PixieNet_File* pf)
  {
    file* f = static_cast<file*>(pf->data);
    return f->write(ptr, size, memb);
  }

  static int
  file_printf(::PixieNet_File* pf,  const char* format, ...)
  {
    file* f = static_cast<file*>(pf->data);
    va_list ap;
    va_start(ap, format);
    int len = f->vprintf(format, ap);
    va_end(ap);
    return len;
  }

  static size_t
  mem_to_blocks(size_t mem)
  {
    return ((mem - 1) / block_size) + 1;
  }

  block::block()
  {
    clear();
  }

  void
  block::clear()
  {
    in = 0;
    out = 0;
    memset(data, 0, sizeof(data));
  }

  bool
  block::full() const
  {
    return in == block_size;
  }

  size_t
  block::write(const void* ptr, size_t size)
  {
    size_t available = block_size - in;
    size_t copy = available;
    if (available > 0) {
      copy = size > available ? available : size;
      memcpy(&data[in], ptr, copy);
      in += copy;
    }
    return copy;
  }

  size_t
  block::read(void* ptr, size_t size)
  {
    size_t available = in - out;
    size_t copy = available;
    if (available > 0) {
      copy = size > available ? available : size;
      std::memcpy(ptr, &data[out], copy);
      out += copy;
    }
    return copy;
  }

  blocks::blocks(size_t max_mem)
    : max_blocks(mem_to_blocks(max_mem)),
      block_count(0),
      bytes_in(0),
      bytes_out(0)
  {
  }

  size_t
  blocks::write(const void* ptr, size_t size)
  {
    size_t bytes_written = 0;

    auto last = data.end();

    if (data.begin() == data.end()) {
      data.push_back(std::make_unique<block>());
      last = data.end();
      block_count++;
    }

    --last;

    const char* p = static_cast<const char*>(ptr);

    while (size > 0) {
      if (block_count == max_blocks)
        break;
      if ((*last)->full()) {
        data.push_back(std::make_unique<block>());
        last = data.end();
        --last;
      }
      size_t copied = (*last)->write(ptr, size);
      p += copied;
      size -= copied;
      bytes_written += copied;
    }

    bytes_in += bytes_written;

    return bytes_written;
  }

  size_t
  blocks::read(void* , size_t )
  {
    return 0;
  }

  file::file(size_t max_mem_)
    : max_mem(max_mem_),
      bin(true)
  {
    pn.open = file_open;
    pn.close = file_close;
    pn.write = file_write;
    pn.printf = file_printf;
    pn.data = static_cast<void*>(this);
  }

  int
  file::open(const char* name_, const char* mode)
  {
    if (data) {
      errno = EBUSY;
      return -1;
    }
    name = name_;
    bin = ::strchr(mode, 'b') != nullptr;
    buffer = std::make_unique<char[]>(block_size);
    data = std::make_unique<blocks>(max_mem);
    return 0;
  }

  int
  file::close()
  {
    if (!data) {
      errno = EBADF;
      return -1;
    }
    data.release();
    return 0;
  }

  ssize_t
  file::write(const void* ptr, size_t size, size_t memb)
  {
    if (!data) {
      errno = EBADF;
      return -1;
    }
    return static_cast<ssize_t>(data->write(ptr, size * memb));
  }

  int
  file::vprintf(const char* format, va_list ap)
  {
    if (!data) {
      errno = EBADF;
      return -1;
    }
    int len = ::vsnprintf(buffer.get(), block_size, format, ap);
    len = static_cast<int>(data->write(buffer.get(), static_cast<size_t>(len)));
    return len;
  }
}
}
}
}
