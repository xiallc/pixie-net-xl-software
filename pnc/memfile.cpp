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
    files* f = static_cast<files*>(pf->data);
    return f->open(name, mode);
  }

  static int
  file_close(::PixieNet_File* pf)
  {
    files* f = static_cast<files*>(pf->data);
    return f->close();
  }

  static ssize_t
  file_write(const void* ptr,
             size_t size,
             size_t memb,
             ::PixieNet_File* pf)
  {
    files* f = static_cast<files*>(pf->data);
    return f->write(ptr, size, memb);
  }

  static int
  file_printf(::PixieNet_File* pf,  const char* format, ...)
  {
    files* f = static_cast<files*>(pf->data);
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

  file::file(const char* name_, bool bin_, size_t max_mem)
    : name(name_),
      bin(bin_),
      max_blocks(mem_to_blocks(max_mem)),
      block_count(0),
      bytes_in(0),
      bytes_out(0)
  {
  }

  size_t
  file::write(const void* ptr, size_t size)
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
  file::read(void* , size_t )
  {
    return 0;
  }

  size_t
  file::size() const
  {
    return data.size();
  }

  files::files(size_t max_mem_)
    : max_mem(max_mem_),
      bytes_in_(0)
  {
    pn.open = file_open;
    pn.close = file_close;
    pn.write = file_write;
    pn.printf = file_printf;
    pn.data = static_cast<void*>(this);
  }

  int
  files::open(const char* name, const char* mode)
  {
    std::lock_guard<std::mutex> guard(lock);
    if (file_) {
      errno = EBUSY;
      return -1;
    }
    bool bin = ::strchr(mode, 'b') != nullptr;
    buffer = std::make_unique<char[]>(block_size);
    file_ = std::make_unique<file>(name, bin, max_mem);
    return 0;
  }

  int
  files::close()
  {
    std::lock_guard<std::mutex> guard(lock);
    if (!file_) {
      errno = EBADF;
      return -1;
    }
    bytes_in_ += file_->bytes_in;
    files_.push_back(std::move(file_));
    return 0;
  }

  ssize_t
  files::write(const void* ptr, size_t size, size_t memb)
  {
    std::lock_guard<std::mutex> guard(lock);
    if (!file_) {
      errno = EBADF;
      return -1;
    }
    return static_cast<ssize_t>(file_->write(ptr, size * memb));
  }

  int
  files::vprintf(const char* format, va_list ap)
  {
    std::lock_guard<std::mutex> guard(lock);
    if (!file_) {
      errno = EBADF;
      return -1;
    }
    int len = ::vsnprintf(buffer.get(), block_size, format, ap);
    len = static_cast<int>(file_->write(buffer.get(), static_cast<size_t>(len)));
    return len;
  }

  size_t
  files::count()
  {
    std::lock_guard<std::mutex> guard(lock);
    return files_.size();
  }

  size_t
  files::total_bytes_in()
  {
    std::lock_guard<std::mutex> guard(lock);
    return bytes_in_;
  }

  size_t
  files::size()
  {
    std::lock_guard<std::mutex> guard(lock);
    if (!file_)
      return 0;
    return file_->size();
  }

  size_t
  files::bytes_in()
  {
    std::lock_guard<std::mutex> guard(lock);
    if (!file_)
      return 0;
    return file_->bytes_in;
  }

  size_t
  files::bytes_out()
  {
    std::lock_guard<std::mutex> guard(lock);
    if (!file_)
      return 0;
    return file_->bytes_out;
  }

}
}
}
}
