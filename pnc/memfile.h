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

#if !defined(MEMFILE_H)
#define MEMFILE_H

#include <list>
#include <memory>

#include <PixieNetCommon.h>

namespace xia
{
namespace pixie
{
namespace net
{
namespace memfile
{
  const size_t block_size = 4096;

  struct block
  {
    typedef std::unique_ptr<block> ptr;

    size_t in;
    size_t out;
    unsigned char data[block_size];

    block();

    void clear();
    bool full() const;

    size_t write(const void* ptr, size_t size);
    size_t read(void* ptr, size_t size);
  };

  struct blocks
  {
    typedef std::list<block::ptr> block_data;

    block_data data;

    size_t max_blocks;
    size_t block_count;

    size_t bytes_in;
    size_t bytes_out;

    blocks(size_t max_mem);

    size_t write(const void* ptr, size_t size);
    size_t read(void* ptr, size_t size);
  };

  struct file
  {
    typedef std::unique_ptr<file> ptr;

    const size_t max_mem;

    std::string name;
    bool bin;

    ::PixieNet_File pn;

    file(size_t max_mem);

    int open(const char* name, const char* mode);
    int close();
    ssize_t write(const void* ptr, size_t size, size_t memb);
    int vprintf(const char* format, va_list ap);

  private:

    std::unique_ptr<blocks> data;
    std::unique_ptr<char[]> buffer;
  };
}
}
}
}

#endif
