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

#if !defined(COMMAND_H)
#define COMMAND_H

#include <memory>
#include <string>
#include <vector>

namespace xia
{
namespace util
{
namespace commands
{
  /*
   * Command line arguments.
   */
  typedef std::vector<std::string> arguments;

  struct argv
  {
    arguments args;
    arguments flags;
    arguments options;
    argv(const char* command);
  };

  /*
   * A command. Create an instance of a command with te member of a
   * class as the handler:
   *
   * class foo
   * {
   *   public:
   *     foo();
   *   private:
   *     int hello(int argc, char** argv);
   *     command hello;
   * };
   *
   * foo::foo()
   *   : bar("bar", "The bar command")
   * {
   *   hello_ = command("hello", "The bar's hello sub-command",
   *                    *this, &foo::hello);
   * }
   *
   * int foo::hello(int , char* )
   * {
   *   std::cout << "Bar's hello" << std::endl;
   *   return 0;
   * }
   */
  struct command
  {
    struct handler_base
    {
      const std::string name;
      const std::string description;

      handler_base(const char* name_,
                   const char* description);
      virtual ~handler_base();
      virtual int dispatch(const argv& args) = 0;

      bool operator==(const handler_base& other) {
        return name == other.name;
      }
      bool operator<(const handler_base& other) {
        return name < other.name;
      }
    };

    using handler_ptr = std::shared_ptr<handler_base>;

    template<typename T, typename Callable>
    explicit command(const char* name_,
                     const char* description,
                     T&          t,
                     Callable&&  func)
      : handler_(make_handler(name_, description,
                              t, std::forward<Callable>(func))) {
      add();
    }

    command();
    command(command&& other);
    ~command() {
      remove();
    }

    command& operator=(command&& other);

    /*
     * Constrain how to use this struct.
     */
    command(command&) = delete;
    command(const command&) = delete;
    command(const command&&) = delete;
    command& operator=(const command&) = delete;

  private:

    handler_ptr handler_;

    void add();
    void remove();

    template<typename T, typename Callable>
    struct handler : handler_base
    {
      T&       t;
      Callable func;

      handler(const char* name_,
              const char* description_,
              T&          t_,
              Callable&&  func_)
        : handler_base(name_, description_),
          t(t_),
          func(std::forward<Callable>(func_)) {
      }

      int dispatch(const argv& args) {
        return (t.*func)(args);
      }
    };

    template<typename T, typename Callable>
    static handler_ptr
    make_handler(const char* name_,
                 const char* description_,
                 T&          t,
                 Callable&&  func) {
      using CallableHandler = handler<T, Callable>;
      return handler_ptr{new CallableHandler{name_,
            description_,
            t,
            std::forward<Callable>(func)}};
    }
  };

  struct help
  {
    util::commands::command command;

    help();

    int handler(const util::commands::argv& args);
  };

  int execute(const char* command_);

  /*
   * Case insensitive compare.
   */
  bool iequals(const std::string& a, const std::string& b);

}
}
}

#endif
