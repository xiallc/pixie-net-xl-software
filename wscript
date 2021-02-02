#
# Copyright (c) 2020 XIA LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms,
# with or without modification, are permitted provided
# that the following conditions are met:
#
#   * Redistributions of source code must retain the above
#     copyright notice, this list of conditions and the
#     following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions and the
#     following disclaimer in the documentation and/or other
#     materials provided with the distribution.
#   * Neither the name of XIA LLC
#     nor the names of its contributors may be used to endorse
#     or promote products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

import os

out = 'build-' + os.uname()[0].lower()

def options(opt):
    opt.load('compiler_c')
    opt.load('compiler_cxx')

def configure(conf):
    conf.load('compiler_c')
    conf.load('compiler_cxx')

def crossline(bld):
    bld.stlib(source='util/crossline.c',
              target='crossline')

def pixienetpl(bld):
    bld.stlib(feature='c c++',
              source=['PixieNetCommon.c',
                      'PixieNetConfig.cpp',
                      'progfippi.c'],
              defines=['EMBED_FIPPI'],
              target='pixienetpl')

def pixie_tools(bld):
    bld.program(feature='c c++',
                source='progfippi.c',
                target='progfippi',
                lib=['m', 'stdc++'],
                use='pixienetpl')
    bld.program(feature='c',
                source='bootfpga.c',
                target='pnkintex',
                lib=['m', 'stdc++'],
                use='pixienetpl')

def build(bld):
    if bld.env['COMPILER_CXX'] == 'clang++':
        warnings = ['-Weverything',
                    '-Wno-c++98-compat',
                    '-Wno-exit-time-destructors',
                    '-Wno-global-constructors',
                    '-Wno-reserved-id-macro',
                    '-Wno-c++98-compat-pedantic',
                    '-Wno-padded']
    else:
        warnings = ['-Wall',
                    '-Wextra']

    cxxflags = ['-g', '-std=c++17'] + warnings

    crossline(bld)
    pixienetpl(bld)

    bld(rule = 'python ../c-to-config.py ${SRC} > ${TGT}',
        target='pnc/hw-elements.cpp',
        source='PixieNetConfig.cpp',
        color = 'BLUE')

    source = ['pnc/pnc.cpp',
              'pnc/set.cpp',
              'pnc/program.cpp',
              'pnc/run.cpp',
              'pnc/status.cpp',
              'pnc/report.cpp',
              'pnc/exit.cpp',
              'pnc/commands.cpp',
              'pnc/hw.cpp',
              'pnc/hw-elements.cpp']

    bld.program(source=source,
                target='pncontrol',
                cxxflags=cxxflags,
                includes='.',
                use=['pixienetpl', 'crossline'])

    pixie_tools(bld)
