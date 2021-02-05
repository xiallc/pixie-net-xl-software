#
# Take PixieNetConfig.cpp and crate a table of
# config values and types
#

from __future__ import print_function

import os
import re
import sys

if len(sys.argv) != 2:
    print('error: invalid arguments', file=sys.stderr)
    sys.exit(1)

def header():
    o = []
    o += ['/*']
    o += [' * Generated, do not edit. See c-to-config.py']
    o += [' */']
    o += ['']
    o += ['#include <pnc/hw.h>']
    o += ['']
    o += ['namespace xia']
    o += ['{']
    o += ['namespace pixie']
    o += ['{']
    o += ['namespace net']
    o += ['{']
    o += ['namespace hw']
    o += ['{']
    o += ['  void load(elements& e)']
    o += ['  {']
    return o

def footer():
    o = []
    o += ['  }']
    o += ['}']
    o += ['}']
    o += ['}']
    o += ['}']
    return o

def process(name):
    ret = re.compile('\s+ret\s+=\s+parse_')

    config = {}

    with open(name, 'r') as c:
        for l in c.readlines():
            if ret.match(l):
                half = l.split('(')
                call = half[0].strip().split(' ')[2]
                if call.split('_')[1] == 'single':
                    single = 'true'
                else:
                    single = 'false'
                dtype = call.split('_')[2]
                label = half[1].split('"')[1]
                config[label] = (single, dtype)

    o = header()

    for k in sorted(list(config.keys())):
        e = config[k]
        o += ['    e.push_back(element("%s", %s, "%s"));' % (k, e[0], e[1])]

    o += footer()

    return o

o = process(sys.argv[1])
print(os.linesep.join(o))

sys.exit(0)
