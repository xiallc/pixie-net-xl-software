# Pixie Net XL Software

Welcome to the Pixie Net Xl Software. The software builds a number of Debian
packages that can be installed on a MicroZed running Linux. The software
assume the Zynq and Kintex FPGAs are loaded with a suitable bitfiles.

## Building

To build run Waf. Waf assumes there is a valid `python` command. If your host
has remove `python` as a command as such distributions are create a `venv` Python
environment using `python3 -m venv py`.

```
 $ ./waf configure build
```

The following programs are built and available under `build-linux`:

- `bootfpgs`
- `progfippi`
- `pncontrol`

## PixieNet Control

Pixie Net Control or `pncontrol` is an interactive command line oriented
progrem that allows you control a PixieNet. The program does not save the
state of any changes. You are responsible for this.

PixieNet Control has a number of command line arguments:

```
./build-linux/pncontrol --help
pnc [options]
Options and arguments:
 -h          : help (also --help)
 -V          : print version and exit (also --version)
 -v          : verbose, can supply multiple times
               to increase verbosity (also --verbose)
 -d file     : fippi defaults (default: defaults.ini)
 -u path     : IO device (default: /dev/uio0)
```

The defaults can be provided using the `-d` option.

To exit `pncontrol` enter `^D` or Ctrl-D.

The `pncontrol` program provides a command line editor. It supports arrow keys,
home, end and history.

The command line has a prompt:

```
# ./build-linux/pncontrol
pnet #
```

`pncontrol` is ready to accept a new command then the prompt is displayed. 

You can also control the system remotely using telnet (or an equivalent program). 
An example telnet command looks like this
```shell
user@localhost:~/>telnet <IP-OF-PIXIE_NET_XL> 9999
```
Where you replace `<IP-OF-PIXIE_NET_XL>` with the actual IP of the Pixie-Net XL

### Commands

Command are used to control the PixieNet. A command has a general format of:

```
command [options] [cmds ...] [options]
```

You need to check each command to see the specific options and commands. Not
all commands will support options. An option starts with a `-`. A `cmd` is the
command line option that does not have a `-`.

A command will finish with either `ok` to indicate the command has finished
and was successful or it will end with `error: message` to indicate the
command failed. You will need to investiage the output and error message to
determine the reason why the command failed.

#### `help`

Display a simple help. Some commands support a more detailed help by issuing
the command with `-h`.

```
pnet # help
    set Set a configuration value
   help Print the help
    run Run control, try 'run -h'
 report Report the configuration
 status Status, try 'status -h'
```

### Set

The `set` command sets a parameter. You can list the parameters with the `-l`
option:

```
pnet # set -l
```

The list will indicate is the parameter requires a single or multiple options
and the type.

To set a parameter enter the parameter and the value or values:

```
pnet # set UDP_PAUSE 100
ok
```

You can optionally add the `-v` option:

```
pnet # set  -v UDP_PAUSE 100
UDP_PAUSE <= 100
ok
```

### Run

The `run` command starts and stops a run:

```
pnet # run start
ANALOG_GAIN = 1.600000 not matching available gains exactly, please choose from this list:
    2.400000
    5.400000
error: code: -8000
```

### Report

The `report` command reports the current parameter settings. The output can be
save new defaults file:

```
pnet # report
ANALOG_GAIN             1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 ....
AUX_CTRL                1
BASELINE_PERCENT        5.0 5.0 5.0 5.0 5.0 5.0 5.0 5.0 10.0 10.0 10.0 ...
 ...
```

### Status

The `status` command reports the status the PixieNet. With no arguments the
command the run status.

The `runstats` reports the running statistics:

```
pnet # status runstats
{ParameterCo:"TOTAL_TIME",Controller:3.6E-07,ParameterSy:"RUN_TIME",System0:4.08E-07,System1:4E-07,ParameterCh:"COUNT_TIME",Channel0:4.16E-07,Channel1:4.16E-07,Channel2:4.16E-07,Channel3:4.16E-07,Channel4:4.08E-07,Channel5:4.08E-07,Channel6:4.08E-07,Channel7:4.08E-07},
{ParameterCo:"PS_CODE_VERSION",Controller:"0x306",ParameterSy:"--",System0:0,System1:0,ParameterCh:"INPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"ACTIVE",Controller:"0",ParameterSy:"--",System0:0,System1:0,ParameterCh:"OUTPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"CSROUT",Controller:"0xC200",ParameterSy:"CSROUT",System0:"0x20",System1:"0x20",ParameterCh:"COUNTTIME",Channel0:52,Channel1:52,Channel2:52,Channel3:52,Channel4:51,Channel5:51,Channel6:51,Channel7:51},
{ParameterCo:"reserved",Controller:"0xBE00",ParameterSy:"sysstatus",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x7",ParameterSy:"MEM_I_CNT",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
ok
```

The `-r` or raw option can be added for the raw values:

```
pnet # status runstats
{ParameterCo:"TOTAL_TIME",Controller:3.6E-07,ParameterSy:"RUN_TIME",System0:4.08E-07,System1:4E-07,ParameterCh:"COUNT_TIME",Channel0:4.16E-07,Channel1:4.16E-07,Channel2:4.16E-07,Channel3:4.16E-07,Channel4:4.08E-07,Channel5:4.08E-07,Channel6:4.08E-07,Channel7:4.08E-07},
{ParameterCo:"PS_CODE_VERSION",Controller:"0x306",ParameterSy:"--",System0:0,System1:0,ParameterCh:"INPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"ACTIVE",Controller:"0",ParameterSy:"--",System0:0,System1:0,ParameterCh:"OUTPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"CSROUT",Controller:"0xC200",ParameterSy:"CSROUT",System0:"0x20",System1:"0x20",ParameterCh:"COUNTTIME",Channel0:52,Channel1:52,Channel2:52,Channel3:52,Channel4:51,Channel5:51,Channel6:51,Channel7:51},
{ParameterCo:"reserved",Controller:"0xBE00",ParameterSy:"sysstatus",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x7",ParameterSy:"MEM_I_CNT",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
ok
pnet # status runstats -r
{ParameterCo:"TOTAL_TIME",Controller:3.6E-07,ParameterSy:"RUN_TIME",System0:4.08E-07,System1:4E-07,ParameterCh:"COUNT_TIME",Channel0:4.16E-07,Channel1:4.16E-07,Channel2:4.16E-07,Channel3:4.16E-07,Channel4:4.08E-07,Channel5:4.08E-07,Channel6:4.08E-07,Channel7:4.08E-07},
{ParameterCo:"PS_CODE_VERSION",Controller:"0x306",ParameterSy:"--",System0:0,System1:0,ParameterCh:"INPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"ACTIVE",Controller:"0",ParameterSy:"--",System0:0,System1:0,ParameterCh:"OUTPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"CSROUT",Controller:"0xC200",ParameterSy:"CSROUT",System0:"0x20",System1:"0x20",ParameterCh:"COUNTTIME",Channel0:52,Channel1:52,Channel2:52,Channel3:52,Channel4:51,Channel5:51,Channel6:51,Channel7:51},
{ParameterCo:"reserved",Controller:"0xBE00",ParameterSy:"sysstatus",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x7",ParameterSy:"MEM_I_CNT",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"FPGA BOOTED",Controller:"0x1",ParameterSy:"MEM_I_CNT",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x0",ParameterSy:"MEM_O_CNT",System0:"0x0",System1:"0x0",ParameterCh:"NTRIG",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x0",ParameterSy:"MEM_O_CNT",System0:"0x0",System1:"0x0",ParameterCh:"NTRIG",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"SysTime",Controller:"0x5C80",ParameterSy:"ADCframe",System0:"0x0",System1:"0x0",ParameterCh:"NTRIG",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"SysTime",Controller:"0xBB05",ParameterSy:"reserved",System0:"0xABCD",System1:"0xABCD",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"FW_VERSION",Controller:"0x400",ParameterSy:"RunTime",System0:"0x33",System1:"0x32",ParameterCh:"NOUT",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"HW_VERSION",Controller:"0xA100",ParameterSy:"RunTime",System0:"0x0",System1:"0x0",ParameterCh:"NOUT",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"TotalTime",Controller:"0x168",ParameterSy:"RunTime",System0:"0x0",System1:"0x0",ParameterCh:"NOUT",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"TotalTime",Controller:"0x0",ParameterSy:"reserved",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"TotalTime",Controller:"0x0",ParameterSy:"FW_VERSION",System0:"0x861",System1:"0x861",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"TotalTime",Controller:"0x0",ParameterSy:"WR_TM_TAI",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x7D7",ParameterSy:"WR_TM_TAI",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x0",ParameterSy:"WR_TM_TAI",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"reserved",Controller:"0x0",ParameterSy:"WR_TM_CYC",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"PCB_VERSION",Controller:"0xA161",ParameterSy:"WR_TM_CYC",System0:"0x0",System1:"0x0",ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"PCB_SNUM",Controller:2007,ParameterSy:"dpmstatus",System0:0,System1:0,ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"SNUM",Controller:2007,ParameterSy:"dpmstatus",System0:0,System1:0,ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"T_BOARD",Controller:32,ParameterSy:"T_ADC",System0:511,System1:60,ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
{ParameterCo:"T_ZYNQ",Controller:51,ParameterSy:"T_WR",System0:0,System1:0,ParameterCh:"reserved",Channel0:0,Channel1:0,Channel2:0,Channel3:0,Channel4:0,Channel5:0,Channel6:0,Channel7:0},
ok
```
