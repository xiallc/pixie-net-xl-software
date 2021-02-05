# Pixie-Net XL Software

The Pixie-Net Xl Software provides users an interface to configure and acquire data from a 
Pixie-Net XL device. The software can be compiled locally and run from the project root directory. 
We also provide the ability to create a Debian package for easy installation onto Debian based Linux
systems. 

Firmware files for the Kintex FPGAs are provided separately from this distribution. You can download 
these files from our [Public File Browser](http://files.xia.com/#hardware/pixie/pixie-net-xl/). Users
will need to ensure that the FPGA firmware files are installed into the root level of this project. 
The Pixie-Net XL User's Manual is available from [here](http://files.xia.com/#hardware/pixie/pixie-net-xl/documentation/).

## Dependencies
You will need to have at minimum the following dependencies installed on your system:
* gcc
* python 3.6+
* GNU make

## Installation
There are two methods to install the software.

### Installation from source
This is the traditional method for installation. In this case, you simply need to download the repository
and ensure that the project root directory is `/var/www`. Then you'll build the software using the instructions
below. 

#### Building
The software's build system is GNU make. You can build the software using the following command in
the project root directory:
```shell
root@PixieNet:~/pixie-net-xl-software#make
```

### Installation via debian package
You can download the latest Debian package from our [Support page](https://xiallc.atlassian.net/wiki/spaces/SUPPORT/pages/207552533/Pixie-Net+XL+Software).
Once the package is on your system. You can install it with 
```shell
root@PixieNet:~#dpkg -i pixie-net-xl_X.Y.Z_armhf.deb
```
Where X.Y.Z is the version number.

## PixieNet Control
Pixie-Net Control or `pncontrol` is an interactive command-line program that allows control of a 
Pixie-Net XL. The program can be used over a TCP connection to remotely configure the system and 
start data runs. **The program does not save the state of any changes. You are responsible for this.**

### Command line Arguments
```
root@PixieNet:~# pncontrol -h
pnc [options]
Options and arguments:
 -h          : help (also --help)
 -V          : print version and exit (also --version)
 -v          : verbose, can supply multiple times
               to increase verbosity (also --verbose)
 -c file     : fippi default config (default: defaults.ini)
 -u path     : IO device (default: /dev/uio0)
```

### Command prompt
`pnconrtol` provides an interactive command prompt. It supports arrow keys, home, end and history. 
To exit `pncontrol` enter `^D` or Ctrl-D.
```
root@PixieNet:~# pncontrol -c /etc/xia/pixie-net-xl/defaults.ini
pnet #
```

`pncontrol` is ready to accept a command then the prompt is displayed. 

### Remote connections
We use systemd to enable TCP communication with the system. To enable this support you should 
either 
1. install 
[the Debian package](https://xiallc.atlassian.net/wiki/spaces/SUPPORT/pages/207552533/Pixie-Net+XL+Software)
2. or install the systemd scripts located in `<project root>/systemd/system`.

Once configured the system will be listening on port `31057`. You can connect to the remote session
via telnet or other appropriate method.

We recommend using this configuration only with `DATA_FLOW=4` and `RUN_TYPE=0x404`. 

#### Telnet example
```shell
user@localhost:~/>telnet X.X.X.X 31057
Trying X.X.X.X...
Connected to X.X.X.X.
Escape character is '^]'.
Pixie Net Control 1.0
 defaults: /etc/xia/pixie-net-xl/defaults.ini
 user-io:  /dev/uio0
 editing:  false
pnet #
```
Where you replace `X.X.X.X` with the actual IP of the Pixie-Net XL in the telnet command.

### Commands
`pncontrol` recognizes a number of commands. A command has a general format of:
```
command [options] [cmds ...] [options]
```

You need to check each command to see the specific options and commands. Not
all commands support options. Options start with a `-`. A `cmd` is the
command line option that does not have a `-`.

A command finishes with `ok` to indicate a success. It will end with `error: message` to indicate 
failure. You will need to investigate error messages to determine the failure's cause.

#### `help`
Display a simple help. Some commands support a more detailed help by issuing
the command with `-h`.

```
pnet # help
  status Status, try 'status -h'
     set Set a configuration value
    help Print the help
    exit Exit the program
     run Run control, try 'run -h'
  report Report the configuration
 program Program the FIPPI. Use -v for verbose
```

### Set
The `set` command sets a parameter. You can list the parameters with the `-l` option:
```
pnet # set -l
```
The list will indicate is the parameter requires a single or multiple options and the type. To set 
a parameter enter the parameter and the value or values. **Note**: You must execute `program` to 
apply the settings to the FPGAs. 

```
pnet # set UDP_PAUSE 100
ok
pnet # program
```

You can optionally add the `-v` option:

```
pnet # set  -v UDP_PAUSE 100
UDP_PAUSE <= 100
ok
pnet # program
```

### Run
The `run` command starts and stops a run. Subsequent calls provide users with basic run information.

```
pnet # run start
ok
pnet # run
run: RUNNING period:1.72701s files:(data:c:0,i:0,s:437 mca:1,i:88790,s:0)
pnet #
```

### Report
The `report` command reports the current parameter settings. The output can be used to create a new
configuration file. 

```
pnet # report
ANALOG_GAIN             1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 1.6 ....
AUX_CTRL                1
BASELINE_PERCENT        5.0 5.0 5.0 5.0 5.0 5.0 5.0 5.0 10.0 10.0 10.0 ...
 ...
```

### Status
The `status` command reports the status the PixieNet.

```
pnet # status
{ParameterCo:"TOTAL_TIME",Controller:49011.4,ParameterSy:"RUN_TIME",System0:42802.7,System1:42802.7,ParameterCh:"COUNT_TIME",Channel0:42802.7,Channel1:42802.7,Channel2:42802.7,Channel3:42802.7,Channel4:42802.7,Channel5:42802.7,Channel6:42802.7,Channel7:42802.7},
{ParameterCo:"PS_CODE_VERSION",Controller:"0x313",ParameterSy:"--",System0:0,System1:0,ParameterCh:"INPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"ACTIVE",Controller:"1",ParameterSy:"--",System0:0,System1:0,ParameterCh:"OUTPUT_COUNT_RATE",Channel0:   0,Channel1:   0,Channel2:   0,Channel3:   0,Channel4:   0,Channel5:   0,Channel6:   0,Channel7:   0},
{ParameterCo:"CSROUT",Controller:"0xE001",ParameterSy:"CSROUT",System0:"0x2000",System1:"0x2000",ParameterCh:"COUNTTIME",Channel0:22053,Channel1:22938,Channel2:23803,Channel3:24663,Channel4:27551,Channel5:28411,Channel6:29276,Channel7:30141},
{ParameterCo:"reserved",Controller:"0xBE00",ParameterSy:"sysstatus",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:47289,Channel1:47289,Channel2:47289,Channel3:47289,Channel4:47289,Channel5:47289,Channel6:47289,Channel7:47289},
{ParameterCo:"reserved",Controller:"0x7",ParameterSy:"MEM_I_CNT",System0:"0x0",System1:"0x0",ParameterCh:"COUNTTIME",Channel0:1245,Channel1:1245,Channel2:1245,Channel3:1245,Channel4:1245,Channel5:1245,Channel6:1245,Channel7:1245},
ok
```