TARGET = adcinit bootfpga cgitraces.cgi gettraces progfippi runstats cgistats.cgi startdaq findsettings
LIBS = -lm 
CFLAGS = -std=c99 -Wall
CXXFLAGS = -Wall -O3 -DNDEBUG   -pthread -std=gnu++98
INCDIRS = -I/usr  -I/usr/include -I/usr/local/include
LINKFLAGS =  -static -static-libstdc++
BOOSTLIBS = -L/usr/local/lib -lboost_date_time -lboost_chrono -lboost_atomic -lboost_program_options -lboost_system -lboost_thread -lrt -pthread

.PHONY: default all clean

default: $(TARGET)
all: default

%.o: %.c 
	gcc  $(CFLAGS) -c $< -o $@

%.o: %.cpp 
	g++  $(CXXFLAGS) $(INCDIRS) -c $< -o $@

adcinit: adcinit.o PixieNetCommon.o PixieNetDefs.h
	gcc adcinit.o PixieNetCommon.o $(LIBS) -o adcinit

bootfpga: bootfpga.o PixieNetCommon.o PixieNetDefs.h
	gcc bootfpga.o PixieNetCommon.o $(LIBS) -o bootfpga
	
cgitraces.cgi: cgitraces.o PixieNetConfig.o PixieNetDefs.h
	g++ cgitraces.o PixieNetConfig.o $(LIBS) -o cgitraces.cgi

gettraces: gettraces.o PixieNetConfig.o PixieNetDefs.h
	g++ gettraces.o PixieNetConfig.o $(LIBS) -o gettraces

progfippi: progfippi.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ progfippi.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o progfippi

runstats: runstats.o PixieNetCommon.o PixieNetDefs.h
	gcc runstats.o PixieNetCommon.o $(LIBS) -o runstats

cgistats.cgi: cgistats.o PixieNetCommon.o PixieNetDefs.h
	gcc cgistats.o PixieNetCommon.o $(LIBS) -o cgistats.cgi

startdaq: startdaq.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ startdaq.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o startdaq

coincdaq: coincdaq.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ coincdaq.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o coincdaq

findsettings: findsettings.o PixieNetDefs.h
	gcc findsettings.o PixieNetCommon.o $(LIBS) -o findsettings

acquire: acquire.o PixieNetConfig.o PixieNetCommon.o PixieNetDefs.h
	g++ acquire.o PixieNetCommon.o PixieNetConfig.o -rdynamic $(LINKFLAGS) $(LIBS) $(BOOSTLIBS) -o acquire

cgiwaveforms.cgi: cgiwaveforms.o PixieNetCommon.o PixieNetDefs.h
	gcc cgiwaveforms.o PixieNetCommon.o $(LIBS) -o cgiwaveforms.cgi

clockprog: clockprog.o PixieNetCommon.o PixieNetDefs.h
	gcc clockprog.o PixieNetCommon.o $(LIBS) -o clockprog

pollcsr: pollcsr.o PixieNetDefs.h
	gcc pollcsr.o $(LIBS) -o pollcsr

clean:
	-rm -f *.o
	-rm -f $(TARGET)
