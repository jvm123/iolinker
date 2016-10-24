CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-std=gnu++11 -g # $(shell root-config --cflags) -lm
LDFLAGS=-g #$(shell root-config --ldflags) -lboost_unit_test_framework -lm
LDLIBS=#$(shell root-config --libs)

SRCS=IOLinker.cpp
SRCS_unittest=$(SRCS) wiringSerial.cpp iolinker_unittest.cpp
SRCS_pi=$(SRCS) wiringSerial.cpp wiringPiSPI.cpp wiringPiI2C.cpp raspberry_test.cpp

OBJS=$(subst .cpp,.o,$(SRCS))
OBJS_unittest=$(subst .cpp,.o,$(SRCS_unittest))
OBJS_pi=$(subst .cpp,.o,$(SRCS_pi))

all: unittest

unittest: $(OBJS_unittest)
	$(CXX) $(LDFLAGS) -o iolinker_unittest $(OBJS_unittest) $(LDLIBS) 

test: unittest
	./iolinker_unittest

depend: .depend

.depend: $(SRCS_unittest)
	rm -f ./.depend
	$(CXX) $(CPPFLAGS) -MM $^>>./.depend;

clean:
	$(RM) $(OBJS_unittest)

dist-clean: clean
	$(RM) *~ .depend

	include .depend

