# Makefile — Host-Simulator des Pico (echte Firmwarelogik, simulierte Hardware).
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror
SRC = ../followme_mcu/src/proto.cpp ../followme_mcu/src/supervisor.cpp \
      ../followme_mcu/src/wheel_ctrl.cpp ../followme_mcu/src/app.cpp

mcu_sim: mcu_sim.cpp wheel_plant.h $(SRC) ../followme_mcu/config.h ../followme_mcu/src/*.h
	$(CXX) $(CXXFLAGS) -o $@ mcu_sim.cpp $(SRC) -lm

clean:
	rm -f mcu_sim

.PHONY: clean
