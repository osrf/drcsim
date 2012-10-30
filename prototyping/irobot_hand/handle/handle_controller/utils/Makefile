
objects := motortest parametertest sensor_poll sensor_spin stopall

all: $(objects)

% : %.c
	arm-commonos-linux-gnueabi-g++ $< ../../handle_lib/src/packing.cpp -Wall -pthread -ggdb3 -o $@

clean:
	rm *~ $(objects)

install:
	scp $(objects) root@armH-palm-1:
