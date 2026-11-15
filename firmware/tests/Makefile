# Host-side tests for firmware/lib (no hardware needed): make -C firmware/tests
CC     ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -I../lib

test: test_core
	./test_core

test_core sim_loop: %: %.c sim.h ../lib/cnc_core.c ../lib/stepgen.c ../lib/*.h
	$(CC) $(CFLAGS) -o $@ $< ../lib/cnc_core.c ../lib/stepgen.c -lm

# Closed-loop timing simulation (host PID + board), prints lag and jitter figures.
sim: sim_loop
	./sim_loop

clean:
	rm -f test_core sim_loop

.PHONY: test sim clean
