# Host-side tests for firmware/lib (no hardware needed): make -C firmware/tests
CC     ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -I../lib

test: test_core
	./test_core

test_core: test_core.c ../lib/cnc_core.c ../lib/stepgen.c ../lib/*.h
	$(CC) $(CFLAGS) -o $@ test_core.c ../lib/cnc_core.c ../lib/stepgen.c

clean:
	rm -f test_core

.PHONY: test clean
