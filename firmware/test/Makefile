# Host-compiled C parity harness (§03/§06). No device — just clang/gcc on the
# host, asserting the C codec produces the SAME bytes as the Elixir parity
# vectors. Wired into `mix test` via a tag and into CI.
#
#   make            # build + run the harness
#   make clean

CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -O2
ROOT    := ../..
# The parity harness asserts the test FIXTURE robot's generated artifacts (its
# robot-scoped gen dir, §09); the include path adds firmware/gen/robot so
# wire_contract.h + parity_vectors.h resolve there (no stale
# firmware/include/wire_contract.h). The Follower is retired (ADR-0003); the
# fixture is the library's host-only drift + C-parity witness.
GEN     := $(ROOT)/firmware/gen/robot
INC     := -I$(ROOT)/firmware/include -I$(GEN) -I$(ROOT)/firmware/test
PARITY_SRC := $(ROOT)/firmware/src/crc16.c \
              $(ROOT)/firmware/src/cobs.c \
              $(ROOT)/firmware/src/frame.c \
              $(ROOT)/firmware/src/transport.c \
              test_parity.c
FLOOR_SRC  := $(ROOT)/firmware/src/floor.c $(ROOT)/firmware/src/frame.c test_floor.c
ROUTER_SRC := $(ROOT)/firmware/src/router.c test_router.c
SEGMENT_SRC := $(ROOT)/firmware/src/segment.c $(ROOT)/firmware/src/crc16.c \
               $(ROOT)/firmware/src/frame.c test_segment.c
UART_BACKPLANE_SRC := $(ROOT)/firmware/src/transport.c $(ROOT)/firmware/src/crc16.c \
                      $(ROOT)/firmware/src/cobs.c $(ROOT)/firmware/src/frame.c \
                      test_uart_backplane.c
SELFTEST_SRC := $(ROOT)/firmware/src/selftest.c $(ROOT)/firmware/src/transport.c \
                $(ROOT)/firmware/src/crc16.c $(ROOT)/firmware/src/cobs.c \
                $(ROOT)/firmware/src/frame.c test_selftest.c

.PHONY: all run clean
all: run

parity_harness: $(PARITY_SRC) $(GEN)/parity_vectors.h $(GEN)/wire_contract.h
	$(CC) $(CFLAGS) $(INC) $(PARITY_SRC) -o parity_harness

floor_harness: $(FLOOR_SRC) $(ROOT)/firmware/include/floor.h $(ROOT)/firmware/include/frame.h
	$(CC) $(CFLAGS) $(INC) $(FLOOR_SRC) -o floor_harness

router_harness: $(ROUTER_SRC) $(ROOT)/firmware/include/router.h
	$(CC) $(CFLAGS) $(INC) $(ROUTER_SRC) -o router_harness

segment_harness: $(SEGMENT_SRC) $(ROOT)/firmware/include/segment.h
	$(CC) $(CFLAGS) $(INC) $(SEGMENT_SRC) -o segment_harness

uart_backplane_harness: $(UART_BACKPLANE_SRC) $(ROOT)/firmware/include/transport.h
	$(CC) $(CFLAGS) $(INC) $(UART_BACKPLANE_SRC) -o uart_backplane_harness

selftest_harness: $(SELFTEST_SRC) $(ROOT)/firmware/include/selftest.h
	$(CC) $(CFLAGS) $(INC) $(SELFTEST_SRC) -o selftest_harness

run: parity_harness floor_harness router_harness segment_harness uart_backplane_harness selftest_harness
	./parity_harness
	@echo
	./floor_harness
	@echo
	./router_harness
	@echo
	./segment_harness
	@echo
	./uart_backplane_harness
	@echo
	./selftest_harness

clean:
	rm -f parity_harness floor_harness router_harness segment_harness uart_backplane_harness selftest_harness
