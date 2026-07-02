# Builds the TEST-ONLY virtual-hub NIF (Option B): the real firmware C floor +
# wire path, wrapped for the host-side e2e suite. Same chassis sources the
# firmware/test harnesses host-compile — so the e2e runs the ACTUAL safety code.
#
# Driven by elixir_make from the library's mix.exs, in :test only. Output goes to
# the app's priv/ where VHubNif.load_nif/0 finds it via :code.priv_dir.

# This Makefile lives at test/support/c_src/; the repo root is three up.
ROOT     := ../../..
PRIV     := $(ROOT)/priv
FW_SRC   := $(ROOT)/firmware/src
FW_INC   := $(ROOT)/firmware/include

# ERTS headers for erl_nif.h. ERL_EI_INCLUDE_DIR / ERTS_INCLUDE_DIR are exported
# by elixir_make; fall back to deriving them from `erl` for a bare `make`.
ERLANG_PATH := $(shell erl -eval 'io:format("~s", [lists:concat([code:root_dir(), "/erts-", erlang:system_info(version), "/include"])])' -s init stop -noshell)
ERTS_INCLUDE_DIR ?= $(ERLANG_PATH)

CC       ?= cc
CFLAGS   ?= -std=c11 -Wall -Wextra -O2 -fPIC
INC      := -I$(FW_INC) -I$(ERTS_INCLUDE_DIR)

# The chassis sources the NIF links — the SAME ones firmware/test compiles.
CHASSIS  := $(FW_SRC)/floor.c \
            $(FW_SRC)/frame.c \
            $(FW_SRC)/crc16.c \
            $(FW_SRC)/cobs.c \
            $(FW_SRC)/transport.c \
            $(FW_SRC)/router.c

SRC      := vhub_nif.c $(CHASSIS)
TARGET   := $(PRIV)/vhub_nif.so

# macOS needs -dynamiclib + undefined-dynamic-lookup for NIF symbols; Linux -shared.
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
  LDFLAGS ?= -dynamiclib -undefined dynamic_lookup
else
  LDFLAGS ?= -shared
endif

.PHONY: all clean
all: $(TARGET)

$(TARGET): $(SRC) $(FW_INC)/floor.h $(FW_INC)/frame.h $(FW_INC)/transport.h $(FW_INC)/router.h
	mkdir -p $(PRIV)
	$(CC) $(CFLAGS) $(INC) $(SRC) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)
