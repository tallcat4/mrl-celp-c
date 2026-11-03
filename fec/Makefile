# mrl-celp-c の誤り訂正(無線の 384 bit ⇔ 音声の 20 byte): 静的ライブラリ・テスト。   make; make test
CC ?= cc
CFLAGS ?= -O2
FEC_CFLAGS = $(CFLAGS) -std=c99 -Wall -Wextra -Iinclude
OBJ = src/fec.o

all: libmrlfec.a tests/test_fec

%.o: %.c
	$(CC) $(FEC_CFLAGS) -c $< -o $@

libmrlfec.a: $(OBJ)
	ar rcs $@ $^

tests/test_fec: tests/test_fec.c libmrlfec.a
	$(CC) $(FEC_CFLAGS) $< libmrlfec.a -o $@

# 原本(Rust の voice_frame.rs)の結果と比べる: 誤りなし・散らばった誤り・当てにならない信頼度・バースト・乱数の 8000 フレーム
test: tests/test_fec
	./tests/test_fec tests/vectors/fec_reference.txt

clean:
	rm -f src/*.o libmrlfec.a tests/test_fec

.PHONY: all test clean
