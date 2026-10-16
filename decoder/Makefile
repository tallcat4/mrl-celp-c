# mrl-celp-c のデコーダ: 静的ライブラリ・mrl2wav・テスト。   make; make test
CC ?= cc
CFLAGS ?= -O2
# -ffp-contract=off: ゲインと LSF の予測(f32)を原本と同じ丸めにする(積和の融合を禁止)
DEC_CFLAGS = $(CFLAGS) -std=c99 -Wall -Wextra -Wno-unused-parameter -ffp-contract=off -Iinclude -Isrc
SRC = src/dsp.c src/bits.c src/tables.c src/decoder.c
OBJ = $(SRC:.c=.o)

all: libmrldec.a mrl2wav tests/test_vectors

%.o: %.c
	$(CC) $(DEC_CFLAGS) -c $< -o $@

libmrldec.a: $(OBJ)
	ar rcs $@ $^

mrl2wav: examples/mrl2wav.c libmrldec.a
	$(CC) $(DEC_CFLAGS) $< libmrldec.a -lm -o $@

tests/test_vectors: tests/test_vectors.c libmrldec.a
	$(CC) $(DEC_CFLAGS) $< libmrldec.a -lm -o $@

# 原本(jr-voice の Rust 版)が作った参照ベクトル: 乱数・少しずつ変化・全 0・交互の 12 列 720 フレーム
test: tests/test_vectors
	./tests/test_vectors tests/vectors/celp_reference.txt

clean:
	rm -f src/*.o libmrldec.a mrl2wav tests/test_vectors

.PHONY: all test clean
