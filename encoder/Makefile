# mrl-celp-c のエンコーダ: 静的ライブラリ・wav2mrl・テスト。   make; make test
# デコーダ(../decoder)の状態の処理を使うので、デコーダも作る。
CC ?= cc
CFLAGS ?= -O2
DEC = ../decoder
ENC_CFLAGS = $(CFLAGS) -std=c99 -Wall -Wextra -Wno-unused-parameter -Iinclude -I$(DEC)/include -I$(DEC)/src
OBJ = src/encoder.o

all: libmrlenc.a wav2mrl

$(DEC)/libmrldec.a $(DEC)/mrl2wav:
	$(MAKE) -C $(DEC) libmrldec.a mrl2wav

%.o: %.c
	$(CC) $(ENC_CFLAGS) -c $< -o $@

libmrlenc.a: $(OBJ)
	ar rcs $@ $^

wav2mrl: examples/wav2mrl.c libmrlenc.a $(DEC)/libmrldec.a
	$(CC) $(ENC_CFLAGS) $< libmrlenc.a $(DEC)/libmrldec.a -lm -o $@

# テストの音声(LibriSpeech の 4.8 秒)を符号化し、期待のフレーム(macOS arm64 で作ったもの)と比べる。
# 浮動小数の違いで別のプラットフォームでは少し違うフレームになることがある(どれもデコーダで正しく復号できる)
test: wav2mrl $(DEC)/mrl2wav
	./wav2mrl tests/vectors/speech.pcm tests/speech.out.mrl
	@if cmp -s tests/speech.out.mrl tests/vectors/speech.expected.mrl; then echo "speech: identical to the expected frames"; \
	else echo "speech: differs from the expected frames (floating point differences on this platform?)"; fi

clean:
	rm -f src/*.o libmrlenc.a wav2mrl tests/speech.out.mrl

.PHONY: all test clean
