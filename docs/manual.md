# 詳細マニュアル

- [フレームの構成](#フレームの構成)
- [デコーダ](#デコーダ)
- [エンコーダ](#エンコーダ)
- [MCU での実測](#mcu-での実測)

## フレームの構成

40 ms のフレームを 20 ms の半分 2 つに分け、半分をさらに 10 ms(80 sample)のサブフレーム 2 つに分けます。

| 項目 | 単位 | ビット数 |
| --- | --- | --- |
| モード(LSF の移動平均予測の係数の組) | 半分 | 1 |
| LSF の 2 段ベクトル量子化(1 段 128 通り × 10 次、2 段 64 通り × 5 次 × 2) | 半分 | 7 + 6 + 6 |
| ピッチ(1/3 sample の分解能、20〜143) | サブフレーム | 絶対 8 / 差分 5 |
| 固定符号帳(パルスの形 2〜3 個の区分 0〜4、単位パルス 5 + 5 個の区分 5) | サブフレーム | 16 |
| ゲイン(ピッチ利得と、予測つきの符号帳利得の組、128 通り) | サブフレーム | 7 |

計 158 ビットを 20 バイトに LSB first で詰めます(上位 2 ビットは 0)。ビットの並びは `decoder/src/bits.c` にあります。
後半 2 つ目のゲインの下位 4 ビットの位置と、前半の絶対ピッチの下位 2 ビットの反転は、原本の注記どおり経験的に確かめた仮定です。

## デコーダ

jr-conventional-line-radio-receiver の `crates/jr-voice/src/celp`(commit `dfc7440`)を C99 に移植したものです。
16 ビット固定小数点 DSP(40 ビット累算器、小数乗算)の処理を `int64_t` で再現し、LSF とゲインの予測だけが単精度浮動小数です。
状態は 868 バイト、速度は Apple M5 で 1 フレーム 5.8 µs です。

```c
#include "mrlcelp.h"
static double mem[1024 / sizeof(double)];      /* mrl_decoder_size()(868)バイト以上、8 バイト境界 */
mrl_decoder *d = mrl_decoder_init(mem);
int16_t pcm[MRL_FRAME_SAMPLES];                /* 320 sample = 40 ms */
if (mrl_decode(d, payload20, pcm) != 0) { /* 上位 2 ビットが 0 でない(不正なフレーム) */ }
```

```bash
decoder/mrl2wav in.mrl out.raw          # 見出しなしの PCM
decoder/mrl2wav in.mrl out.wav --pre    # 出力フィルタの前の合成(評価用)
```

- 自分のビルドシステムに組み込むときは `decoder/src/*.c` と `decoder/include/` を加え、**`-ffp-contract=off`** でコンパイルします(積和を融合すると、ゲインと LSF の予測の丸めが原本と変わります)。
- `-DMRL_MAC16` を付けると、Xtensa(ESP32 など)の MAC16 の 40 ビット累算器で積和をします。
- 40 ビット累算器の巻き戻しは、値がその範囲に収まることが分かる所では省いています(結果は同じ)。
- `logf` / `expf` はプラットフォームの libm を使います(原本と同じ)。libm が違うと、ごくまれに PCM が ±1〜2 違うことがあります。参照ベクトルは macOS(arm64)で作りました。
- `decoder/src/tables.c` は原本の表から自動生成したもので、手で編集しません。
- jr-voice の学習済みの後処理(先読み 280 ms)は含みません。

## エンコーダ

本物の(無線機の)エンコーダの手順は公開されていないので、デコーダのビット列の意味と状態遷移に合わせて、分析による合成(A-b-S)の CELP として作りました。
デコーダの状態の処理(`decoder/src/core.h`)で状態を進めるので、エンコーダとデコーダの状態は常に一致します。

```c
#include "mrlcelp_enc.h"
static double mem[10240 / sizeof(double)];          /* mrl_encoder_size() バイト以上、8 バイト境界 */
mrl_encoder *e = mrl_encoder_init(mem, NULL);       /* NULL で既定の設定 */
uint8_t payload[MRL_FRAME_BYTES];
if (mrl_encode_push(e, pcm320, payload)) send(payload);   /* 320 sample ずつ。1 つ前のフレームが出る(先読みのため) */
if (mrl_encode_flush(e, payload)) send(payload);          /* 最後のフレーム。半端な最後のフレームは 0 で埋めて push する */
```

- 遅れはフレーム 40 ms + 先読み 10 ms。状態は 9.5 KB(と、探索に使う static の 25 KB)。速度は Apple M5 で 1 フレーム 92 µs です。
- `mrl_enc_config`(`mrl_enc_default_config` で既定値)で探索の幅を変えられます。既定値は、評価用の別の 130 発話で品質をほとんど落とさずに絞った値です。
- 組み込むときは `encoder/src/encoder.c` と `encoder/include/`、デコーダの `src/*.c` と `include/`・`src/` を加えます。エンコーダは積和を融合してよく、ESP32 では速くなります。

品質:テスト用の 32 発話(LibriSpeech test-clean の英語 24 発話、macOS の `say` の日本語 8 発話)で、PESQ(P.862 狭帯域)2.87、合成の SNR 9.6 dB です。
研究用の Python 版では、最初の試作が PESQ 2.74、ビーム探索つきの重い設定が 3.07 でした。ただ、この程度の差は聴き比べではほとんど区別できませんでした。

手順(半フレーム 20 ms ごと):
1. 線形予測:サブフレームの中心に 240 sample の窓(後半は 10 ms の先読み)、Chebyshev 多項式の根の探索で LSF
2. LSF の量子化:デコーダの移動平均予測を裏返した目標に対して 1 段・2 段の上位を組み合わせる。後半の LSF と前半(前の半フレームとの中点)の LSF の両方で誤差を測る
3. 開ループのピッチ:聴感の重み付けをした音声の正規化相関

サブフレーム 10 ms ごと:

4. 聴感の重み付け W(z) = A(z/0.94)/A(z/0.6) をかけた目標
5. ピッチ:整数のラグは、分母を補間フィルタを通した励振の再帰(G.729 と同じ)、分子を正確な適応符号帳で比べ、小数は前後の得点が高い側だけ調べる
6. 固定符号帳:パルスの形をフィルタに通したものを位置をずらして使い、相関の大きい位置だけ組み合わせる
7. 上位の候補を 128 通りのゲインと比べ、選んだものだけ正確な符号ベクトルでゲインを選び直す
8. デコーダの状態を進める

## MCU での実測

テストの音声(LibriSpeech の 4.8 秒、120 フレーム)を符号化してデスクトップと同じフレームか、そのフレームを復号した PCM のハッシュが同じかを確かめ、
1 フレーム(40 ms)あたりのサイクル数を数えます(`encoder/tests/mcu/mcu_test.c`)。

| ボード | エンコーダ | デコーダ | 合計(CPU の割合) |
| --- | --- | --- | --- |
| Arduino Portenta H7(Cortex-M7、400 MHz) | 270 万サイクル(最大 283 万) | 22 万 | 292 万(18%) |
| ESP32-WROOM-32(240 MHz、1 コア、コードと表は内部 RAM) | 319 万(最大 333 万) | 38 万 | 357 万(37%、最も重いフレームで 39%) |

どちらもフレームは 120/120 がデスクトップと同じで、PCM もビット一致しました。

```bash
encoder/examples/esp32/run.sh /dev/cu.usbserial-XXXX   # ESP-IDF 5.4。MRL_PROFILE=1 で段階ごと、MRL_MICRO=1 で部品ごと
encoder/tests/mcu/portenta/run_portenta.sh             # arduino-cli と arduino:mbed_portenta、pyserial
encoder/tests/mcu/update_expected.sh                   # エンコーダを変えたら期待値を作り直す
```

ESP32 で効いたこと(最初の版の 1750 万サイクルから 320 万へ):
- 探索の幅を、評価用の発話で品質を見ながら絞る(開ループの 2 つ目のピッチ候補は、調べない方が品質も良かった)
- ピッチの分母は再帰、分子だけ正確に。符号 ↔ ラグは表と算術式で
- 固定符号帳の相関は H^T x と形の内積で。ゲインは近似で比べ、選んだものだけ正確に
- 割り算を避ける(ESP32 の浮動小数の割り算はソフトウェア)。内積の形にして累積を 4 本に分ける
- エンコーダだけ積和を 1 命令(`-ffp-contract=fast`、`madd.s`)にする
- デコーダの固定小数点:巻き戻しの省略と MAC16(ビット一致のまま、63 万 → 38 万サイクル)
- コードと表を内部 RAM に置く
