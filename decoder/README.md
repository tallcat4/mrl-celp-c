# decoder — 改良型 RL-CELP のデコーダ(jr-voice の CELP 本体の C 移植)

[jr-conventional-line-radio-receiver](https://github.com/soltia48/jr-conventional-line-radio-receiver)(Rust、MIT)の
`crates/jr-voice/src/celp`(commit `dfc7440`)を、C99 にビット単位で一致するよう移植したものです。

- **一致**:原本が作った参照ベクトル(乱数・少しずつ変化・全 0・交互の 12 列 720 フレーム)と、PCM がビット単位で一致します(`make test`)。
- **軽さ**:16 ビット固定小数点 DSP(40 ビット累算器、小数乗算)の処理を `int64_t` で再現し、LSF とゲインの予測だけが単精度浮動小数です。
  動的メモリ確保はなく、libm は `logf` / `expf` だけを使います。状態は 868 バイトです。
  速度は Apple M5 で 1 フレーム(40 ms)5.8 µs です。MCU の実機では、Cortex-M7(400 MHz)で 22 万サイクル(0.54 ms)、
  ESP32(240 MHz、コードと表を RAM に配置)で 38 万サイクル(1.6 ms)です([`../encoder/README.md`](../encoder/README.md#mcu-での実測))。
- jr-voice の学習済みの後処理(音質の補正、先読み 280 ms)は含みません。出力は CELP 本体の出力そのものです。

## ビルド
```
make            # libmrldec.a、mrl2wav(フレーム列 → WAV)、テスト用プログラム
make test       # 原本の参照ベクトルとのビット一致
```
ソースは `src/*.c` だけで、依存はありません。自分のビルドシステムに `src/*.c` と `include/` を加えても使えます。
その場合も **`-ffp-contract=off`**(積和の融合を禁止)でコンパイルしてください。ゲインと LSF の予測の丸めが原本と変わります。

## 使い方
```
./mrl2wav in.mrl out.wav          # 20 バイト/40 ms のフレーム列 → 8 kHz・16 ビット・モノラルの WAV
./mrl2wav in.mrl out.raw          # 見出しなしの PCM
./mrl2wav in.mrl out.wav --pre    # 出力フィルタの前の合成(評価用)
```
```c
#include "mrlcelp.h"
static double mem[1024 / sizeof(double)];      /* mrl_decoder_size()(868)バイト以上、8 バイト境界 */
mrl_decoder *d = mrl_decoder_init(mem);
int16_t pcm[MRL_FRAME_SAMPLES];                /* 320 sample = 40 ms */
if (mrl_decode(d, payload20, pcm) != 0) { /* 上位 2 ビットが 0 でない(不正なフレーム) */ }
```

## フレームの構成
40 ms のフレームを 20 ms の半分 2 つに分け、半分をさらに 10 ms(80 sample)のサブフレーム 2 つに分けます。

| 項目 | 単位 | ビット数 |
|---|---|---|
| モード(LSF の移動平均予測の係数の組) | 半分 | 1 |
| LSF の 2 段ベクトル量子化(1 段 128 通り × 10 次、2 段 64 通り × 5 次 × 2) | 半分 | 7 + 6 + 6 |
| ピッチ(1/3 sample の分解能、20〜143) | サブフレーム | 絶対 8 / 差分 5 |
| 固定符号帳(パルスの形 2〜3 個の区分 0〜4、単位パルス 5 + 5 個の区分 5) | サブフレーム | 16 |
| ゲイン(ピッチ利得と、予測つきの符号帳利得の組、128 通り) | サブフレーム | 7 |

計 158 ビットです。ビットの並びは `src/bits.c`(`mrl_unpack` / `mrl_pack`)にあります。

## 移植の注意
- 40 ビット累算器の巻き戻しは、値の範囲からその範囲に収まることが分かる所(適応符号帳の補間、合成フィルタ、出力フィルタ、`mul32x16`)では省いています。結果は同じで、テストで確かめています。
- `-DMRL_MAC16` を付けると、Xtensa(ESP32 など)の MAC16 の 40 ビット累算器で積和をします。巻き戻しの省略と合わせて、ESP32 でデコーダが 63 万 → 38 万サイクルになりました。
- `logf` / `expf` はプラットフォームの libm を使います(原本の Rust 版の `f32::ln` / `f32::exp` と同じ)。libm が違うと、ごくまれに最下位ビットの違いで PCM が ±1〜2 違うことがあります。参照ベクトルは macOS(arm64)で作ったものです(`tests/vectors/platform`)。
- `src/tables.c` は原本の表から自動生成したもので、手で編集しません。
- `src/core.h` の関数(サブフレームの段階ごとの処理)は、エンコーダが候補ごとにデコーダの状態を進めるのに使います。
