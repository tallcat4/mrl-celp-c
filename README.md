# mrl-celp-c — 改良型 RL-CELP のデコーダとエンコーダ(純 C)

改良型 RL-CELP(Modified RL-CELP)の CELP 本体を、C99 だけで実装したものです。JR 在来線デジタル列車無線の音声コーデックで、
1 フレーム 40 ms を 158 ビット(20 バイト、3.95 kbps)で送ります。

| | ディレクトリ | 中身 | 品質 | 速度(Apple M5、1 フレーム 40 ms あたり) |
|---|---|---|---|---|
| **デコーダ** | [`decoder/`](decoder/README.md) | jr-voice(Rust、MIT)の CELP 本体の C 移植。固定小数点の処理をそのまま移したもの | 原本の参照ベクトル 720 フレームと PCM がビット一致 | 5.8 µs(約 7000 倍速)、状態 868 バイト、malloc なし |
| **エンコーダ** | [`encoder/`](encoder/README.md) | オリジナル実装(分析による合成の CELP、float)。デコーダの状態の処理をそのまま使うので、エンコーダとデコーダの状態は常に一致 | PESQ 2.87(LibriSpeech と日本語の合成音声 32 発話) | 92 µs(約 430 倍速)、状態 9.5 KB、malloc なし |

- フレームは 20 バイトで、158 ビットを LSB first で詰めます(上位 2 ビットは 0)。ビットの並びは原本(jr-voice の `bits.rs`)と同じです。
- jr-voice の学習済みの後処理(音質の補正)は含みません。デコーダの出力は CELP 本体の出力そのものです。
- **MCU でも動きます**(実機で実測、エンコード + デコード、40 ms のフレームあたり):
  - Arduino Portenta H7(Cortex-M7、400 MHz):292 万サイクル(CPU の 18%)
  - ESP32-WROOM-32(240 MHz、コードと表を RAM に配置):357 万サイクル(1 コアの 37%、最も重いフレームで 39%)
  - どちらも、デスクトップと同じフレームを出し、復号した PCM もビット一致しました。詳細は [`encoder/README.md`](encoder/README.md#mcu-での実測) にあります。

```
make            # decoder/libmrldec.a、decoder/mrl2wav、encoder/libmrlenc.a、encoder/wav2mrl
make test       # デコーダと原本の参照ベクトルのビット一致、エンコーダと期待のフレームの一致
./encoder/wav2mrl in.wav out.mrl     # 8 kHz・16 ビット・モノラルの WAV → 20 バイト/40 ms のフレーム列
./decoder/mrl2wav out.mrl out.wav    # フレーム列 → 8 kHz の WAV
```
C のアプリへの組み込み方は、それぞれの README を見てください。

## 経緯
デコーダの Python 版への移植、エンコーダの研究(探索の方法、評価の指標、学習を使う試み)は、別のリポジトリ(mrl-celp、Python)で行いました。
エンコーダは、そこで作った Python 版の考え方を、マイコンで動くように探索を絞って C にしたものです。

## ライセンス・注意
- ライセンスは MIT です([`LICENSE`](LICENSE))。
  - `decoder/` は [jr-conventional-line-radio-receiver](https://github.com/soltia48/jr-conventional-line-radio-receiver)(MIT、© 2026 KIRISHIKI Yudai)の
    `crates/jr-voice/src/celp` の派生物で、その固定小数点の DSP と表は [mcelp-codec](https://github.com/soltia48/mcelp-codec)(MIT、同じ作者)に由来します。
    著作権表示は [`decoder/LICENSE`](decoder/LICENSE) にあります。解析の成果を公開してくださった KIRISHIKI Yudai 氏に感謝します。
- テスト用の音声(`encoder/tests/vectors/speech.pcm`)は、LibriSpeech(CC BY 4.0、V. Panayotov ほか、LibriVox の朗読から作成)の抜粋です。
- 原本の注記どおり、後半 2 つ目のゲインの下位 4 ビットの位置と、前半の絶対ピッチの下位 2 ビットの反転は「経験的に確かめた仮定」で、仕様を完全に復元したものではありません。
- コーデックの方式に特許があるかどうかは確認していません。
- 鉄道無線の周波数での送信は、電波法上の免許が無いため違法です。列車の運行の安全も損ないます。このリポジトリは、コーデックの研究と、自作の無線(自分の免許や免許不要の範囲)での利用を目的としています。
