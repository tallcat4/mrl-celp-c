# mrl-celp-c

改良型 RL-CELP(JR 在来線デジタル列車無線の音声コーデック、3.95 kbps)のデコーダとエンコーダを、C99 だけで実装したものです。

## 主な機能

- **ビット一致のデコーダ** — jr-voice(Rust)の CELP 本体の移植。原本の参照ベクトルと PCM がビット単位で一致します。
- **オリジナルのエンコーダ** — 分析による合成の CELP。出すフレームはこのデコーダでそのまま復号できます。
- **マイコンで動く** — エンコードとデコードを同時に動かして、ESP32-WROOM-32 で CPU の 37%、Portenta H7 で 18%。
- **依存なし** — malloc なし、外部ライブラリなし(libm の `logf` / `expf` だけ)。

## 動作環境

- C99 のコンパイラ(macOS / Linux で `cc`、ESP-IDF、Arduino で確認)

## ビルド

```bash
make         # decoder/mrl2wav、encoder/wav2mrl、ライブラリ
make test    # 原本の参照ベクトルとのビット一致、期待のフレームとの一致
```

## 使い方

```bash
encoder/wav2mrl in.wav out.mrl     # 8 kHz・16 ビット・モノラルの WAV → 20 バイト/40 ms のフレーム列
decoder/mrl2wav out.mrl out.wav    # フレーム列 → WAV
```

C の API、フレームの構成、エンコーダの手順、マイコンでの実測は [詳細マニュアル](docs/manual.md) を参照してください。

## ライセンス

MIT。詳細は [`LICENSE`](LICENSE) を参照してください。

- `decoder/` は [jr-conventional-line-radio-receiver](https://github.com/soltia48/jr-conventional-line-radio-receiver)(MIT、© 2026 KIRISHIKI Yudai)の派生物です([`decoder/LICENSE`](decoder/LICENSE))。
- テスト用の音声は LibriSpeech(CC BY 4.0)の抜粋です。
- 鉄道無線の周波数での送信は、電波法上の免許が無いため違法です。コーデックの研究と、自作の無線での利用を目的としています。
