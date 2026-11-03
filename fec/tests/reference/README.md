# 参照の作り方

`fec_reference.txt` は、原本(Rust)の復号の結果から作ります。

```bash
git clone https://github.com/soltia48/jr-conventional-line-radio-receiver jr
git -C jr checkout 9945870
cp jr/crates/receiver-core/src/{voice_frame.rs,coding.rs,voice_coordinates.in} fec/tests/reference/src/
cd fec/tests/reference
cargo run --release             # 5 行のハッシュ(fec_reference.txt の中身)
cargo run --release -- -v       # ケースごとの結果(../test_fec -v と同じ書式)
```

`src/main.rs` の乱数とケースの作り方は `../test_fec.c` と同じです。片方を変えたら、もう片方も同じように変えてください。
