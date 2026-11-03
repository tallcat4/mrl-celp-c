// 参照の生成器:上流の voice_frame.rs と coding.rs を写したクレートで、test_fec.c と同じケースを復号する。
#![allow(dead_code)]
mod coding;
mod voice_frame;
use voice_frame::{decode, decode_soft, encode};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 { self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17; self.0 }
    fn below(&mut self, n: u64) -> u64 { self.next() % n }
}
struct Fnv(u64);
impl Fnv {
    fn byte(&mut self, b: u8) { self.0 ^= b as u64; self.0 = self.0.wrapping_mul(0x100000001b3); }
    fn bytes(&mut self, b: &[u8]) { for &x in b { self.byte(x) } }
}
fn flip(env: &mut [u8; 48], pos: usize) { env[pos / 8] ^= 1 << (7 - pos % 8); }
fn data_pos(i: usize) -> usize { 128 * (i / 112) + 16 + i % 112 }
fn record(h: &mut Fnv, s: &Option<voice_frame::Speech>, n: &mut u32) {
    match s {
        Some(s) => { *n += 1; h.byte(1); h.bytes(&s.payload); h.bytes(&s.corrected_bits.to_le_bytes()); }
        None => h.byte(0),
    }
}
fn main() {
    let verbose = std::env::args().nth(1).is_some();
    let mut r = Rng(0x9e37_79b9_7f4a_7c15);
    // category: name, count
    let cats: [(&str, usize); 5] = [("clean", 500), ("scattered", 3000), ("uninformed", 1000), ("burst", 1500), ("random", 2000)];
    for (cat, &(name, count)) in cats.iter().enumerate() {
        let mut h = Fnv(0xcbf29ce484222325);
        let (mut nh, mut ns) = (0u32, 0u32);
        for case in 0..count {
            let mut env = [0u8; 48];
            let mut conf = [0u8; 384];
            if cat == 4 {
                for b in env.iter_mut() { *b = r.next() as u8; }
                for c in conf.iter_mut() { *c = r.next() as u8; }
            } else {
                let mut payload = [0u8; 20];
                for b in payload.iter_mut() { *b = r.next() as u8; }
                payload[19] &= 0x3f;
                let alternate = r.below(2) == 1;
                env = encode(&payload, alternate);
                if cat == 0 { h.bytes(&env); }
                for c in conf.iter_mut() { *c = (60 + r.below(196)) as u8; }
                match cat {
                    1 | 2 => {
                        let k = 1 + r.below(30) as usize;
                        for _ in 0..k {
                            let p = data_pos(r.below(336) as usize);
                            flip(&mut env, p);
                            conf[p] = r.below(41) as u8;
                        }
                        for c in conf.iter_mut() { if r.below(10) == 0 { *c = r.below(60) as u8; } }
                        if cat == 2 { for c in conf.iter_mut() { *c = r.next() as u8; } }
                    }
                    3 => {
                        let len = 1 + r.below(48) as usize;
                        let start = r.below((384 - len + 1) as u64) as usize;
                        for p in start..start + len { flip(&mut env, p); conf[p] = r.below(30) as u8; }
                    }
                    _ => {}
                }
            }
            let hard = decode(&env);
            let soft = decode_soft(&env, &conf);
            if verbose {
                let f = |s: &Option<voice_frame::Speech>| match s { Some(s) => format!("{} {}", s.payload.iter().map(|b| format!("{b:02x}")).collect::<String>(), s.corrected_bits), None => "-".into() };
                println!("{name} {case} H {} S {}", f(&hard), f(&soft));
            }
            record(&mut h, &hard, &mut nh);
            record(&mut h, &soft, &mut ns);
        }
        if !verbose { println!("{name} {count} {nh} {ns} {:016x}", h.0); }
    }
}
