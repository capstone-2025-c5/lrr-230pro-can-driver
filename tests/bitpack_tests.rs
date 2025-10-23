use lrr_230pro_can_driver::bitpack::{get_bits_m, set_bits_m, sign_extend};

#[test]
fn motorola_roundtrip_simple() {
    let mut d = [0u8; 8];
    // put 0b10101 (21) into startbit 3, len 5
    set_bits_m(&mut d, 3, 5, 0b10101);
    assert_eq!(get_bits_m(&d, 3, 5), 0b10101);
}

#[test]
fn sign_extend_ok() {
    // 11-bit signed: -1 should be 0x7FF -> -1
    assert_eq!(sign_extend(0x7FF, 11), -1);
    // 11-bit signed: 0x400 -> -1024?
    assert_eq!(sign_extend(0x400, 11), -1024);
}
