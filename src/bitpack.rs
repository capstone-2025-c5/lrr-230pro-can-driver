// src/bitpack.rs
// Minimal helpers for Motorola (big-endian) bit numbering as in DBC "startbit/length" fields.
// Assumptions: startbit 0 = MSB of byte 0; bytes are in network order in CAN payload.

pub fn get_bits_m(data: &[u8], start: u16, len: u16) -> u64 {
    assert!(len > 0 && len <= 64);
    let mut v: u64 = 0;
    for i in 0..len {
        // Motorola: walk bits from start..start+len-1
        let bit_index = start + i;
        let byte = (bit_index / 8) as usize;
        let bit_in_byte = 7 - (bit_index % 8); // MSB-first in each byte
        let bit = ((data[byte] >> bit_in_byte) & 0x1) as u64;
        v = (v << 1) | bit;
    }
    v
}

pub fn set_bits_m(data: &mut [u8], start: u16, len: u16, mut value: u64) {
    assert!(len > 0 && len <= 64);
    // place 'value' into bits [start..start+len) (Motorola). Highest bit of 'value' goes to start.
    for i in (0..len).rev() {
        let bit_index = start + (len - 1 - i);
        let byte = (bit_index / 8) as usize;
        let bit_in_byte = 7 - (bit_index % 8);
        let bit = ((value >> i) & 0x1) as u8;
        let mask = 1u8 << bit_in_byte;
        data[byte] = (data[byte] & !mask) | (bit * mask);
    }
}

// Sign-extend a raw field read with 'len' bits to i64
pub fn sign_extend(value: u64, len: u16) -> i64 {
    let shift = 64 - len;
    ((value << shift) as i64) >> shift
}
