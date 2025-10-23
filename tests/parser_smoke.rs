use lrr_230pro_can_driver::radar::{parse_object_part2, parse_frs_status};
use anyhow::Result;

#[test]
fn p2_parse_smoke() -> Result<()> {
    let d = [0u8; 8]; // all zeros → boundary values
    let p2 = parse_object_part2(&d)?;
    // With zeros, signed fields should decode to offset-minimums; we just assert no crash.
    assert!(p2.x_pos_m >= 0.0);
    Ok(())
}

#[test]
fn frs_status_smoke() -> Result<()> {
    let d = [0u8; 8];
    let st = parse_frs_status(&d)?;
    assert!(st.latency_ms >= 0.0);
    Ok(())
}
