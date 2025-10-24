# LRR 230Pro CAN Driver
Automotive 77GHz Long-Range Radar CAN driver in Rust.

Requires a Linux system with SocketCAN support.

Quick run: 
# Bring up CAN @ 500kbps (adjust if your radar differs)
sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 500000 dbitrate 2000000 fd off

# Build & run
cargo build --release
RUST_LOG=info ./target/release/lrr-230pro-can-driver