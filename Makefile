build:
	cargo build

test:
	cargo test --all-features

clippy:
	cargo clippy --all-targets --all-features

lint:
	cargo clippy --all-targets --all-features
	cargo test --all-features

build-frp:
	cargo build --release --features frp --bin ironsight-frp

build-windows:
	cargo build --release --features frp --bin ironsight-frp --target x86_64-pc-windows-gnu

publish:
	cargo publish --dry-run
	cargo publish

clean:
	cargo clean

.PHONY: build test clippy lint build-frp build-windows publish clean
