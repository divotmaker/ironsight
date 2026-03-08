build:
	cargo build

test:
	cargo test

clippy:
	cargo clippy

lint:
	cargo clippy --all-targets --features gvp,frp
	cargo test --lib --features gvp,frp
	cargo test --doc --features gvp,frp

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
