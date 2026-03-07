build:
	cargo build

test:
	cargo test

clippy:
	cargo clippy

lint:
	cargo clippy --all-targets --features gvp
	cargo test --lib --features gvp
	cargo test --doc --features gvp

publish:
	cargo publish --dry-run
	cargo publish

clean:
	cargo clean

.PHONY: build test clippy lint publish clean
