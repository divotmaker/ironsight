build:
	cargo build

test:
	cargo test

clippy:
	cargo clippy

publish:
	cargo publish --dry-run
	cargo publish

clean:
	cargo clean

.PHONY: build test clippy publish clean
