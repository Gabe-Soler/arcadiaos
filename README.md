# ArcadiaOS

ArcadiaOS: an operating system for robot crews building on Mars.

ArcadiaOS decides which robot does what, when, and with how much power, while Earth supervises across a 4–24 minute delay and multi-week blackouts. It is measured on [ArcadiaBench](https://github.com/Gabe-Soler/ArcadiaBench), and it talks to the world only through [arcadia-protocol](https://github.com/Gabe-Soler/arcadia-protocol). That means the same coordinator can drive the simulator now and real hardware later.

## Status

**Week 1 scaffold.** The protocol session works: version handshake, then the observation → decision loop, then end. The coordinator doesn't plan yet; it acknowledges every observation with an empty decision. Task allocation and scheduling arrive in weeks 3–4.

## Run

ArcadiaOS is a coordinator process that speaks newline-delimited JSON on stdin/stdout and logs to stderr. A simulator launches it, for example:

```sh
cargo build --release
arcadiabench run -- ./target/release/arcadiaos
```

## Develop

```sh
cargo test     # drives the real binary over stdio, playing the simulator's side
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --check
```

## License

MIT
