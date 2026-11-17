# ArcadiaOS as a container: the coordinator, speaking arcadia-protocol on
# standard input and output. This is the image ArcadiaBench's leaderboard
# builds and runs an entry from; it needs no network, no writable files and
# no privileges at run time.
#
#   docker build -t arcadiaos .
#   arcadiabench run --seed 1 -- docker run --rm -i --network none --read-only arcadiaos
FROM rust:1-bookworm AS build
WORKDIR /src
COPY . .
RUN cargo build --release --locked --bin arcadiaos

FROM debian:bookworm-slim
COPY --from=build /src/target/release/arcadiaos /usr/local/bin/arcadiaos
USER 65534:65534
ENTRYPOINT ["/usr/local/bin/arcadiaos"]
