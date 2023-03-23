FROM rust:1.67

RUN apt-get update -yqq && apt-get install -yqq --no-install-recommends build-essential && rm -rf /var/lib/apt/lists/*
RUN rustup target add thumbv6m-none-eabi
RUN rustup component add clippy rustfmt rust-src



