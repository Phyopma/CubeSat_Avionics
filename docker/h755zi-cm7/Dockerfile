# syntax=docker/dockerfile:1
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        binutils-arm-none-eabi \
        ca-certificates \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
