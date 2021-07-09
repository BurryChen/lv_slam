#!/usr/bin/env bash

set -e

# Install prerequisite packages for apt
apt-get -y update && \
    apt-get install -y --no-install-recommends \
    apt-utils 

# Install a few prerequisite packages which let apt use packages over HTTPS
apt-get -y update && \
    apt-get install -y --no-install-recommends \
    ca-certificates apt-transport-https

# Change apt source to Tsinghua University mirror
sed -i "s@http://archive.ubuntu.com/ubuntu/@https://mirrors.tuna.tsinghua.edu.cn/ubuntu/@g" /etc/apt/sources.list

apt-get -y update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    git    \
    sudo   \
    xz-utils

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*