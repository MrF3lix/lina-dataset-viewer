#!/bin/bash
# One-time setup on the cluster's login node (it needs internet): install a Node.js binary
# into ~/.local/node when none is on PATH, then install the viewer's one dependency (h5wasm)
# into node_modules/. The home directory is shared with the compute nodes, so the jobs find both.
#
#   cluster/install_node.sh                   # Node 22 LTS into ~/.local/node, then npm ci
#   NODE_VERSION=v22.11.0 VIEWER_NODE_DIR=/raid/.../node cluster/install_node.sh

set -euo pipefail
cd "$(dirname "$0")/.."

NODE_VERSION="${NODE_VERSION:-v22.11.0}"
NODE_DIR="${VIEWER_NODE_DIR:-$HOME/.local/node}"
export PATH="${NODE_DIR}/bin:$PATH"

if command -v node >/dev/null 2>&1 && [ "$(node -p 'process.versions.node.split(".")[0] >= 18')" = "true" ]; then
    echo "node $(node --version) found at $(command -v node)"
else
    case "$(uname -m)" in
        x86_64) ARCH=x64 ;;
        aarch64) ARCH=arm64 ;;
        *) echo "unsupported architecture $(uname -m)" >&2; exit 1 ;;
    esac
    NAME="node-${NODE_VERSION}-linux-${ARCH}"
    URL="https://nodejs.org/dist/${NODE_VERSION}/${NAME}.tar.xz"
    TMP="$(mktemp -d)"
    trap 'rm -rf "${TMP}"' EXIT
    echo "downloading ${URL}"
    curl -fsSL "${URL}" -o "${TMP}/${NAME}.tar.xz"
    curl -fsSL "https://nodejs.org/dist/${NODE_VERSION}/SHASUMS256.txt" -o "${TMP}/SHASUMS256.txt"
    (cd "${TMP}" && grep " ${NAME}.tar.xz\$" SHASUMS256.txt | sha256sum -c -)
    mkdir -p "${NODE_DIR}"
    tar -xJf "${TMP}/${NAME}.tar.xz" -C "${NODE_DIR}" --strip-components=1
    echo "installed node $(node --version) into ${NODE_DIR}"
fi

npm ci --omit=dev --no-audit --no-fund
echo "ready: DATASET=/path/to/dataset sbatch cluster/viewer.submit   (or cluster/view.sh /path/to/dataset)"
