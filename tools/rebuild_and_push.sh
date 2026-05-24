#!/usr/bin/env bash
# rebuild_and_push.sh — Build the SANCHO Docker image and publish it to Docker Hub.
#
# Usage: run from any directory; the script always operates on the repo root.
# Requires: docker login already performed for docker.io/diego586 before running.
#
# Steps:
#   1. Pull latest code from GitHub
#   2. Build the image from the repo root (docker/Dockerfile needs ros2_ws/ in context)
#   3. Push the image to Docker Hub
#   4. Verify: pull the image back and confirm motor_bridge_node prints dry_run=True

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="diego586/sancho:latest"

echo "=== SANCHO rebuild and push ==="
echo "Repo root: ${REPO_ROOT}"
echo "Image:     ${IMAGE}"
echo

# ── Step 1: pull latest code ──────────────────────────────────────────────────
echo "[1/4] Pulling latest code from GitHub..."
git -C "${REPO_ROOT}" pull
echo

# ── Step 2: build the Docker image ───────────────────────────────────────────
# Must be built from the repo root so that COPY ros2_ws/ in the Dockerfile
# can find the source tree. The Dockerfile lives at docker/Dockerfile.
echo "[2/4] Building Docker image ${IMAGE}..."
docker build \
    -f "${REPO_ROOT}/docker/Dockerfile" \
    -t "${IMAGE}" \
    "${REPO_ROOT}"
echo

# ── Step 3: push to Docker Hub ────────────────────────────────────────────────
echo "[3/4] Pushing ${IMAGE} to Docker Hub..."
docker push "${IMAGE}"
echo

# ── Step 4: verify the pushed image ──────────────────────────────────────────
# Pull the image back from the registry and run motor_bridge_node.
# The node must print dry_run=True (Bridge library not present in image,
# so the node falls back to dry-run mode automatically).
echo "[4/4] Verifying image from Docker Hub..."
docker pull "${IMAGE}"

OUTPUT=$(docker run --rm "${IMAGE}" \
    ros2 run sancho_bridge motor_bridge_node 2>&1 || true)

echo "${OUTPUT}"

if echo "${OUTPUT}" | grep -q "dry_run=True"; then
    echo
    echo "Verification passed — motor_bridge_node reports dry_run=True."
else
    echo
    echo "ERROR: verification failed — 'dry_run=True' not found in node output." >&2
    echo "Full output was:" >&2
    echo "${OUTPUT}" >&2
    exit 1
fi

echo
echo "=== Done. ${IMAGE} is live on Docker Hub. ==="
