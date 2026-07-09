BIN_OUTPUT = bin/robotiq-epick
MODULE_ARCHIVE = bin/module.tar.gz

GOOS ?= $(shell go env GOOS)
GOARCH ?= $(shell go env GOARCH)

MESH_VENV = bin/mesh-venv
MESH_DIR = epick/meshes

.PHONY: module build build-go test lint clean setup decimate

setup:
	apt-get update && apt-get install -y gcc-aarch64-linux-gnu gcc-x86-64-linux-gnu || true

module: build
	rm -f $(MODULE_ARCHIVE)
	tar czf $(MODULE_ARCHIVE) $(BIN_OUTPUT) meta.json

build: build-go

build-go:
	mkdir -p bin
	CGO_ENABLED=0 GOOS=$(GOOS) GOARCH=$(GOARCH) go build -o $(BIN_OUTPUT) .

test:
	go test -race ./...

lint:
	go mod tidy
	go vet ./...

# Regenerate the embedded visualization meshes from the full-resolution CAD exports
# in epick/meshes/. Deterministic: re-running produces byte-identical STLs. The
# default mesh is capped at 500 triangles because Geometries() is polled by the app
# on every frame update.
decimate: $(MESH_VENV)
	$(MESH_VENV)/bin/python scripts/decimate_stl.py \
		$(MESH_DIR)/epick_full.stl epick/epick_simplified.stl --faces 500
	$(MESH_VENV)/bin/python scripts/decimate_stl.py \
		$(MESH_DIR)/epick_with_realsense_full.stl epick/epick_simplified_with_realsense.stl --faces 2000

$(MESH_VENV): scripts/requirements.txt
	python3 -m venv $(MESH_VENV)
	$(MESH_VENV)/bin/pip install -q --upgrade pip
	$(MESH_VENV)/bin/pip install -q -r scripts/requirements.txt
	touch $(MESH_VENV)

clean:
	rm -rf bin/
