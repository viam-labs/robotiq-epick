BIN_OUTPUT = bin/robotiq-epick
MODULE_ARCHIVE = bin/module.tar.gz

GOOS ?= $(shell go env GOOS)
GOARCH ?= $(shell go env GOARCH)

.PHONY: module build build-go test lint clean setup

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

clean:
	rm -rf bin/
