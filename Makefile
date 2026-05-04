BIN_OUTPUT = bin/robotiq-epick
MODULE_ARCHIVE = bin/module.tar.gz

.PHONY: module build build-go test lint clean

module: build
	rm -f $(MODULE_ARCHIVE)
	tar czf $(MODULE_ARCHIVE) $(BIN_OUTPUT) meta.json

build: build-go

build-go:
	mkdir -p bin
	go build -o $(BIN_OUTPUT) .

test:
	go test -race ./...

lint:
	go mod tidy
	go vet ./...

clean:
	rm -rf bin/
