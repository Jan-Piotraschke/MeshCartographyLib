# Makefile for building MeshCartographyLib

SHELL := /bin/bash

# Paths
PROJECT_DIR := $(shell pwd)
ARCHITECTURE := $(shell uname -m)

# Platform selection
PLATFORM ?= executive
ifeq ($(PLATFORM), wasm)
	CMAKE_CMD = emcmake cmake
	BUILD_CMD = emmake ninja
else
	CMAKE_CMD = cmake
	BUILD_CMD = ninja
endif

# Determine OS
OS := $(shell uname -s)

# Compiler paths
ifeq ($(OS), Darwin)
	C_COMPILER=$(shell brew --prefix llvm)/bin/clang
	CXX_COMPILER=$(shell brew --prefix llvm)/bin/clang++
else ifeq ($(OS), Linux)
	C_COMPILER=/usr/bin/gcc
	CXX_COMPILER=/usr/bin/g++
endif

.PHONY: all
all: check_submodule build

.PHONY: check_submodule
check_submodule:
	@if [ ! "$(shell git submodule status | grep pmp-library | cut -c 1)" = "-" ]; then \
		echo "PMP library submodule already initialized and updated."; \
	else \
		echo "PMP library submodule is empty. Initializing and updating..."; \
		git submodule update --init -- pmp-library; \
		$(MAKE) install_pmp; \
	fi

.PHONY: update_submodule
update_submodule:
	@echo "Updating PMP library submodule..."; \
	git submodule update --remote pmp-library;

.PHONY: install_pmp
install_pmp:
	@echo "Installing PMP library..."; \
	mkdir -p build/pmp-library; \
	cd build/pmp-library && $(CMAKE_CMD) $(PROJECT_DIR)/pmp-library -DCMAKE_BUILD_TYPE=Release && make && sudo make install;

.PHONY: build
build:
	echo "Building for platform: $(PLATFORM)"; \
	$(CMAKE_CMD) -S $(PROJECT_DIR) \
			-B $(PROJECT_DIR)/build \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_C_COMPILER=$(C_COMPILER) \
			-DCMAKE_CXX_COMPILER=$(CXX_COMPILER) \
			-DCMAKE_CXX_STANDARD=20 \
			-DCMAKE_OSX_ARCHITECTURES=$(ARCHITECTURE) \
			-GNinja
ifeq ($(OS), Darwin)
	$(BUILD_CMD) -C $(PROJECT_DIR)/build -j $(shell sysctl -n hw.logicalcpu)
else ifeq ($(OS), Linux)
	$(BUILD_CMD) -C $(PROJECT_DIR)/build -j $(shell nproc)
endif

# Cleaning
.PHONY: clean
clean:
	rm -rf $(PROJECT_DIR)/build
