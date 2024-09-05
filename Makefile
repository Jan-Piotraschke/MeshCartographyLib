# Makefile for building MeshCartographyLib

SHELL := /bin/bash

# Paths
PROJECT_DIR := $(shell pwd)
ARCHITECTURE := $(shell uname -m)
export Meshes_Dir := $(PROJECT_DIR)/meshes

# Platform selection
PLATFORM ?= executive
BUILD_DIR = build
ifeq ($(PLATFORM), wasm)
	CMAKE_CMD = emcmake cmake
	BUILD_CMD = emmake ninja
	BUILD_DIR = embuild
else
	CMAKE_CMD = cmake
	BUILD_CMD = ninja
endif

# Determine OS
OS := $(shell uname -s)

# Compiler paths
ifeq ($(OS), Darwin)
	C_COMPILER=$(shell xcrun --find clang)
	CXX_COMPILER=$(shell xcrun --find clang++)
	# Set the correct macOS SDK path
	# export CMAKE_OSX_SYSROOT = /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.5.sdk
else ifeq ($(OS), Linux)
	C_COMPILER=/usr/bin/gcc
	CXX_COMPILER=/usr/bin/g++
endif

.PHONY: all
all: check_submodule install_glog build

.PHONY: check_submodule
check_submodule:
	@if [ ! "$(shell git submodule status | grep pmp-library | cut -c 1)" = "-" ]; then \
		echo "PMP library submodule already initialized and updated."; \
	else \
		echo "PMP library submodule is empty. Initializing and updating..."; \
		git submodule update --init -- pmp-library; \
		$(MAKE) install_pmp; \
	fi
	@if [ ! "$(shell git submodule status | grep ceres-solver | cut -c 1)" = "-" ]; then \
		echo "Ceres solver submodule already initialized and updated."; \
	else \
		echo "Ceres solver submodule is empty. Initializing and updating..."; \
		git submodule update --init -- ceres-solver; \
		$(MAKE) install_ceres; \
	fi
	@if [ ! "$(shell git submodule status | grep glog | cut -c 1)" = "-" ]; then \
		echo "Glog submodule already initialized and updated."; \
	else \
		echo "Glog submodule is empty. Initializing and updating..."; \
		git submodule update --init -- glog; \
		$(MAKE) install_glog; \
	fi

.PHONY: update_pmp
update_pmp:
	@echo "Updating PMP library submodule..."; \
	git submodule update --remote pmp-library;

.PHONY: update_ceres
update_ceres:
	@echo "Updating Ceres solver submodule..."; \
	git submodule update --remote ceres-solver;

.PHONY: install_pmp
install_pmp:
	@echo "Installing PMP library..."; \
	mkdir -p $(BUILD_DIR)/pmp-library; \
	cd $(BUILD_DIR)/pmp-library && \
	$(CMAKE_CMD) -G Ninja $(PROJECT_DIR)/pmp-library -DCMAKE_BUILD_TYPE=Release && \
	ninja && \
	sudo ninja install;

.PHONY: install_glog
install_glog:
	@echo "Installing Google glog library..."; \
	mkdir -p $(PROJECT_DIR)/glog/$(BUILD_DIR); \
	cd $(PROJECT_DIR)/glog && \
	$(CMAKE_CMD) -S . -B $(BUILD_DIR) -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=$(PROJECT_DIR)/glog/install && \
	cmake --build $(BUILD_DIR) && \
	cmake --build $(BUILD_DIR) --target install;

.PHONY: install_ceres
install_ceres:
	@echo "Installing Ceres solver..."; \
	mkdir -p $(BUILD_DIR)/ceres-solver; \
	cd $(BUILD_DIR)/ceres-solver && \
	$(CMAKE_CMD) -G Ninja $(PROJECT_DIR)/ceres-solver -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCERES_BUILD_EXAMPLES=OFF -DCERES_BUILD_TESTS=OFF && \
	ninja && \
	sudo ninja install;

.PHONY: build
build:
	@echo "Building for platform: $(PLATFORM)"; \
	$(CMAKE_CMD) -S $(PROJECT_DIR) \
			-B $(PROJECT_DIR)/$(BUILD_DIR) \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_C_COMPILER=$(C_COMPILER) \
			-DCMAKE_CXX_COMPILER=$(CXX_COMPILER) \
			-DCMAKE_CXX_STANDARD=20 \
			-DCMAKE_OSX_ARCHITECTURES=$(ARCHITECTURE) \
			-GNinja
ifeq ($(OS), Darwin)
	$(BUILD_CMD) -C $(PROJECT_DIR)/$(BUILD_DIR) -j $(shell sysctl -n hw.logicalcpu)
else ifeq ($(OS), Linux)
	$(BUILD_CMD) -C $(PROJECT_DIR)/$(BUILD_DIR) -j $(shell nproc)
endif

.PHONY: wasm
wasm:
	wasm-pack build

# Cleaning
.PHONY: clean
clean:
	rm -rf $(PROJECT_DIR)/build
	rm -rf $(PROJECT_DIR)/embuild
	rm -rf $(PROJECT_DIR)/glog/build
	rm -rf $(PROJECT_DIR)/glog/install
	rm -rf $(PROJECT_DIR)/glog/embuild
