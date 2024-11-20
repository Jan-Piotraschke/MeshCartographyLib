# Makefile for building MeshCartographyLib

SHELL := /bin/bash

# Paths
PROJECT_DIR := $(shell pwd)
ARCHITECTURE := $(shell uname -m)
export Meshes_Dir := $(PROJECT_DIR)/meshes

CMAKE_CMD := cmake

# Determine OS
OS := $(shell uname -s)

# Compiler paths and vcpkg settings
ifeq ($(OS), Darwin)
    C_COMPILER=$(shell brew --prefix llvm)/bin/clang
    CXX_COMPILER=$(shell brew --prefix llvm)/bin/clang++
	VCPKG_DEFAULT_TRIPLET := x64-osx
else ifeq ($(OS), Linux)
    C_COMPILER=/usr/bin/gcc
    CXX_COMPILER=/usr/bin/g++
    VCPKG_DEFAULT_TRIPLET := x64-linux
endif

VCPKG_ROOT := $(PROJECT_DIR)/vcpkg
VCPKG_TOOLCHAIN := $(VCPKG_ROOT)/scripts/buildsystems/vcpkg.cmake

.PHONY: all
all: init_vcpkg build

.PHONY: update_submodule
update_submodule:
	@echo "Updating vcpkg submodule..."
	git submodule update --remote vcpkg

# Initialize and set up vcpkg
.PHONY: init_vcpkg
init_vcpkg:
	@echo "Initializing vcpkg..."
	cd $(VCPKG_ROOT) && ./bootstrap-vcpkg.sh
	@echo "Integrating vcpkg with system..."
	$(VCPKG_ROOT)/vcpkg integrate install
	@echo "Installing libraries via vcpkg..."
	$(VCPKG_ROOT)/vcpkg install boost opencv4 glog pmp-library ceres
	@echo "vcpkg initialization and library installation complete."


.PHONY: build
build:
	@echo "Building for platform: $(OS)"
	$(CMAKE_CMD) -S $(PROJECT_DIR) \
			-B $(PROJECT_DIR)/build \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_C_COMPILER=$(C_COMPILER) \
			-DCMAKE_CXX_COMPILER=$(CXX_COMPILER) \
			-DCMAKE_CXX_STANDARD=20 \
			-DCMAKE_OSX_ARCHITECTURES=$(ARCHITECTURE) \
			-GNinja
	ninja -C $(PROJECT_DIR)/build -j $(shell sysctl -n hw.logicalcpu || nproc)

# Cleaning
.PHONY: clean
clean:
	rm -rf $(PROJECT_DIR)/build
