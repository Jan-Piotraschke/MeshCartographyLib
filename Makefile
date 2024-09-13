# Makefile for building MeshCartographyLib

SHELL := /bin/bash

# Paths
PROJECT_DIR := $(shell pwd)
ARCHITECTURE := $(shell uname -m)
CMAKE_CMD := cmake

# Determine OS
OS := $(shell uname -s)

# Compiler paths
ifeq ($(OS), Darwin)
	C_COMPILER=$(shell brew --prefix llvm)/bin/clang
	CXX_COMPILER=$(shell brew --prefix llvm)/bin/clang++
	VCPKG_ROOT := $(PROJECT_DIR)/vcpkg
	VCPKG_TOOLCHAIN := $(VCPKG_ROOT)/scripts/buildsystems/vcpkg.cmake
else ifeq ($(OS), Linux)
	C_COMPILER=/usr/bin/gcc
	CXX_COMPILER=/usr/bin/g++
	VCPKG_ROOT := $(PROJECT_DIR)/vcpkg
	VCPKG_TOOLCHAIN := $(VCPKG_ROOT)/scripts/buildsystems/vcpkg.cmake
endif

.PHONY: all
all: init_vcpkg run_leli build

.PHONY: update_submodule
update_submodule:
	@echo "Updating vcpkg submodule..."; \
	git submodule update --remote vcpkg;

# Initialize and set up vcpkg
.PHONY: init_vcpkg
init_vcpkg:
	@echo "Initializing vcpkg..."
	cd $(VCPKG_ROOT) && ./bootstrap-vcpkg.sh
	@echo "Integrating vcpkg with system..."
	$(VCPKG_ROOT)/vcpkg integrate install
	@echo "Installing librdkafka via vcpkg..."
	$(VCPKG_ROOT)/vcpkg install opencv4 glog cgal ceres
	@echo "vcpkg initialization and library installation complete."

.PHONY: run_leli
run_leli:
	@echo "Running leli from $(PROJECT_DIR)..."
	@ls -l $(PROJECT_DIR)/leli
	$(PROJECT_DIR)/leli extract --folder dev --output src


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
			-DCMAKE_TOOLCHAIN_FILE=$(VCPKG_TOOLCHAIN) \
			-GNinja
ifeq ($(OS), Darwin)
	ninja -C $(PROJECT_DIR)/build -j $(shell sysctl -n hw.logicalcpu)
else ifeq ($(OS), Linux)
	ninja -C $(PROJECT_DIR)/build -j $(shell nproc)
endif

# Cleaning
.PHONY: clean
clean:
	rm -rf $(PROJECT_DIR)/build
