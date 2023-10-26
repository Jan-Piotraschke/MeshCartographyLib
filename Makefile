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
	@if [ ! "$(shell git submodule status | grep ceres-solver | cut -c 1)" = "-" ]; then \
		echo "Ceres solver submodule already initialized and updated."; \
	else \
		echo "Ceres solver submodule is empty. Initializing and updating..."; \
		git submodule update --init -- ceres-solver; \
		$(MAKE) install_ceres; \
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
	mkdir -p build/pmp-library; \
	cd build/pmp-library && \
	$(CMAKE_CMD) -G Ninja $(PROJECT_DIR)/pmp-library -DCMAKE_BUILD_TYPE=Release && \
	ninja && \
	sudo ninja install;

.PHONY: install_ceres
install_ceres:
	@echo "Installing Ceres solver..."; \
	mkdir -p build/ceres-solver; \
	cd build/ceres-solver && \
	$(CMAKE_CMD) -G Ninja $(PROJECT_DIR)/ceres-solver -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCERES_BUILD_EXAMPLES=OFF -DCERES_BUILD_TESTS=OFF && \
	ninja && \
	sudo ninja install;

.PHONY: build
build:
	@echo "Building for platform: $(PLATFORM)"; \
	$(CMAKE_CMD) -S $(PROJECT_DIR) \
			-B $(PROJECT_DIR)/build \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_C_COMPILER=$(C_COMPILER) \
			-DCMAKE_CXX_COMPILER=$(CXX_COMPILER) \
			-DCMAKE_CXX_STANDARD=20 \
			-DCMAKE_OSX_ARCHITECTURES=$(ARCHITECTURE) \
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
