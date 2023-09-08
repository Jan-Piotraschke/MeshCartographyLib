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
	@if [ ! "$(shell git submodule status | grep cgal | cut -c 1)" = "-" ]; then \
		echo "CGAL submodule already initialized and updated."; \
	else \
		echo "CGAL submodule is empty. Initializing and updating..."; \
		git submodule update --init -- cgal; \
		$(MAKE) install_cgal; \
	fi

.PHONY: update_submodule
update_submodule:
	@echo "Updating CGAL submodule..."; \
	git submodule update --remote cgal;

.PHONY: install_cgal
install_cgal:
	@echo "Installing CGAL..."; \
	mkdir -p build/cgal; \
	cd build/cgal && $(CMAKE_CMD) $(PROJECT_DIR)/cgal -DCMAKE_BUILD_TYPE=Release -DCGAL_HEADER_ONLY=OFF && make && sudo make install;

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
	ninja -C $(PROJECT_DIR)/build -j $(shell sysctl -n hw.logicalcpu)
else ifeq ($(OS), Linux)
	ninja -C $(PROJECT_DIR)/build -j $(shell nproc)
endif

# Cleaning
.PHONY: clean
clean:
	rm -rf $(PROJECT_DIR)/build
