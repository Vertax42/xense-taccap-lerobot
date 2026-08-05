# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# NOTE: upstream's end-to-end targets (test-act-ete-train, test-diffusion-ete-eval,
# and the rest) drove `lerobot-train` / `lerobot-eval` over the bundled policies.
# This fork ships no policies and no training entry points, so every one of those
# targets invoked a command that does not exist. They are gone; `test` below runs
# the suite that this build actually has, the same one CI runs.

.PHONY: test test-fast lint format build-user

DEVICE ?= cpu

PYTHON_PATH := $(shell which python)

# If uv is installed and a virtual environment exists, use it
UV_CHECK := $(shell command -v uv)
ifneq ($(UV_CHECK),)
	PYTHON_PATH := $(shell .venv/bin/python)
endif

export PATH := $(dir $(PYTHON_PATH)):$(PATH)

test:
	LEROBOT_TEST_DEVICE=$(DEVICE) pytest tests/ -v --timeout=300

# The fork's own code — hardware-free and quick, so it is the one to run while
# working on the grippers, discovery rules or the camera read path.
test-fast:
	LEROBOT_TEST_DEVICE=$(DEVICE) pytest tests/robots/ tests/utils/ -q

lint:
	ruff check .

format:
	ruff format .

build-user:
	docker build -f docker/Dockerfile.user -t lerobot-user .
