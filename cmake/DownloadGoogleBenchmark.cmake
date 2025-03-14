# Copyright (c) 2017 Facebook Inc.
# Copyright (c) 2015-2017 Georgia Institute of Technology
# All rights reserved.
#
# Copyright 2019 Google LLC
#
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

CMAKE_MINIMUM_REQUIRED(VERSION 3.5 FATAL_ERROR)

PROJECT(googlebenchmark-download NONE)

INCLUDE(ExternalProject)
ExternalProject_Add(googlebenchmark
	URL https://github.com/google/benchmark/archive/v1.5.3.zip
	URL_HASH SHA256=bdefa4b03c32d1a27bd50e37ca466d8127c1688d834800c38f3c587a396188ee
	SOURCE_DIR "${CMAKE_BINARY_DIR}/googlebenchmark-source"
	BINARY_DIR "${CMAKE_BINARY_DIR}/googlebenchmark"
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
	TEST_COMMAND ""
)
