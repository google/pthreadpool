# Copyright (c) 2017 Facebook Inc.
# Copyright (c) 2015-2017 Georgia Institute of Technology
# All rights reserved.
#
# Copyright 2019 Google LLC
#
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

CMAKE_MINIMUM_REQUIRED(VERSION 3.5 FATAL_ERROR)

PROJECT(cpuinfo-download NONE)

INCLUDE(ExternalProject)
ExternalProject_Add(cpuinfo
	URL https://github.com/pytorch/cpuinfo/archive/19b9316c71e4e45b170a664bf62ddefd7ac9feb5.zip
	URL_HASH SHA256=e0a485c072de957668eb324c49d726dc0fd736cfb9436b334325f20d93085003
	SOURCE_DIR "${CMAKE_BINARY_DIR}/cpuinfo-source"
	BINARY_DIR "${CMAKE_BINARY_DIR}/cpuinfo"
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
	TEST_COMMAND ""
)
