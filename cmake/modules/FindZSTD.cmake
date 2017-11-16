# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

# - Find ZSTD (zstd.h, libzstd.a, libzstd.so, and libzstd.so.0)
# This module defines
#  ZSTD_INCLUDE_DIR, directory containing headers
#  ZSTD_SHARED_LIB, path to libzstd shared library
#  ZSTD_STATIC_LIB, path to libzstd static library
#  ZSTD_FOUND, whether zstd has been found

find_path(ZSTD_INCLUDE_DIR NAMES zstd.h
    PATH_SUFFIXES "include")

set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
find_library(ZSTD_STATIC_LIB NAMES zstd
  PATH_SUFFIXES "lib")

set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_SHARED_LIBRARY_SUFFIX})
find_library(ZSTD_SHARED_LIB NAMES zstd
  PATH_SUFFIXES "lib")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZSTD REQUIRED_VARS
  ZSTD_STATIC_LIB ZSTD_SHARED_LIB ZSTD_INCLUDE_DIR)
