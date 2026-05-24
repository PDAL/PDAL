#!/bin/bash
ctest -VV --output-on-failure --schedule-random -E "^pdal_io_copc_remote_reader_test$"
