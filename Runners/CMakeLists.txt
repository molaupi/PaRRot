# ******************************************************************************
# MIT License
#
# Copyright (c) 2020 Valentin Buchhold
# Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
# Copyright (c) 2024 Patrick Steil
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ******************************************************************************


if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(KASSERT_ASSERTION_LEVEL 40)
else ()
    set(KASSERT_ASSERTION_LEVEL 20)
endif ()
message("Using kassert assertion level ${KASSERT_ASSERTION_LEVEL}")

add_executable(PTaxi RunPTaxi.cpp)
target_compile_definitions(PTaxi PRIVATE CSV_IO_NO_THREAD)
set_target_properties(PTaxi PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(PTaxi routingkit vectorclass fast_cpp_csv_parser kassert)
target_include_directories(PTaxi PUBLIC ${PARROT_INCLUDE_DIR})
if(OpenMP_FOUND)
    target_link_libraries(PTaxi OpenMP::OpenMP_CXX)
endif()

include(KaRRi_compile_params_PTaxi.cmake)