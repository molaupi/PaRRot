# ******************************************************************************
# MIT License
#
# Copyright (c) 2020 Valentin Buchhold
# Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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


add_executable(ConvertGraph ConvertGraph.cc)
target_compile_definitions(ConvertGraph PRIVATE CSV_IO_NO_THREAD)
set_target_properties(ConvertGraph PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(ConvertGraph ${PROJ_LIBRARY} routingkit kassert rapidxml fast_cpp_csv_parser)
target_include_directories(ConvertGraph PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(GenerateODPairs GenerateODPairs.cc)
target_compile_definitions(GenerateODPairs PRIVATE CSV_IO_NO_THREAD)
set_target_properties(GenerateODPairs PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(GenerateODPairs kassert fast_cpp_csv_parser)
if(OpenMP_FOUND)
  target_link_libraries(GenerateODPairs OpenMP::OpenMP_CXX)
endif()
target_include_directories(GenerateODPairs PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(GenerateRandomVehicles GenerateRandomVehicles.cc)
target_compile_definitions(GenerateRandomVehicles PRIVATE CSV_IO_NO_THREAD)
set_target_properties(GenerateRandomVehicles PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(GenerateRandomVehicles kassert fast_cpp_csv_parser)
target_include_directories(GenerateRandomVehicles PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(DrawRandomDepartureTimes DrawRandomDepartureTimes.cc)
target_compile_definitions(DrawRandomDepartureTimes PRIVATE CSV_IO_NO_THREAD)
set_target_properties(DrawRandomDepartureTimes PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(DrawRandomDepartureTimes fast_cpp_csv_parser)
target_include_directories(DrawRandomDepartureTimes PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(OsmToCarAndPassengerGraph OsmToCarAndPassengerGraph.cc)
target_compile_definitions(OsmToCarAndPassengerGraph PRIVATE CSV_IO_NO_THREAD)
set_target_properties(OsmToCarAndPassengerGraph PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(OsmToCarAndPassengerGraph proj routingkit kassert fast_cpp_csv_parser)
target_include_directories(OsmToCarAndPassengerGraph PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(OsmToPassengerGraph OsmToPassengerGraph.cc)
target_compile_definitions(OsmToPassengerGraph PRIVATE CSV_IO_NO_THREAD)
set_target_properties(OsmToPassengerGraph PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(OsmToPassengerGraph proj routingkit kassert fast_cpp_csv_parser)
target_include_directories(OsmToPassengerGraph PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(TransformLocations TransformLocations.cc)
target_compile_definitions(TransformLocations PRIVATE CSV_IO_NO_THREAD)
set_target_properties(TransformLocations PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(TransformLocations proj routingkit kassert fast_cpp_csv_parser)
target_include_directories(TransformLocations PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(CheckExistingAttributesInNetworkFile CheckExistingAttributesInNetworkFile.cc)
target_compile_definitions(CheckExistingAttributesInNetworkFile PRIVATE CSV_IO_NO_THREAD)
set_target_properties(CheckExistingAttributesInNetworkFile PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(CheckExistingAttributesInNetworkFile kassert fast_cpp_csv_parser)
target_include_directories(CheckExistingAttributesInNetworkFile PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(GenerateGeoJsonForGraph GenerateGeoJsonForGraph.cc)
target_compile_definitions(GenerateGeoJsonForGraph PRIVATE CSV_IO_NO_THREAD)
set_target_properties(GenerateGeoJsonForGraph PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(GenerateGeoJsonForGraph PRIVATE nlohmann_json kassert fast_cpp_csv_parser)
target_include_directories(GenerateGeoJsonForGraph PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(GenerateGeoJsonForVehicles GenerateGeoJsonForVehicles.cc)
target_compile_definitions(GenerateGeoJsonForVehicles PRIVATE CSV_IO_NO_THREAD)
set_target_properties(GenerateGeoJsonForVehicles PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(GenerateGeoJsonForVehicles PRIVATE nlohmann_json kassert fast_cpp_csv_parser)
target_include_directories(GenerateGeoJsonForVehicles PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(GenerateGeoJsonForRequests GenerateGeoJsonForRequests.cc)
target_compile_definitions(GenerateGeoJsonForRequests PRIVATE CSV_IO_NO_THREAD)
set_target_properties(GenerateGeoJsonForRequests PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(GenerateGeoJsonForRequests PRIVATE nlohmann_json kassert fast_cpp_csv_parser)
target_include_directories(GenerateGeoJsonForRequests PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(BuildStaticBuckets BuildStaticBuckets.cc)
target_compile_definitions(BuildStaticBuckets PRIVATE CSV_IO_NO_THREAD)
set_target_properties(BuildStaticBuckets PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(BuildStaticBuckets routingkit vectorclass fast_cpp_csv_parser kassert)
target_include_directories(BuildStaticBuckets PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(TransformRequests TransformRequests.cc)
target_compile_definitions(TransformRequests PRIVATE CSV_IO_NO_THREAD)
set_target_properties(TransformRequests PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(TransformRequests fast_cpp_csv_parser)
target_include_directories(TransformRequests PUBLIC ${PARROT_INCLUDE_DIR})

add_executable(KarriRequestsToParrotRawFormat KarriRequestsToParrotRawFormat.cc)
target_compile_definitions(KarriRequestsToParrotRawFormat PRIVATE CSV_IO_NO_THREAD)
set_target_properties(KarriRequestsToParrotRawFormat PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(KarriRequestsToParrotRawFormat PRIVATE kassert fast_cpp_csv_parser)
target_include_directories(KarriRequestsToParrotRawFormat PUBLIC ${PARROT_INCLUDE_DIR})

# Executable for P2P Algorithms
add_executable(RunP2PAlgo RunP2PAlgo.cc)
target_compile_definitions(RunP2PAlgo PRIVATE CSV_IO_NO_THREAD)
set_target_properties(RunP2PAlgo PROPERTIES KASSERT_ASSERTION_LEVEL ${KASSERT_ASSERTION_LEVEL})
target_link_libraries(RunP2PAlgo routingkit kassert fast_cpp_csv_parser)
target_include_directories(RunP2PAlgo PUBLIC ${PARROT_INCLUDE_DIR})
if(OpenMP_FOUND)
  target_link_libraries(RunP2PAlgo OpenMP::OpenMP_CXX)
endif()