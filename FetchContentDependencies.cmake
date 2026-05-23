# Specify all dependencies fetched at configure time using FetchContent
include(FetchContent)

# Declare proj dependency
FetchContent_Declare(
        proj
        URL https://download.osgeo.org/proj/proj-9.5.0.tar.gz
        URL_MD5 ac46b4e31562890d012ea6b31e579cf6
)

# Fetch proj
message("Fetching proj library...")
set(BUILD_APPS OFF)
set(BUILD_TESTING OFF)
set(ENABLE_CURL OFF)
set(ENABLE_TIFF OFF)
set(TESTING_USE_NETWORK OFF)
FetchContent_MakeAvailable(proj)