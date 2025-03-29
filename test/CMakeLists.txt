cmake_minimum_required(VERSION 3.14)

project(weather_routing_pi_tests)
set(CMAKE_CXX_STANDARD 17)
message(STATUS "Building weather routing plugin tests")

enable_testing ()

# Add the GoogleTest library subdirectory
# As per https://google.github.io/googletest/quickstart-cmake.html
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Find required packages
find_package(GTest REQUIRED)
find_package(wxWidgets COMPONENTS 
core 
base 
net
xml
html
adv
aui
REQUIRED)

set(TEST_SRC
    Polar_tests.cpp
    PolygonRegion_tests.cpp
    Utilities_tests.cpp
    ${CMAKE_SOURCE_DIR}/src/Utilities.cpp
    ${CMAKE_SOURCE_DIR}/src/PolygonRegion.cpp
    ${CMAKE_SOURCE_DIR}/src/Polar.cpp
    ${CMAKE_SOURCE_DIR}/src/zuFile.cpp
   )
   
   add_executable(weather_routing_pi_tests ${TEST_SRC})

   # Uncomment include directories as needed - @todo tests should inherit this from main plugin
   target_include_directories(weather_routing_pi_tests 
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src
   #         ${CMAKE_SOURCE_DIR}/opencpn-libs/api-18/include
            ${CMAKE_SOURCE_DIR}/opencpn-libs/libtess2/include
            ${CMAKE_SOURCE_DIR}/opencpn-libs/tinyxml/include
   #         ${CMAKE_SOURCE_DIR}/opencpn-libs/plugin_dc/dc_utils/include
   #         ${CMAKE_SOURCE_DIR}/opencpn-libs/jsonlib/include
   #         ${CMAKE_SOURCE_DIR}/opencpn-libs/odapi
   #         ${CMAKE_SOURCE_DIR}/opencpn-libs/pugixml
            ${GTEST_INCLUDE_DIRS}
            ${wxWidgets_INCLUDE_DIRS}
    )
   
   # Uncomment required libraries as needed - @todo tests should inherit this from main plugin
   target_link_libraries(weather_routing_pi_tests 
       PRIVATE 
           GTest::gtest_main
           ${wxWidgets_LIBRARIES}
           ocpn::libtess2
           ocpn::tinyxml
           # ocpn::dc-utils
           # ocpn::jsonlib
           # ocpn::api_wx32
           # ocpn::iso8211
           # ocpn::json-schema-validator
           # ocpn::wxsvg
           ocpn::bzip2
           ${ZLIB_LIBRARIES}
   )
   
   # Workaround as per https://discourse.cmake.org/t/how-to-get-an-lc-rpath-and-rpath-prefix-on-a-dylib-on-macos/5540/5
   # TODO: Quinton: Make this robust on non-Mac platforms.
   set_target_properties(weather_routing_pi_tests PROPERTIES
       BUILD_RPATH "../lib"
       INSTALL_RPATH "../lib"    
       )
   
# Add the test
add_test(NAME weather_routing_pi_tests COMMAND weather_routing_pi_tests)

target_compile_definitions(weather_routing_pi_tests
    PUBLIC
        USE_MOCK_DEFS CMAKE_BINARY_DIR="${CMAKE_BINARY_DIR}"
        TESTDATADIR="${CMAKE_CURRENT_LIST_DIR}/testdata"
)

include(GoogleTest)
gtest_discover_tests(weather_routing_pi_tests)