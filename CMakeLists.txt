cmake_minimum_required(VERSION 3.20)
project(FusionForce LANGUAGES CXX)

find_package(nlohmann_json 3.11 REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(SQLite3 REQUIRED)
find_package(ZLIB REQUIRED)

add_library(fusionforce
    src/cloud_store.cpp
    src/placement.cpp
    src/projects.cpp
    src/rigid_body.cpp
    src/route_destination.cpp
    src/settling.cpp
    src/simulation.cpp
    src/simulation_csv.cpp
    src/stability.cpp
    src/terrain_patch.cpp
    src/truck_geometry.cpp
    src/truck_step.cpp
    src/viewer_inputs.cpp
    src/viewer_results.cpp
    src/viewer_workspace.cpp
)
target_compile_features(fusionforce PUBLIC cxx_std_20)
target_include_directories(fusionforce PUBLIC include)
target_link_libraries(fusionforce
    PUBLIC nlohmann_json::nlohmann_json
    PRIVATE OpenSSL::Crypto SQLite::SQLite3 ZLIB::ZLIB
)

option(FUSIONFORCE_BUILD_VIEWER "Build the loopback web server" ON)
if(FUSIONFORCE_BUILD_VIEWER)
    find_path(CPP_HTTPLIB_INCLUDE_DIR NAMES httplib.h REQUIRED)
    find_library(CPP_HTTPLIB_LIBRARY NAMES cpp-httplib REQUIRED)
    find_package(Threads REQUIRED)
    add_executable(fusionforce-viewer src/viewer_main.cpp)
    target_include_directories(fusionforce-viewer PRIVATE "${CPP_HTTPLIB_INCLUDE_DIR}")
    target_compile_definitions(fusionforce-viewer PRIVATE
        CPPHTTPLIB_OPENSSL_SUPPORT CPPHTTPLIB_ZLIB_SUPPORT CPPHTTPLIB_BROTLI_SUPPORT
        FUSIONFORCE_VIEWER_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/viewer"
    )
    target_link_libraries(fusionforce-viewer PRIVATE fusionforce "${CPP_HTTPLIB_LIBRARY}" Threads::Threads)
endif()

include(CTest)
if(BUILD_TESTING)
    foreach(component IN ITEMS cloud_store placement projects rigid_body route_destination settling simulation stability truck_geometry truck_step viewer_inputs viewer_results)
        add_executable(test_${component} tests/${component}.cpp)
        target_link_libraries(test_${component} PRIVATE fusionforce)
        add_test(NAME ${component} COMMAND test_${component})
    endforeach()
    set_tests_properties(settling truck_geometry PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    if(FUSIONFORCE_BUILD_VIEWER AND UNIX)
        find_program(CURL_PROGRAM curl)
        find_program(JQ_PROGRAM jq)
        if(CURL_PROGRAM AND JQ_PROGRAM)
            add_test(NAME viewer_http COMMAND sh "${CMAKE_CURRENT_SOURCE_DIR}/tests/viewer_http.sh"
                "$<TARGET_FILE:fusionforce-viewer>" "${CMAKE_CURRENT_SOURCE_DIR}/viewer" "${CURL_PROGRAM}" "${JQ_PROGRAM}")
            set_tests_properties(viewer_http PROPERTIES TIMEOUT 45)
        endif()
    endif()
endif()
