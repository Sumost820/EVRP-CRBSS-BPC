cmake_minimum_required(VERSION 3.20)
project(EVRP-CRBSS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

if(NOT MSVC)
    message(FATAL_ERROR "Select the Visual Studio toolchain in CLion; the supplied Gurobi libraries require MSVC.")
endif()

set(GUROBI_HOME "D:/Gurobi/win64" CACHE PATH "Gurobi installation directory")
find_path(gurobiInclude gurobi_c++.h PATHS "${GUROBI_HOME}/include" REQUIRED NO_DEFAULT_PATH)
find_library(gurobiCore NAMES gurobi130 PATHS "${GUROBI_HOME}/lib" REQUIRED NO_DEFAULT_PATH)
find_library(gurobiRelease NAMES gurobi_c++md2017 PATHS "${GUROBI_HOME}/lib" REQUIRED NO_DEFAULT_PATH)
find_library(gurobiDebug NAMES gurobi_c++mdd2017 PATHS "${GUROBI_HOME}/lib" REQUIRED NO_DEFAULT_PATH)

# CVRPSEP is kept separate from the project code and retains its original license.
file(GLOB cvrpsepSources CONFIGURE_DEPENDS "third_party/cvrpsep/*.cpp" "third_party/cvrpsep/*.CPP")
list(REMOVE_DUPLICATES cvrpsepSources)
add_library(cvrpsep STATIC ${cvrpsepSources})
target_include_directories(cvrpsep PUBLIC third_party/cvrpsep)
target_compile_definitions(cvrpsep PRIVATE _CRT_SECURE_NO_WARNINGS)

add_library(evrpCore STATIC
    src/instance.cpp
    src/energy.cpp
    src/labels.cpp
    src/pricing.cpp
    src/master.cpp
    src/cuts.cpp
    src/schedule.cpp
    src/timeLimit.cpp
    src/solver.cpp
    src/initialRoutes.cpp
    src/columnGeneration.cpp
    src/branching.cpp
    src/report.cpp
)
target_include_directories(evrpCore PUBLIC src "${gurobiInclude}")
target_compile_options(evrpCore PUBLIC /utf-8 /EHsc)
target_compile_definitions(evrpCore PUBLIC NOMINMAX _CRT_SECURE_NO_WARNINGS)
target_link_libraries(evrpCore PUBLIC cvrpsep
    optimized "${gurobiRelease}" debug "${gurobiDebug}" "${gurobiCore}")

add_executable(EVRP-CRBSS src/main.cpp src/runner.cpp src/app.manifest)
target_link_libraries(EVRP-CRBSS PRIVATE evrpCore)
target_compile_definitions(EVRP-CRBSS PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_custom_command(TARGET EVRP-CRBSS POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${GUROBI_HOME}/bin/gurobi130.dll" "$<TARGET_FILE_DIR:EVRP-CRBSS>")

# Existing checker regression cases; ordinary batch runs only need test.py.
option(BUILD_CHECKS "Build the existing schedule regression checks" OFF)
if(BUILD_CHECKS)
    enable_testing()
    add_executable(checkSchedule checks/testSchedule.cpp src/app.manifest)
    target_link_libraries(checkSchedule PRIVATE evrpCore)
    add_custom_command(TARGET checkSchedule POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GUROBI_HOME}/bin/gurobi130.dll" "$<TARGET_FILE_DIR:checkSchedule>")
    add_test(NAME schedule COMMAND checkSchedule "${CMAKE_CURRENT_SOURCE_DIR}/checks/schedule_seed.txt")
endif()
