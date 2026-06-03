include(GNUInstallDirs)

# libslvs
add_library(slvs-interface INTERFACE)
target_sources(slvs-interface INTERFACE lib.cpp)
target_compile_definitions(slvs-interface INTERFACE -DLIBRARY)
target_include_directories(slvs-interface INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(slvs-interface INTERFACE slvs-solver mimalloc-static)

if(ENABLE_PYTHON_LIB)
    add_custom_command(
        OUTPUT cylib.c
        COMMAND cython lib.pyx -o ${CMAKE_CURRENT_BINARY_DIR}/cylib.c --include-dir ${PROJECT_SOURCE_DIR}/include --module-name solvespace
        DEPENDS lib.pyx
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    # Cross-compile opt-in. When SLVS_PY_PYTHON_INCLUDE_DIR /
    # SLVS_PY_PYTHON_LIBRARY are pre-populated by the caller's
    # toolchain file, skip find_package(Python) (which insists on a
    # runnable Interpreter matching the *target* ABI — fatal in a
    # MinGW-w64-on-Linux cross-build for a Windows-ABI module).
    # The caller pre-supplies the header + import-lib paths, and we
    # build slvs-py as a plain MODULE library.
    if(DEFINED SLVS_PY_PYTHON_INCLUDE_DIR AND DEFINED SLVS_PY_PYTHON_LIBRARY)
        add_library(slvs-py MODULE cylib.c)
        target_include_directories(slvs-py PRIVATE ${SLVS_PY_PYTHON_INCLUDE_DIR})
        target_link_libraries(slvs-py PRIVATE slvs-interface ${SLVS_PY_PYTHON_LIBRARY})
        # `.pyd` on Windows, no `lib` prefix on either platform.
        if(WIN32)
            set_target_properties(slvs-py PROPERTIES PREFIX "" SUFFIX ".pyd")
        else()
            set_target_properties(slvs-py PROPERTIES PREFIX "")
        endif()
    else()
        find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
        Python_add_library(slvs-py MODULE cylib.c)
        target_link_libraries(slvs-py PRIVATE Python::Module)
    endif()
    set_target_properties(slvs-py PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_compile_definitions(slvs-py PRIVATE -DSTATIC_LIB)
    # Stable ABI (abi3) opt-in. When SLVS_PY_LIMITED_API is set to a
    # hex Python version (e.g. 0x030B0000 for 3.11), the Cython module
    # is compiled against the CPython limited API, so a single wheel
    # runs on that minor version and every newer one. Cython
    # auto-switches to limited-API code paths once Py_LIMITED_API is
    # defined. Leave unset for a regular version-specific (cp3x) build.
    if(SLVS_PY_LIMITED_API)
        target_compile_definitions(slvs-py PRIVATE Py_LIMITED_API=${SLVS_PY_LIMITED_API})
    endif()
    target_link_libraries(slvs-py PRIVATE slvs-interface)
    set_target_properties(slvs-py PROPERTIES LIBRARY_OUTPUT_NAME solvespace)
    # Where to install the slvs-py module + its Python shim files.
    # Honour an external `SLVS_PY_DESTINATION` override first — used
    # by embedding builds that want the module at e.g.
    # `<pkg>/_slvs/` instead of the top-level. Falls back to the
    # original standalone-build behaviour otherwise.
    if(DEFINED SLVS_PY_DESTINATION)
        install(TARGETS slvs-py
            DESTINATION ${SLVS_PY_DESTINATION}
            COMPONENT   python
        )
        install(
            DIRECTORY ${PROJECT_SOURCE_DIR}/python/slvs/
            DESTINATION ${SLVS_PY_DESTINATION}
            COMPONENT   python
        )
    elseif(SKBUILD)
        install(TARGETS slvs-py
            DESTINATION ${SKBUILD_PROJECT_NAME}
            COMPONENT   python
        )
    else()
        install(TARGETS slvs-py
            DESTINATION "slvs"
            COMPONENT   python
        )
        install(
            DIRECTORY ${PROJECT_SOURCE_DIR}/python/slvs/
            DESTINATION "slvs"
            COMPONENT   python
        )
    endif()
endif()

if(EMSCRIPTEN)
    add_executable(slvs-wasm jslib.cpp)
    target_compile_features(slvs-wasm PRIVATE cxx_std_17)
    target_link_libraries(slvs-wasm PRIVATE slvs-interface embind)
    target_link_options(slvs-wasm PRIVATE
        "SHELL:-s MODULARIZE=1"
        "SHELL:-s EXPORT_NAME=solvespace"
        "SHELL:-s SINGLE_FILE=1"
        "SHELL:-s INITIAL_MEMORY=512MB"
        "SHELL:-s ALLOW_MEMORY_GROWTH"
        -O3
        -flto
        --closure 1)
    set_target_properties(slvs-wasm PROPERTIES OUTPUT_NAME "slvs" SUFFIX ".js")
endif()

add_library(slvs SHARED)
target_compile_definitions(slvs PRIVATE -DEXPORT_DLL)
target_link_libraries(slvs PRIVATE slvs-interface)
set_target_properties(slvs PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(slvs PUBLIC ${PROJECT_SOURCE_DIR}/include)

set_target_properties(slvs PROPERTIES
    PUBLIC_HEADER ${PROJECT_SOURCE_DIR}/include/slvs.h
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

# if(NOT WIN32)
install(TARGETS slvs
    COMPONENT     slvs
    LIBRARY       DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE       DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# endif()
