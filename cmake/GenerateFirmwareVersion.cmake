find_package(Git QUIET)

set(commit "unknown")
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
        WORKING_DIRECTORY "${SOURCE_DIR}"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE commit
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT result EQUAL 0)
        set(commit "unknown")
    else()
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
            WORKING_DIRECTORY "${SOURCE_DIR}"
            OUTPUT_VARIABLE dirty
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(NOT dirty STREQUAL "")
            string(APPEND commit "-dirty")
        endif()
    endif()
endif()

get_filename_component(output_dir "${OUTPUT_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${output_dir}")
set(temporary "${OUTPUT_FILE}.tmp")
file(WRITE "${temporary}" "#pragma once\n\n#define VB_FIRMWARE_COMMIT \"${commit}\"\n")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${temporary}" "${OUTPUT_FILE}")
file(REMOVE "${temporary}")
