
set(vscode_dir ${CMAKE_SOURCE_DIR}/.vscode)
set(vscode_integ ${CMAKE_SOURCE_DIR}/_vscode)

file(MAKE_DIRECTORY ${vscode_dir})

foreach(file "settings.json" "tasks.json" "launch.json")
    if (NOT EXISTS ${vscode_dir}/${file})
        file(COPY ${vscode_integ}/${file} DESTINATION ${vscode_dir})
    endif()
endforeach()
