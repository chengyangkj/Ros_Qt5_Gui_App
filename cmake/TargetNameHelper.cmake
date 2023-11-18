#根据当前的文件夹相对路径 生成target name:(父级文件夹_当前文件夹)
macro(GetTargetName)
    file(RELATIVE_PATH CURRENT_RELATIVE_PATH  "${PROJECT_ROOT}" "${CMAKE_CURRENT_LIST_DIR}")
    string(REPLACE "/" "_" DIRS_UNDERSCORE ${CURRENT_RELATIVE_PATH})
    set(TARGET_NAME ${DIRS_UNDERSCORE}) 
endmacro()


