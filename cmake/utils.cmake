# cmake/utils.cmake

# ====================================================================
#  ==== 自动复制运行时依赖 (DLLs) 的辅助函数 ====
# ====================================================================
#
# 这个函数为目标程序添加一个“构建后”步骤，
# 它会把依赖库的 DLL 文件复制到可执行文件所在的目录。
#
# @param target_name       你的可执行目标
# @param dependency_target 要复制其 DLL 的依赖库目标
#

function(add_runtime_dependency_copy_step target_name dependency_target)
    if(NOT TARGET ${dependency_target})
        message(WARNING "Cannot copy runtime for '${dependency_target}' because it is not a valid target.")
        return()
    endif()

    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        add_custom_command(
            TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:${dependency_target}>"
                "$<TARGET_FILE_DIR:${target_name}>"
            COMMENT "Copying runtime dependency ${dependency_target} for ${target_name}"
        )
    endif()
endfunction()