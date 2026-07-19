# 说明：
# 这个文件目前保留 Astra SDK 官方 C API 示例的 CMake 结构，方便后续迁移到 SDK 示例工程中编译。
# 当前树莓派第一版联调仍以 gcc 直接生成 depth_xyz_reader 为主。
# 后续如果要独立 CMake 构建，需要补充 Astra SDK include/lib 路径和运行时库路径。
set (_projname "DepthReaderPoll")

set (${_projname}_SOURCES
  main.c
  ${SAMPLE_COMMON_DIR}/key_handler.h
  )

add_executable(${_projname} ${${_projname}_SOURCES})

set_target_properties(${_projname} PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}c-api")

target_link_libraries(${_projname} ${ASTRA_LIBRARIES})

set_debug_working_dir(${_projname} "$(TargetDir)")

add_astra_sdk_dependency(${_projname})

install_sample(${_projname})
