# 主机侧单元测试（PC 上跑，不需要硬件、不需要 ARM 工具链）
#
# 只编译「纯逻辑」的源文件：
#   - RobotComponents/component/module_state.hpp       （纯类型 + 数学）
#   - RobotComponents/component/gimbal_chassis_comm.cpp（双板协议编解码）
#   - Chassis/inc/pid.hpp + Chassis/src/pid.cpp        （PID）
#   - Chassis/RobotModules/component/chassis_ik_solver.hpp（麦轮逆解）
#   - Chassis/inc/imu_bias.hpp                         （零漂估计）
#
# 用法：在 omni_skeleton 目录下，逐行执行（PowerShell 和 cmd 通用，不含续行符）：
#
#   cmake -S host_tests -B host_tests/build -G Ninja -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe
#   cmake --build host_tests/build
#   host_tests/build/host_tests.exe
#
# PowerShell 里最后一行也可以写成 .\host_tests\build\host_tests.exe
# 注意：不要用 ^ 续行（那是 cmd 的写法），PowerShell 的续行符是反引号 ` ，直接分行更省事。
#
cmake_minimum_required(VERSION 3.16)
project(omni_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OMNI_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(host_tests
  main.cpp
  stub_tick.cpp
  ${OMNI_ROOT}/RobotComponents/component/gimbal_chassis_comm.cpp
  ${OMNI_ROOT}/Chassis/src/pid.cpp
)

target_include_directories(host_tests PRIVATE
  ${OMNI_ROOT}/RobotComponents/component
  ${OMNI_ROOT}/Chassis/inc
  ${OMNI_ROOT}/Chassis/RobotModules/component
  ${OMNI_ROOT}/Gimbal/RobotModules/component
)
