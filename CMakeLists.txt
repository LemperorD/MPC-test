cmake_minimum_required(VERSION 3.14)
project(CartPoleMPC CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 禁用 Eigen 未使用变量警告
add_compile_options(-Wno-unused-variable -Wno-unused-but-set-variable)

# ── 依赖 ──
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Pangolin REQUIRED)

# ──────────────────────────────────────────
# 核心库 — 倒立摆动力学
# ──────────────────────────────────────────
add_library(cart_pole STATIC
  src/cart_pole.cpp
)
target_include_directories(cart_pole PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ──────────────────────────────────────────
# PID 控制器库
# ──────────────────────────────────────────
add_library(pid_controller STATIC
  src/pid_controller.cpp
)
target_include_directories(pid_controller PUBLIC include)

# ──────────────────────────────────────────
# YAML 配置加载库
# ──────────────────────────────────────────
add_library(config_loader STATIC
  src/config_loader.cpp
)
target_include_directories(config_loader PUBLIC
  include
)
target_link_libraries(config_loader PUBLIC
  yaml-cpp
)

# ──────────────────────────────────────────
# 终端 ASCII PID 演示 (保留, 向后兼容)
# ──────────────────────────────────────────
add_executable(pid_cart_pole_demo
  src/pid_cart_pole_demo.cpp
)
target_link_libraries(pid_cart_pole_demo
  cart_pole
  pid_controller
)
target_include_directories(pid_cart_pole_demo PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)
set_target_properties(pid_cart_pole_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin
)

# ──────────────────────────────────────────
# Pangolin 可视化 PID 演示 (Cascaded PID)
# ──────────────────────────────────────────
add_executable(pangolin_cart_pole_demo
  src/pangolin_cart_pole_demo.cpp
)
target_link_libraries(pangolin_cart_pole_demo
  cart_pole
  pid_controller
  config_loader
  ${Pangolin_LIBRARIES}
)
target_include_directories(pangolin_cart_pole_demo PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)
set_target_properties(pangolin_cart_pole_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin
)