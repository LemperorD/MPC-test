cmake_minimum_required(VERSION 3.14)
project(CartPoleMPC CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen3
find_package(Eigen3 REQUIRED)

# 禁用 Eigen 未使用变量警告
add_compile_options(-Wno-unused-variable -Wno-unused-but-set-variable)

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
# PID 倒立摆演示可执行文件
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
