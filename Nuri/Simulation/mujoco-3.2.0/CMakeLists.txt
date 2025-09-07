cmake_minimum_required(VERSION 3.16)
project(mujoco-3.2.0 LANGUAGES C CXX OBJCXX)

# ---- 공통 컴파일 설정 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall)

# ---- 경로들 ----
set(MUJOCO_PATH "/Users/shinseungmin/Documents/Github/Nuri/Simulation/mujoco-3.2.0")
set(SIMULATE_PATH "${MUJOCO_PATH}/simulate")
# Homebrew로 설치한 pinocchio를 쓴다면 보통 아래 경로에서 자동 탐지됨
# 필요 시 명시:
#set(pinocchio_DIR "/opt/homebrew/opt/pinocchio/lib/cmake/pinocchio")

# ---- 패키지 찾기 ----
find_package(Curses REQUIRED)
find_package(glfw3 REQUIRED)       # IMPORTED target: glfw
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# ---- MuJoCo 찾기 (macOS: framework 우선, 아니면 dylib) ----
set(MUJOCO_LINK "")   # 최종적으로 여기에 링크 토큰을 넣음
if(APPLE)
  # .dmg에서 설치한 MuJoCo.app / 프레임워크 위치들
  list(APPEND CMAKE_FRAMEWORK_PATH
    "${MUJOCO_PATH}"
    "${MUJOCO_PATH}/MuJoCo.app/Contents/Frameworks"
    "$ENV{HOME}/Library/Frameworks"
    "/Library/Frameworks"
  )
  find_library(MUJOCO_FRAMEWORK MuJoCo)
  if(MUJOCO_FRAMEWORK)
    # 프레임워크로 링크
    set(MUJOCO_LINK "-framework MuJoCo")
  else()
    # 프레임워크를 못 찾으면 dylib 시도
    find_library(MUJOCO_DYLIB
      NAMES mujoco libmujoco.dylib
      PATHS "${MUJOCO_PATH}/lib" "/opt/homebrew/lib" "/usr/local/lib"
    )
    if(NOT MUJOCO_DYLIB)
      message(FATAL_ERROR "MuJoCo framework/dylib을 찾지 못했습니다. MUJOCO_PATH를 확인하세요.")
    endif()
    set(MUJOCO_LINK "${MUJOCO_DYLIB}")
  endif()
else()
  # 리눅스 등의 경우 .so
  find_library(MUJOCO_SO
    NAMES mujoco libmujoco.so
    PATHS "${MUJOCO_PATH}/lib" "/usr/local/lib" "/usr/lib"
    REQUIRED
  )
  set(MUJOCO_LINK "${MUJOCO_SO}")
endif()

# ---- lodepng ----
add_library(lodepng STATIC
  ${SIMULATE_PATH}/lodepng.cpp
  ${SIMULATE_PATH}/lodepng.h
)

# ---- libsimulate (먼저 "정의"하고, 그 다음 "링크") ----
add_library(libsimulate STATIC
  ${SIMULATE_PATH}/simulate.cc
  ${SIMULATE_PATH}/glfw_adapter.cc
  ${SIMULATE_PATH}/platform_ui_adapter.cc
  ${SIMULATE_PATH}/glfw_dispatch.cc

  ${SIMULATE_PATH}/simulate.h
  ${SIMULATE_PATH}/array_safety.h
  ${SIMULATE_PATH}/glfw_adapter.h
  ${SIMULATE_PATH}/platform_ui_adapter.h
  ${SIMULATE_PATH}/glfw_dispatch.h
)

target_include_directories(libsimulate PUBLIC
  ${SIMULATE_PATH}
  ${MUJOCO_PATH}/include
  ${MUJOCO_PATH}/simulate
  ${MUJOCO_PATH}/sample
)

# 필요 시 유지 (정의되어 있을 경우에만 적용되도록)
if(DEFINED MUJOCO_SIMULATE_COMPILE_OPTIONS)
  target_compile_options(libsimulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
endif()
if(DEFINED MUJOCO_SIMULATE_LINK_OPTIONS)
  target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
endif()

# 이제 링크 (타깃 정의 후!)
target_link_libraries(libsimulate PUBLIC
  ${MUJOCO_LINK}
  glfw
  lodepng
  Threads::Threads
  ${CURSES_LIBRARY}
)

# ---- control 라이브러리 ----
add_library(control STATIC
  ${MUJOCO_PATH}/include/Controller.cpp
)
target_link_libraries(control PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  ${MUJOCO_LINK}
)

# ---- 실행파일 ----
add_executable(${PROJECT_NAME}
  ${SIMULATE_PATH}/main.cc

  # 실행파일에도 .mm가 필요하다면 유지 (중복 컴파일 방지를 위해 보통 libsimulate로 묶는 걸 권장)
  # ${SIMULATE_PATH}/glfw_corevideo.mm
  # ${SIMULATE_PATH}/macos_gui.mm
)

# 실행파일 링크
target_link_libraries(${PROJECT_NAME} PRIVATE
  libsimulate
  control
  Eigen3::Eigen
  pinocchio::pinocchio
  glfw
  Threads::Threads
  ${CURSES_LIBRARY}
  ${MUJOCO_LINK}
)

# macOS에서 프레임워크 런타임 탐색이 필요할 수 있으니 RPATH 조정(선택)
if(APPLE AND MUJOCO_FRAMEWORK)
  # 실행 시 프레임워크를 찾기 쉽게 @executable_path 기준 RPATH 추가
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "@executable_path/../Frameworks;@loader_path"
    INSTALL_RPATH "@executable_path/../Frameworks;@loader_path"
  )
endif()