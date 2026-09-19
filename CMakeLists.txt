cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(final_radar_system)

target_sources(app PRIVATE
    src/main.c
    src/gui_radar.c
    src/algo_object_detect.c
)
# 注：font_chip.c 字库驱动保留在 src/ 但暂不编入——排查花屏期间停用字库，
# 屏上只画雷达图。恢复时把 src/font_chip.c 加回 target_sources 即可。
