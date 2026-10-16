# ESP-IDF の計測アプリ(tests/mcu/mcu_test.c): 音声を符号化・復号し、1 フレームあたりのサイクル数を出す。
#   . ~/esp/idf_env.sh && idf.py set-target esp32 build flash monitor
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mrl_test)
