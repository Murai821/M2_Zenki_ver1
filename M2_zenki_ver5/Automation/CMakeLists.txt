cmake_minimum_required(VERSION 3.10)
project(my_program)

# C標準の設定
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# ソースファイルを指定
set(SOURCES main.c module.c)

# 実行ファイルを生成
add_executable(my_program ${SOURCES})

# 数学ライブラリをリンク
target_link_libraries(my_program m)
