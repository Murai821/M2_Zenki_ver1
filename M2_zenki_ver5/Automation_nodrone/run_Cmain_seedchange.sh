#!/bin/bash

# シード値の範囲を設定(100回)
start_seed=41
end_seed=140

gcc shell_main_GIF.c module.c -o my_program -lm

# 特定ディレクトリ内のファイルを初期化
> drone_datafile/txtfile/Mean_Medinf_delay.txt
> drone_datafile/txtfile/Mean_Med_re_delay.txt
> drone_datafile/txtfile/Mean_Medinf_collect_delay.txt
> drone_datafile/txtfile/Mean_Med_re_collect_to_delivery_delay.txt

> drone_datafile/txtfile/Etc_histgram_data.txt

> drone_datafile/txtfile/Etd_data.txt
> drone_datafile/txtfile/Eti_data.txt
> drone_datafile/txtfile/Etg_data.txt

for i in $(seq $start_seed $end_seed)
do
    echo "Running with seed $i"
    ./my_program "$i"
done

gcc Etd_average_shell.c
./a.out
gcc Eti_average_shell.c
./a.out
gcc Etg_average_shell.c
./a.out

# Cプログラムをコンパイル
gcc E_MedInf_average_shell.c
./a.out

# Cプログラムをコンパイル
gcc E_Med_re_average_shell.c
./a.out

# Cプログラムをコンパイル
gcc E_Med_re_collect_to_delivery_average_shell.c
./a.out

# Cプログラムをコンパイル
gcc E_Medinf_collect_average_shell.c
./a.out

# E(TC)のヒストグラムを作成
gcc ETC_histgram.c
./a.out
