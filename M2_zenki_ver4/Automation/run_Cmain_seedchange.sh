#!/bin/bash

# シード値の範囲を設定(100回)
start_seed=41
end_seed=45

#gcc shell_main_GIF_ver2.c module.c -o my_program -lm
gcc shell_main_GIF_Ddelivery_DonlyVer3.c module.c -o my_program -lm

# 特定ディレクトリ内のファイルを初期化
> drone_datafile/txtfile/Mean_Medinf_delay.txt
> drone_datafile/txtfile/Mean_Med_re_delay.txt
> drone_datafile/txtfile/Mean_Medinf_collect_delay.txt
> drone_datafile/txtfile/Mean_Med_re_collect_to_delivery_delay.txt

for i in $(seq $start_seed $end_seed)
do
    echo "Running with seed $i"
    ./my_program "$i"
done

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