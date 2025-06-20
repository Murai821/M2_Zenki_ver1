#!/bin/bash

# シード値の範囲を設定(100回)
start_seed=40
end_seed=141

#gcc shell_main_GIF_ver2.c module.c -o my_program -lm
#gcc shell_main_GIF_Ddelivery_DonlyVer5.c module.c -o my_program -lm
#gcc shell_main_GIF_Ddelivery_DonlyVer5_nocharge.c module.c -o my_program -lm
#gcc shell_main_GIF_Ver5_multiD.c module.c -o my_program -lm #ドローン複数（TVと逆周り）
#gcc shell_main_GIF_Ver5_multiD_gyakumawari.c module.c -o my_program -lm #ドローン複数（TVと同じ周り）
#gcc shell_main_GIF_Ver7_3.c module.c -o my_program -lm
gcc shell_main_GIF_Ver7_4_1.c module.c -o my_program -lm

# 特定ディレクトリ内のファイルを初期化
> drone_datafile/txtfile/Mean_Medinf_delay.txt
> drone_datafile/txtfile/Mean_Med_re_delay.txt
> drone_datafile/txtfile/Mean_Medinf_collect_delay.txt
> drone_datafile/txtfile/Mean_Med_re_collect_to_delivery_delay.txt
> drone_datafile/txtfile/Drone_deliveryProbability.txt
> drone_datafile/txtfile/Prob_Medinf_drone.txt

> drone_datafile/txtfile/Etd_data.txt
> drone_datafile/txtfile/Eti_data.txt
> drone_datafile/txtfile/Etg_data.txt

> drone_datafile/txtfile/Mean_ETC_dro.txt
> drone_datafile/txtfile/Mean_ETC_Vehicle.txt
> drone_datafile/txtfile/Mean_ETC_to_Vehicle.txt
> drone_datafile/txtfile/drone_trip_ave_data.txt
> drone_datafile/txtfile/Mean_Jyunkai_time_InfCdrone.txt
> drone_datafile/txtfile/Etc_histgram_data.txt

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
gcc Etc_dro_average_shell.c
./a.out
gcc Etc_Vehicle_average_shell.c
./a.out
gcc Etc_to_Vehicle_average_shell.c
./a.out
gcc E_InfCdrone_TripTime_shell.c
./a.out
gcc E_Jyunkai_time_InfCdrone_shell.c
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

# Cプログラムをコンパイル
gcc E_Drone_deliveryProbability_shell.c
./a.out
gcc E_Prob_Medinf_drone_shell.c
./a.out

# E(TC)のヒストグラムを作成
gcc ETC_histgram.c
./a.out
