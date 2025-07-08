#!/bin/bash

./run_excel_data_reset.sh

# シード値の範囲を設定(100回)
start_seed=40
end_seed=141

for sn in $(seq 2 2 18); do

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

    echo "Setting S_N = $sn"

    # header.h の中の S_N 定義を変更
    sed -i "s/^#define S_N .*/#define S_N $sn/" header.h

    # 再コンパイル
    #gcc shell_main_GIF_Ver7_4_1.c module.c -o my_program -lm
    gcc shell_main_GIF_Ver7_3.c module.c -o my_program -lm


    # シードを変えながら実行
    for seed in $(seq $start_seed $end_seed); do
        echo "Running with S_N = $sn and seed = $seed"
        ./my_program "$seed"
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
    gcc E_MedInf_average_shell.c
    ./a.out
    gcc E_Med_re_average_shell.c
    ./a.out
    gcc E_Med_re_collect_to_delivery_average_shell.c
    ./a.out
    gcc E_Medinf_collect_average_shell.c
    ./a.out
    gcc E_Drone_deliveryProbability_shell.c
    ./a.out
    gcc E_Prob_Medinf_drone_shell.c
    ./a.out
    gcc ETC_histgram.c
    ./a.out

done



