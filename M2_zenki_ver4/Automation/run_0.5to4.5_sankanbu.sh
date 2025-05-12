#!/bin/bash

# シード値の範囲を設定
start_seed=41
end_seed=90
#end_seed=140

# circle_radiusの範囲を設定
start_radius=0.5 #開始半径
end_radius=4.5 #終了半径
step_radius=4.0

# シード値を変更しながら実行
for radius in $(seq $start_radius $step_radius $end_radius)
do
  echo "Running with circle_radius $radius"

  # 特定ディレクトリ内の「Etd_data.txt」ファイルを初期化
  > drone_datafile/txtfile/Etd_data.txt
  > drone_datafile/txtfile/flight_ave.txt
  > drone_datafile/txtfile/TV_chargeCount.txt
  > drone_datafile/txtfile/TV_chargeAmount.txt
  
  for seed in $(seq $start_seed $end_seed)
  do
    echo "Running with seed $seed and circle_radius $radius"

    # Pythonスクリプトを実行（シード値とcircle_radiusを渡す）
    python3 shell_0.5to4.5_sankanbu.py $seed $radius

    # Cプログラムをコンパイル
    #gcc main.c module.c -lm #gif画像も表示する場合
    gcc main_noGIF.c module.c -lm #gif画像表示しない場合


    # Cプログラムを実行
    ./a.out

    echo "Completed run with seed $seed and circle_radius $radius"
  done

  # ループで求めたEtdの平均値を「Shell_Etd_average.txt」に書き込むプログラムを実行
  echo "Calculating average for circle_radius $radius"

  # Cプログラムをコンパイル
  gcc Etd_average_shell.c
  ./a.out

  gcc E_flight_time_average_shell.c
  # Cプログラムを実行
  ./a.out

  gcc E_TV_chargeCount_ave_shell.c
  # Cプログラムを実行
  ./a.out

  gcc E_TV_chargeAmount_ave_shell.c
  # Cプログラムを実行
  ./a.out

  echo "Completed average calculation for circle_radius $radius"
done
