#!/bin/bash

# シード値の範囲を設定
start_seed=41
end_seed=90
#end_seed=140

# Nmの範囲を設定
start_Nm=5 #開始山間部避難所数
end_Nm=30 #終了山間部避難所数
step_Nm=5

# シード値を変更しながら実行
for Nm in $(seq $start_Nm $step_Nm $end_Nm)
do
  echo "Running with Nm $Nm"

  # 特定ディレクトリ内の「Etd_data.txt」ファイルを初期化
  > drone_datafile/txtfile/Etd_data.txt
  > drone_datafile/txtfile/flight_ave.txt
  > drone_datafile/txtfile/TV_chargeCount.txt
  > drone_datafile/txtfile/TV_chargeAmount.txt

  for seed in $(seq $start_seed $end_seed)
  do
    echo "Running with seed $seed and Nm $Nm"

    # Pythonスクリプトを実行（シード値とNmを渡す）
    python3 shell_Nm_5to25.py $seed $Nm

    # Cプログラムをコンパイル
    #gcc main.c module.c -lm #gif画像も表示する場合
    gcc main_noGIF.c module.c -lm #gif画像表示しない場合


    # Cプログラムを実行
    ./a.out

    echo "Completed run with seed $seed and Nm $Nm"
  done

  # ループで求めたEtdの平均値を「Shell_Etd_average.txt」に書き込むプログラムを実行
  echo "Calculating average for Nm $Nm"

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


  echo "Completed average calculation for Nm $Nm"
done
