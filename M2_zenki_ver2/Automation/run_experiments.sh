#!/bin/bash

# シード値の範囲を設定
start_seed=41
end_seed=90

# 特定ディレクトリ内の「Etd_data.txt」ファイルを初期化
> drone_datafile/txtfile/Etd_data.txt
> drone_datafile/txtfile/flight_ave.txt
> drone_datafile/txtfile/TV_chargeCount.txt
> drone_datafile/txtfile/TV_chargeAmount.txt

# シード値を変更しながら実行
for seed in $(seq $start_seed $end_seed)
do
  echo "Running with seed $seed"

  # Pythonスクリプトを実行（シード値を渡す）
  python3 shell_Network_solveTSP_sankanbu.py $seed

  # Cプログラムをコンパイル
  #gcc main.c module.c -lm #gif画像も表示する場合
  gcc main_noGIF.c module.c -lm #gif画像表示しない場合

  # Cプログラムを実行
  ./a.out

  echo "Completed run with seed $seed"
done

# ループで求めたEtdの平均値を「Shell_Etd_average.txt」に書き込むプログラムを実行
# Cプログラムをコンパイル
gcc Etd_average_shell.c

# Cプログラムを実行
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