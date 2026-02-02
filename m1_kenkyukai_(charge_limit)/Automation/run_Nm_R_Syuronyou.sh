#!/bin/bash
#r=0.5~4.5, Nm =5~30まで変更し、異なるシード値に対するシミュレーション結果の平均値をファイルに格納するシェルスクリプト

# シード値の範囲を設定
start_seed=41
end_seed=90
#end_seed=140

# Nmの範囲を設定
start_Nm=5 #開始山間部避難所数
end_Nm=30 #終了山間部避難所数
step_Nm=25

# circle_radiusの範囲を設定
start_r=0.5 #開始circle_radius値
end_r=4.5   #終了circle_radius値
step_r=4.0  #circle_radiusのステップサイズ

# circle_radiusを変更しながら実行
for r in $(seq $start_r $step_r $end_r)
do
  echo "Running with circle_radius $r"
  
  # shell_and_Nm5to25_ND1to8.py内のcircle_radiusを書き換える
  #sed -i "s/circle_radius = [0-9]*\.*[0-9]*/circle_radius = $r/" shell_and_Nm5to25_ND1to8.py
  
  # Nmを変更しながら実行
  for Nm in $(seq $start_Nm $step_Nm $end_Nm)
do
  echo "Running with Nm $Nm"

  # 特定ディレクトリ内の「Etd_data.txt」ファイルを初期化
  > drone_datafile/txtfile/Etd_data.txt
  > drone_datafile/txtfile/Etg_data.txt
  > drone_datafile/txtfile/Eti_data.txt
  > drone_datafile/txtfile/flight_ave.txt
  > drone_datafile/txtfile/TV_chargeCount.txt
  > drone_datafile/txtfile/TV_chargeAmount.txt

  for seed in $(seq $start_seed $end_seed)
  do
    echo "Running with seed $seed and Nm $Nm"

    # Pythonスクリプトを実行（シード値とNmを渡す）
    python3 shell_Nm_R_Syuronyou.py $seed $Nm $r

    # Cプログラムをコンパイル

    gcc Syuronyou_main_noGIF.c Syuronyou_module.c -lm


    # Cプログラムを実行
    ./a.out

    echo "Completed run with seed $seed and Nm $Nm"
  done

  # ループで求めたEtdの平均値を「Shell_Etd_average.txt」に書き込むプログラムを実行
  echo "Calculating average for Nm $Nm"

  # Cプログラムをコンパイル
  gcc Etd_average_shell.c
  ./a.out
  gcc Etg_average_shell.c
  ./a.out
  gcc Eti_average_shell.c
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
  
  echo "Completed all runs for circle_radius $r"
done

echo "All experiments completed"
