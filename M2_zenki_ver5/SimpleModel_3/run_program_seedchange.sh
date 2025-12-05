#!/bin/bash

# ドローン台数NDを0から8まで1ずつ増やしながらプログラムを実行

echo "=== 一括実行スクリプト ==="
echo "ドローン台数0から8まで順次実行開始..."

# プログラムをコンパイル
echo "プログラムをコンパイル中..."
#gcc -o shell_simple_ver9_4 shell_simple_ver9_4.c -lm
#gcc -o shell_simple_ver9_5 shell_simple_ver9_5.c -lm
gcc -o shell_simple_ver9_5_1 shell_simple_ver9_5_1.c -lm
# プログラム変更はseed値変更のfor文の中でも変更する（プログラム名変更箇所は二箇所）

if [ $? -ne 0 ]; then
    echo "エラー: コンパイルに失敗しました"
    exit 1
fi

# Resultsディレクトリの準備
mkdir -p Results

# numerical_data.txtを初期化（ヘッダー行も削除してリセット）
> Results/numerical_data.txt
> Results/ETa.txt
> Results/ETb.txt
> Results/Rdro.txt
> Results/Rsh.txt
> Results/Tc_dro.txt
> Results/Tc_TV.txt
> Results/Ave_ETa.txt
> Results/Ave_ETb.txt
> Results/Ave_Rdro.txt
> Results/Ave_Rsh.txt
> Results/Ave_Tc_dro.txt
> Results/Ave_Tc_TV.txt
> Results/tb_shelter_avg_total.txt

echo "実行開始..."
echo

# ドローン台数0から20まで、各ドローン台数でseed値1から10まで順次実行
for nd in {0..8}
do
    echo "========================================"
    echo "ドローン台数: ${nd}台での実行開始"
    echo "========================================"

    > Results/tb_shelter_avg_total.txt

    # seed値1から10まで順次実行
    for seed in {12..63}
    do
        echo "----------------------------------------"
        echo "ドローン台数: ${nd}台, Seed値: ${seed}"
        echo "----------------------------------------"
        
        # プログラム実行（標準出力を抑制し、エラーのみ表示）
        #./shell_simple_ver9_4 ${nd} ${seed} > /dev/null 2>&1
        #./shell_simple_ver9_5 ${nd} ${seed} > /dev/null 2>&1
        ./shell_simple_ver9_5_1 ${nd} ${seed} > /dev/null 2>&1

        #> Results/ETr.txt

        if [ $? -eq 0 ]; then
            echo "ドローン台数${nd}台, Seed${seed}: 実行完了"
        else
            echo "エラー: ドローン台数${nd}台, Seed${seed}での実行に失敗しました"
        fi
        
        echo
    done

    #統計処理のプログラム実行
    gcc ETa_ave.c -o ETa_ave -lm
    ./ETa_ave

    gcc ETb_ave.c -o ETb_ave -lm
    ./ETb_ave

    gcc ERdro_ave.c -o ERdro_ave -lm
    ./ERdro_ave

    gcc ERsh_ave.c -o ERsh_ave -lm
    ./ERsh_ave

    gcc ETc_dro_ave.c -o ETc_dro_ave -lm
    ./ETc_dro_ave

    gcc ETc_TV_ave.c -o ETc_TV_ave -lm
    ./ETc_TV_ave

    gcc ETb_shelter.c -o ETb_shelter -lm
    ./ETb_shelter

    # 次のドローン台数のために、ETa.txtとETb.txtをクリア
    > Results/ETa.txt
    > Results/ETb.txt
    > Results/Rdro.txt
    > Results/Rsh.txt
    > Results/Tc_dro.txt
    > Results/Tc_TV.txt
    > Results/tb_shelter_avg_total.txt
    
    echo "ドローン台数${nd}台の全Seed値での実行完了"
    echo
done

echo "========================================"
echo "全ての実行が完了しました"
echo "実行概要:"
echo "  - ドローン台数: 0〜20台"
echo "  - 各台数でSeed値: 1〜10"
echo "  - 総実行回数: 210回 (21台数 × 10seed値)"
echo "========================================"
