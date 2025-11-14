#!/bin/bash

# ドローン台数NDを0から8まで1ずつ増やしながらプログラムを実行

echo "=== 一括実行スクリプト ==="
echo "ドローン台数0から8まで順次実行開始..."

# プログラムをコンパイル
echo "プログラムをコンパイル中..."
#gcc -o shell_simple_ver8_3_1 shell_simple_ver8_3_1.c -lm
gcc -o shell_simple_seedchange shell_simple_seedchange.c -lm
#gcc -o shell_simple_ver9_2 shell_simple_ver9_2.c -lm

if [ $? -ne 0 ]; then
    echo "エラー: コンパイルに失敗しました"
    exit 1
fi

# Resultsディレクトリの準備
mkdir -p Results

# numerical_data.txtを初期化（ヘッダー行も削除してリセット）
> Results/numerical_data.txt
> Results/ETr.txt
> Results/Ave_ETr.txt

echo "実行開始..."
echo

# ドローン台数0から20まで、各ドローン台数でseed値1から10まで順次実行
for nd in {0..20}
do
    echo "========================================"
    echo "ドローン台数: ${nd}台での実行開始"
    echo "========================================"
    
    # seed値1から10まで順次実行
    for seed in {12..27}
    do
        echo "----------------------------------------"
        echo "ドローン台数: ${nd}台, Seed値: ${seed}"
        echo "----------------------------------------"
        
        # プログラム実行（標準出力を抑制し、エラーのみ表示）
        #./shell_simple_ver8_3_1 ${nd} ${seed} > /dev/null 2>&1
        ./shell_simple_seedchange ${nd} ${seed} > /dev/null 2>&1
        #./shell_simple_ver9_2 ${nd} ${seed} > /dev/null 2>&1

        #> Results/ETr.txt

        if [ $? -eq 0 ]; then
            echo "ドローン台数${nd}台, Seed${seed}: 実行完了"
        else
            echo "エラー: ドローン台数${nd}台, Seed${seed}での実行に失敗しました"
        fi
        
        echo
    done

    gcc ETr_ave.c -o ETr_ave -lm
    ./ETr_ave 

    > Results/ETr.txt
    
    echo "ドローン台数${nd}台の全Seed値での実行完了"
    echo
done

echo "========================================"
echo "全ての実行が完了しました"
echo "実行概要:"
echo "  - ドローン台数: 0〜20台"
echo "  - 各台数でSeed値: 1〜10"
echo "  - 総実行回数: 210回 (21台数 × 10seed値)"
echo "結果ファイル:"
echo "  - Results/numerical_data.txt (数値データ)"
echo "  - Results/ETr.txt (Tr平均値データ)"
echo "========================================"

# 結果の概要表示
if [ -f "Results/numerical_data.txt" ]; then
    echo
    echo "=== 実行結果概要 ==="
    echo "総データ数: $(wc -l < Results/numerical_data.txt)行"
    echo "ETrデータ数: $(wc -l < Results/ETr.txt)行"
    echo
    echo "numerical_data.txt の最初の5行:"
    head -5 Results/numerical_data.txt | nl -w2 -s": "
fi
