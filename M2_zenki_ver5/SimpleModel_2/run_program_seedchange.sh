#!/bin/bash

# shell_simple_ver8_2.c用実行スクリプト
# ドローン台数NDを0から8まで1ずつ増やしながらプログラムを実行

echo "=== shell_simple_ver8_3_1.c 一括実行スクリプト ==="
echo "ドローン台数0から8まで順次実行開始..."

# プログラムをコンパイル
echo "プログラムをコンパイル中..."
#gcc -o shell_simple_ver8_3_1 shell_simple_ver8_3_1.c -lm
gcc -o shell_simple_ver8_3_2 shell_simple_ver8_3_2.c -lm
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

echo "実行開始..."
echo

# ドローン台数0から8まで順次実行
for nd in {0..20}
do
    echo "----------------------------------------"
    echo "ドローン台数: ${nd}台での実行開始"
    echo "----------------------------------------"
    
    # プログラム実行（標準出力を抑制し、エラーのみ表示）
    #./shell_simple_ver8_3_1 ${nd} > /dev/null 2>&1
    ./shell_simple_ver8_3_2 ${nd} > /dev/null 2>&1
    #./shell_simple_ver9_2 ${nd} > /dev/null 2>&1

    if [ $? -eq 0 ]; then
        echo "ドローン台数${nd}台: 実行完了"
    else
        echo "エラー: ドローン台数${nd}台での実行に失敗しました"
    fi
    
    echo
done

echo "========================================="
echo "全ての実行が完了しました"
echo "結果ファイル:"
echo "  - Results/numerical_data.txt (数値データ)"
echo "  - Results/tr_values.txt (最後の実行のTr値)"
echo "========================================="

# 結果の概要表示
if [ -f "Results/numerical_data.txt" ]; then
    echo
    echo "=== 実行結果概要 ==="
    echo "ドローン台数別データ (車両配送量[t], ドローン配送量[t], 総配送量[t], 時間[h], 生成要求数, 完了要求数, ドローン台数, 平均配送時間[h]):"
    cat Results/numerical_data.txt | nl -w2 -s": ND="
fi
