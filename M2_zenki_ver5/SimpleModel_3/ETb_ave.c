/**
 * @file ETb_ave.c
 * @brief Results/ETb.txtの値の平均値を計算してResults/Ave_ETb.txtに出力するプログラム
 *
 * 【概要】
 * - Results/ETb.txtから数値データを読み取る
 * - 読み取った数値の平均値を計算する
 * - 平均値をResults/Ave_ETb.txtに出力する
 *
 * コンパイル: gcc ETb_ave.c -o ETb_ave -lm
 * 実行: ./ETb_ave
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_VALUES 10000 // 最大読み取り値数
#define INPUT_FILE "Results/ETb.txt"
#define OUTPUT_FILE "Results/Ave_ETb.txt"

int main()
{
    FILE *input_file, *output_file;
    double values[MAX_VALUES];
    double value;
    int count = 0;
    double sum = 0.0;
    double average = 0.0;

    // === ETr.txtファイルを開く ===
    input_file = fopen(INPUT_FILE, "r");
    if (input_file == NULL)
    {
        return 1;
    }

    // === データを読み取り ===
    while (fscanf(input_file, "%lf", &value) == 1 && count < MAX_VALUES)
    {
        values[count] = value;
        sum += value;
        count++;
    }
    fclose(input_file);

    // === 読み取り結果の確認 ===
    if (count == 0)
    {
        return 1;
    }

    // === 平均値計算 ===
    average = sum / count;

    // === Ave_ETr.txtに平均値を出力 ===
    output_file = fopen(OUTPUT_FILE, "a");
    if (output_file == NULL)
    {
        return 1;
    }

    fprintf(output_file, "%.3f\n", average);
    fclose(output_file);

    return 0;
}
