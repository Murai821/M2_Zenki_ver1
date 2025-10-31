// shellスクリプトで求めた「drone_datafile/txtfile/totalMax_ave_data.txt」内の平均値を求め「Shell_totalMax_average.txt」に書き込むプログラム
// 各順回路の総距離の中で最大値の平均を導出プログラム
#include <stdio.h>
#include <stdlib.h>

int main()
{
    FILE *inputFile, *outputFile;
    char inputFilePath[] = "drone_datafile/txtfile/totalMax_ave_data.txt";                  // 読み込むファイルのパス
    char outputFilePath[] = "drone_datafile/txtfile/excel_data/Shell_totalMax_average.txt"; // 書き込むファイルのパス
    double value, sum = 0.0;
    int count = 0;

    // 入力ファイルを開く
    inputFile = fopen(inputFilePath, "r");
    if (inputFile == NULL)
    {
        printf("totalMax_ave_data.txt ファイルを開けませんでした。\n");
        return 1;
    }

    // データを読み込み、合計と数を計算する
    while (fscanf(inputFile, "%lf", &value) == 1)
    {
        sum += value;
        count++;
    }

    // 入力ファイルを閉じる
    fclose(inputFile);

    // 平均値を計算する
    double average = sum / count;

    // 出力ファイルを開いて平均値を書き込む
    // outputFile = fopen(outputFilePath, "w");
    outputFile = fopen(outputFilePath, "a+");
    if (outputFile == NULL)
    {
        printf("Shell_totalMax_average.txt ファイルを開けませんでした。\n");
        return 1;
    }

    // 平均値を書き込む
    fprintf(outputFile, "%.6f\n", average);

    // 出力ファイルを閉じる
    fclose(outputFile);

    printf("平均値 %.6f をファイルに書き込みました。\n", average);

    return 0;
}
