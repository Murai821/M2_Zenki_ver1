// 医療品の配送平均遅延時間算出
#include <stdio.h>
#include <stdlib.h>

int main()
{
    FILE *inputFile, *outputFile;
    char inputFilePath[] = "drone_datafile/txtfile/Mean_Med_re_delay.txt";                // 読み込むファイルのパス
    char outputFilePath[] = "drone_datafile/txtfile/excel_data/Shell_Med_re_average.txt"; // 書き込むファイルのパス
    double value, sum = 0.0;
    int count = 0;

    // 入力ファイルを開く
    inputFile = fopen(inputFilePath, "r");
    if (inputFile == NULL)
    {
        printf("ファイルを開けませんでした。\n");
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
    outputFile = fopen(outputFilePath, "a+");
    if (outputFile == NULL)
    {
        printf("ファイルを開けませんでした。\n");
        return 1;
    }

    // 平均値を書き込む
    fprintf(outputFile, "%.6f\n", average);

    // 出力ファイルを閉じる
    fclose(outputFile);

    // printf("医療品の配送遅延時間の平均値 %.6f をファイルに書き込みました。\n", average);

    return 0;
}
