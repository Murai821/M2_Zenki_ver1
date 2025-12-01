#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINES 1000
#define MAX_LINE_LENGTH 256

int main()
{
    FILE *input_file, *output_file;
    char line[MAX_LINE_LENGTH];
    double sum_value, count_value, average;

    // 入力ファイルを開く
    input_file = fopen("Results/tb_shelter_avg_total.txt", "r");
    if (input_file == NULL)
    {
        printf("Error: Cannot open Results/tb_shelter_avg_total.txt\n");
        return 1;
    }

    // 出力ファイルを開く
    output_file = fopen("Results/Ave_ETb_shelter.txt", "w");
    if (output_file == NULL)
    {
        printf("Error: Cannot create Results/Ave_ETb_shelter.txt\n");
        fclose(input_file);
        return 1;
    }

    printf("Processing tb_shelter_avg_total.txt...\n");

    // ファイルを行ごとに読み込み処理
    while (fgets(line, sizeof(line), input_file))
    {
        // 空行をスキップ
        if (strlen(line) <= 1)
        {
            continue;
        }

        // 第一列（合計値）と第二列（カウント）を読み取り
        if (sscanf(line, "%lf\t%lf", &sum_value, &count_value) == 2)
        {
            // ゼロ除算チェック
            if (count_value > 0)
            {
                average = sum_value / count_value;
                fprintf(output_file, "%.6f\n", average);
                printf("Sum: %.6f, Count: %.0f, Average: %.6f\n",
                       sum_value, count_value, average);
            }
            else
            {
                printf("Warning: Count is zero for line: %s", line);
                fprintf(output_file, "0.000000\n");
            }
        }
        else
        {
            printf("Warning: Could not parse line: %s", line);
        }
    }

    // ファイルを閉じる
    fclose(input_file);
    fclose(output_file);

    printf("Average calculation completed. Results saved to Results/Ave_ETb_shelter.txt\n");

    return 0;
}
