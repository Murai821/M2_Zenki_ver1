#include <stdio.h>
#include <stdlib.h>

#define MAX_CLASSES 9 // 0~10時間の11階級

int main()
{
    FILE *fp = fopen("drone_datafile/txtfile/Etc_histgram_data.txt", "r");
    if (fp == NULL)
    {
        perror("Failed to open data file");
        return 1;
    }

    int class_counts[MAX_CLASSES] = {0};
    int total_count = 0;
    double value;

    // データ読み込み・階級化
    while (fscanf(fp, "%lf", &value) == 1)
    {
        int class_index = (int)(value / 3600);
        if (class_index >= MAX_CLASSES)
            continue;
        class_counts[class_index]++;
        total_count++;
    }
    fclose(fp);

    // 一時データファイル作成（割合で出力）
    FILE *data = fopen("tmp_data.txt", "w");
    for (int i = 0; i < MAX_CLASSES; i++)
    {
        double ratio = (total_count > 0) ? (class_counts[i] * 1.0) / total_count : 0.0;
        fprintf(data, "%d %.6f\n", i, ratio);
    }
    fclose(data);

    // Gnuplot を使って PNG 出力
    FILE *gp = popen("gnuplot -persist", "w");
    if (gp == NULL)
    {
        perror("Failed to launch gnuplot");
        return 1;
    }

    fprintf(gp, "set terminal pngcairo size 800,600 enhanced font 'DejaVu Sans,12'\n");
    fprintf(gp, "set output 'drone_datafile/histogram_ratio.png'\n");

    fprintf(gp, "set xrange [-0.5:8.5]\n");
    fprintf(gp, "set yrange [0:1]\n");
    fprintf(gp, "set xlabel 'Time range (hours)'\n");
    fprintf(gp, "set ylabel 'Relative Frequency'\n");
    fprintf(gp, "set style fill solid 1.0\n");
    fprintf(gp, "set boxwidth 0.8\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set xtics rotate by -45\n");

    // X軸ラベル定義
    fprintf(gp, "set xtics (");
    for (int i = 0; i < MAX_CLASSES; i++)
    {
        if (i > 0)
            fprintf(gp, ", ");
        fprintf(gp, "'%d〜%d' %d", i, i + 1, i);
    }
    fprintf(gp, ")\n");

    // プロット
    fprintf(gp, "plot 'tmp_data.txt' using 1:2 with boxes lc rgb 'skyblue'\n");

    pclose(gp);
    remove("tmp_data.txt");

    printf("Histogram (ratio) exported to 'drone_datafile/histogram_ratio.png'.\n");
    return 0;
}
