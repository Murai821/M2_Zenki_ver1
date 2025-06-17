// コンパイル方法「gcc -o my_program main.c module.c -lm」
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "header.h"

/**************************************メイン関数******************************************************/
int main(int argc, char *argv[])
{
    int i, j, k, m;
    char *data_file;     // 座標プロット用
    char *ad_file;       // road-network表示用
    char *jyunkai_file;  // 巡回ルート表示用
    char *new_data_file; // 新たに番号を付した配送センター、避難所の表示用
    char *new_ad_file;
    char *y_file;
    char *v1_file; // 配送車１の順回路表示用ファイル
    char *v2_file;
    char *v3_file;
    char *v4_file;
    char *v5_file;
    char *inf_interval_file;   // 平均情報到着間隔
    char *inf_delay_file;      // 平均情報遅延時間
    char *inf_delay_part_file; // 個別の避難所Td
    char *re_interval_file;    // 平均物資到着間隔
    char *Etd_data_file;       // シミュレーションごとに平均情報遅延時間を格納してくファイル
    char *Eti_data_file;
    char *Etg_data_file;
    char *Medinf_delay_file;                     // 薬情報の平均遅延時間
    char *Med_re_delay_file;                     // 医療品の配送平均遅延時間
    char *Medinf_collect_delay_file;             // 薬情報の収集平均遅延時間
    char *Med_re_collect_to_delivery_delay_file; // 医療品の収集平均遅延時間

    double COST[N]; /*距離*/
    int VIA[N];     /*経由点*/
    char USED[N];   /*確定か未確定か*/

    // シェルスクリプトにおける記述
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <seed>\n", argv[0]);
        return 1;
    }
    int seed = atoi(argv[1]);
    srand(seed); // シード値

    point p[N];   // 避難所と配送センターの宣言
    vehicle v[M]; // 配送車の宣言

    // 配送センターの座標初期化
    p[0].x = L / 2;
    p[0].y = L / 2;

    p[0].re = 0;

    p[0].re_req = 0;

    p[0].re_req_sum = 0;

    p[0].re_deli = 0;

    for (i = 0; i < N; i++)
    {
        p[0].i_ptr[i] = 0;
        p[0].i_med_ptr[i] = 0;
    }

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < I_SIZE; j++)
        {
            p[0].inf[i][j] = 0;
        }
    }

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < Y_SIZE; j++)
        {
            for (k = 0; k < Z_SIZE; k++)
            {
                p[0].inf_med[i][j][k] = 0.0;
            }
        }
    }

    // 避難所の要素初期化 i=0は集積所のため省く
    /*中心市街地*/
    for (i = 1; i < C_N; i++)
    {
        // 一度ランダムに生成
        p[i].x = (double)rand() / RAND_MAX * (2 * R);
        p[i].y = (double)rand() / RAND_MAX * (2 * R);
        p[i].x += L / 2 - R; // 中心をL/2に修正
        p[i].y += L / 2 - R;

        while (retDis(p[0].x, p[0].y, p[i].x, p[i].y) > R) // 中心市街地内に生成されるまで繰り返す
        {
            p[i].x = (double)rand() / RAND_MAX * (2 * R);
            p[i].y = (double)rand() / RAND_MAX * (2 * R);

            p[i].x += L / 2 - R; // 中心をL/2に修正
            p[i].y += L / 2 - R;
        }

        p[i].re = 0;

        p[i].re_req = 0;

        p[i].re_req_sum = 0;

        p[i].re_deli = 0;

        for (j = 0; j < N; j++)
        {
            p[i].i_ptr[j] = 0;
            p[i].i_med_ptr[j] = 0;
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < I_SIZE; k++)
            {
                p[i].inf[j][k] = 0;
            }
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < Y_SIZE; k++)
            {
                for (m = 0; m < Z_SIZE; m++)
                {
                    p[i].inf_med[j][k][m] = 0.0;
                }
            }
        }
    }
    /*山間部の避難所初期化*/
    for (i = C_N; i < N; i++)
    {
        p[i].x = L / 2; // 初期値
        p[i].y = L / 2;

        while (retDis(p[0].x, p[0].y, p[i].x, p[i].y) < (R + A_R)) // (R+A_R)の範囲外に生成されるまで繰り返す
        {
            // printf("i=%d : %f\n", i, retDis(p[0].x, p[0].y, p[i].x, p[i].y));
            p[i].x = (double)rand() / RAND_MAX * L;
            p[i].y = (double)rand() / RAND_MAX * L;
        }

        p[i].re = 0;

        p[i].re_req = 0;

        p[i].re_req_sum = 0;

        p[i].re_deli = 0;

        for (j = 0; j < N; j++)
        {
            p[i].i_ptr[j] = 0;
            p[i].i_med_ptr[j] = 0.0;
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < I_SIZE; k++)
            {
                p[i].inf[j][k] = 0;
            }
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < Y_SIZE; k++)
            {
                for (m = 0; m < Z_SIZE; m++)
                {
                    p[i].inf_med[j][k][m] = 0.0;
                }
            }
        }
    }

    // 配送車初期化
    for (i = 0; i < M; i++)
    {
        v[i].x = L / 2;
        v[i].y = L / 2;

        v[i].re = 0;

        v[i].Med_re = 0;

        v[i].next_wait_flag = FALSE;

        for (j = 0; j < N; j++)
        {
            v[i].i_ptr[j] = 0;
            v[i].i_med_ptr[j] = 0;
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < I_SIZE; k++)
            {
                v[i].inf[j][k] = 0;
            }
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < Y_SIZE; k++)
            {
                for (m = 0; m < Z_SIZE; m++)
                {
                    v[i].inf_med[j][k][m] = 0.0;
                }
            }
        }

        for (j = 0; j < QUEUE_SIZE; j++)
        {
            v[i].Med_delivery_queue[j] = 0;
        }

        v[i].queue_ptr = 0;

        v[i].queue_Notdelivery_ptr = 0;
    }

    /*********************************************** pythonの出力ファイルから点の「座標」と「隣接行列」を読み込む **************************************************************************/

    /*********************************************** adjacency_matrix.txt ファイルからconnect[][]へ要素を格納 **************************************************************/
    int connect[N][N] = {0}; // つながっている点同士の関係を表す配列

    // adjacency_matrix.txtからデータを読み込んでconnect配列に格納
    readAdjacencyMatrix("pythonfile/adjacency_matrix.txt", connect);

    /*********************************************** points.txt ファイルから p[].x, p[].y へ要素を格納 **************************************************************/
    // ファイルを開く
    FILE *file = fopen("pythonfile/points.txt", "r");
    if (file == NULL)
    {
        printf("ファイルを開けませんでした。\n");
        return 1;
    }
    // 座標を読み込んで構造体に格納する
    for (i = 0; i < N; i++)
    {
        if (fscanf(file, "%lf %lf", &p[i].x, &p[i].y) != 2)
        {
            printf("座標を読み込めませんでした。\n");
            return 1;
        }
    }
    // ファイルを閉じる
    fclose(file);

    /**********************************************************　各点間の距離をdi[][]に格納 *********************************************************************/
    // 各避難所、配送センター間を格納
    double di[N][N];

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            di[i][j] = sqrt(pow(p[i].x - p[j].x, 2) + pow(p[i].y - p[j].y, 2));
        }
    }

    double ad[N][N]; // 隣接行列

    // 初期化
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            if (i == j)
            {
                ad[i][j] = 0;
            }
            else
            {
                ad[i][j] = INF;
            }
        }
    }

    // connect[i][j]をもとに隣接行列に値を格納
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            if ((i != j) && connect[i][j] == TRUE)
            {
                ad[i][j] = di[i][j];
                ad[j][i] = di[i][j]; // ad[][]どちらにも値を格納
            }
        }
    }

    // 繋がっている辺同士の組合わせをファイルに格納
    FILE *fp_ad;
    ad_file = "drone_datafile/txtfile/ad.txt";
    fp_ad = fopen(ad_file, "w");
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            if (ad[i][j] != 0 && ad[i][j] != INF)
            {
                fprintf(fp_ad, "%lf %lf\n", p[i].x, p[i].y);
                fprintf(fp_ad, "%lf %lf\n", p[j].x, p[j].y);
                fprintf(fp_ad, "\n");
            }
        }
    }
    fclose(fp_ad);

    /****************************************************************************************************************************/
    /***** pythonファイルの出力ファイルから巡回セールスマン問題の近似解の巡回路を読み込む*******/
    // ファイルポインタを定義
    FILE *file_python_jyunkairo;
    // 読み込む配列データを格納する配列を定義
    int jyunkai_keiro[MAX_ELEMENTS];
    // 読み込んだ要素数を保持する変数を定義
    int python_jyunkairo_count = 0;

    // ファイルを読み取りモードで開く
    file_python_jyunkairo = fopen("pythonfile/tsp_result.txt", "r");
    if (file_python_jyunkairo == NULL)
    {
        printf("ファイルを開けませんでした。\n");
        return 1;
    }

    // "int tsp_route[] = {" まで読み飛ばす
    fscanf(file_python_jyunkairo, "int tsp_route[] = {");

    // ファイルから整数を読み込み配列に格納する
    while (fscanf(file_python_jyunkairo, "%d,", &jyunkai_keiro[python_jyunkairo_count]) == 1)
    {
        python_jyunkairo_count++;
        if (python_jyunkairo_count >= MAX_ELEMENTS)
        {
            printf("配列の最大要素数を超えました。\n");
            break;
        }
    }

    printf("python_jyunkairo_count:%d\n", python_jyunkairo_count);

    // ファイルを閉じる
    fclose(file_python_jyunkairo);

    int jyunkai_keiro_size = sizeof(jyunkai_keiro) / sizeof(jyunkai_keiro[0]); // 配列のサイズを決定

    int new_jyunkai_keiro_size = jyunkai_keiro_size; // 番号を更新した巡回経路の配列のサイズ

    int new_jyunkai_keiro[new_jyunkai_keiro_size]; // 番号を更新した巡回路の配列

    /****************************************************************************************************************************/
    FILE *fp_jyunkai;
    jyunkai_file = "drone_datafile/txtfile/jyunkai.txt";
    fp_jyunkai = fopen(jyunkai_file, "w");
    for (i = 0; i < jyunkai_keiro_size - 1; i++)
    {
        fprintf(fp_jyunkai, "%lf %lf\n", p[jyunkai_keiro[i]].x, p[jyunkai_keiro[i]].y);
        fprintf(fp_jyunkai, "%lf %lf\n", p[jyunkai_keiro[i + 1]].x, p[jyunkai_keiro[i + 1]].y);
        fprintf(fp_jyunkai, "\n");
    }

    fclose(fp_jyunkai);

    /*****新たに避難所の番号を定義******/
    point new_p[N];
    double new_ad[N][N];
    double new_di[N][N];
    FILE *fp_new;
    FILE *fp_new_ad;

    // new_p 初期化
    for (i = 0; i < N; i++)
    {

        new_p[i].re = 0;

        for (j = 0; j < N; j++)
        {
            new_p[i].i_ptr[j] = 0;
            new_p[i].i_med_ptr[j] = 0.0;
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < I_SIZE; k++)
            {
                new_p[i].inf[j][k] = 0.0;
            }
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < Y_SIZE; k++)
            {
                for (m = 0; m < Z_SIZE; m++)
                {
                    new_p[i].inf_med[j][k][m] = 0.0;
                }
            }
        }
    }

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            if (i == j)
            {
                new_ad[i][j] = 0;
            }
            else
            {
                new_ad[i][j] = INF;
            }
        }
    }

    /*番号を更新*/
    int flag[N] = {0};   // 再定義した座標new_p[N]に格納済みか
    int new_p_index = 0; // new_pに格納していくインデックス
    int corres[N];       // p[i]とnew_p[i]の対応関係を格納: 元：6　更新後：1 → corres[1] = 6
    int re_corres[N];    // p[i]とnew_p[i]の対応関係を格納: 元：6　更新後：1 → re_corres[6] = 1
    for (i = 0; i < jyunkai_keiro_size; i++)
    {
        if (flag[jyunkai_keiro[i]] == 0)
        {
            new_p[new_p_index].x = p[jyunkai_keiro[i]].x;
            new_p[new_p_index].y = p[jyunkai_keiro[i]].y;
            flag[jyunkai_keiro[i]] = 1;
            corres[new_p_index] = jyunkai_keiro[i];
            re_corres[jyunkai_keiro[i]] = new_p_index;
            new_p_index += 1;
        }
    }

    /******** new_jyunkai_keiro[]更新 ********/
    for (i = 0; i < jyunkai_keiro_size; i++)
    {
        new_jyunkai_keiro[i] = re_corres[jyunkai_keiro[i]];
    }

    /*隣接行列を再定義*/

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            new_ad[i][j] = ad[corres[i]][corres[j]];
        }
    }

    /*新たな座標の各点間の距離*/
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            new_di[i][j] = sqrt(pow(new_p[i].x - new_p[j].x, 2) + pow(new_p[i].y - new_p[j].y, 2));
        }
    }

    new_data_file = "drone_datafile/txtfile/new_data.txt";
    new_ad_file = "drone_datafile/txtfile/new_ad.txt";

    fp_new = fopen(new_data_file, "w");

    for (i = 0; i < N; i++)
    {
        fprintf(fp_new, "%d\t%lf\t%lf\n", i, new_p[i].x, new_p[i].y);
    }

    fclose(fp_new);

    // 番号を更新した隣接行列表示ファイル
    fp_new_ad = fopen(new_ad_file, "w");

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            if (i != 0 && new_ad[i][j] != INF)
            {
                fprintf(fp_new_ad, "%lf %lf\n", new_p[i].x, new_p[i].y);
                fprintf(fp_new_ad, "%lf %lf\n", new_p[j].x, new_p[j].y);
                fprintf(fp_new_ad, "\n");
            }
        }
    }

    fclose(fp_new_ad);

    /************************* RFCS法による巡回路分割 *******************************/
    int rfcs_re[M + 1] = {0}; // RFCS法の結果を格納する配列
    int rfcs_re_size = 0;     // RFCS法の結果を格納する配列のサイズ
    RFCS_method(new_ad, N, rfcs_re, &rfcs_re_size);
    printf("RFCS result: ");
    for (i = 0; i < rfcs_re_size; i++)
    {
        if (i == rfcs_re_size - 1)
        {
            printf("%d", rfcs_re[i]);
        }
        else
        {
            printf("%d -> ", rfcs_re[i]);
        }
    }
    printf("\n");

    /********************************************************ダイクストラ法を行う関数による経由地点追加配列処理***********************************************************************:*/

    // サブ配列の定義
    int size[MAX_SUBARRAYS] = {0}; // 各サブ配列のサイズを記録する配列

    /******************************** 配列動的確保 *******************************************************/
    int **cir;
    int **cir_flag;

    // cir初期化
    cir = (int **)calloc(MAX_SUBARRAYS, sizeof(int *));      // MAX10
    cir_flag = (int **)calloc(MAX_SUBARRAYS, sizeof(int *)); // MAX10
    for (i = 0; i < MAX_SUBARRAYS; i++)
    {
        cir[i] = (int *)calloc(MAX_SIZE, sizeof(int));      // MAX100
        cir_flag[i] = (int *)calloc(MAX_SIZE, sizeof(int)); // MAX100
    }

    split_array(new_jyunkai_keiro, python_jyunkairo_count, cir, size); // 関数呼び出し

    /*****要素追加****/
    int *path;       // サブ配列に追加する配列
    int path_length; // 追加する配列のサイズ

    for (i = 0; i < M; i++) // M個分のサブ配列の各巡回路にダイクストラ法で導出した経由地点を追加する
    {
        if (i == 0) // 一番目の分割配列には最後の避難所から集積所までの経由地点を配列の最後に追加
        {
            // 後ろに経由地点追加
            dijkstraArrayAdd(cir[i][size[i] - 1], 0, new_ad, N, &path, &path_length); // 10→0までの経由地点を配列に格納
            add_to_row(&cir, size, i, path, path_length);                             // cirの i=0 行の配列の最後のにpathを追加
        }
        else if (i == M - 1) // 一番最後のサブ配列には集積所からの経由地点を追加
        {
            // 前に経由地点追加
            dijkstraArrayAppend(0, cir[i][0], new_ad, N, &path, &path_length); // 0→50までの経由地点を配列に格納
            prepend_to_row(&cir, size, i, path, path_length);
        }
        else // その他のサブ配列には前後に経由地点を追加
        {
            // 前に経由地点追加
            dijkstraArrayAppend(0, cir[i][0], new_ad, N, &path, &path_length); // 0→50までの経由地点を配列に格納
            prepend_to_row(&cir, size, i, path, path_length);
            // 後ろに経由地点追加
            dijkstraArrayAdd(cir[i][size[i] - 1], 0, new_ad, N, &path, &path_length); // 10→0までの経由地点を配列に格納
            add_to_row(&cir, size, i, path, path_length);                             // cirの i=0 行の配列の最後のにpathを追加
        }
    }

    // 元の配列を表示

    for (int i = 0; i < new_jyunkai_keiro_size; i++)
    {
        printf("%d ", new_jyunkai_keiro[i]);
    }
    printf("\n");

    // 分割されたサブ配列を表示

    for (int i = 0; i < MAX_SUBARRAYS && size[i] > 0; i++)
    {
        printf("Subarray cir[%d][]: ", i + 1); // サブ配列の番号を表示
        for (int j = 0; j < size[i]; j++)
        {
            printf("%d ", cir[i][j]); // サブ配列の内容を表示
        }
        printf("\n");
    }

    // size確認
    for (i = 0; i < M; i++)
    {
        printf("size[%d]:%d\n", i, size[i]);
    }

    /***************************************************************************************************************************************************************************/

    /******************各小回線の最後に物資を下ろす避難所のindex*********************************/
    int ind_last[M] = {0};        // 各小回線の最後に物資を下ろす避難所のindex
    int size_indlast[M] = {0};    // indexのカウント用変数
    int indlast_current[M] = {0}; // 現在の物資を下ろす避難所番号、順に一づつ増える
    int lastflag[M] = {0};

    for (i = 0; i < M; i++)
    {

        indlast_current[i] = i * J_N + 1; // 初期化

        for (j = 0; j < size[i]; j++)
        {
            if (indlast_current[i] == (i + 1) * J_N && cir[i][j] == (i + 1) * J_N) // 物資を運ぶ避難所に到達した and J_N箇所の避難所を巡回しているならそのときのindexを格納
            {
                ind_last[i] = size_indlast[i];
            }

            if (cir[i][j] == indlast_current[i])
            {
                indlast_current[i] = indlast_current[i] + 1;
            }

            size_indlast[i] += 1;
        }
        // printf("%d\n", ind_last[i]); // 動作確認
    }

    /*****************************cir_flag[][] : cir[][]において物資を荷降ろしするindex*******************************/
    // cir[][]において物資を荷降ろしする添字はTRUE

    int cirflag_current[M] = {0};

    for (i = 0; i < M; i++)
    {
        cirflag_current[i] = i * 10 + 1; // 初期化

        for (j = 0; j < size[i]; j++)
        {
            if (cir[i][j] == cirflag_current[i] && cir[i][j] <= (i + 1) * 10)
            {
                cir_flag[i][j] = TRUE;
                cirflag_current[i] += 1;
            }
            // 動作確認
            if (cir_flag[i][j] == TRUE)
            {
                // printf("TRUE,");
            }
            else
            {
                // printf("%d,", cir_flag[i][j]);
            }
        }
        // printf("\n");
    }

    /***********************************各小回線の総距離********************************************/
    double total_di[M] = {0}; // 各小回線の総距離
    // 各小回線の総距離の導出
    for (i = 0; i < M; i++)
    {
        for (j = 0; j < size[i] - 1; j++)
        {
            total_di[i] += new_di[cir[i][j]][cir[i][j + 1]];
        }
    }

    // 回線の総距離表示
    printf("配送車の回線の総距離[km]\n");
    for (i = 0; i < M; i++)
    {
        // printf("配送車%dの回線の総距離: %f\n", i + 1, total_di[i]);
        printf("%f\n", total_di[i]);
    }

    /***********************************各小回線の総距離の平均値********************************************/
    double total_di_sum = 0; // 各小回線の総距離の合計値
    double total_di_ave = 0; // 各小回線の総距離の平均値

    for (i = 0; i < M; i++)
    {
        total_di_sum += total_di[i];
    }
    total_di_ave = total_di_sum / M;
    printf("各小回線の総距離の平均値:%lf[km]\n", total_di_ave);

    // 各小回線の総距離の平均値をファイルに格納（シミュレーションごとに格納していく）
    FILE *fp_totaldi_data;
    char *totaldi_ave_data_file = "drone_datafile/txtfile/totaldi_ave_data.txt";

    fp_totaldi_data = fopen(totaldi_ave_data_file, "a+");
    fprintf(fp_totaldi_data, "%f\n", total_di_ave);
    fclose(fp_totaldi_data);

    /***********************************各小回線の総距離における最大値の出力********************************************/
    double max_value = total_di[0]; // 初期値として配列の最初の要素を最大値とする

    // 配列の要素を順に比較して最大値を探す
    for (int i = 1; i < M; i++)
    {
        if (total_di[i] > max_value)
        {
            max_value = total_di[i]; // より大きい値があれば更新
        }
    }

    // 最大値を出力
    printf("巡回路の総距離の最大値: %.2f[km]\n", max_value);

    // 各小回線の総距離の最大値をファイルに格納（シミュレーションごとに格納していく）
    FILE *fp_totalMax_data;
    char *totalMax_ave_data_file = "drone_datafile/txtfile/totalMax_ave_data.txt";

    fp_totalMax_data = fopen(totalMax_ave_data_file, "a+");
    fprintf(fp_totalMax_data, "%f\n", max_value);
    fclose(fp_totalMax_data);

    /***********************************各小回線の一周の所要時間********************************************/
    for (i = 0; i < M; i++)
    {
        printf("配送車%dの回線の所要時間: %f\n", i + 1, total_di[i] * 360.0);
    }

    // 各配送車の順回路表示用ファイル
    FILE *fp_v1, *fp_v2, *fp_v3, *fp_v4, *fp_v5;

    v1_file = "drone_datafile/txtfile/data_v1.txt";
    fp_v1 = fopen(v1_file, "w");
    for (i = 0; i < size[0] - 1; i++)
    {
        // 表示用に巡回路１を全体的に0.05右上にずらす
        fprintf(fp_v1, "%f %f\n", new_p[cir[0][i]].x + 0.05, new_p[cir[0][i]].y + 0.05);
        fprintf(fp_v1, "%f %f\n", new_p[cir[0][i + 1]].x + 0.05, new_p[cir[0][i + 1]].y + 0.05);
        fprintf(fp_v1, "\n");
    }
    fclose(fp_v1);

    v2_file = "drone_datafile/txtfile/data_v2.txt";
    fp_v2 = fopen(v2_file, "w");
    for (i = 0; i < size[1] - 1; i++)
    {
        fprintf(fp_v2, "%f %f\n", new_p[cir[1][i]].x, new_p[cir[1][i]].y);
        fprintf(fp_v2, "%f %f\n", new_p[cir[1][i + 1]].x, new_p[cir[1][i + 1]].y);
        fprintf(fp_v2, "\n");
    }
    fclose(fp_v2);

    v3_file = "drone_datafile/txtfile/data_v3.txt";
    fp_v3 = fopen(v3_file, "w");
    for (i = 0; i < size[2] - 1; i++)
    {
        fprintf(fp_v3, "%f %f\n", new_p[cir[2][i]].x, new_p[cir[2][i]].y);
        fprintf(fp_v3, "%f %f\n", new_p[cir[2][i + 1]].x, new_p[cir[2][i + 1]].y);
        fprintf(fp_v3, "\n");
    }
    fclose(fp_v3);

    v4_file = "drone_datafile/txtfile/data_v4.txt";
    fp_v4 = fopen(v4_file, "w");
    for (i = 0; i < size[3] - 1; i++)
    {
        // 表示用に巡回路4を全体的に0.05右上にずらす
        fprintf(fp_v4, "%f %f\n", new_p[cir[3][i]].x + 0.05, new_p[cir[3][i]].y - 0.05);
        fprintf(fp_v4, "%f %f\n", new_p[cir[3][i + 1]].x + 0.05, new_p[cir[3][i + 1]].y - 0.05);
        fprintf(fp_v4, "\n");
    }
    fclose(fp_v4);

    v5_file = "drone_datafile/txtfile/data_v5.txt";
    fp_v5 = fopen(v5_file, "w");
    for (i = 0; i < size[4] - 1; i++)
    {
        fprintf(fp_v5, "%f %f\n", new_p[cir[4][i]].x, new_p[cir[4][i]].y);
        fprintf(fp_v5, "%f %f\n", new_p[cir[4][i + 1]].x, new_p[cir[4][i + 1]].y);
        fprintf(fp_v5, "\n");
    }
    fclose(fp_v5);

    /**************************** 各避難所の物資要求量をランダムに決定 *****************************************/
    double lambda_re = 1; // 物資要求量変化のラムダ
    // 各避難所の物資量を生成
    for (int i = 1; i < N; i++)
    {
        // new_p[i].re_req = generate_normal(MEAN, STD_DEV); // 各避難所の必要物資量をランダムに生成(正規分布)
        new_p[i].re_req = MEAN;                      // 初期値MEAN(=50)に設定
        new_p[i].re_req += (int)rand_exp(lambda_re); // 指数分布による増加分
        new_p[i].re_req_sum += new_p[i].re_req;      // 各避難所の総必要物資量
    }

    // 結果を表示
    int sum_re = 0; // 物資総要求量
    printf("避難所ごとの必要物資量:\n");
    for (int i = 1; i < N; i++)
    {
        sum_re += new_p[i].re_req;
        printf("避難所 %d: %d\n", i, new_p[i].re_req);
    }
    printf("物資総要求量：%d\n", sum_re);

    /********************************************************************　シミュレーション（これより上を「titibumodel_douromou_kettei2.c」からコピペ）　*************************************************************************************/
    /******************GNUPLOT**************************/
    FILE *gp, *fp;

    // 各避難所、配送センターの座標をプロットするためにファイルに書き込む

    data_file = "drone_datafile/txtfile/data.txt";
    fp = fopen(data_file, "w");
    for (i = 0; i < N; i++)
    {
        fprintf(fp, "%d\t%lf\t%lf\n", i, p[i].x, p[i].y);
    }
    fclose(fp);
    // #if 0
    //  gnuplotの設定
    /*
        gp = popen("gnuplot -persist", "w");
        fprintf(gp, "set xrange [0:10]\n");
        fprintf(gp, "set yrange [0:10]\n");
        fprintf(gp, "set size square\n");
        fprintf(gp, "unset key\n");

        fprintf(gp, "set term gif animate delay 5 optimize size 640,480\n"); //
        fprintf(gp, "set output 'drone_datafile/test.gif'\n");

        // ラベルの表示
        for (i = 0; i < N; i++)
        {
            fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
        }
            */

    /************************************シミュレーション******************************************/

    double total_t = 0;                                  // シミュレーションの合計時間
    double termination_t = 500000;                       // シミュレーション強制終了時間
    double r_velo = 360;                                 // 速度の逆数
    double part_t[M] = {0};                              // 二点間の経過時間
    int current[M] = {0};                                // 始点
    int target[M];                                       // 終点
    double d[M];                                         // ２点間の距離
    double n_sin[M];                                     // サイン
    double n_cos[M];                                     // コサイン
    double n_tan[M];                                     // タンジェント
    int ind[M] = {0};                                    // targetのindex
    double time_span = 10;                               // 増加時間
    double stay_t[M] = {0};                              // 避難所待機時間カウンター
    double stay = 1800;                                  // 避難所,配送センターでの物資積載時間
    double dis_stay_t[M] = {0};                          // 配送センター待機時間カウンター
    double t_wait = 20000;                               // additional waiting time
    double dis_stay = stay + t_wait;                     // 配送センターでの待機時間
    double lambda_g = 0.25;                              // 配送センターへの物資到着率
    double lambda_i = 0.25;                              // 各避難所での情報生成率
    double lambda_i_med = 0.15;                          // 各避難所での薬の情報性成立
    double poisson_re_total = rand_exp(lambda_g) * 3600; // 物資の到着間隔
    double poisson_re_count = 0;
    double poisson_inf_total = rand_exp(lambda_i) * 3600; // 情報の到着
    double poisson_inf_count = 0;
    double poisson_Medinf_total = rand_exp(lambda_i_med) * 3600; // 薬の情報の到着
    double poisson_Medinf_count = 0;
    int re_load_num = 10 * MEAN;   // 配送センターで一度に積載する物資の数
    int re_finish_num = 10 * MEAN; // シミュレーション終了物資量(避難所に物資届ける回数×MEAN)
    int ind_relief[M];             // 物資を避難所に下ろすindex :配送車1なら1,2,3,,,
    int re_wait_flag[M] = {FALSE}; // 配送センターの物資存在フラグ
    // 平均情報到着間隔
    double current_inf_arrival_time[N] = {0}; // 各避難所の情報到着時刻
    FILE *fp_inf_interval;
    inf_interval_file = "drone_datafile/txtfile/inf_interval.txt";
    int count = 0;
    // 平均情報遅延間隔
    FILE *fp_inf_delay;
    inf_delay_file = "drone_datafile/txtfile/inf_delay.txt";
    // 平均情報遅延時間(各避難所)
    FILE *fp_inf_delay_part;
    inf_delay_part_file = "drone_datafile/txtfile/inf_delay_part.txt";
    int analyse_num = 1; // 分析する避難所番号(ヒストグラム)
    // 平均物資到着間隔
    double current_re_arrival_time[N] = {0};
    FILE *fp_re_interval;
    re_interval_file = "drone_datafile/txtfile/re_interval.txt";
    // 最終的に求まった平均情報遅延時間をファイルに格納（シミュレーションごとに格納していく）
    FILE *fp_Etd_data;
    Etd_data_file = "drone_datafile/txtfile/Etd_data.txt";
    FILE *fp_Eti_data;
    Eti_data_file = "drone_datafile/txtfile/Eti_data.txt";
    FILE *fp_Etg_data;
    Etg_data_file = "drone_datafile/txtfile/Etg_data.txt";
    // 平均薬情報遅延間隔
    FILE *fp_Medinf_delay;
    Medinf_delay_file = "drone_datafile/txtfile/Medinf_delay.txt";
    // 医療品の配送遅延時間
    FILE *fp_Med_re_delay;
    Med_re_delay_file = "drone_datafile/txtfile/Med_re_delay.txt";
    // 医療品情報の収集遅延時間（避難所から配送車への情報共有の遅延時間限定）
    FILE *fp_Medinf_collect_delay;
    Medinf_collect_delay_file = "drone_datafile/txtfile/Medinf_collect_delay.txt";
    // 医療品の配送遅延時間（要求情報発生が回収されてから実際に医療品が避難所へ届けられるまでの遅延時間）
    FILE *fp_Med_re_collect_to_delivery_delay;
    Med_re_collect_to_delivery_delay_file = "drone_datafile/txtfile/Med_re_collect_to_delivery_delay.txt";

    // 平均配送車マッチング数
    double meet_vehicle_num[M] = {0};
    double max_num[M] = {0};
    double total_meet_vehicle_num[M] = {0};
    double total_count[M] = {0};
    int re_departure_flag[M] = {FALSE}; // 物資を受け取って出発するフラグ
    int re_depature_count[M] = {0};     // 1800->0になるまで待機
    int vehicle_merge_flag = FALSE;     // すべての配送車が合流
    // 平均周回時間
    double total_time_trip[M] = {0};
    int total_vehicle_num = 0; // 巡回が終了して配送センターに到着した配送車数
    double trip_count[M] = {0};
    double previous_time[M] = {0};
    // 物資数が100の避難所のカウンター
    int relief_count;
    // ドローンに関する変数
    double r_d_velo = 180;                // 配送車の2倍
    double v_d_ratio = r_velo / r_d_velo; // 配送車の速度とドローンの速度の比率
    double d_d[D];
    double d_n_sin[D];
    double d_n_cos[D];
    int shelter_num[D] = {0};                                                        // ドローンが出発する避難所
    int current_returnnum[D] = {0};                                                  // ドローンが一周して帰ってくる配送車番号
    int change_follow = 0;                                                           // 配送センターでドローンのfollowする配送車を切り替える
    int total_inf_num = 0;                                                           // 避難所に共有される情報の総数
    double flight_time[D] = {0};                                                     // ドローンの飛行時間(目的地まで1飛行あたり)
    double total_flight_time[D] = {0};                                               // flight_timeの合計値
    double flight_count[D] = {0};                                                    // ドローンの飛行回数
    double charge_constant = 2;                                                      // ドローンの充電時間の定数倍(２なら飛行時間の２倍充電時間が掛かる)
    double m_v_f = 60 * 14.5;                                                        // ドローンの配送車間の平均飛行時間（分）mean_value_flight 14分
    double flight_time_lag = removeOnePlace((m_v_f * M + m_v_f * 2 * (M - 1)) / SD); // ドローンの飛行開始時間の時間差(60秒 * 分)導出用

    // ドローンの合流点を決める際のパラメータ
    int answerFlag = 0;    // 関数の戻り値を格納
    double d_sol[D] = {0}; // ドローンが配送車に追いつけない場合の処理用
    double sin_sol[D] = {0};
    double cos_sol[D] = {0};
    double d_d_v[D] = {0}; // ドローンと配送車の向かう避難所の間の距離
    double xv, yv, xs, ys, xs_dash, ys_dash;

    // 各避難所のTD
    int total_td[N] = {0};
    int total_td_count[N] = {0};
    // 各避難所のTG
    int total_tg[N] = {0};
    int total_tg_count[N] = {0};
    // 各避難所のTI
    int total_ti[N] = {0};
    int total_ti_count[N] = {0};

    // targetのindex初期化
    for (i = 0; i < M; i++)
    {
        ind[i] = 1;
    }

    // targetの初期化
    for (i = 0; i < M; i++)
    {
        target[i] = cir[i][1];
    }

    // 配送車の物資を初期化
    for (i = 0; i < M; i++)
    {
        v[i].re = 10 * MEAN; // それぞれの避難所にMEAN個ずつ物資を届ける
    }

    // ind_reliefの初期化
    for (i = 0; i < M; i++)
    {
        ind_relief[i] = 10 * i + 1;
    }

    // 配送センターの物資の初期値
    new_p[0].re = 0;

    // ドローンの出発避難所
    shelter_num[0] = 1;

    // current_retnum初期化
    for (i = 0; i < SD; i++)
    {
        current_returnnum[i] = 0;
    }

    fp_inf_interval = fopen(inf_interval_file, "w"); // 平均情報到着間隔ファイルのオープン
    fp_inf_delay = fopen(inf_delay_file, "w");       // 平均情報遅延間隔ファイルのオープン
    fp_inf_delay_part = fopen(inf_delay_part_file, "w");
    fp_re_interval = fopen(re_interval_file, "w");                                           // 平均物資到着間隔のファイルオープン
    fp_Medinf_delay = fopen(Medinf_delay_file, "w");                                         // 薬情報の遅延間隔ファイルのオープン
    fp_Med_re_delay = fopen(Med_re_delay_file, "w");                                         // 医療品の配送遅延間隔ファイルのオープン
    fp_Medinf_collect_delay = fopen(Medinf_collect_delay_file, "w");                         // 医療品情報の収集遅延間隔ファイルのオープン
    fp_Med_re_collect_to_delivery_delay = fopen(Med_re_collect_to_delivery_delay_file, "w"); // 医療品の収集から配送への遅延間隔ファイルのオープン

    /************************************ ループ処理 ***********************************************************/
    while (1)
    {
        for (i = 0; i < M; i++)
        {
            d[i] = sqrt(pow(new_p[target[i]].x - new_p[current[i]].x, 2) + pow(new_p[target[i]].y - new_p[current[i]].y, 2));
            n_sin[i] = (new_p[target[i]].x - new_p[current[i]].x) / d[i];
            n_cos[i] = (new_p[target[i]].y - new_p[current[i]].y) / d[i];
            n_tan[i] = n_sin[i] / n_cos[i];
        }

#if 0
        if (total_t >= 30000 && total_t <= 65000)
        {
            if ((int)(total_t) % 50 == 0)
            { // 50sごとに描画

                // ラベルの表示
                for (j = 0; j < N; j++)
                {
                    // fprintf(gp, "set label %d at first %f,%f '%d / %d'\n", j + 51, new_p[j].x + 0.1, new_p[j].y + 0.1, new_p[j].re, new_p[j].re_req_sum);
                }

                // 薬の情報ラベル(薬の情報を持っている避難所を表示)
                for (i = 0; i < N; i++)
                {
                    if (new_p[i].i_med_ptr[i] != 0)
                    {
                        fprintf(gp, "set label %d at first %f,%f 'Med:%d'\n", i + 151, new_p[i].x - 0.1, new_p[i].y - 0.1, new_p[i].i_med_ptr[i]);
                    }
                }

                // 薬の情報ラベル(薬の情報を持っている車を表示)
                int c_label = 0; // ラベル表示かぶらないようにするためのカウンター
                for (i = 0; i < M; i++)
                {
                    for (j = 0; j < N; j++)
                    {
                        if (v[i].i_med_ptr[j] != 0)
                        {
                            fprintf(gp, "set label %d at first %f,%f ':%d'\n", c_label + 251 + 40 * i, v[i].x + 0.55 * c_label, v[i].y + 0.1, j);
                            c_label += 1;
                        }
                    }
                    c_label = 0; // 初期化
                }

                // ドローン8台
                fprintf(gp, "set title 't = %f'\n", total_t);
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'gold','-' pt 5 lt rgbcolor'dark-turquoise'\n", new_data_file, new_ad_file);
                fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black'\n", new_data_file, new_ad_file);
                fprintf(gp, "%f %f\n", v[0].x, v[0].y);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", v[1].x, v[1].y);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", v[2].x, v[2].y);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", v[3].x, v[3].y);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", v[4].x, v[4].y);
                fprintf(gp, "e\n");
            }
        }
#endif

        /**************配送車の座標更新*****************/
        for (i = 0; i < M; i++)
        {
            // 待機中でないなら座標更新
            if (stay_t[i] == 0 && dis_stay_t[i] == 0 && re_wait_flag[i] == FALSE)
            {
                v[i].x = new_p[current[i]].x + n_sin[i] * part_t[i] / r_velo;
                v[i].y = new_p[current[i]].y + n_cos[i] * part_t[i] / r_velo;
            }
            else if (stay_t[i] != 0 && dis_stay_t[i] == 0)
            { // 避難所で待機中なら待機時間カウンターを減らす
                stay_t[i] -= time_span;
            }
            else if (stay_t[i] == 0 && dis_stay_t[i] != 0)
            { // 配送センターで待機中なら待機時間カウンターを減らす
                dis_stay_t[i] -= time_span;
            }
        }

        /*****************各配送車において始点から終点へ到達したときの処理****************************/
        for (i = 0; i < M; i++)
        {
            if ((ind[i] == size[i] - 1 && n_cos[i] < 0 && v[i].y < new_p[cir[i][0]].y) || (ind[i] == size[i] - 1 && n_cos[i] > 0 && v[i].y > new_p[cir[i][0]].y))
            { // ①一周したら、ループを初期化してもう一周
                ind[i] = 1;
                current[i] = 0;
                target[i] = cir[i][ind[i]];
                v[i].x = new_p[current[i]].x;
                v[i].y = new_p[current[i]].y;
                part_t[i] = 0;
                dis_stay_t[i] = dis_stay; // 配送センターでの待機時間セット
                // printf("配送車%dの到着時間: %fhour : %fs\n", i + 1, total_t / 3600, total_t);
                //  周回時間の計算
                total_time_trip[i] += total_t - previous_time[i];
                previous_time[i] = total_t;
                trip_count[i] += 1;
                // 周回時間からadditionalwaitingtime算出
                total_vehicle_num += 1; // 配送センターに到着

                // ind_reliefの初期化
                ind_relief[i] = i * 10 + 1;

                // 配送センターから物資を積載
                if (new_p[0].re > 0)
                { // 配送センターに物資があれば積載(到着したときに配送センターに物資存在した場合に一度だけ処理)
                    re_wait_flag[i] = FALSE;
                    v[i].re += re_load_num;
                    new_p[0].re -= re_load_num;
                }
                else
                {                           // 配送センターに物資がなければ
                    re_wait_flag[i] = TRUE; // 物資が届くまで待機
                }

                vehicle_merge_flag = FALSE; // 配送車合流フラグを0にしておく
            }
            else if ((n_cos[i] < 0 && v[i].y < new_p[target[i]].y) || (n_cos[i] > 0 && v[i].y > new_p[target[i]].y))
            { // ③それ以外において、targetに到達したらcurrentとtarget更新
                ind[i] += 1;
                current[i] = target[i];
                target[i] = cir[i][ind[i]];
                v[i].x = new_p[current[i]].x;
                v[i].y = new_p[current[i]].y;
                part_t[i] = 0;

                // 物資を下ろす
                if (current[i] == ind_relief[i] && ind_relief[i] <= (i + 1) * 10)
                {
                    stay_t[i] = stay; // 避難所待機時間をセット

                    if (v[i].re > new_p[current[i]].re_req) // 配送車が避難所に物資を要求分届けることができるとき
                    {
                        // パターン１:前半の避難所から物資必要量すべて運搬
                        /*
                        v[i].re -= new_p[current[i]].re_req;                  // 配送車の物資減少
                        new_p[ind_relief[i]].re += new_p[current[i]].re_req;  // 避難所の物資増加
                        new_p[current[i]].re_deli = new_p[current[i]].re_req; // 避難所に届けられた物資量記録
                        ind_relief[i] += 1;
                        */

                        // パターン２:すべての避難所に平等にMEANだけ物資運搬
                        v[i].re -= MEAN;                  // 配送車の物資減少
                        new_p[ind_relief[i]].re += MEAN;  // 避難所の物資増加
                        new_p[current[i]].re_deli = MEAN; // 避難所に届けられた物資量記録
                        ind_relief[i] += 1;
                    }
                    else // 配送車上の物資量では避難所への物資量が不足する場合
                    {
                        new_p[ind_relief[i]].re += v[i].re;  // 避難所の物資増加
                        new_p[current[i]].re_deli = v[i].re; // 避難所に届けられた物資量記録
                        v[i].re -= v[i].re;                  // 配送車の物資減少
                        ind_relief[i] += 1;
                    }

                    // 物資到着間隔をファイルに書き込む
                    fprintf(fp_re_interval, "%f\n", total_t - current_re_arrival_time[current[i]]);
                    total_tg[current[i]] += total_t - current_re_arrival_time[current[i]];
                    total_tg_count[current[i]] += 1;
                    current_re_arrival_time[current[i]] = total_t;

                    /**************** 避難所 -> 配送車 (各避難所において)************/
                    for (j = 0; j < N; j++)
                    {
                        // 避難所情報の交換
                        if (new_p[current[i]].i_ptr[j] > v[i].i_ptr[j])
                        { // 避難所の情報配列に要素が入っているならその分の情報を配送車に渡す
                            if (i == 0)
                            {
                                // printf("配送車：%d 避難所：%d i_ptr[%d]\n",i+1,current[i],j);
                            }
                            for (k = v[i].i_ptr[j]; k < new_p[current[i]].i_ptr[j]; k++)
                            {

                                v[i].inf[j][v[i].i_ptr[j]] = new_p[current[i]].inf[j][k];
                                v[i].i_ptr[j] += 1;
                                // 配列の容量オーバー
                                if (v[i].i_ptr[j] == I_SIZE)
                                {
                                    printf("配列要素数オーバー\n");
                                    break;
                                }
                            }
                        }

                        // 避難所情報（薬の情報）の交換
                        if (new_p[current[i]].i_med_ptr[j] > v[i].i_med_ptr[j])
                        {
                            for (k = v[i].i_med_ptr[j]; k < new_p[current[i]].i_med_ptr[j]; k++)
                            {
                                v[i].inf_med[j][v[i].i_med_ptr[j]][0] = new_p[current[i]].inf_med[j][k][0]; // 情報の生成時間
                                v[i].inf_med[j][v[i].i_med_ptr[j]][1] = new_p[current[i]].inf_med[j][k][1]; // 薬の緊急度
                                v[i].i_med_ptr[j] += 1;

                                // 医療品の配送先をキューに保存
                                v[i].Med_delivery_queue[v[i].queue_ptr] = current[i]; // 医療品の配送先をキューに格納
                                v[i].queue_ptr += 1;                                  // キューのポインタを進める
                                if (v[i].queue_ptr == QUEUE_SIZE)
                                {
                                    printf("キューの要素数オーバー\n");
                                    break;
                                }

                                // debug
                                // printf("%d:%d********p[%d]toV[%d]%lf\n", v[i].i_med_ptr[j] - 1, new_p[current[i]].i_med_ptr[j], current[i], i, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]);

                                // ファイルへの書き込み
                                // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf new_p[%d]->v[%d]\n", total_t, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0], current[i], i);
                                fprintf(fp_Medinf_delay, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間
                                // fprintf(fp_Medinf_collect_delay, "t=%lf generate_time:%lf new_p[%d]->v[%d]\n", total_t, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0], current[i], i);
                                fprintf(fp_Medinf_collect_delay, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                                v[i].inf_med[j][v[i].i_med_ptr[j] - 1][2] = total_t;
                            }
                        }
                    }

                    /**************************医療品の避難所への配達処理***********************************/
                    if (v[i].Med_re > 0 && v[i].i_med_ptr[current[i]] > 0 && v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][3] != TRUE && current[i] == v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]) // 配送車が集積所で物資を積載してまた戻って来たなら
                    {
                        // 医療品の配達
                        v[i].Med_re -= 1;
                        v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][3] = TRUE; // 配送車が医療品を届けたことを記録
                        printf("配送車%d 避難所%d 医療品%d\n", i, current[i], v[i].Med_re);
                        v[i].queue_Notdelivery_ptr += 1; // 配達が完了したキュー内の避難所のポインタを進める

                        // ファイルへの書き込み処理
                        fprintf(fp_Med_re_delay, "%lf\n", total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][0]);
                        // fprintf(fp_Med_re_delay, "t=%lf v[%d] -> new_p[%d] : %lf\n", total_t, i, current[i], total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][0]);
                        fprintf(fp_Med_re_collect_to_delivery_delay, "%lf\n", total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][2]); // 医療品の収集から配送への遅延時間
                        // fprintf(fp_Med_re_collect_to_delivery_delay, "t=%lf v[%d] -> new_p[%d] : %lf\n", total_t, i, current[i], total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][2]); // 医療品の収集から配送への遅延時間
                        // fprintf(fp_Med_re_collect_to_delivery_delay, "t=%lf v[%d] -> new_p[%d] : 情報が回収された時間 %lf\n", total_t, i, current[i], v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][2]); // 医療品の収集から配送への遅延時間
                    }

                    /***************** 配送車 -> 避難所（各避難所において）*********/

                    count = 0;
                    for (j = 0; j < N; j++)
                    {
                        if (v[i].i_ptr[j] > new_p[current[i]].i_ptr[j])
                        {
                            // 平均情報到着間隔のデータをファイルに格納
                            if (count == 0)
                            {
                                fprintf(fp_inf_interval, "%f\n", total_t - current_inf_arrival_time[current[i]]);
                                total_ti[current[i]] += total_t - current_inf_arrival_time[current[i]];
                                total_ti_count[current[i]] += 1;
                                current_inf_arrival_time[current[i]] = total_t;
                            }
                            count++;

                            for (k = new_p[current[i]].i_ptr[j]; k < v[i].i_ptr[j]; k++)
                            {

                                new_p[current[i]].inf[j][k] = v[i].inf[j][k];

                                // 情報遅延間隔ファイルへの書き込み
                                fprintf(fp_inf_delay, "%d\n", (int)total_t - v[i].inf[j][k]);
                                // 避難所1
                                if (current[i] == analyse_num)
                                {
                                    // fprintf(fp_inf_delay_part, "%d\n", (int)total_t - v[i].inf[j][k]);
                                    fprintf(fp_inf_delay_part, "%f\n", (total_t - (double)v[i].inf[j][k]) / 3600);
                                }

                                // 各避難所のtd求める
                                total_td[current[i]] += (int)total_t - new_p[current[i]].inf[j][k];
                                total_td_count[current[i]] += 1;
                                new_p[current[i]].i_ptr[j] += 1; // ポインタ+1

                                // 配列の容量オーバー
                                if (new_p[current[i]].i_ptr[j] == I_SIZE)
                                {
                                    printf("配列要素数オーバー\n");
                                    break;
                                }

                                total_inf_num += 1; // 避難所への情報加算
                            }
                        }
                    }
                }
            }
        }

        /**************** 配送センターにすべての配送車が集まった場合 ********************/
        if (total_vehicle_num == M)
        {
            // printf("t_wait: %f\n", findMax(total_time_trip, M) - findMin(total_time_trip, M));

            total_vehicle_num = 0;

            /**************************************** 各避難所の要求物資量と実際に届けられた物資の比較結果表示 **************************************************/
            // printf("%-8s %-8s %-8s %-8s %-8s %-8s %-8s\n", "避難所", "必要物資量", "配達物資量", "物資不足量", "総物資必要量", "総物資配達量", "総物資不足量");
            for (int i = 1; i < N; i++)
            {
                // printf("%-8d %-8d %-8d    %-8d   %-8d         %-8d %-8d\n", i, new_p[i].re_req, new_p[i].re_deli, new_p[i].re_req - new_p[i].re_deli, new_p[i].re_req_sum, new_p[i].re, new_p[i].re_req_sum - new_p[i].re);
            }

            // 各避難所の要求物資量を更新
            for (int i = 1; i < N; i++)
            {
                // new_p[i].re_req = generate_normal(MEAN, STD_DEV); // 各避難所の必要物資量をランダムに生成(正規分布)
                new_p[i].re_req += (int)rand_exp(lambda_re); // 指数分布による増加分
                new_p[i].re_req_sum += new_p[i].re_req;      // 各避難所の総必要物資量
            }

            // 医療品の積載について
            for (int i = 0; i < M; i++)
            {
                for (j = 1; j < N; j++)
                {
                    if (v[i].i_med_ptr[j] > 0 && v[i].inf_med[j][v[i].i_med_ptr[j] - 1][3] != TRUE && j <= (i + 1) * 10 && j >= i * 10 + 1)
                    {
                        v[i].Med_re += 1;
                        printf("配送車%d:避難所[%d]への物資積載\n", i, j);
                    }
                }
            }
        }

        /**************************************** 情報交換に関する処理 ********************************************/

        /********************* 配送車 -> 配送センター（配送センターにおいて） ***********************/

        for (i = 0; i < M; i++)
        {
            if (fabs(v[i].x - new_p[0].x) < 0.001 && fabs(v[i].y - new_p[0].y) < 0.001 && total_t != 0)
            {
                for (j = 0; j < N; j++)
                {
                    if (v[i].i_ptr[j] > new_p[0].i_ptr[j])
                    {
                        for (k = new_p[0].i_ptr[j]; k < v[i].i_ptr[j]; k++)
                        {
                            new_p[0].inf[j][k] = v[i].inf[j][k];
                            new_p[0].i_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (new_p[0].i_ptr[j] == I_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }
            }
        }

        /********************* 配送センター -> 配送車（配送センターにおいて）****************************/

        for (i = 0; i < M; i++)
        {
            if (fabs(v[i].x - new_p[0].x) < 0.001 && fabs(v[i].y - new_p[0].y) < 0.001 && total_t != 0)
            {
                for (j = 0; j < N; j++)
                {
                    if (new_p[0].i_ptr[j] > v[i].i_ptr[j])
                    {
                        for (k = v[i].i_ptr[j]; k < new_p[0].i_ptr[j]; k++)
                        {
                            v[i].inf[j][k] = new_p[0].inf[j][k];
                            v[i].i_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (v[i].i_ptr[j] == I_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }
            }
        }

        /********************* 配送車 -> 配送車 (配送センターにおいて)*****************************/

        // 避難所情報の交換
        for (i = 0; i < M; i++)
        {
            for (j = 0; j < M; j++)
            {
                if (fabs(v[i].x - new_p[0].x) < 0.001 && fabs(v[j].x - new_p[0].x) < 0.001 && i != j)
                {
                    for (k = 0; k < N; k++)
                    {
                        if (v[i].i_ptr[k] > v[j].i_ptr[k])
                        {
                            for (m = v[j].i_ptr[k]; m < v[i].i_ptr[k]; m++)
                            {
                                v[j].inf[k][m] = v[i].inf[k][m];
                                v[j].i_ptr[k] += 1;
                                // 配列の容量オーバー
                                if (v[j].i_ptr[k] == I_SIZE)
                                {
                                    printf("配列要素数オーバー\n");
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        // 薬の情報配列の交換
        for (i = 0; i < M; i++)
        {
            for (j = 0; j < M; j++)
            {
                if (fabs(v[i].x - new_p[0].x) < 0.001 && fabs(v[j].x - new_p[0].x) < 0.001 && i != j)
                {
                    for (k = 0; k < N; k++)
                    {
                        if (v[i].i_med_ptr[k] > v[j].i_med_ptr[k])
                        {
                            for (m = v[j].i_med_ptr[k]; m < v[i].i_med_ptr[k]; m++)
                            {
                                // debug
                                // printf("%d:%d***************************(i,j,k,m)=(%d,%d,%d,%d) %lf\n", v[j].i_med_ptr[k], v[i].i_med_ptr[k], i, j, k, m, v[i].inf_med[k][m][0]);

                                v[j].inf_med[k][m][0] = v[i].inf_med[k][m][0];
                                v[j].inf_med[k][m][1] = v[i].inf_med[k][m][1];
                                v[j].i_med_ptr[k] += 1;

                                // ファイルへの書き込み
                                // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf v[%d]->v[%d]\n", total_t, v[i].inf_med[k][m][0], i, j);
                                fprintf(fp_Medinf_delay, "%lf\n", total_t - v[i].inf_med[k][m][0]); // 生成されてから配送車で回収されるまでの遅延時間

                                // 配列の容量オーバー
                                if (v[j].i_med_ptr[k] == Y_SIZE)
                                {
                                    printf("配列要素数オーバー\n");
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        /*************************** ドローン -> 配送車(ドローンが配送車にいるとき(followモード)または充電中のときは 配送車の情報をドローンにコピー) *****************************/

        /***************************************** 避難所 -> 配送車（避難所で待機中にその避難所で情報が生成されたときの処理） *****************************************************/
        for (i = 0; i < M; i++)
        {
            if ((int)stay_t[i] != 0 && fabs(v[i].x - new_p[current[i]].x) < 0.001 && fabs(v[i].y - new_p[current[i]].y) < 0.001)
            {
                for (j = 0; j < N; j++)
                {
                    if (new_p[current[i]].i_ptr[j] > v[i].i_ptr[j])
                    { // 避難所の情報配列に要素が入っているならその分の情報を配送車に渡す
                        if (i == 0)
                        {
                            // printf("配送車：%d 避難所：%d i_ptr[%d]\n",i+1,current[i],j);
                        }
                        for (k = v[i].i_ptr[j]; k < new_p[current[i]].i_ptr[j]; k++)
                        {
                            v[i].inf[j][v[i].i_ptr[j]] = new_p[current[i]].inf[j][k];
                            v[i].i_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (v[i].i_ptr[j] == I_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }
            }
        }

        /*****************避難所への情報の到着******************/
        if (total_t != 0 && (int)(total_t) % ((int)(poisson_inf_total) - (int)(poisson_inf_total) % 10) == 0)
        {
            poisson_inf_total += rand_exp(lambda_i) * 3600;
            poisson_inf_count += 1;

            for (i = 1; i < N; i++)
            {
                new_p[i].inf[i][new_p[i].i_ptr[i]] = (int)total_t;
                new_p[i].i_ptr[i] += 1; //
            }
        }

        /*****************避難所への情報の到着(薬の情報について)******************/
        if (total_t != 0 && (int)(total_t) % ((int)(poisson_Medinf_total) - (int)(poisson_Medinf_total) % 10) == 0)
        {
            poisson_Medinf_total += rand_exp(lambda_i_med) * 3600;
            poisson_Medinf_count += 1;

            int med_num = get_random_int(1, N - 1); // 薬の情報が発生する避難所番号をランダムに決定

            printf("避難所[%d]で薬情報発生\n", med_num);

            new_p[med_num].inf_med[med_num][new_p[med_num].i_med_ptr[med_num]][0] = total_t; // 情報の生成時間を格納
            new_p[med_num].inf_med[med_num][new_p[med_num].i_med_ptr[med_num]][1] = 60 * 60; // 薬の緊急時間を生成
            new_p[med_num].i_med_ptr[med_num] += 1;                                          // ポインタ更新

            // 表示確認用
            /*
                        printf("薬の情報配列表示 (t=%lfs)\n", total_t);
                        for (i = 0; i < N; i++)
                        {
                            if (i == med_num) // 避難所１の薬情報配列
                            {
                                for (j = 0; j < N; j++)
                                {
                                    for (k = 0; k < 10; k++)
                                    {
                                        printf("%.1lf ", new_p[i].inf_med[j][k][0]);
                                    }
                                    printf("ptr:%d\n", new_p[i].i_med_ptr[j]);
                                }
                            }
                        }*/
        }

        /*****配送センターへの物資の到着******/
        if (total_t != 0 && (int)(total_t) % ((int)(poisson_re_total) - (int)(poisson_re_total) % 10) == 0)
        {
            poisson_re_total += rand_exp(lambda_g) * 3600;
            poisson_re_count += 1;
            new_p[0].re += 50 * MEAN; // 物資到着（50箇所の避難所分の物資）

            for (i = 0; i < M; i++)
            {
                if (re_wait_flag[i] == TRUE)
                { // 配送車が物資待機中なら物資を積載する
                    v[i].re += re_load_num;
                    new_p[0].re -= re_load_num;
                    re_wait_flag[i] = FALSE;
                    re_departure_flag[i] = TRUE;
                    re_depature_count[i] = (int)stay;
                }
            }
        }

        // 物資を受け取ったあとに出発する配送車の場合
        for (i = 0; i < M; i++)
        {
            if (re_departure_flag[i] == TRUE)
            {
                dis_stay_t[i] = dis_stay;
                re_departure_flag[i] = FALSE;
            }
        }

        /*******************　配送車が出会う配送車の数の処理　（すべての配送車が配送センターに集まったらその時点で出発）　************************/
        // 初期化
        int vehicle_num = 0;
        for (i = 0; i < M; i++)
        {
            meet_vehicle_num[i] = 0;
        }
        // 各配送車の配送センターでの出会った配送車の数を導出
        for (i = 0; i < M; i++)
        {
            for (j = 0; j < M; j++)
            {
                if (i != j && fabs(v[i].x - new_p[0].x) < 0.001 && fabs(v[j].x - new_p[0].x) < 0.001 && total_t != 0)
                {
                    meet_vehicle_num[i] += 1;
                }
            }
            if (meet_vehicle_num[i] > max_num[i])
            {
                max_num[i] = meet_vehicle_num[i];
            }
        }
        // 配送センターに存在する配送車の合計数
        for (i = 0; i < M; i++)
        {
            if (v[i].x == new_p[0].x)
            {
                vehicle_num += 1;
            }
        }
        if (total_t != 0 && vehicle_num == 5 && vehicle_merge_flag == FALSE && re_wait_flag[0] == FALSE && re_wait_flag[1] == FALSE && re_wait_flag[2] == FALSE && re_wait_flag[3] == FALSE && re_wait_flag[4] == FALSE)
        {

            for (i = 0; i < M; i++)
            {
                dis_stay_t[i] = stay; // すべての配送車が合流し、物資到着待ちでないなら同時に出発
            }
            // printf("合流時間：%f\n", total_t);

            vehicle_merge_flag = TRUE;
        }

        /**************************************** 配送車が配送センターから出発する ****************************************************/
        // 配送車が配送センターから出発する際におけるmeet_vehicle_numの処理と表示

        for (i = 0; i < M; i++)
        {
            if (total_t != 0 && fabs(v[i].x - new_p[0].x) < 0.001 && dis_stay_t[i] == 0 && re_wait_flag[i] == FALSE)
            {
                total_meet_vehicle_num[i] += max_num[i];
                total_count[i] += 1;
                max_num[i] = 0; // max_num初期化
            }
        }

        /************************終了条件********************************/
        relief_count = 0;

        // シミュレーション強制終了条件
        if (total_t >= termination_t)
        {
            printf("シミュレーション時間が規定を超過したため強制終了\n");
            break;
        }

        // 規定物資量を運搬し終えた避難所をカウント
        for (i = 1; i < N; i++)
        {
            if (new_p[i].re >= re_finish_num)
            { // 物資が規定個以上の避難所のカウント
                relief_count += 1;
            }
        }

        if (relief_count == 50) // すべての避難所に物資が行き渡ったら
        {
            // 平均配送車マッチング数を表示
            for (i = 0; i < M; i++)
            {
                if (total_count[i] != 0)
                {
                    printf("mean meet num 配送車%d: %f\n", i + 1, total_meet_vehicle_num[i] / total_count[i]);
                }
            }

            // 平均周回時間の表示
            for (i = 0; i < M; i++)
            {
                printf("平均周回時間 配送車%d: %f\n", i + 1, total_time_trip[i] / trip_count[i] / 3600);
            }

            printf("シミュレーション時間:%f\n", total_t);

            printf("配送センター reliefnum:%d\n", new_p[0].re);

            // 各避難所のtd
            printf("各避難所のE(TD)\n");
            for (i = 1; i < N; i++)
            {
                // printf("%f\n", (double)(total_td[i] / total_td_count[i]) / 3600);
            }

            // printf("ポアソン到着（物資）:%f\n", poisson_re_total / poisson_re_count / 3600);
            // printf("ポアソン到着（情報）:%f\n", poisson_inf_total / poisson_inf_count / 3600);

            break; // ループ終了
        }

        total_t += time_span; // 総合計時間を更新

        // 部分時間を更新(待機していないなら更新)
        for (i = 0; i < M; i++)
        {
            if (stay_t[i] == 0 && dis_stay_t[i] == 0 && re_wait_flag[i] == FALSE)
            {
                part_t[i] += time_span;
            }
        }
    }
    /*********************************** while文処理終了 ********************************************************************/

    fclose(fp_re_interval); // 平均物資到着間隔ファイルクローズ
    fclose(fp_inf_delay);   // 平均情報遅延時間ファイルクローズ
    fclose(fp_inf_delay_part);
    fclose(fp_inf_interval); // 平均情報到着間隔ファイルクローズ
    fclose(fp_Medinf_delay);
    fclose(fp_Med_re_delay);
    fclose(fp_Medinf_collect_delay);
    fclose(fp_Med_re_collect_to_delivery_delay);
    // pclose(gp);

    /*********平均値の導出**********/

    /********情報の共有割合導出***********/
    printf("避難所への情報共有数: %d\n", total_inf_num);

    /*********平均物資到着間隔***************/
    double value3 = 0;
    double sum3 = 0;
    double count3 = 0;
    fp_re_interval = fopen(re_interval_file, "r"); // 平均物資到着間隔のファイルオープン
    if (fp_re_interval == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_re_interval, "%lf", &value3) == 1)
    {
        sum3 += value3;
        count3++;
    }
    fclose(fp_re_interval); // 平均物資到着間隔ファイルクローズ

    if (count3 > 0)
    {
        double average3 = sum3 / count3;
        printf("平均物資到着間隔：%f\n", average3 / 3600);
        // 各シミュレーションごとのE(TG)のデータを格納する
        fp_Etg_data = fopen(Etg_data_file, "a+");
        fprintf(fp_Etg_data, "%f\n", average3 / 3600);
        fclose(fp_Etg_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /*****平均情報到着間隔*******/
    double value1 = 0;
    double sum1 = 0;
    double count1 = 0;

    fp_inf_interval = fopen(inf_interval_file, "r"); // 平均情報到着間隔ファイルのオープン
    if (fp_inf_interval == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_inf_interval, "%lf", &value1) == 1)
    {
        sum1 += value1;
        count1++;
    }
    fclose(fp_inf_interval); // 平均情報到着間隔ファイルクローズ

    if (count1 > 0)
    {
        double average1 = sum1 / count1;
        printf("平均情報到着間隔：%f\n", average1 / 3600);
        // 各シミュレーションごとのE(TI)のデータを格納する
        fp_Eti_data = fopen(Eti_data_file, "a+");
        fprintf(fp_Eti_data, "%f\n", average1 / 3600);
        fclose(fp_Eti_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /*******平均情報遅延*******/
    double value2 = 0;
    double sum2 = 0;
    double count2 = 0;
    double average2;

    fp_inf_delay = fopen(inf_delay_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_inf_delay == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_inf_delay, "%lf", &value2) == 1)
    {
        sum2 += value2;
        count2++;
    }
    fclose(fp_inf_delay); // 平均情報遅延時間ファイルクローズ

    if (count2 > 0)
    {
        average2 = sum2 / count2;
        printf("平均情報遅延時間：%f\n", average2 / 3600);
        // 各シミュレーションごとのE(TD)のデータを格納する
        fp_Etd_data = fopen(Etd_data_file, "a+");
        fprintf(fp_Etd_data, "%f\n", average2 / 3600);
        fclose(fp_Etd_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /*********平均情報遅延時間（各避難所）************/
    double value4 = 0;
    double sum4 = 0;
    double count4 = 0;
    double td_max = 0;
    double td_min = INF;

    fp_inf_delay_part = fopen(inf_delay_part_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_inf_delay_part == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_inf_delay_part, "%lf", &value4) == 1)
    {
        sum4 += value4;
        count4++;

        if (value4 > td_max)
        {
            td_max = value4;
        }
        if (value4 < td_min)
        {
            td_min = value4;
        }
    }
    fclose(fp_inf_delay_part); // 平均情報遅延時間ファイルクローズ

    if (count4 > 0)
    {
        double average4 = sum4 / count4;
        // printf("平均情報遅延時間（避難所%d）:%f\n", analyse_num, average4 / 3600);
        printf("平均情報遅延時間（避難所%d）:%f\n", analyse_num, average4);
        printf("MAX:%f\n", td_max);
        printf("min:%f\n", td_min);
    }
    else
    {
        printf("データがありません\n");
    }

    /******** 薬情報の平均情報遅延時間 *********/
    double value5 = 0;
    double sum5 = 0;
    double count5 = 0;
    double average5;

    fp_Medinf_delay = fopen(Medinf_delay_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_Medinf_delay == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_Medinf_delay, "%lf", &value5) == 1)
    {
        sum5 += value5;
        count5++;
    }
    fclose(fp_Medinf_delay); // 平均情報遅延時間ファイルクローズ

    if (count5 > 0)
    {
        average5 = sum5 / count5;
        printf("薬情報の平均情報遅延時間：%f [h]\n", average5 / 3600);
        // 各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_MedInf_data;
        char *Mean_MedInf_file = "drone_datafile/txtfile/Mean_Medinf_delay.txt";
        fp_Mean_MedInf_data = fopen(Mean_MedInf_file, "a+");
        fprintf(fp_Mean_MedInf_data, "%f\n", average5 / 3600);
        fclose(fp_Mean_MedInf_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /******** 薬の配送平均情報遅延時間 *********/
    double value6 = 0;
    double sum6 = 0;
    double count6 = 0;
    double average6;

    fp_Med_re_delay = fopen(Med_re_delay_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_Med_re_delay == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_Med_re_delay, "%lf", &value6) == 1)
    {
        sum6 += value6;
        count6++;
    }
    fclose(fp_Med_re_delay); // 平均情報遅延時間ファイルクローズ

    if (count6 > 0)
    {
        average6 = sum6 / count6;
        printf("薬の配送平均情報遅延時間：%f [h]\n", average6 / 3600);
        // 各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_Med_re_data;
        char *Mean_Med_re_file = "drone_datafile/txtfile/Mean_Med_re_delay.txt";
        fp_Mean_Med_re_data = fopen(Mean_Med_re_file, "a+");
        fprintf(fp_Mean_Med_re_data, "%f\n", average6 / 3600);
        fclose(fp_Mean_Med_re_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /******** 医療品情報の回収における平均情報遅延時間 *********/
    double value7 = 0;
    double sum7 = 0;
    double count7 = 0;
    double average7;

    fp_Medinf_collect_delay = fopen(Medinf_collect_delay_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_Medinf_collect_delay == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_Medinf_collect_delay, "%lf", &value7) == 1)
    {
        sum7 += value7;
        count7++;
    }
    fclose(fp_Medinf_collect_delay); // 平均情報遅延時間ファイルクローズ

    if (count7 > 0)
    {
        average7 = sum7 / count7;
        printf("医療品情報の回収における平均情報遅延時間：%f [h]\n", average7 / 3600);
        // 各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_Medinf_collect_data;
        char *Mean_Medinf_collect_delay_file = "drone_datafile/txtfile/Mean_Medinf_collect_delay.txt";
        fp_Mean_Medinf_collect_data = fopen(Mean_Medinf_collect_delay_file, "a+");
        fprintf(fp_Mean_Medinf_collect_data, "%f\n", average7 / 3600);
        fclose(fp_Mean_Medinf_collect_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /******  シミュレーション通してのE(TC)のヒストグラムを表示する ******/
    FILE *fp_Etc_histgram_data;
    char *Etc_histgram_file = "drone_datafile/txtfile/Etc_histgram_data.txt";
    fp_Etc_histgram_data = fopen(Etc_histgram_file, "a+");
    // Medinf_collect_delay.txtの内容をEtc_histgram_data.txtにコピー
    FILE *fp_src = fopen(Medinf_collect_delay_file, "r");
    if (fp_src != NULL && fp_Etc_histgram_data != NULL)
    {
        char buf[256];
        while (fgets(buf, sizeof(buf), fp_src) != NULL)
        {
            fputs(buf, fp_Etc_histgram_data);
        }
        fclose(fp_src);
    }
    fclose(fp_Etc_histgram_data);

    /******** 医療品情報の回収から配達における平均情報遅延時間 *********/
    double value8 = 0;
    double sum8 = 0;
    double count8 = 0;
    double average8;

    fp_Med_re_collect_to_delivery_delay = fopen(Med_re_collect_to_delivery_delay_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_Med_re_collect_to_delivery_delay == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_Med_re_collect_to_delivery_delay, "%lf", &value8) == 1)
    {
        sum8 += value8;
        count8++;
    }
    fclose(fp_Med_re_collect_to_delivery_delay); // 平均情報遅延時間ファイルクローズ

    if (count8 > 0)
    {
        average8 = sum8 / count8;
        printf("医療品情報の回収から配達までにおける平均情報遅延時間：%f [h]\n", average8 / 3600);
        // 各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_Med_re_collect_to_delivery_data;
        char *Mean_Med_re_collect_to_derivery_delay_file = "drone_datafile/txtfile/Mean_Med_re_collect_to_delivery_delay.txt";
        fp_Mean_Med_re_collect_to_delivery_data = fopen(Mean_Med_re_collect_to_derivery_delay_file, "a+");
        fprintf(fp_Mean_Med_re_collect_to_delivery_data, "%f\n", average8 / 3600);
        fclose(fp_Mean_Med_re_collect_to_delivery_data);
    }
    else
    {
        printf("データがありません\n");
    }

    /********************************************************************　シミュレーション終了　**************************************************************************************************/
    // #endif
    /*************************** gnuplot表示 *************************************/

    // 各避難所、配送センターの座標をプロットするためにファイルに書き込む

    data_file = "drone_datafile/txtfile/data.txt";
    fp = fopen(data_file, "w");
    for (i = 0; i < N; i++)
    {
        fprintf(fp, "%d\t%lf\t%lf\n", i, p[i].x, p[i].y);
    }
    fclose(fp);

    /************************ 点プロット(番号更新前) ********************************************/
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set parametric\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/zahyou.png'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, p[i].x + 0.1, p[i].y + 0.1, i);
    }

    // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7 lt rgbcolor'black'\n", data_file);
    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", data_file, R, R);

    pclose(gp);

    /************************ 点プロット(中心市街地のみ拡大) ********************************************/
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [%f/2-%f:%f/2+%f]\n", L, R, L, R);
    fprintf(gp, "set yrange [%f/2-%f:%f/2+%f]\n", L, R, L, R);
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set parametric\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/zahyou_Rkakudai.png'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, p[i].x + 0.1, p[i].y + 0.1, i);
    }

    // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7 lt rgbcolor'black'\n", data_file);
    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", data_file, R, R);

    pclose(gp);

    /************************ 道路網プロット(番号更新前) ********************************************/

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set parametric\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/douromou.png'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, p[i].x + 0.1, p[i].y + 0.1, i);
    }

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", ad_file, data_file, R, R);

    pclose(gp);

    /************************ 道路網プロット(番号更新後) ********************************************/

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set parametric\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/douromou_new.png'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
    }

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", ad_file, new_data_file, R, R);

    pclose(gp);

    /************************ 点プロット(番号更新後) ********************************************/
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set parametric\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/zahyou_kousin.png'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
    }

    // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7 lt rgbcolor'black'\n", data_file);
    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red',\'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black'\n", new_data_file, R, R, jyunkai_file);

    pclose(gp);

    /***************************** 巡回路（道路網含める）*****************************************/
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/jyunkairo_douromou.png'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
    }

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'gray',\'%s\' u 1:2 with linespoints linewidth 2 pt 7 lt rgbcolor'black'\n", new_ad_file, jyunkai_file);

    pclose(gp);

    /************** それぞれの配送車の巡回路について *********************/

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/v1_v4_jyunkairo.png'\n");

    for (i = 0; i < 10; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
    }

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black'\n", v1_file, v4_file);
    pclose(gp);

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/v3_v5_jyunkairo.png'\n");

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black'\n", v3_file, v5_file);
    pclose(gp);

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/v2_jyunkairo.png'\n");

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black'\n", v2_file);
    pclose(gp);

    /************************* 全巡回路表示（各TVについて） ******************************/
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set noxtics\n"); // 目盛り非表示
    fprintf(gp, "set noytics\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set terminal png\n");
    fprintf(gp, "set output 'drone_datafile/all_jyunkairo.png'\n");

    for (i = 0; i < N; i++)
    {
        fprintf(gp, "set label %d at first %f,%f '%d' front\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
    }

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints linewidth 2 pt 7 lt rgbcolor'green',\'%s\' u 1:2 with linespoints linewidth 2 pt 7 lt rgbcolor'red',\'%s\' u 1:2 with linespoints linewidth 2 pt 7 lt rgbcolor'blue',\'%s\' u 1:2 with linespoints linewidth 2 pt 7 lt rgbcolor'orange',\'%s\' u 1:2 with linespoints linewidth 2 pt 7 lt rgbcolor'black'\n", v1_file, v2_file, v3_file, v4_file, v5_file);
    pclose(gp);

    /**************************動的確保した配列をfree*****************************/

    for (i = 0; i < MAX_SUBARRAYS; i++)
    {
        free(cir_flag[i]);
    }
    free(cir_flag);

    for (i = 0; i < MAX_SUBARRAYS; i++)
    {
        free(cir[i]);
    }
    free(cir);

    return 0;
}