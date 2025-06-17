// 新手法　ドローンが独立して避難所間を飛行しながら要求情報を回収する手法：物資運搬車両の数個前の避難所を重点的に飛行する手法：巡回路で集積所巡回し終えたらTVに届けに行く手法（避難所のバッテリーの数考慮したver）
// GIFアニメーションで表示するプログラム
// コンパイル方法「gcc -o my_program main.c module.c -lm」
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "header.h"

/**************************************メイン関数******************************************************/
int main(void)
{
    int i, j, k, m, n;
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

    srand(821); // シード値
    // srand(57); // シード値

    point p[N];          // 避難所と配送センターの宣言
    vehicle v[M];        // 配送車の宣言
    dro drone[D];        // ドローンの宣言
    dro infC_drone[C_D]; // 避難所の要求情報を回収するドローンの宣言

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
    }

    for (i = 0; i < N; i++)
    {
        for (j = 0; j < I_SIZE; j++)
        {
            p[0].inf[i][j] = 0;
            p[0].i_med_ptr[i] = 0;
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
    for (i = 1; i < N; i++)
    {
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

        p[i].battery_count = INITIAL_BATTERY_COUNT; // 避難所の初期バッテリー数
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

        v[i].drone_charge_count = 0;

        v[i].charge_amount = 0;

        v[i].chargeable_flag = TRUE;

        for (j = 0; j < QUEUE_SIZE; j++)
        {
            v[i].Med_delivery_queue[j] = 0;
        }

        v[i].queue_ptr = 0;

        v[i].queue_Notdelivery_ptr = 0;

        v[i].battery_count = DELIVERY_BATTERY_COUNT * ((N - 1) / M) + ADDITIONAL_BATTERY_COUNT; // 配送車の初期積載バッテリー数:それぞれの避難所へ INITIAL_BATTERY_COUNT + TV上での交換用の余分なバッテリー
    }

    // ドローン初期化
    for (i = 0; i < D; i++)
    {
        drone[i].x = L / 2;
        drone[i].y = L / 2;

        drone[i].xt = 0;
        drone[i].yt = 0;

        drone[i].re = 0;

        drone[i].wait_flag = FALSE;

        for (j = 0; j < N; j++)
        {
            drone[i].i_ptr[j] = 0;
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < I_SIZE; k++)
            {
                drone[i].inf[j][k] = 0;
            }
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < Y_SIZE; k++)
            {
                for (m = 0; m < Z_SIZE; m++)
                {
                    drone[i].inf_med[j][k][m] = 0.0;
                }
            }
        }

        drone[i].follow_num = i % M;

        drone[i].target_num = 1;

        drone[i].free_mode = FALSE;

        drone[i].charge_time = 0;

        drone[i].flight_start_time = 0;

        drone[i].FtoDiscenter_mode = FALSE; // ドローンが配送センターに向かうモード(避難所から集積所)（FALSE:配送車に従う、TRUE:ドローン単独で配送センターへ向かう）

        drone[i].delivery_mode = FALSE; // ドローンの配達モード(集積所から避難所)（FALSE:配送車に従う、TRUE:ドローン単独で配達)

        drone[i].Med_re = 0;

        drone[i].target_shelter_num = 0;

        drone[i].stay_Medload_time = 0;

        drone[i].TV_wait_flag = FALSE;

        drone[i].cannot_fly_judge_flag = FALSE;
    }

    // 要求情報回収ドローン初期化
    for (i = 0; i < C_D; i++)
    {
        infC_drone[i].x = L / 2;
        infC_drone[i].y = L / 2;

        infC_drone[i].xt = 0;
        infC_drone[i].yt = 0;

        infC_drone[i].re = 0;

        infC_drone[i].wait_flag = FALSE;

        for (j = 0; j < N; j++)
        {
            infC_drone[i].i_ptr[j] = 0;
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < I_SIZE; k++)
            {
                infC_drone[i].inf[j][k] = 0;
            }
        }

        for (j = 0; j < N; j++)
        {
            for (k = 0; k < Y_SIZE; k++)
            {
                for (m = 0; m < Z_SIZE; m++)
                {
                    infC_drone[i].inf_med[j][k][m] = 0.0;
                }
            }
        }

        infC_drone[i].follow_num = 0; // 要求情報回収ドローンは初期状態として、配送車[0]の巡回路に従うため

        infC_drone[i].target_num = 1;

        infC_drone[i].free_mode = FALSE;

        infC_drone[i].charge_time = 0;

        infC_drone[i].flight_start_time = 0;

        infC_drone[i].FtoDiscenter_mode = FALSE; // ドローンが配送センターに向かうモード(避難所から集積所)（FALSE:配送車に従う、TRUE:ドローン単独で配送センターへ向かう）

        infC_drone[i].delivery_mode = FALSE; // ドローンの配達モード(集積所から避難所)（FALSE:配送車に従う、TRUE:ドローン単独で配達)

        infC_drone[i].Med_re = 0;

        infC_drone[i].target_shelter_num = 0;

        infC_drone[i].stay_Medload_time = 0;

        infC_drone[i].TV_wait_flag = FALSE;

        infC_drone[i].cannot_fly_judge_flag = FALSE;

        infC_drone[i].crossing_cir_flag = TRUE; // 初期状態のとき、必ず避難所の初期地点に飛行するため TRUE

        infC_drone[i].FtoVehicle_mode = FALSE; // ドローンが配送車に向かうモード（FALSE:配送車に従う、TRUE:ドローン単独で配送車へ向かう）

        for (j = 0; j < N; j++)
        {
            infC_drone[i].shelter_visit_counter[j] = 0;
        }

        infC_drone[i].bat_swap_onTV_flag = FALSE;

        infC_drone[i].bat_swap_follow_num = 0;

        infC_drone[i].bat_swap_counter = 0;

        infC_drone[i].batDel_wait_flag = FALSE;
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

        new_p[i].battery_count = INITIAL_BATTERY_COUNT; // 避難所の初期バッテリー数
    }

    new_p[0].battery_count = INF; // 集積所のバッテリー数はINFに設定

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

    /***************** 巡回路を逆順に格納する配列 reverse_cir[][] を定義 ****************/
    // reverse_cir[M][]の各配列を逆順に格納するため動的確保
    int **reverse_cir = (int **)calloc(M, sizeof(int *));
    for (i = 0; i < M; i++)
    {
        reverse_cir[i] = (int *)calloc(size[i], sizeof(int));
    }

    // cir[M][]の各配列を逆順にしてreverse_cir[M][]に格納
    for (i = 0; i < M; i++)
    {
        for (j = 0; j < size[i]; j++)
        {
            reverse_cir[i][j] = cir[i][size[i] - 1 - j];
        }
    }

    // reverse_cir[][]の配列を表示
    /*
    for (i = 0; i < M; i++)
    {
        printf("reverse_cir[%d]: ", i);
        for (j = 0; j < size[i]; j++)
        {
            printf("%d ", reverse_cir[i][j]);
        }
        printf("\n");
    }*/

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

    // 各巡回路上の情報収集ドローンが一巡回するのにかかる時間の平均（純粋に飛行だけした場合の時間（充電などは考慮なし））
    double vd = 20.0; // ドローンの速度[km/h]
    FILE *fp_drone_trip_data;
    char *drone_trip_data_file = "drone_datafile/txtfile/drone_trip_ave_data.txt";

    fp_drone_trip_data = fopen(drone_trip_data_file, "a+");
    fprintf(fp_drone_trip_data, "%f\n", total_di_ave / vd * 60); // ドローンの巡回路の平均所要時間をファイルに格納
    fclose(fp_drone_trip_data);

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

    /**************************** 各避難所の物資要求量を決定 *****************************************/
    // 各避難所の物資量を生成
    for (int i = 1; i < N; i++)
    {
        new_p[i].re_req = MEAN; // 初期値MEAN(=50)に設定
    }

    // 結果を表示
    int sum_re = 0; // 物資総要求量
    // printf("避難所ごとの必要物資量:\n");
    for (int i = 1; i < N; i++)
    {
        sum_re += new_p[i].re_req;
        // printf("避難所 %d: %d\n", i, new_p[i].re_req);
    }
    // printf("物資総要求量：%d\n", sum_re);

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

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [0:10]\n");
    fprintf(gp, "set yrange [0:10]\n");
    fprintf(gp, "set size square\n");
    fprintf(gp, "unset key\n");

    // fprintf(gp, "set term gif animate delay 5 optimize size 640,480\n");
    fprintf(gp, "set term gif animate delay 10 optimize size 640,480 font 'DejaVu Sans,12'\n");
    fprintf(gp, "set output 'drone_datafile/test.gif'\n");

    // ラベルの表示
    for (i = 0; i < N; i++)
    {
        // fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 1, new_p[i].x + 0.1, new_p[i].y + 0.1, i);
    }

    /************************************シミュレーション******************************************/

    double total_t = 0;                                  // シミュレーションの合計時間
    double termination_t = 500000;                       // シミュレーション強制終了時間
    double r_velo = 360;                                 // 速度の逆数
    double part_t[M] = {0};                              // 二点間の経過時間(配送車)
    double part_t_dro[C_D] = {0};                        // 二点間の経過時間(ドローン)
    int current[M] = {0};                                // 始点
    int target[M];                                       // 終点
    int current_dro[C_D];                                // ドローンの始点
    int target_dro[C_D];                                 // ドローンの終点
    double d[M];                                         // ２点間の距離(配送車)
    double d_dro[C_D];                                   // ２点間の距離(ドローン)
    double n_sin[M];                                     // サイン
    double n_cos[M];                                     // コサイン
    double n_tan[M];                                     // タンジェント
    double n_sin_dro[C_D];                               // ドローンのサイン
    double n_cos_dro[C_D];                               // ドローンのコサイン
    double n_tan_dro[C_D];                               // ドローンのタンジェント
    int ind[M] = {0};                                    // targetのindex(配送車)
    int ind_dro[C_D] = {0};                              // targetのindex(ドローン)
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
    double poisson_inf_total = rand_exp(lambda_i) * 3600;
    double poisson_inf_count = 0;
    double poisson_Medinf_total = rand_exp(lambda_i_med) * 3600; // 薬の情報の到着
    double poisson_Medinf_count = 0;
    int re_load_num = 10 * MEAN;           // 配送センターで一度に積載する物資の数
    int re_finish_num = 10 * MEAN;         // シミュレーション終了物資量(避難所に物資届ける回数×MEAN)
    int ind_relief[M];                     // 物資を避難所に下ろすindex :配送車1なら1,2,3,,,
    int re_wait_flag[M] = {FALSE};         // 配送センターの物資存在フラグ
    int counter_Med_re_delivery = 0;       // 医療物資の避難所への配送回数（配送車＋ドローン）
    int counter_Med_re_Drone_delivery = 0; // 医療物資のドローンによる配送回数（ドローンのみ）
    int Medinf_collect_count = 0;          // 医療物資の要求情報を収集する回数（避難所から配送車またはドローンへの情報共有の回数）
    int Medinf_collect_count_dro = 0;      // 医療物資の要求情報を収集する回数（避難所からドローンへの情報共有の回数）

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
    // 医療品情報の収集遅延時間 E(TC)（避難所から配送車またはドローンへの情報共有の遅延時間限定）
    FILE *fp_Medinf_collect_delay;
    Medinf_collect_delay_file = "drone_datafile/txtfile/Medinf_collect_delay.txt";
    // E(TC)（避難所からドローン）
    FILE *fp_ETC_dro;
    char *ETC_dro_data_file = "drone_datafile/txtfile/ETC_dro_data.txt";
    // E(TC) (避難所から物資運搬車両のみ)
    FILE *fp_ETC_Vehicle;
    char *ETC_Vehicle_data_file = "drone_datafile/txtfile/ETC_Vehicle_data.txt";
    // E(TC)（避難所から物資運搬車両（避難所→ドローン→TV）と（避難所→TV）の遅延時間）
    FILE *fp_ETC_to_Vehicle;
    char *ETC_to_Vehicle_data_file = "drone_datafile/txtfile/ETC_to_Vehicle_data.txt";
    // 医療品の配送遅延時間（要求情報発生が回収されてから実際に医療品が避難所へ届けられるまでの遅延時間）
    FILE *fp_Med_re_collect_to_delivery_delay;
    Med_re_collect_to_delivery_delay_file = "drone_datafile/txtfile/Med_re_collect_to_delivery_delay.txt";
    // 各ドローンの一巡回にかかる時間をファイルに書き込む
    FILE *fp_infC_jyunkai_time;
    char *infC_jyunkai_time_file = "drone_datafile/txtfile/infC_jyunkai_time.txt";

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
    double r_d_velo = 180;                    // 配送車の2倍（ドローンの時速の逆数：）一キロ飛行するのにかかる秒数
    double v_d_ratio = r_velo / r_d_velo;     // 配送車の速度とドローンの速度の比率
    double capable_flight_time = 60 * 30;     // ドローンの最大飛行時間(３０分)
    double drone_Med_loding_time = 60 * 10;   // ドローンの医療物資積載時間(10分) 60*10
    double drone_Med_Unloding_time = 60 * 10; // ドローンの避難所での医療物資荷降ろし時間(10分) 60*10
    double battery_swap_time = 60 * 10;       // ドローンのバッテリー交換時間(5分) 60*5
    double addtional_time = 0;                // ドローンの配送による配送車の避難所での追加待機時間
    double d_d[D];                            // ドローンと集積所との距離
    double d_s_dis;                           // 集積所と任意の避難所の距離
    double d_n_sin[D];
    double d_n_cos[D];
    int drone_depature_flag[D] = {FALSE};                                            // ドローンが飛び出したことを示すフラグ
    int shelter_num[D] = {0};                                                        // ドローンが出発する避難所
    int drone_roop_count[D] = {0};                                                   // ドローンが周回する回数
    int current_returnnum[D] = {0};                                                  // ドローンが一周して帰ってくる配送車番号
    int change_follow = 0;                                                           // 配送センターでドローンのfollowする配送車を切り替える
    int total_inf_num = 0;                                                           // 避難所に共有される情報の総数
    int drone_inf_num = 0;                                                           // 避難所に共有される情報のうちドローンによって共有される情報数
    int drone_target[D] = {0};                                                       // ドローンが先回りして向かう避難所番号
    double drone_flight_distanece[D] = {0};                                          // ドローンの総飛行距離
    double flight_time[D] = {0};                                                     // ドローンの飛行時間(目的地まで1飛行あたり)
    double total_flight_time[D] = {0};                                               // flight_timeの合計値
    double flight_count[D] = {0};                                                    // ドローンの飛行回数
    double charge_constant = 2;                                                      // ドローンの充電時間の定数倍(２なら飛行時間の２倍充電時間が掛かる)
    double m_v_f = 60 * 14.5;                                                        // ドローンの配送車間の平均飛行時間（分）mean_value_flight 14分
    double flight_time_lag = removeOnePlace((m_v_f * M + m_v_f * 2 * (M - 1)) / SD); // ドローンの飛行開始時間の時間差(60秒 * 分)導出用
    int drone_next_target[M] = {1, 2, 3, 4, 0};                                      // ドローンが巡回路をまたいで向かう配送車の番号の対応配列：
    double infC_drone_flight_time[C_D] = {0};                                        // 要求情報回収ドローンの飛行時間
    double infC_drone_jyunkai_time[C_D] = {0};                                       // 要求情報回収ドローンの巡回時間

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

    /********************全TVのドローン総充電台数計算*********************/
    double all_TV_chargecount = 0;     // 全てのドローンでの充電台数（一巡回）
    double all_TV_chargecount_ave = 0; // 全てのドローンでの充電台数の平均（一巡回）
    double TV_chargeAmount = 0;        // 各TVにおけるドローン総充電時間(一巡回):単位は[min]
    double ave_TV_chargeAmount;        // 各TVにおけるドローン総充電時間の平均値(一巡回)

    // target について（配送車）
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

    // target_dro について（ドローン）
    //  target_droのindex初期化
    for (i = 0; i < C_D; i++)
    {
        ind_dro[i] = 1;
    }
    // target_droの初期化: 巡回路1(TV[0])における避難所から巡回
    int reverse_num = 0;       // 巡回路の配列の最大要素数から戻す数
    int pass_count[C_D] = {0}; // ドローンがある巡回路で通過した避難所の数：S_Nと比較用
    for (i = 0; i < C_D; i++)
    {
        current_dro[i] = 0;

        if (S_N >= size[0] && ind[0] - (S_N + 1) < 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以上のとき
        {
            reverse_num = S_N + 1 - ind[0]; // 巡回路の配列の最大要素数から戻す数
            target_dro[i] = cir[0][(2 * size[0] - 1) - reverse_num];
            ind_dro[i] = 2 * size[0] - reverse_num - 1; // ドローンのtarget_droのindexを更新
        }
        else if (ind[0] - (S_N + 1) < 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以下のとき
        {
            reverse_num = S_N + 1 - ind[0]; // 巡回路の配列の最大要素数から戻す数
            target_dro[i] = cir[0][(size[0] - 1) - reverse_num];
            ind_dro[i] = size[0] - reverse_num - 1; // ドローンのtarget_droのindexを更新
        }
        else
        {
            target_dro[i] = cir[0][ind[0] - (S_N + 1)];
        }

        infC_drone[i].xt = new_p[target_dro[i]].x; // ドローンの目的座標更新
        infC_drone[i].yt = new_p[target_dro[i]].y;
    }

    // 情報回収用ドローンの出発時間初期化
    double devision = C_D / M;     // ドローンの数を配送車の数で割る
    double total_di_time[M] = {0}; // 各小回線の総時間
    double f_s_time = 0;
    for (i = 0; i < M; i++)
    {
        total_di_time[i] = total_di[i] * r_d_velo;                   // 各小回線の総時間を計算
        total_di_time[i] = total_di_time[i] * (charge_constant + 1); // 充電時間も考慮した飛行間隔に（充電なしの場合*(充電係数+1)）
        // printf("total_di_time[%d]: %f\n", i, total_di_time[i]);
    }
    for (i = 0; i < C_D; i++)
    {
        f_s_time = total_di_time[i % M] / (double)devision * (double)(i - i % M) / M;
        infC_drone[i].flight_start_time = (int)f_s_time - (int)f_s_time % 10; // ドローンの出発時間を設定(少数第一位を0にする)
        // printf("infC_drone[%d].flight_start_time: %f\n", i, infC_drone[i].flight_start_time);
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
    fp_re_interval = fopen(re_interval_file, "w");                                           // 平均物資到着間隔のファイルオープン
    fp_Medinf_delay = fopen(Medinf_delay_file, "w");                                         // 薬情報の遅延間隔ファイルのオープン
    fp_Med_re_delay = fopen(Med_re_delay_file, "w");                                         // 医療品の配送遅延間隔ファイルのオープン
    fp_Medinf_collect_delay = fopen(Medinf_collect_delay_file, "w");                         // 医療品情報の収集遅延間隔ファイルのオープン
    fp_ETC_dro = fopen(ETC_dro_data_file, "w");                                              // ドローンの避難所からの配送遅延間隔ファイルのオープン
    fp_ETC_Vehicle = fopen(ETC_Vehicle_data_file, "w");                                      // 避難所から配送車への配送遅延間隔ファイルのオープン
    fp_ETC_to_Vehicle = fopen(ETC_to_Vehicle_data_file, "w");                                // 避難所から配送車への配送遅延間隔ファイルのオープン
    fp_Med_re_collect_to_delivery_delay = fopen(Med_re_collect_to_delivery_delay_file, "w"); // 医療品の収集から配送への遅延間隔ファイルのオープン
    fp_infC_jyunkai_time = fopen(infC_jyunkai_time_file, "w");                               // ドローンの巡回時間ファイルのオープン

    /************************************ ループ処理 ***********************************************************/
    while (1)
    {
        // debug
        // printf("total_t: %f\n", total_t);

        // 配送車のサインコサイン
        for (i = 0; i < M; i++)
        {
            d[i] = sqrt(pow(new_p[target[i]].x - new_p[current[i]].x, 2) + pow(new_p[target[i]].y - new_p[current[i]].y, 2));
            n_sin[i] = (new_p[target[i]].x - new_p[current[i]].x) / d[i];
            n_cos[i] = (new_p[target[i]].y - new_p[current[i]].y) / d[i];
            n_tan[i] = n_sin[i] / n_cos[i];
        }
        // ドローンのサインコサイン
        for (i = 0; i < S_C_D; i++)
        {
            if (target_dro[i] == current_dro[i] || ((infC_drone[i].x == infC_drone[i].xt) && (infC_drone[i].y == infC_drone[i].yt))) // ドローンの目的地と現在地が重複してしまった場合は即座に到着したものとみなす（ドローンと避難所の座標が一致 or ドローンと物資運搬車両の座標が一致）
            {
                n_sin_dro[i] = 0.001;
                n_cos_dro[i] = 0.001;
            }
            else // ドローンが避難所 or 物資運搬車両へ飛行する場合
            {
                d_dro[i] = sqrt(pow(infC_drone[i].xt - infC_drone[i].x, 2) + pow(infC_drone[i].yt - infC_drone[i].y, 2));
                n_sin_dro[i] = (infC_drone[i].xt - infC_drone[i].x) / d_dro[i];
                n_cos_dro[i] = (infC_drone[i].yt - infC_drone[i].y) / d_dro[i];
                n_tan_dro[i] = n_sin_dro[i] / n_cos_dro[i];
            }
        }

        // if (total_t >= 0 && total_t <= 20000)
        if (total_t >= 0 && total_t <= 30000)
        {
            if ((int)(total_t) % 50 == 0)
            { // 50sごとに描画

                // ラベルの表示
                for (j = 0; j < N; j++)
                {
                    // fprintf(gp, "set label %d at first %f,%f '%d'\n", j + 51, new_p[j].x + 0.1, new_p[j].y + 0.1, new_p[j].re);
                }

                // 避難所のバッテリー数を表示
                for (i = 1; i < N; i++)
                {
                    fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 101, new_p[i].x - 0.2, new_p[i].y - 0.2, new_p[i].battery_count);
                }

                // 物資運搬車両のバッテリー数を表示
                for (i = 0; i < M; i++)
                {
                    fprintf(gp, "set label %d at first %f,%f '%d'\n", i + 201, v[i].x + 0.2, v[i].y + 0.2, v[i].battery_count);
                }

                // ドローン8台
                fprintf(gp, "set title 't = %f'\n", total_t);
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'gold','-' pt 5 lt rgbcolor'dark-turquoise'\n", new_data_file, new_ad_file);
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta'\n", new_data_file, new_ad_file);
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta'\n", new_data_file, new_ad_file);
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red'\n", new_data_file, new_ad_file); // ５台
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green'\n", new_data_file, new_ad_file); // 通常ドローンと情報収集ドローン
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 lt rgbcolor'green','-' pt 9 lt rgbcolor'red','-' pt 9 lt rgbcolor'blue','-' pt 9 lt rgbcolor'orange','-' pt 9 lt rgbcolor'black','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'orange-red','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green','-' pt 5 lt rgbcolor'light-green'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ
                // fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 ps 1.5 lt rgbcolor'green','-' pt 9 ps 1.5 lt rgbcolor'red','-' pt 9 ps 1.5 lt rgbcolor'blue','-' pt 9 ps 1.5 lt rgbcolor'orange','-' pt 9 ps 1.5 lt rgbcolor'black','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'dark-magenta','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ(10台)

                // infC_drone[0]のfollow_numに応じてプロットの色を変更
                if (infC_drone[0].follow_num == 0)
                {
                    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 ps 1.5 lt rgbcolor'green','-' pt 9 ps 1.5 lt rgbcolor'red','-' pt 9 ps 1.5 lt rgbcolor'blue','-' pt 9 ps 1.5 lt rgbcolor'orange','-' pt 9 ps 1.5 lt rgbcolor'black','-' pt 5 lt rgbcolor'green'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ(1台のみ)
                }
                else if (infC_drone[0].follow_num == 1)
                {
                    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 ps 1.5 lt rgbcolor'green','-' pt 9 ps 1.5 lt rgbcolor'red','-' pt 9 ps 1.5 lt rgbcolor'blue','-' pt 9 ps 1.5 lt rgbcolor'orange','-' pt 9 ps 1.5 lt rgbcolor'black','-' pt 5 lt rgbcolor'red'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ(1台のみ)
                }
                else if (infC_drone[0].follow_num == 2)
                {
                    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 ps 1.5 lt rgbcolor'green','-' pt 9 ps 1.5 lt rgbcolor'red','-' pt 9 ps 1.5 lt rgbcolor'blue','-' pt 9 ps 1.5 lt rgbcolor'orange','-' pt 9 ps 1.5 lt rgbcolor'black','-' pt 5 lt rgbcolor'blue'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ(1台のみ)
                }
                else if (infC_drone[0].follow_num == 3)
                {
                    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 ps 1.5 lt rgbcolor'green','-' pt 9 ps 1.5 lt rgbcolor'red','-' pt 9 ps 1.5 lt rgbcolor'blue','-' pt 9 ps 1.5 lt rgbcolor'orange','-' pt 9 ps 1.5 lt rgbcolor'black','-' pt 5 lt rgbcolor'orange'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ(1台のみ)
                }
                else if (infC_drone[0].follow_num == 4)
                {
                    fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 9 ps 1.5 lt rgbcolor'green','-' pt 9 ps 1.5 lt rgbcolor'red','-' pt 9 ps 1.5 lt rgbcolor'blue','-' pt 9 ps 1.5 lt rgbcolor'orange','-' pt 9 ps 1.5 lt rgbcolor'black','-' pt 5 lt rgbcolor'black'\n", new_data_file, new_ad_file); // 情報収集ドローンのみ(1台のみ)
                }

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
                /*
                fprintf(gp, "%f %f\n", drone[0].x + 0.1, drone[0].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[1].x + 0.1, drone[1].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[2].x + 0.1, drone[2].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[3].x + 0.1, drone[3].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[4].x + 0.1, drone[4].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[5].x + 0.1, drone[5].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[6].x + 0.1, drone[6].y + 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", drone[7].x + 0.1, drone[7].y + 0.1);
                fprintf(gp, "e\n");
                */
                fprintf(gp, "%f %f\n", infC_drone[0].x - 0.1, infC_drone[0].y - 0.1);
                fprintf(gp, "e\n");
                /*
                fprintf(gp, "%f %f\n", infC_drone[1].x - 0.1, infC_drone[1].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[2].x - 0.1, infC_drone[2].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[3].x - 0.1, infC_drone[3].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[4].x - 0.1, infC_drone[4].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[5].x - 0.1, infC_drone[5].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[6].x - 0.1, infC_drone[6].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[7].x - 0.1, infC_drone[7].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[8].x - 0.1, infC_drone[8].y - 0.1);
                fprintf(gp, "e\n");
                fprintf(gp, "%f %f\n", infC_drone[9].x - 0.1, infC_drone[9].y - 0.1);
                fprintf(gp, "e\n");
                */

                /*
                // ドローン1台
                fprintf(gp, "set title 't = %f'\n", total_t);
                fprintf(gp, "plot \'%s\' u 2:3 with points pt 7, \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'grey','-' pt 5 lt rgbcolor'green','-' pt 5 lt rgbcolor'red','-' pt 5 lt rgbcolor'blue','-' pt 5 lt rgbcolor'orange','-' pt 5 lt rgbcolor'black','-' pt 5 lt rgbcolor'dark-magenta'\n", new_data_file, new_ad_file);
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
                fprintf(gp, "%f %f\n", drone[0].x + 0.1, drone[0].y + 0.1);
                fprintf(gp, "e\n");
                */
            }
        }

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
        /**************ドローンの座標更新*****************/
        for (i = 0; i < S_C_D; i++)
        {
            // 飛行開始時間まで待機
            if (infC_drone[i].flight_start_time != 0)
            {
                infC_drone[i].flight_start_time -= time_span; // ドローンの飛行開始時間を減らす
                if (infC_drone[i].flight_start_time == 0)     // 飛行開始時間が0以下になったら
                {
                    infC_drone[i].flight_start_time = 0; // 飛行開始時間を0にする
                    printf("drone[%d]の飛行開始時間: %lf\n", i, total_t);
                }
            }
            else if (infC_drone[i].batDel_wait_flag == TRUE) // ドローンが避難所でTVによるバッテリー配達を待っている場合
            {
                if (new_p[current_dro[i]].battery_count > 0) // 避難所にバッテリーが届けられた場合
                {
                    infC_drone[i].batDel_wait_flag = FALSE; // バッテリー配達待ちフラグを解除

                    new_p[current_dro[i]].battery_count -= 1; // 避難所のバッテリー数を減らす

                    infC_drone[i].bat_swap_counter += 1; // バッテリー交換回数をカウント
                }
            }
            else if (infC_drone[i].charge_time == 0) // 避難所へ飛行する場合：充電する必要ないなら目的の避難所へ飛行
            {
                infC_drone[i].x = infC_drone[i].x + n_sin_dro[i] * time_span / r_d_velo;
                infC_drone[i].y = infC_drone[i].y + n_cos_dro[i] * time_span / r_d_velo;

                infC_drone_flight_time[i] += time_span; // ドローンの飛行時間を加算

                infC_drone_jyunkai_time[i] += time_span; // ドローンの巡回時間を加算
            }
            else if (infC_drone[i].charge_time != 0) // 充電する必要があるなら
            {
                infC_drone[i].charge_time -= time_span; // ドローンの充電時間を減らす

                infC_drone_jyunkai_time[i] += time_span; // ドローンの巡回時間を加算

                if (infC_drone[i].charge_time == 0 && infC_drone[i].x == new_p[0].x && infC_drone[i].y == new_p[0].y) // 集積所にいながら飛行時間加算してしまっている場合
                {
                    infC_drone_jyunkai_time[i] = 0;
                }

                if (infC_drone[i].bat_swap_onTV_flag == TRUE) // TV上でバッテリー交換を行っている場合はTVに追従
                {
                    infC_drone[i].x = v[infC_drone[i].bat_swap_follow_num].x;
                    infC_drone[i].y = v[infC_drone[i].bat_swap_follow_num].y;

                    if (infC_drone[i].charge_time == 0) // バッテリー交換が完了したら
                    {
                        infC_drone[i].bat_swap_onTV_flag = FALSE;
                    }
                }

                // 物資運搬車両へ飛行する前に充電を行ったため、改めて物資運搬車両との合流地点を導出する場合
                if (infC_drone[i].charge_time == 0 && infC_drone[i].FtoVehicle_mode == TRUE)
                {
                    solveConfluenceVer2(v[infC_drone[i].follow_num].x, v[infC_drone[i].follow_num].y, infC_drone[i].x, infC_drone[i].y, 1.0, v_d_ratio, new_p[target[infC_drone[i].follow_num]].x, new_p[target[infC_drone[i].follow_num]].y, &infC_drone[i].xt, &infC_drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, dis_stay_t, new_p, infC_drone, i, v, infC_drone[i].follow_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size); // follow_numにおける配送車との合流地点

                    // ドローンと合流地点のサインコサインを改めて計算
                    d_dro[i] = sqrt(pow(infC_drone[i].xt - infC_drone[i].x, 2) + pow(infC_drone[i].yt - infC_drone[i].y, 2));
                    n_sin_dro[i] = (infC_drone[i].xt - infC_drone[i].x) / d_dro[i];
                    n_cos_dro[i] = (infC_drone[i].yt - infC_drone[i].y) / d_dro[i];
                    n_tan_dro[i] = n_sin_dro[i] / n_cos_dro[i];
                }
            }
        }

/********************************************** 情報回収ドローンの避難所間飛行処理 **************************************************************************************************/
/*****************(追加 5/23)各ドローンにおいて始点から終点へ到達したときの処理****************************/
#if 0
        for (i = 0; i < S_C_D; i++)
        {
            if ((ind_dro[i] == size[i % M] - 1 && n_cos_dro[i] < 0 && infC_drone[i].y < new_p[reverse_cir[i % M][0]].y) || (ind_dro[i] == size[i % M] - 1 && n_cos_dro[i] > 0 && infC_drone[i].y > new_p[reverse_cir[i % M][0]].y))
            { // ①一周したら、ループを初期化してもう一周
                ind_dro[i] = 1;
                current_dro[i] = 0;
                target_dro[i] = reverse_cir[i % M][ind_dro[i]]; // 目的避難所変更
                infC_drone[i].x = new_p[current_dro[i]].x;      // 座標修正
                infC_drone[i].y = new_p[current_dro[i]].y;
                part_t_dro[i] = 0;

                // printf(" fff 収集ドローン[%d]の周回時間: %lf[min]\n", i, infC_drone_jyunkai_time[i] / 60);
                //   各ドローンの一巡回にかかる時間をファイルに書き込む
                fprintf(fp_infC_jyunkai_time, "%lf\n", infC_drone_jyunkai_time[i] / 60);
                infC_drone_jyunkai_time[i] = 0; // ドローンの飛行時間を初期化

                // 次の避難所へ行く間に充電量が不足する場合はその避難所で充電する
                if (infC_drone_flight_time[i] + new_di[current_dro[i]][target_dro[i]] * r_d_velo > capable_flight_time)
                {
                    infC_drone[i].charge_time = infC_drone_flight_time[i] * charge_constant; // 充電時間を設定
                    infC_drone_flight_time[i] = 0;                                           // ドローンの飛行時間を初期化
                    // printf("ドローン[%d]は集積所で充電開始  充電時間: %lf[min]\n", i, (double)infC_drone[i].charge_time / 60);
                }
            }
            else if ((n_cos_dro[i] < 0 && infC_drone[i].y < new_p[target_dro[i]].y) || (n_cos_dro[i] > 0 && infC_drone[i].y > new_p[target_dro[i]].y))
            { // ③それ以外において、targetに到達したらcurrentとtarget更新
                ind_dro[i] += 1;
                current_dro[i] = target_dro[i];
                target_dro[i] = reverse_cir[i % M][ind_dro[i]]; // 目的避難所変更
                infC_drone[i].x = new_p[current_dro[i]].x;      // 座標修正
                infC_drone[i].y = new_p[current_dro[i]].y;
                part_t_dro[i] = 0;

                // 次の避難所へ行く間に充電量が不足する場合はその避難所で充電する
                if (infC_drone_flight_time[i] + new_di[current_dro[i]][target_dro[i]] * r_d_velo > capable_flight_time)
                {
                    infC_drone[i].charge_time = infC_drone_flight_time[i] * charge_constant; // 充電時間を設定
                    infC_drone_flight_time[i] = 0;                                           // ドローンの飛行時間を初期化
                    // printf("ドローン[%d]は避難所[%d]で充電開始  充電時間: %lf[min]\n", i, current_dro[i], (double)infC_drone[i].charge_time / 60);
                }

                /***********避難所 ↔ ドローン間情報交換**********/

                // 避難所との薬の情報配列の交換(避難所→ドローン)
                for (j = 0; j < N; j++)
                {
                    if (new_p[current_dro[i]].i_med_ptr[j] > infC_drone[i].i_med_ptr[j] && current_dro[i] >= (i % M) * 10 + 1 && current_dro[i] <= ((i % M) + 1) * 10) // 同じ巡回路内の避難所の要求情報を回収
                    {
                        for (k = infC_drone[i].i_med_ptr[j]; k < new_p[current_dro[i]].i_med_ptr[j]; k++)
                        {
                            if (new_p[current_dro[i]].inf_med[j][k][5] == FALSE) // 既に他の配送車やドローンに要求情報が回収されていなければ
                            {
                                new_p[current_dro[i]].inf_med[j][k][5] = TRUE; // 回収済みのフラグを立てる

                                for (m = 0; m < Z_SIZE; m++)
                                {
                                    infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j]][m] = new_p[current_dro[i]].inf_med[j][k][m]; // 薬の情報コピー
                                }
                                infC_drone[i].i_med_ptr[j] += 1;

                                // 医療品の配送先をキューに保存
                                // v[i].Med_delivery_queue[v[i].queue_ptr] = current[i]; // 医療品の配送先をキューに格納
                                // v[i].queue_ptr += 1;                                  // キューのポインタを進める
                                /*if (v[i].queue_ptr == QUEUE_SIZE)
                                {
                                    printf("キューの要素数オーバー\n");
                                    break;
                                }*/

                                printf(" **** t=%lf : 避難所[%d]の情報を ドローン[%d]が回収\n", total_t, current_dro[i], i);

                                // ファイルへの書き込み
                                // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf new_p[%d]->drone[%d]\n", total_t, infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0], current_dro[i], i);
                                // fprintf(fp_Medinf_delay, "%lf\n", total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]); // 生成されてからドローンで回収されるまでの遅延時間
                                // fprintf(fp_ETC_dro, "t=%lf generate_time:%lf new_p[%d]->drone[%d] %lf\n", total_t, infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0], current_dro[i], i, total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]);
                                fprintf(fp_ETC_dro, "%lf\n", total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]); // 生成されてからドローンで回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                                infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][4] = total_t;

                                // fprintf(fp_Medinf_collect_delay, "t=%lf generate_time:%lf new_p[%d]->drone[%d] %lf\n", total_t, infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0], current_dro[i], i, total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]);
                                fprintf(fp_Medinf_collect_delay, "%lf\n", total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]); // 生成されてからドローンで回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                            }
                        }
                    }
                }
            }
        }
#endif
        /************************************ 情報収集ドローンで各巡回路においてTVの S_N 個前の避難所を順に巡回していく処理：避難所 or 集積所へ到達した場合の処理 *******************************************************************************************************************************************/
        for (i = 0; i < S_C_D; i++)
        {
            /*******************物資運搬車両へドローンが到達した場合 *****************************/
            if (((n_cos_dro[i] < 0 && infC_drone[i].y < infC_drone[i].yt) || (n_cos_dro[i] > 0 && infC_drone[i].y > infC_drone[i].yt)) && infC_drone[i].FtoVehicle_mode == TRUE) // 目的の物資運搬車両に到達した場合
            {

                infC_drone[i].x = infC_drone[i].xt; // 座標修正
                infC_drone[i].y = infC_drone[i].yt;

                infC_drone[i].FtoVehicle_mode = FALSE; // 物資運搬車両への飛行モードをリセット

                current_dro[i] = INF; // ドローンの現在値は物資運搬車両であるため、INFに設定

                // ドローンが次の巡回路へ飛行するための処理追加
                infC_drone[i].bat_swap_follow_num = infC_drone[i].follow_num; // バッテリー交換を行う物資運搬車両の番号を保存しておく

                pass_count[i] = 0;             // 避難所通過回数をリセット
                infC_drone[i].follow_num += 1; // 一つ隣の巡回路の避難所巡回に変更

                if (infC_drone[i].follow_num >= M) // 最後の巡回路を超えたら
                {
                    infC_drone[i].follow_num = 0; // 巡回路0からにする
                }

                infC_drone[i].crossing_cir_flag = TRUE; // 巡回路間飛行中フラグを立てる

                // 隣の巡回路において、TVの S_N 個前の避難所を目的地にする
                if (stay_t[infC_drone[i].follow_num] != 0) // 物資運搬車両が避難所で物資におろし中なら、(S_N + 1)個前から始める
                {
                    /*
                    if (S_N + 1 >= (size[infC_drone[i].follow_num] + (ind[i] - 1)) && ind[infC_drone[i].follow_num] - (S_N + 1) <= 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以上のとき
                    {
                        reverse_num = (S_N + 1) - ind[infC_drone[i].follow_num] + 1; // 巡回路の配列の最大要素数から戻す数
                        target_dro[i] = cir[infC_drone[i].follow_num][(2 * size[infC_drone[i].follow_num] - 1) - reverse_num] - ((ind[i] - 1) - 1);
                        ind_dro[i] = (2 * size[infC_drone[i].follow_num] - 1) - reverse_num - ((ind[i] - 1) - 1); // ドローンのtarget_droのindexを更新
                    }*/
                    if (S_N - ind[i] + 1 >= size[infC_drone[i].follow_num] && ind[infC_drone[i].follow_num] - (S_N + 1) <= 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以上のとき
                    {
                        // reverse_num = (S_N + 1) - ind[infC_drone[i].follow_num] + 1; // 巡回路の配列の最大要素数から戻す数
                        // target_dro[i] = cir[infC_drone[i].follow_num][(2 * size[infC_drone[i].follow_num] - 1) - reverse_num] - ((ind[i] - 1) - 1);
                        // ind_dro[i] = (2 * size[infC_drone[i].follow_num] - 1) - reverse_num - ((ind[i] - 1) - 1); // ドローンのtarget_droのindexを更新
                        target_dro[i] = cir[infC_drone[i].follow_num][2 * (size[infC_drone[i].follow_num] - 1) + ind[infC_drone[i].follow_num] - (S_N + 1)];
                        ind_dro[i] = 2 * (size[infC_drone[i].follow_num] - 1) + ind[infC_drone[i].follow_num] - (S_N + 1); // ドローンのtarget_droのindexを更新
                    }
                    else if (ind[infC_drone[i].follow_num] - (S_N + 1) <= 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以下のとき
                    {
                        reverse_num = (S_N + 1) - ind[infC_drone[i].follow_num] + 1; // 巡回路の配列の最大要素数から戻す数
                        target_dro[i] = cir[infC_drone[i].follow_num][(size[infC_drone[i].follow_num] - 1) - reverse_num];
                        ind_dro[i] = size[infC_drone[i].follow_num] - 1 - reverse_num; // ドローンのtarget_droのindexを更新
                    }
                    else
                    {
                        target_dro[i] = cir[infC_drone[i].follow_num][ind[infC_drone[i].follow_num] - (S_N + 1)];
                        ind_dro[i] = ind[infC_drone[i].follow_num] - (S_N + 1);
                    }
                }
                else // 物資運搬車両が避難所で物資におろし中でないなら、(S_N)個前から始める
                {
                    /*
                    if (S_N >= (size[infC_drone[i].follow_num] + (ind[i] - 1)) && ind[infC_drone[i].follow_num] - (S_N + 1) <= 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以上のとき
                    {
                        reverse_num = (S_N ) - ind[infC_drone[i].follow_num] + 1; // 巡回路の配列の最大要素数から戻す数
                        target_dro[i] = cir[infC_drone[i].follow_num][(2 * size[infC_drone[i].follow_num] - 1) - reverse_num] - ((ind[i] - 1) - 1);
                        ind_dro[i] = (2 * size[infC_drone[i].follow_num] - 1) - reverse_num - ((ind[i] - 1) - 1); // ドローンのtarget_droのindexを更新
                    }*/
                    if (S_N - ind[i] >= size[infC_drone[i].follow_num] && ind[infC_drone[i].follow_num] - (S_N + 1) <= 0) // 集積所を超えて戻る場合：S_Nが巡回路のサイズ以上のとき
                    {
                        // reverse_num = (S_N)-ind[infC_drone[i].follow_num] + 1; // 巡回路の配列の最大要素数から戻す数
                        target_dro[i] = cir[infC_drone[i].follow_num][2 * (size[infC_drone[i].follow_num] - 1) + ind[infC_drone[i].follow_num] - S_N];
                        ind_dro[i] = 2 * (size[infC_drone[i].follow_num] - 1) + ind[infC_drone[i].follow_num] - S_N; // ドローンのtarget_droのindexを更新
                    }
                    else if (ind[infC_drone[i].follow_num] - S_N <= 0) // 集積所を超えて戻る場合
                    {
                        reverse_num = S_N - ind[infC_drone[i].follow_num] + 1; // 巡回路の配列の最大要素数から戻す数集積所を超えて戻る場合：S_Nが巡回路のサイズ以下のとき
                        target_dro[i] = cir[infC_drone[i].follow_num][(size[infC_drone[i].follow_num] - 1) - reverse_num];
                        ind_dro[i] = size[infC_drone[i].follow_num] - 1 - reverse_num; // ドローンのtarget_droのindexを更新
                    }
                    else
                    {
                        target_dro[i] = cir[infC_drone[i].follow_num][ind[infC_drone[i].follow_num] - S_N];
                        ind_dro[i] = ind[infC_drone[i].follow_num] - S_N;
                    }
                }

                infC_drone[i].xt = new_p[target_dro[i]].x; // ドローンの目的座標更新
                infC_drone[i].yt = new_p[target_dro[i]].y;

                /************ ドローンの充電・バッテリー交換処理 *************/
                // 次の避難所へ行く間に充電量が不足する場合は物資運搬車両で充電する
                if (infC_drone_flight_time[i] + retDis(infC_drone[i].xt, infC_drone[i].yt, infC_drone[i].x, infC_drone[i].y) * r_d_velo > capable_flight_time)
                {
                    // debug
                    // printf("drone[%d]の飛行時間: %lf [min]\n", i, infC_drone_flight_time[i] / 60);

                    // infC_drone[i].charge_time = infC_drone_flight_time[i] * charge_constant; // 充電時間を設定
                    infC_drone[i].charge_time = battery_swap_time; // バッテリー交換時間
                    infC_drone_flight_time[i] = 0;                 // ドローンの飛行時間を初期化

                    infC_drone[i].bat_swap_counter += 1; // バッテリー交換回数をカウント

                    infC_drone[i].bat_swap_onTV_flag = TRUE; // TV上でバッテリー交換を行う

                    v[infC_drone[i].bat_swap_follow_num].battery_count -= 1; // 物資運搬車両上でバッテリー交換を行うため、物資運搬車両のバッテリー数を減らす
                }

                // 物資運搬車両との情報交換
                /************************** 情報交換： 情報収集ドローン　→　物資運搬車両 *****************************/
                // 薬の情報配列の交換
                for (j = 0; j < N; j++)
                {
                    if (infC_drone[i].i_med_ptr[j] > v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j])
                    {
                        for (k = v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j]; k < infC_drone[i].i_med_ptr[j]; k++)
                        {
                            for (m = 0; m < Z_SIZE; m++)
                            {
                                v[infC_drone[i].bat_swap_follow_num].inf_med[j][k][m] = infC_drone[i].inf_med[j][k][m];
                            }
                            v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j] == Y_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }

                            // ドローンが情報を渡す物資運搬車両は、その物資運搬車両が担当する巡回路内の避難所に関する情報のみにする（その物資運搬車両から配送ドローンが飛行する）
                            if (j >= infC_drone[i].bat_swap_follow_num * 10 + 1 && j <= (infC_drone[i].bat_swap_follow_num + 1) * 10)
                            {
                                printf(" t=%lf : ドローン[%d]が物資運搬車両[%d]に 避難所[%d] の情報を伝達\n", total_t, i, infC_drone[i].bat_swap_follow_num, j);

                                printf(" %lf\n", v[infC_drone[i].bat_swap_follow_num].inf_med[j][v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j] - 1][0]);

                                // fprintf(fp_ETC_to_Vehicle, "t=%lf generate_time:%lf 避難所[%d]の情報 : drone[%d]->vehicle[%d] %lf\n", total_t, v[infC_drone[i].bat_swap_follow_num].inf_med[j][v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j] - 1][0], j, i, infC_drone[i].bat_swap_follow_num, total_t - v[infC_drone[i].bat_swap_follow_num].inf_med[j][v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j] - 1][0]);
                                fprintf(fp_ETC_to_Vehicle, "%lf\n", total_t - v[infC_drone[i].bat_swap_follow_num].inf_med[j][v[infC_drone[i].bat_swap_follow_num].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                            }
                        }
                    }
                }
            }
            /****************** 避難所へドローンが到達した場合 ***********************************************************************************************/
            else if (((n_cos_dro[i] < 0 && infC_drone[i].y < infC_drone[i].yt) || (n_cos_dro[i] > 0 && infC_drone[i].y > infC_drone[i].yt)) && infC_drone[i].FtoVehicle_mode == FALSE) // 目的の避難所に到達した場合
            {
                infC_drone[i].x = infC_drone[i].xt; // 座標修正
                infC_drone[i].y = infC_drone[i].yt;

                /**************************** S_N = 1のとき ***********************************************************************************/
                if (S_N == 1 && infC_drone[i].crossing_cir_flag == TRUE) // S_N=1のときは、常に巡回路をまたぐ飛行
                {
                    infC_drone[i].FtoVehicle_mode = TRUE; // 避難所から物資運搬車両へ情報を渡すモードにする（物資運搬車両に情報を渡しに行く場合）

                    solveConfluenceVer2(v[infC_drone[i].follow_num].x, v[infC_drone[i].follow_num].y, infC_drone[i].x, infC_drone[i].y, 1.0, v_d_ratio, new_p[target[infC_drone[i].follow_num]].x, new_p[target[infC_drone[i].follow_num]].y, &infC_drone[i].xt, &infC_drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, dis_stay_t, new_p, infC_drone, i, v, infC_drone[i].follow_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size); // follow_numにおける配送車との合流地点

                    // infC_drone[i].xt = new_p[target_dro[i]].x; // ドローンの目的座標更新
                    // infC_drone[i].yt = new_p[target_dro[i]].y;
                    current_dro[i] = target_dro[i]; // 現在の避難所を目的避難所にする
                    target_dro[i] = INF;            // 目的は物資運搬車両なので、target_dro[i]はINFにする
                }
                /************************* S_N = 1 以外のとき ***********************************************************************/
                else if (infC_drone[i].crossing_cir_flag == TRUE) // ドローンが巡回路間をまたいで飛行していた場合
                {
                    pass_count[i] += 1; // 避難所通過回数をカウントアップ

                    infC_drone[i].crossing_cir_flag = FALSE; // フラグをリセット

                    // ind_dro[i] += 1;
                    if (ind_dro[i] >= size[infC_drone[i].follow_num] - 1) // 目的が巡回路最後（集積所）である場合
                    {
                        ind_dro[i] = 1; // 巡回路の最初の避難所
                    }
                    else
                    {
                        ind_dro[i] += 1;
                    }

                    current_dro[i] = target_dro[i];
                    target_dro[i] = cir[infC_drone[i].follow_num][ind_dro[i]]; // 目的避難所変更
                    infC_drone[i].xt = new_p[target_dro[i]].x;                 // ドローンの目的座標更新
                    infC_drone[i].yt = new_p[target_dro[i]].y;
                }
                else // 同巡回路で S_N 箇所の避難所を巡回中のとき
                {
                    if (target_dro[i] != 0)
                    {
                        pass_count[i] += 1; // 避難所通過回数をカウントアップ
                    }

                    if (pass_count[i] == S_N) // 同巡回路で S_N 箇所の避難所を通過した場合は、物資運搬車両を経由したのち、次の隣の巡回路へ
                    {
                        infC_drone[i].FtoVehicle_mode = TRUE; // 避難所から物資運搬車両へ情報を渡すモードにする（物資運搬車両に情報を渡しに行く場合）

                        solveConfluenceVer2(v[infC_drone[i].follow_num].x, v[infC_drone[i].follow_num].y, infC_drone[i].x, infC_drone[i].y, 1.0, v_d_ratio, new_p[target[infC_drone[i].follow_num]].x, new_p[target[infC_drone[i].follow_num]].y, &infC_drone[i].xt, &infC_drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, dis_stay_t, new_p, infC_drone, i, v, infC_drone[i].follow_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size); // follow_numにおける配送車との合流地点

                        current_dro[i] = target_dro[i]; // 現在の避難所を目的避難所にする
                        target_dro[i] = INF;            // 目的は物資運搬車両なので、target_dro[i]はINFにする

                        // 物資運搬車両まで充電量足りなければバッテリー交換：バッテリー交換終わると改めて合流地点導出

                        // ここ以下は適宜消去
                        pass_count[i] = 0; // 避難所通過回数をリセット
                    }
                    else // pass_count[i]が S_N より小さい場合:同巡回路で S_N 箇所の避難所を巡回中のとき
                    {

                        if (ind_dro[i] >= size[infC_drone[i].follow_num] - 1) // 目的が巡回路最後（集積所）である場合
                        {
                            ind_dro[i] = 1; // 巡回路の最初の避難所
                        }
                        else
                        {
                            ind_dro[i] += 1;
                        }

                        current_dro[i] = target_dro[i];
                        target_dro[i] = cir[infC_drone[i].follow_num][ind_dro[i]]; // 目的避難所変更
                        infC_drone[i].x = new_p[current_dro[i]].x;                 // 座標修正
                        infC_drone[i].y = new_p[current_dro[i]].y;

                        infC_drone[i].xt = new_p[target_dro[i]].x; // ドローンの目的座標更新
                        infC_drone[i].yt = new_p[target_dro[i]].y;

                        part_t_dro[i] = 0;
                    }
                }

                /************ ドローンが避難所へ訪れた回数カウント ************/
                if (current_dro[i] != INF)
                {
                    infC_drone[i].shelter_visit_counter[current_dro[i]] += 1; // ドローンが避難所へ訪れた回数をカウント
                }

                /************ ドローンの充電・バッテリー交換処理 *************/
                // debug
                // infC_drone[i].charge_time = battery_swap_time; // 充電時間を5分に設定(テスト用)

                // 次の避難所へ行く間に充電量が不足する場合はその避難所で充電する
                if (infC_drone_flight_time[i] + retDis(infC_drone[i].xt, infC_drone[i].yt, infC_drone[i].x, infC_drone[i].y) * r_d_velo > capable_flight_time)
                {
                    // debug
                    // printf("drone[%d]の飛行時間: %lf [min]\n", i, infC_drone_flight_time[i] / 60);

                    // infC_drone[i].charge_time = infC_drone_flight_time[i] * charge_constant; // 充電時間を設定

                    infC_drone[i].charge_time = battery_swap_time; // バッテリー交換時間
                    infC_drone_flight_time[i] = 0;                 // ドローンの飛行時間を初期化

                    if (new_p[current_dro[i]].battery_count >= 1) // 避難所に交換バッテリーがある場合は、その場で交換
                    {
                        infC_drone[i].bat_swap_counter += 1; // バッテリー交換回数をカウント

                        new_p[current_dro[i]].battery_count -= 1; // 避難所のバッテリー数を減らす
                    }
                    else // 避難所に交換バッテリーがない場合は、TVによってバッテリーが運搬されてくるまで避難所で待機
                    {
                        infC_drone[i].batDel_wait_flag = TRUE; // バッテリー交換待ちフラグを立てる

                        printf("避難所[%d]におけるバッテリー数が不足しているためバッテリー交換不可\n", current_dro[i]);
                    }
                }

                /************************** 情報交換： 避難所　→　情報収集ドローン *****************************/
                // 避難所との薬の情報配列の交換(避難所→ドローン)
                for (j = 0; j < N; j++)
                {
                    if (new_p[current_dro[i]].i_med_ptr[j] > infC_drone[i].i_med_ptr[j]) // 同じ巡回路内の避難所の要求情報を回収
                    {
                        for (k = infC_drone[i].i_med_ptr[j]; k < new_p[current_dro[i]].i_med_ptr[j]; k++)
                        {
                            // if (TRUE)
                            if (new_p[current_dro[i]].inf_med[j][k][5] == FALSE) // 既に他の配送車やドローンに要求情報が回収されていなければ
                            {
                                new_p[current_dro[i]].inf_med[j][k][5] = TRUE; // 回収済みのフラグを立てる

                                for (m = 0; m < Z_SIZE; m++)
                                {
                                    infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j]][m] = new_p[current_dro[i]].inf_med[j][k][m]; // 薬の情報コピー
                                }
                                infC_drone[i].i_med_ptr[j] += 1;

                                // 医療品の配送先をキューに保存
                                // v[i].Med_delivery_queue[v[i].queue_ptr] = current[i]; // 医療品の配送先をキューに格納
                                // v[i].queue_ptr += 1;                                  // キューのポインタを進める
                                /*if (v[i].queue_ptr == QUEUE_SIZE)
                                {
                                    printf("キューの要素数オーバー\n");
                                    break;
                                }*/

                                printf("**** t=%lf : 避難所[%d]の情報を ドローン[%d]が回収\n", total_t, current_dro[i], i);
                                // debug
                                if (j == 12)
                                {
                                    printf(" 2 ドローン[%d]のi_med_ptr[%d]: %d , TV[1]のi_med_ptr[%d]: %d\n", i, j, infC_drone[i].i_med_ptr[j], j, v[1].i_med_ptr[j]);
                                }

                                // ファイルへの書き込み
                                // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf new_p[%d]->drone[%d]\n", total_t, infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0], current_dro[i], i);
                                // fprintf(fp_Medinf_delay, "%lf\n", total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]); // 生成されてからドローンで回収されるまでの遅延時間
                                // fprintf(fp_ETC_dro, "t=%lf generate_time:%lf new_p[%d]->drone[%d] %lf\n", total_t, infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0], current_dro[i], i, total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]);
                                fprintf(fp_ETC_dro, "%lf\n", total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]); // 生成されてからドローンで回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                                infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][4] = total_t;

                                // fprintf(fp_Medinf_collect_delay, "t=%lf generate_time:%lf new_p[%d]->drone[%d] %lf\n", total_t, infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0], current_dro[i], i, total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]);
                                fprintf(fp_Medinf_collect_delay, "%lf\n", total_t - infC_drone[i].inf_med[j][infC_drone[i].i_med_ptr[j] - 1][0]); // 生成されてからドローンで回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)

                                Medinf_collect_count++;     // 避難所から要求情報を回収した回数(全体)
                                Medinf_collect_count_dro++; // ドローンごとの避難所から要求情報を回収した回数
                            }
                        }
                    }
                }
            }
        }
        /***********************************************************************************************************************************************************************************************/

        /************************************************** 物資運搬車両の避難所 or 集積所への到達判定処理 ***********************************************************/
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

                v[i].battery_count = DELIVERY_BATTERY_COUNT * ((N - 1) / M) + ADDITIONAL_BATTERY_COUNT; // 集積所にてバッテリー補充：配送車のバッテリー数を初期化
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

                    v[i].battery_count -= DELIVERY_BATTERY_COUNT;              // 配送車のバッテリー数を減らす
                    new_p[current[i]].battery_count += DELIVERY_BATTERY_COUNT; // 避難所のバッテリー数を増やす

                    if (v[i].re >= new_p[current[i]].re_req) // 配送車が避難所に物資を要求分届けることができるとき
                    {
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

                        // 避難所情報（薬の情報）の回収
                        if (new_p[current[i]].i_med_ptr[j] > v[i].i_med_ptr[j])
                        {
                            for (k = v[i].i_med_ptr[j]; k < new_p[current[i]].i_med_ptr[j]; k++)
                            {
                                // 既に他の配送車やドローンに要求情報が回収されていなければ(TV or ドローンが避難所から初めて要求情報を取得するときのみ)
                                if (new_p[current[i]].inf_med[j][k][5] == FALSE)
                                {
                                    new_p[current[i]].inf_med[j][k][5] = TRUE; // 回収済みのフラグを立てる

                                    for (m = 0; m < Z_SIZE; m++)
                                    {
                                        v[i].inf_med[j][v[i].i_med_ptr[j]][m] = new_p[current[i]].inf_med[j][k][m]; // 薬の情報コピー
                                    }
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
                                    printf("t=%lf : 避難所[%d]の情報を 配送車[%d]が回収\n", total_t, current[i], i);

                                    // ファイルへの書き込み
                                    // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf new_p[%d]->v[%d]\n", total_t, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0], current[i], i);
                                    fprintf(fp_Medinf_delay, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間
                                    // fprintf(fp_Medinf_collect_delay, "t=%lf generate_time:%lf new_p[%d]->v[%d]\n", total_t, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0], current[i], i);
                                    fprintf(fp_Medinf_collect_delay, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                                    // E(Tc) Vehicleについてのファイル書き込み追加
                                    fprintf(fp_ETC_Vehicle, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                                    // fprintf(fp_ETC_to_Vehicle, "t=%lf generate_time:%lf 避難所[%d]の情報 : vehicle[%d] %lf\n", total_t, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0], j, i, total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]);
                                    fprintf(fp_ETC_to_Vehicle, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間(避難所から配送車への情報共有の遅延時間)
                                    v[i].inf_med[j][v[i].i_med_ptr[j] - 1][2] = total_t;                                      // 要求情報発生から回収までの遅延時間（避難所→TV）

                                    Medinf_collect_count++; // 避難所から要求情報を回収した回数(全体)
                                }
                            }
                        }
                    }

                    /**************************配送車による医療品の避難所への配達処理***********************************/
                    if (v[i].Med_re > 0 && v[i].i_med_ptr[current[i]] > 0 && v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][3] != TRUE && current[i] == v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]) // 配送車が集積所で物資を積載してまた戻って来たなら
                    {
                        // 医療品の配達
                        v[i].Med_re -= 1;
                        v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][3] = TRUE; // 配送車が医療品を届けたことを記録
                        // printf("t=%.2lf : 配送車[%d]から避難所[%d]医療品配達 残り医療物資量:%d\n", total_t, i, current[i], v[i].Med_re);
                        v[i].queue_Notdelivery_ptr += 1; // 配達が完了したキュー内の避難所のポインタを進める

                        counter_Med_re_delivery++; // 医療品の配送回数をカウント

                        // ファイルへの書き込み処理
                        fprintf(fp_Med_re_delay, "%lf\n", total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][0]);
                        // fprintf(fp_Med_re_delay, "t=%lf v[%d] -> new_p[%d] : %lf\n", total_t, i, current[i], total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][0]);
                        fprintf(fp_Med_re_collect_to_delivery_delay, "%lf\n", total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][2]); // 医療品の収集から配送への遅延時間
                        // fprintf(fp_Med_re_collect_to_delivery_delay, "t=%lf v[%d] -> new_p[%d] : %lf\n", total_t, i, current[i], total_t - v[i].inf_med[current[i]][v[i].i_med_ptr[current[i]] - 1][2]); // 医療品の収集から配送への遅延時間
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

// ドローンがTVから飛行して医療物資届ける手法（5月23日報告会までの手法）
#if 0
        /************************************ドローンの医療物資運搬のための飛行開始に関する処理**************************************************/
        for (i = 0; i < M; i++)
        {
            if (v[i].queue_ptr != v[i].queue_Notdelivery_ptr) // 配送車が避難所の医療物資要求（未配達）の情報を保持しているとき
            {
                for (k = 0; k < SD; k++)
                {
                    if (drone[k].follow_num == i && drone[k].free_mode == FALSE && drone[k].FtoDiscenter_mode == FALSE && drone[k].charge_time == 0 && v[i].x == new_p[v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]].x && v[i].y == new_p[v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]].y) // 現在要求情報を受け取った配送車上にいながら，充電中ではなく飛行可能なとき、かつその避難所に車がいるとき
                    {
                        drone[k].target_shelter_num = v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]; // ドローンの目的地を現在の配送車の避難所(キューの医療物資配送最優先の避難所)に設定

                        drone[k].xt = new_p[0].x;
                        drone[k].yt = new_p[0].y; // ドローンの目的地を配送センターに設定

                        d_d[k] = sqrt(pow(drone[k].xt - drone[k].x, 2) + pow(drone[k].yt - drone[k].y, 2)); // ドローンと集積所との距離算出

                        d_s_dis = sqrt(pow(new_p[v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]].x - new_p[0].x, 2) + pow(new_p[v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]].y - new_p[0].y, 2)); // 避難所と集積所との距離算出

                        // ドローンが充電量で集積所へ飛行して医療物資を補充し避難所へ戻ってくることができる距離 かつ 配送車が避難所で荷降ろししている間に戻ってくることができるなら
                        if (2 * d_d[k] * r_d_velo <= capable_flight_time && 2 * d_d[k] * r_d_velo + drone_Med_loding_time <= stay)
                        {
                            // stay_t[i] += drone_Med_Unloding_time; // ドローンの荷降ろし時間を考慮して配送車の待機時間を延長する
                            // ドローンの医療品の荷降ろし中に物資運搬車両の荷降ろしが終わる場合追加で待機
                            if (2 * d_d[k] * r_d_velo + drone_Med_loding_time + drone_Med_Unloding_time >= stay)
                            {
                                addtional_time = (2 * d_d[k] * r_d_velo + drone_Med_loding_time + drone_Med_Unloding_time) - stay;
                                stay_t[i] += (double)((int)(addtional_time) - (int)(addtional_time) % 10); // ドローンの飛行時間を考慮して配送車の待機時間を延長する
                            }

                            // printf("drone[%d]が集積所へ医療物資を補給しに飛行を開始\n", k);
                            // printf("t=%.2lf : 避難所[%d]にて 配送車[%d] 上の ドローン[%d] が集積所へ飛行開始\n", total_t, v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr], i, k);
                            drone[k].free_mode = TRUE;         // ドローンのフリーモードを有効にする
                            drone[k].FtoDiscenter_mode = TRUE; // ドローンの配送モードを有効にする

                            v[i].inf_med[drone[k].target_shelter_num][v[i].i_med_ptr[drone[k].target_shelter_num] - 1][3] = TRUE; // ドローンが避難所に医療品を届けるため医療品配達フラグを有効にする（配送車は集積所で補充しなくてもよい）
                            v[i].queue_Notdelivery_ptr += 1;                                                                      // 配達が完了したキューの避難所のポインタを進める
                        }
                        else if (2 * d_d[k] * r_d_velo <= capable_flight_time && 2 * d_d[k] * r_d_velo + drone_Med_loding_time > stay) // ドローンが充電量で集積所へ飛行して医療物資を補充し避難所へ戻ってくることができる距離 かつ 配送車が避難所で荷降ろししている間に戻ってくることができるないなら
                        {
                            stay_t[i] += drone_Med_Unloding_time; // ドローンの荷降ろし時間を考慮して配送車の待機時間を延長する

                            // printf("t=%.2lf : 避難所[%d]にて 配送車[%d] 上の ドローン[%d] が集積所へ飛行開始\n", total_t, v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr], i, k);
                            // printf("-> 配送車[%d]の待機時間 %.2lf を %.2lf [min]延長 :延長後待機時間%.2lf\n", i, stay_t[i] / 60, ((2 * d_d[k] * r_d_velo + drone_Med_loding_time) - stay) / 60, (stay_t[i] + ((2 * d_d[k] * r_d_velo + drone_Med_loding_time) - stay)) / 60);

                            addtional_time = (2 * d_d[k] * r_d_velo + drone_Med_loding_time) - stay;
                            stay_t[i] += (double)((int)(addtional_time) - (int)(addtional_time) % 10); // ドローンの飛行時間を考慮して配送車の待機時間を延長する
                            //((int)(poisson_re_total) - (int)(poisson_re_total) % 10)

                            printf("%lf [s]\n", (double)((int)(addtional_time) - (int)(addtional_time) % 10));

                            drone[k].free_mode = TRUE;         // ドローンのフリーモードを有効にする
                            drone[k].FtoDiscenter_mode = TRUE; // ドローンの配送モードを有効にする

                            v[i].inf_med[drone[k].target_shelter_num][v[i].i_med_ptr[drone[k].target_shelter_num] - 1][3] = TRUE; // ドローンが避難所に医療品を届けるため医療品配達フラグを有効にする（配送車は集積所で補充しなくてもよい）
                            v[i].queue_Notdelivery_ptr += 1;
                        }
                        else // 距離的に飛行不可能である場合
                        {
                            drone[k].cannot_fly_judge_flag = TRUE; // ドローンの飛行不可能フラグを有効にする

                            // printf("t=%.2lf : 配送車[%d] 上の ドローン[%d] が避難所[%d] のため集積所へ飛行不能と判断\n", total_t, i, k, v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]);
                        }
                    }
                }
            }
        }

        /*********************　新手法ver2 充電制限により飛行不能で配送車で待機中だったドローンが配送車の移動により、距離的に飛行可能と判断する処理　**************************/
        for (i = 0; i < M; i++)
        {
            if (v[i].queue_ptr != v[i].queue_Notdelivery_ptr) // 配送車が避難所の医療物資要求（未配達）の情報を保持しているとき
            {
                for (k = 0; k < SD; k++)
                {
                    if (drone[k].follow_num == i && drone[k].free_mode == FALSE && drone[k].FtoDiscenter_mode == FALSE && drone[k].charge_time == 0 && drone[k].cannot_fly_judge_flag == TRUE) // 現在要求情報を受け取った配送車上にいながら，充電中ではなく飛行可能なとき、かつ飛行不可能判断フラグが有効なとき
                    {
                        drone[k].xt = new_p[0].x;
                        drone[k].yt = new_p[0].y; // ドローンの目的地を配送センターに設定

                        d_d[k] = sqrt(pow(drone[k].xt - drone[k].x, 2) + pow(drone[k].yt - drone[k].y, 2)); // ドローンと集積所との距離算出

                        d_s_dis = sqrt(pow(new_p[v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]].x - new_p[0].x, 2) + pow(new_p[v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]].y - new_p[0].y, 2)); // 避難所と集積所との距離算出

                        if ((d_d[k] * r_d_velo) + (d_s_dis * r_d_velo) <= capable_flight_time) // 配送車の移動により、集積所へ近づいて避難所へ飛行可能になった場合（集積所-避難所間の飛行は可能な場合）
                        {
                            // printf("t=%.2lf : 配送車[%d] で待機中の ドローン[%d] が集積所へ飛行開始：避難所[%d] のため\n", total_t, i, k, v[i].Med_delivery_queue[v[i].queue_Notdelivery_ptr]);

                            drone[k].free_mode = TRUE;              // ドローンのフリーモードを有効にする
                            drone[k].FtoDiscenter_mode = TRUE;      // ドローンの配送モードを有効にする
                            drone[k].TV_wait_flag = TRUE;           // ドローンの避難所待機フラグを有効にする
                            drone[k].cannot_fly_judge_flag = FALSE; // ドローンの飛行不可能フラグを無効にする

                            v[i].inf_med[drone[k].target_shelter_num][v[i].i_med_ptr[drone[k].target_shelter_num] - 1][3] = TRUE; // ドローンが避難所に医療品を届けるため医療品配達フラグを有効にする（配送車は集積所で補充しなくてもよい）
                            v[i].queue_Notdelivery_ptr += 1;
                        }
                    }
                }
            }
        }
#endif

        /******************************************** ドローンの制御 **********************************************************/
#if 0
        // 配送車0（ドローン積載）が一番始めの避難所１に到達したら
        if (fabs(v[0].x - new_p[shelter_num[0]].x) < 0.001 && fabs(v[0].y - new_p[shelter_num[0]].y) < 0.001 && drone_depature_flag[0] == FALSE)
        {
            drone[0].free_mode = TRUE;
            drone_depature_flag[0] = TRUE;

            for (i = 1; i < SD; i++)
            {
                drone[i].flight_start_time = (flight_time_lag * i); // ドローンの飛行開始時間を時間差を考慮して導出
            }

            for (i = 0; i < 1; i++) // ドローン1のみ
            {
                // ドローンと配送車の合流地点算出
                solveConfluence(v[drone[i].target_num].x, v[drone[i].target_num].y, drone[i].x, drone[i].y, 1.0, v_d_ratio, new_p[target[drone[i].target_num]].x, new_p[target[drone[i].target_num]].y, &drone[i].xt, &drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, new_p, drone, i, v, drone[i].target_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size);
            }
        }

        /******************************* ドローン1 以降は時間差をおいて飛行開始 **********************************/
        for (i = 1; i < SD; i++)
        {
            if (drone_depature_flag[0] == TRUE && (int)(drone[i].flight_start_time) == 0 && drone_depature_flag[i] == FALSE) // ドローン１が飛行済みで時間差分だけ時間が経過したら
            {
                drone_depature_flag[i] = TRUE;
                drone[i].free_mode = TRUE;

                // ドローンと配送車の合流地点算出
                solveConfluence(v[drone[i].target_num].x, v[drone[i].target_num].y, drone[i].x, drone[i].y, 1.0, v_d_ratio, new_p[target[drone[i].target_num]].x, new_p[target[drone[i].target_num]].y, &drone[i].xt, &drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, new_p, drone, i, v, drone[i].target_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size);
            }
        }
#endif
        /************************************************* ドローンの飛行に関する処理（ free_mode によって場合分け） ******************************************************************/
        for (i = 0; i < SD; i++)
        {
            if (drone[i].free_mode == FALSE) // free_modeオフの時
            {                                // 自由飛行モードでないなら配送車に従う
                drone[i].x = v[drone[i].follow_num].x;
                drone[i].y = v[drone[i].follow_num].y;

                if (drone[i].charge_time != 0)
                {
                    drone[i].charge_time -= time_span; // 充電が終わっていなければ充電
                }

                if (drone[i].flight_start_time != 0)
                {
                    drone[i].flight_start_time -= time_span; // 飛行開始時間の時間差の減算
                }
            }
            else if (drone[i].free_mode == TRUE && drone[i].charge_time != 0 && drone[i].TV_wait_flag == FALSE) // ドローンが配送車上にいて、充電中の時
            {
                // ドローンが配送車で充電中なら配送車に従い, 飛行時間の定数倍分充電
                drone[i].x = v[drone[i].follow_num].x;
                drone[i].y = v[drone[i].follow_num].y;

                drone[i].charge_time -= time_span; // 充電時間減算

                // 充電が終了したら
                if (drone[i].charge_time == 0)
                {
                    // ドローンと配送車の合流地点算出
                    // solveConfluence(v[drone[i].target_num].x, v[drone[i].target_num].y, drone[i].x, drone[i].y, 1.0, v_d_ratio, new_p[target[drone[i].target_num]].x, new_p[target[drone[i].target_num]].y, &drone[i].xt, &drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, new_p, drone, i, v, drone[i].target_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size);
                    drone[i].free_mode = FALSE; // ドローンのフリーモードを無効にする
                }
            }
            else if (drone[i].free_mode == TRUE && drone[i].charge_time != 0 && drone[i].TV_wait_flag == TRUE) // ドローンが避難所で配送車を待機中のとき（充電が必要）：配送車がくるまで医療物資を届けた避難所で待機
            {
                if (drone[i].x == v[drone[i].follow_num].x && drone[i].y == v[drone[i].follow_num].y)
                {
                    drone[i].TV_wait_flag = FALSE; // 配送車と合流したら充電開始（ドローンの避難所待機フラグを無効にする）
                    // printf("t=%.2lf : ドローン[%d] が避難所[%d] で配送車 [%d]と合流完了. 充電時間：%lf [min]\n", total_t, i, drone[i].target_shelter_num, drone[i].follow_num, (double)drone[i].charge_time / 60);
                }
            }
            else if (drone[i].free_mode == TRUE && drone[i].FtoDiscenter_mode == TRUE && drone[i].delivery_mode == FALSE) // ドローンが集積所に医療物資を補充しに行くモードに移行したら
            {
                // ドローンが中央の集積所に向かって飛行
                // printf("xt:%lf yt:%lf\n", drone[i].xt, drone[i].yt);
                d_d[i] = sqrt(pow(drone[i].xt - drone[i].x, 2) + pow(drone[i].yt - drone[i].y, 2));
                d_n_sin[i] = (drone[i].xt - drone[i].x) / d_d[i];
                d_n_cos[i] = (drone[i].yt - drone[i].y) / d_d[i];

                drone[i].x = drone[i].x + d_n_sin[i] * time_span / r_d_velo;
                drone[i].y = drone[i].y + d_n_cos[i] * time_span / r_d_velo;

                drone_flight_distanece[i] += time_span / r_d_velo;

                flight_time[i] += time_span; // 飛行時間加算

                /************ ドローンが集積所に到着したら **********/
                if ((d_n_cos[i] < 0 && drone[i].y < drone[i].yt) || (d_n_cos[i] > 0 && drone[i].y > drone[i].yt))
                {
                    drone[i].x = drone[i].xt; // 座標修正
                    drone[i].y = drone[i].yt;

                    drone[i].xt = new_p[drone[i].target_shelter_num].x; // ドローンの目的地を避難所に設定
                    drone[i].yt = new_p[drone[i].target_shelter_num].y; // ドローンの目的地を避難所に設定

                    drone[i].FtoDiscenter_mode = FALSE; // ドローンの医療品回収モードを無効にする

                    drone[i].delivery_mode = TRUE; // ドローンの配送モードを有効にする

                    // 医療物資積載に関する処理
                    drone[i].Med_re += 1;

                    // ドローンの集積所での医療物資積載時間セット
                    drone[i].stay_Medload_time = drone_Med_loding_time; // ドローンの集積所での医療物資積載時間
                }
            }
            else if (drone[i].free_mode == TRUE && drone[i].FtoDiscenter_mode == FALSE && drone[i].delivery_mode == TRUE && drone[i].stay_Medload_time != 0) // ドローンが集積所で医療物資を積載中の時
            {
                drone[i].stay_Medload_time -= time_span; // 医療物資積載時間減算

                if ((int)drone[i].stay_Medload_time == 0)
                {
                    // ドローンの医療物資積載時間が終了したら
                    drone[i].stay_Medload_time = 0; // 医療物資積載時間初期化
                    // printf("t=%.2lf : 集積所にて ドローン[%d] が医療物資を積載完了\n", total_t, i);
                }
            }
            else if (drone[i].free_mode == TRUE && drone[i].FtoDiscenter_mode == FALSE && drone[i].delivery_mode == TRUE && drone[i].stay_Medload_time == 0) // ドローンが避難所に医療物資を届けるモードに移行したら
            {
                // ドローンが避難所に向かって飛行
                d_d[i] = sqrt(pow(drone[i].xt - drone[i].x, 2) + pow(drone[i].yt - drone[i].y, 2));
                d_n_sin[i] = (drone[i].xt - drone[i].x) / d_d[i];
                d_n_cos[i] = (drone[i].yt - drone[i].y) / d_d[i];

                drone[i].x = drone[i].x + d_n_sin[i] * time_span / r_d_velo;
                drone[i].y = drone[i].y + d_n_cos[i] * time_span / r_d_velo;

                drone_flight_distanece[i] += time_span / r_d_velo;

                flight_time[i] += time_span; // 飛行時間加算

                /************ ドローンが目的の避難所に到着したら **********/
                if ((d_n_cos[i] < 0 && drone[i].y < drone[i].yt) || (d_n_cos[i] > 0 && drone[i].y > drone[i].yt))
                {
                    drone[i].x = drone[i].xt; // 座標修正
                    drone[i].y = drone[i].yt;

                    // 飛行時間処理
                    total_flight_time[drone[i].follow_num] += flight_time[i];
                    flight_count[drone[i].follow_num] += 1;

                    // drone[i].free_mode = FALSE;     // ドローンのフリーモードを無効にする
                    drone[i].delivery_mode = FALSE; // ドローンの配送モードを無効にする

                    // ドローンの充電時間処理
                    drone[i].charge_time = flight_time[i] * charge_constant; // ドローンの配送車での充電時間

                    // printf("t=%.2lf : 避難所[%d]にて ドローン[%d] が医療物資を届けた 充電時間 : %lf[min]\n", total_t, drone[i].target_shelter_num, i, (double)drone[i].charge_time / 60);

                    flight_time[i] = 0; // ドローンの飛行時間初期化

                    drone[i].Med_re -= 1; // ドローンの医療物資量を減少

                    counter_Med_re_delivery++;       // 医療品の配送回数をカウント
                    counter_Med_re_Drone_delivery++; // ドローンによる医療品の配送回数をカウント

                    // ドローンによる医療物資の配送遅延時間のファイルへの書き込み
                    fprintf(fp_Med_re_delay, "%lf\n", total_t - v[drone[i].follow_num].inf_med[drone[i].target_shelter_num][v[drone[i].follow_num].i_med_ptr[drone[i].target_shelter_num] - 1][0]);
                    // fprintf(fp_Med_re_delay, "t=%lf drone[%d] -> new_p[%d] : %lf\n", total_t, i, drone[i].target_shelter_num, total_t - v[drone[i].follow_num].inf_med[drone[i].target_shelter_num][v[drone[i].follow_num].i_med_ptr[drone[i].target_shelter_num] - 1][0]);

                    fprintf(fp_Med_re_collect_to_delivery_delay, "%lf\n", total_t - v[drone[i].follow_num].inf_med[drone[i].target_shelter_num][v[drone[i].follow_num].i_med_ptr[drone[i].target_shelter_num] - 1][2]); // 医療品の収集から配送への遅延時間                                                                                                                                               // 医療品の収集から配送への遅延時間
                    // fprintf(fp_Med_re_collect_to_delivery_delay, "t=%lf drone[%d] -> new_p[%d] : %lf\n", total_t, i, drone[i].target_shelter_num, total_t - v[drone[i].follow_num].inf_med[drone[i].target_shelter_num][v[drone[i].follow_num].i_med_ptr[drone[i].target_shelter_num] - 1][2]); // 医療品の収集から配送への遅延時間

                    if (drone[i].TV_wait_flag == TRUE) // ドローンが避難所で配送車を待機する必要があるとき
                    {
                        // printf("t=%.2lf : 避難所[%d]にて ドローン[%d] が配送車 [%d] を待機開始\n", total_t, drone[i].target_shelter_num, i, drone[i].follow_num);
                    }
                }
            }
#if 0
            else if (drone[i].free_mode == TRUE && drone[i].charge_time == 0)
            { // 自由飛行モードならtargetの配送車に向かって飛行

                d_d[i] = sqrt(pow(drone[i].xt - drone[i].x, 2) + pow(drone[i].yt - drone[i].y, 2));
                d_n_sin[i] = (drone[i].xt - drone[i].x) / d_d[i];
                d_n_cos[i] = (drone[i].yt - drone[i].y) / d_d[i];

                drone[i].x = drone[i].x + d_n_sin[i] * time_span / r_d_velo;
                drone[i].y = drone[i].y + d_n_cos[i] * time_span / r_d_velo;

                drone_flight_distanece[i] += time_span / r_d_velo;

                flight_time[i] += time_span; // 飛行時間加算

                /************ ドローンが目的の配送車に到着したら **********/
                if ((d_n_cos[i] < 0 && drone[i].y < drone[i].yt) || (d_n_cos[i] > 0 && drone[i].y > drone[i].yt))
                {
                    drone[i].x = drone[i].xt; // 座標修正
                    drone[i].y = drone[i].yt;

                    // 飛行時間処理
                    drone[i].charge_time = flight_time[i] * charge_constant; // ドローンの配送車での充電時間
                    // printf("飛行時間:%f[min]\n", flight_time[i] / 60);

                    // TVの総充電時間(単位[min])
                    if (v[drone[i].target_num].chargeable_flag == TRUE) // 充電可能フラグが立っていれば総充電に加算
                    {
                        v[drone[i].target_num].charge_amount += drone[i].charge_time / 60;
                    }

                    // printf("ドローン%dの飛行時間:%lf[min]\n", i, flight_time[i] / 60);

                    if (v[drone[i].target_num].charge_amount >= MAX_TVchargeable && v[drone[i].target_num].chargeable_flag == TRUE) // TVの総ドローン充電量が規定量を超えた場合は充電不可能フラグをたてる
                    {
                        v[drone[i].target_num].chargeable_flag = FALSE;
                        // printf("TV[%d]の総充電時間:%lf[min]  -> %lf[min]\n", drone[i].target_num, v[drone[i].target_num].charge_amount - (drone[i].charge_time / 60), v[drone[i].target_num].charge_amount);
                        v[drone[i].target_num].charge_amount -= drone[i].charge_time / 60; // 超過分の総充電量を戻す
                    }

                    // printf("TV[%d]の総充電時間:%lf\n", drone[i].target_num, v[drone[i].target_num].charge_amount);

                    /***************************** TVにおけるドローンの充電制限に関する処理 ***********************************************************************************/
                    if (v[drone[i].target_num].chargeable_flag == FALSE) // TVでの充電量が規定量を超えた場合はそのドローンは充電できずに待機
                    {
                        drone[i].free_mode = FALSE;
                        flight_time[i] = 0; // 飛行時間初期化初期化
                        // printf("drone[%d]はTV[%d]にて飛行やめる\n", i, drone[i].target_num);
                    }
                    else // TVでまだ充電可能な場合
                    {

                        // 飛行時間に関する処理約を生成します
                        total_flight_time[i] += flight_time[i];
                        flight_count[i] += 1;
                        flight_time[i] = 0;

                        // 充電回数処理
                        v[drone[i].target_num].drone_charge_count += 1; // ドローンがTVに到着するとそのTVの充電回数を + 1
                    }

                    /************************** 配送車 -> ドローン **********************************/

                    // 避難所情報の交換
                    for (j = 0; j < N; j++)
                    {
                        if (v[drone[i].target_num].i_ptr[j] > drone[i].i_ptr[j])
                        {
                            for (k = drone[i].i_ptr[j]; k < v[drone[i].target_num].i_ptr[j]; k++)
                            {
                                drone[i].inf[j][k] = v[drone[i].target_num].inf[j][k];
                                drone[i].i_ptr[j] += 1;
                                // 配列の容量オーバー
                                if (drone[i].i_ptr[j] == I_SIZE)
                                {
                                    printf("配列要素数オーバー\n");
                                    break;
                                }
                            }
                        }
                    }

                    /***************************************** ドローン -> 配送車 *****************************************************/
                    for (j = 0; j < N; j++)
                    {
                        if (drone[i].i_ptr[j] > v[drone[i].target_num].i_ptr[j])
                        {
                            for (k = v[drone[i].target_num].i_ptr[j]; k < drone[i].i_ptr[j]; k++)
                            {
                                v[drone[i].target_num].inf[j][k] = drone[i].inf[j][k];
                                v[drone[i].target_num].i_ptr[j] += 1;
                                // 配列の容量オーバー
                                if (v[drone[i].target_num].i_ptr[j] == I_SIZE)
                                {
                                    printf("配列要素数オーバー\n");
                                    break;
                                }
                            }
                        }
                    }

                    if (drone[i].target_num == current_returnnum[i]) // ドローンが一周してきたら
                    {

                        drone[i].follow_num = drone[i].target_num; // follow_num更新

                        drone_roop_count[i] += 1; // ドローン周回数カウント

                        drone[i].target_num = drone_next_target[drone[i].follow_num];

                        // ドローンと配送車の合流地点算出
                        solveConfluence(v[drone[i].target_num].x, v[drone[i].target_num].y, drone[i].x, drone[i].y, 1.0, v_d_ratio, new_p[target[drone[i].target_num]].x, new_p[target[drone[i].target_num]].y, &drone[i].xt, &drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, new_p, drone, i, v, drone[i].target_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size);
                    }
                    else /************ ドローンが一周していないとき ******************************/
                    {
                        drone[i].follow_num = drone[i].target_num; // follow_num更新

                        drone[i].target_num = drone_next_target[drone[i].follow_num];

                        // ドローンと配送車の合流地点算出
                        solveConfluence(v[drone[i].target_num].x, v[drone[i].target_num].y, drone[i].x, drone[i].y, 1.0, v_d_ratio, new_p[target[drone[i].target_num]].x, new_p[target[drone[i].target_num]].y, &drone[i].xt, &drone[i].yt, v_d_ratio, r_d_velo, r_velo, stay_t, new_p, drone, i, v, drone[i].target_num, current, target, cir, cir_flag, ind, ind_last, ind_relief, size);
                    }
                }
            }
#endif
        }

        /**************** 配送センターにすべての配送車が集まった場合 ********************/
        if (total_vehicle_num == M)
        {

            // printf("t_wait: %f\n", findMax(total_time_trip, M) - findMin(total_time_trip, M));

            // follow,target,depature_flag初期化
            for (i = 0; i < SD; i++)
            {
                drone[i].follow_num = i % M;

                drone[i].target_num = drone_next_target[drone[i].follow_num];

                drone_depature_flag[i] = FALSE;
            }

            total_vehicle_num = 0;

            for (i = 0; i < SD; i++)
            {
                drone_roop_count[i] = 0; // ドローン周回カウンタ初期化
            }

            /******************TVにおけるドローンの充電回数制限に関する処理*****************/
            for (i = 0; i < M; i++)
            {
                // printf("TV[%d]のドローン充電回数：%d\n", i, v[i].drone_charge_count);
                v[i].drone_charge_count = 0; // TVの充電回数初期化
            }

            /**********************************TVにおける総充電時間に関する処理(一巡回)*************************************** */
            for (i = 0; i < M; i++)
            {
                TV_chargeAmount += v[i].charge_amount;
                // printf("TV[%d]のドローン総充電時間：%lf\n", i, v[i].charge_amount);
                v[i].charge_amount = 0;      // TVの充電量初期化
                v[i].chargeable_flag = TRUE; // TVの充電可能フラグ初期化
            }
            ave_TV_chargeAmount = TV_chargeAmount / M; // 一巡回におけるTVの平均総ドローン充電時間
            TV_chargeAmount = 0;

            // 医療品の積載について
            for (int i = 0; i < M; i++)
            {
                for (j = 1; j < N; j++)
                {
                    if (v[i].i_med_ptr[j] > 0 && v[i].inf_med[j][v[i].i_med_ptr[j] - 1][3] != TRUE && j <= (i + 1) * 10 && j >= i * 10 + 1) // 巡回経路上の物資を荷降ろしする避難所内で医療物資要求があったにも関わらずまだ医療品を届けることができていないとき
                    {
                        v[i].Med_re += 1;
                        // printf("t=%.2lf : 配送車%d:避難所[%d]への物資積載\n", total_t, i, j);
                        //  debug
                        if (j == 13)
                        {
                            printf("flag: %lf\n", v[i].inf_med[j][v[i].i_med_ptr[j] - 1][3]);
                        }
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

                                // v[j].inf_med[k][m][0] = v[i].inf_med[k][m][0];
                                // v[j].inf_med[k][m][1] = v[i].inf_med[k][m][1];
                                for (n = 0; n < Z_SIZE; n++)
                                {
                                    v[j].inf_med[k][m][n] = v[i].inf_med[k][m][n];
                                }
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

        /************************** 配送車 -> ドローン (ドローンが配送車にいるとき(followモード)または, 充電中のときは 配送車の情報をドローンにコピー) **********************************/

        for (i = 0; i < SD; i++)
        {
            if (drone[i].free_mode == FALSE || (drone[i].free_mode == TRUE && drone[i].charge_time != 0))
            { // followモードならコピー
                // 避難所情報を交換
                for (j = 0; j < N; j++)
                {
                    if (v[drone[i].follow_num].i_ptr[j] > drone[i].i_ptr[j])
                    {
                        for (k = drone[i].i_ptr[j]; k < v[drone[i].follow_num].i_ptr[j]; k++)
                        {
                            drone[i].inf[j][k] = v[drone[i].follow_num].inf[j][k];
                            drone[i].i_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (drone[i].i_ptr[j] == I_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }

                // 薬の情報配列の交換
                for (j = 0; j < N; j++)
                {
                    if (v[drone[i].follow_num].i_med_ptr[j] > drone[i].i_med_ptr[j])
                    {
                        for (k = drone[i].i_med_ptr[j]; k < v[drone[i].follow_num].i_med_ptr[j]; k++)
                        {
                            // drone[i].inf_med[j][k][0] = v[drone[i].follow_num].inf_med[j][k][0]; // 生成時間の情報
                            // drone[i].inf_med[j][k][1] = v[drone[i].follow_num].inf_med[j][k][1]; // 緊急度の情報
                            for (m = 0; m < Z_SIZE; m++)
                            {
                                drone[i].inf_med[j][k][m] = v[drone[i].follow_num].inf_med[j][k][m];
                            }
                            drone[i].i_med_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (drone[i].i_med_ptr[j] == Y_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }
            }
        }

        /*************************** ドローン -> 配送車(ドローンが配送車にいるとき(followモード)または充電中のときは 配送車の情報をドローンにコピー) *****************************/
        for (i = 0; i < SD; i++)
        {
            if (drone[i].free_mode == FALSE || (drone[i].free_mode == TRUE && drone[i].charge_time != 0))
            { // followモードならコピー
                // 　避難所情報の交換
                for (j = 0; j < N; j++)
                {
                    if (drone[i].i_ptr[j] > v[drone[i].follow_num].i_ptr[j])
                    {
                        for (k = v[drone[i].follow_num].i_ptr[j]; k < drone[i].i_ptr[j]; k++)
                        {
                            v[drone[i].follow_num].inf[j][k] = drone[i].inf[j][k] + 2;
                            v[drone[i].follow_num].i_ptr[j] += 1;
                            // 配列の容量オーバー
                            if (v[drone[i].follow_num].i_ptr[j] == I_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }

                // 薬の情報配列を交換

                for (j = 0; j < N; j++)
                {
                    if (drone[i].i_med_ptr[j] > v[drone[i].follow_num].i_med_ptr[j])
                    {
                        for (k = v[drone[i].follow_num].i_med_ptr[j]; k < drone[i].i_med_ptr[j]; k++)
                        {
                            // v[drone[i].follow_num].inf_med[j][k][0] = drone[i].inf_med[j][k][0]; // 生成時間情報
                            // v[drone[i].follow_num].inf_med[j][k][1] = drone[i].inf_med[j][k][1]; // 緊急度情報
                            for (m = 0; m < Z_SIZE; m++)
                            {
                                v[drone[i].follow_num].inf_med[j][k][m] = drone[i].inf_med[j][k][m];
                            }
                            v[drone[i].follow_num].i_med_ptr[j] += 1;
                            //   配列の容量オーバー
                            if (v[drone[i].follow_num].i_med_ptr[j] == Y_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }

                            // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf new_p[%d]->v[%d]\n", total_t, v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0], current[i], i);
                            // fprintf(fp_Medinf_delay, "%lf\n", total_t - v[i].inf_med[j][v[i].i_med_ptr[j] - 1][0]); // 生成されてから配送車で回収されるまでの遅延時間
                            // fprintf(fp_Medinf_delay, "t=%lf generate_time:%lf drone[%d]->v[%d]\n", total_t, drone[i].inf_med[j][k][0], i, drone[i].follow_num);
                            fprintf(fp_Medinf_delay, "%lf\n", total_t - drone[i].inf_med[j][k][0]); // 生成されてから配送車で回収されるまでの遅延時間
                        }
                    }
                }
            }
        }

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

        /************************************ 情報収集ドローン → 配送車**************************************/
        // 情報収集ドローンと配送車が合流したときに、薬情報の配列を共有
        /*
        for (i = 0; i < S_C_D; i++)
        {
            if (fabs(infC_drone[i].x - v[infC_drone[i].follow_num].x) < 0.05 && fabs(infC_drone[i].y - v[infC_drone[i].follow_num].y) < 0.05)
            {
                for (j = 0; j < N; j++)
                {
                    if (infC_drone[i].i_med_ptr[j] > v[infC_drone[i].follow_num].i_med_ptr[j])
                    {
                        for (k = v[infC_drone[i].follow_num].i_med_ptr[j]; k < infC_drone[i].i_med_ptr[j]; k++)
                        {
                            for (m = 0; m < Z_SIZE; m++)
                            {
                                v[infC_drone[i].follow_num].inf_med[j][k][m] = infC_drone[i].inf_med[j][k][m];
                            }
                            // 遅延時間のファイルへの書き込み
                            fprintf(fp_ETC_to_Vehicle, "t = %lf : infC_drone[%d] -> v[%d] shelter[%d]の情報 : %lf\n", total_t, i, infC_drone[i].follow_num, j, total_t - infC_drone[i].inf_med[j][k][0]); // 避難所で生成されてから配送車へ共有されるまでの遅延時間
                            // fprintf(fp_ETC_to_Vehicle, "%lf\n", total_t - infC_drone[i].inf_med[j][k][0]); // 避難所で生成されてから配送車へ共有されるまでの遅延時間

                            // v[infC_drone[i].follow_num].i_med_ptr[j] += 1;
                            //  配列の容量オーバー
                            if (infC_drone[i].i_med_ptr[j] == Y_SIZE)
                            {
                                printf("配列要素数オーバー\n");
                                break;
                            }
                        }
                    }
                }
            }
        }
            */

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

            printf("@@@ t=%.2lf : 避難所[%d]で薬情報発生\n", total_t, med_num);

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
                            printf("%.1lf ", new_p[i].inf_med[j][k][1]);
                        }
                        printf("\n");
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
            { // 物資が1000個以上の避難所のカウント
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

        // 部分時間を更新(待機していないなら更新)：配送車
        for (i = 0; i < M; i++)
        {
            if (stay_t[i] == 0 && dis_stay_t[i] == 0 && re_wait_flag[i] == FALSE)
            {
                part_t[i] += time_span;
            }
        }
        // 部分時間を更新（充電中でないかつ飛行開始しているのなら）：ドローン
        for (i = 0; i < S_C_D; i++)
        {
            if (infC_drone[i].charge_time == 0 && infC_drone[i].flight_start_time == 0 && infC_drone[i].FtoVehicle_mode == FALSE)
            {
                part_t_dro[i] += time_span;
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
    fclose(fp_ETC_Vehicle);
    fclose(fp_ETC_dro);
    fclose(fp_ETC_to_Vehicle);
    fclose(fp_Med_re_collect_to_delivery_delay);
    fclose(fp_infC_jyunkai_time);
    pclose(gp);

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
    /*****************ドローンによる医療物資配送割合の導出*******************/
    printf("ドローンによる医療物資配送割合: %f\n", (double)counter_Med_re_Drone_delivery / (double)counter_Med_re_delivery);
    printf("全体の配送回数: %d\n", counter_Med_re_delivery);
    printf("ドローンによる配送回数: %d\n", counter_Med_re_Drone_delivery);

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
        // 各シミュレーションごとのMed_E(TD)のデータを格納する
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

    /******** 医療品情報の回収における平均情報遅延時間 E(TC) *********/
    double value7 = 0;
    double sum7 = 0;
    double count7 = 0;
    double average7;

    // 避難所から配送車 or 避難所から情報収集ドローン
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
        printf("E(TC) TV + drone：%f [h]\n", average7 / 3600);
        // 各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_Medinf_collect_data;
        char *Mean_Medinf_collect_delay_file = "drone_datafile/txtfile/Mean_Medinf_collect_delay.txt";
        fp_Mean_Medinf_collect_data = fopen(Mean_Medinf_collect_delay_file, "a+");
        fprintf(fp_Mean_Medinf_collect_data, "%f\n", average7 / 3600);
        fclose(fp_Mean_Medinf_collect_data);
    }
    else
    {
        printf("E(TC) TV + drone：データがありません\n");
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

    /*** 避難所から物資運搬車両のみ ***/
    count7 = 0; // 初期化
    sum7 = 0;
    fp_ETC_Vehicle = fopen(ETC_Vehicle_data_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_ETC_Vehicle == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_ETC_Vehicle, "%lf", &value7) == 1)
    {
        sum7 += value7;
        count7++;
    }
    fclose(fp_ETC_Vehicle); // 平均情報遅延時間ファイルクローズ

    if (count7 > 0)
    {
        average7 = sum7 / count7;
        printf("E(TC) Vehicle：%f [hour]\n", average7 / 3600);

        //  各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_ETC_Vehicle_data;
        char *Mean_ETC_Vehicle_file = "drone_datafile/txtfile/Mean_ETC_Vehicle.txt";
        fp_Mean_ETC_Vehicle_data = fopen(Mean_ETC_Vehicle_file, "a+");
        fprintf(fp_Mean_ETC_Vehicle_data, "%f\n", average7 / 3600);
        fclose(fp_Mean_ETC_Vehicle_data);
    }
    else
    {
        printf("E(TC) Vehicle：データがありません\n");
    }

    /***避難所から収集用ドローン***/
    count7 = 0; // 初期化
    sum7 = 0;
    fp_ETC_dro = fopen(ETC_dro_data_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_ETC_dro == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_ETC_dro, "%lf", &value7) == 1)
    {
        sum7 += value7;
        count7++;
    }
    fclose(fp_ETC_dro); // 平均情報遅延時間ファイルクローズ

    if (count7 > 0)
    {
        average7 = sum7 / count7;
        printf("E(TC) drone：%f [hour]\n", average7 / 3600);
        //  各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_ETC_dro_data;
        char *Mean_ETC_dro_file = "drone_datafile/txtfile/Mean_ETC_dro.txt";
        fp_Mean_ETC_dro_data = fopen(Mean_ETC_dro_file, "a+");
        fprintf(fp_Mean_ETC_dro_data, "%f\n", average7 / 3600);
        fclose(fp_Mean_ETC_dro_data);
    }
    else
    {
        printf("E(TC) drone：データがありません\n");
    }

    /****** ドローンによる要求情報の収集率 *******/
    printf("ドローンによる要求情報の収集率：%f\n", (double)Medinf_collect_count_dro / (double)Medinf_collect_count);
    FILE *fp_Medinf_drone_Prob_data;
    char *Mean_Medinf_drone_Prob_file = "drone_datafile/txtfile/Prob_Medinf_drone.txt";
    fp_Medinf_drone_Prob_data = fopen(Mean_Medinf_drone_Prob_file, "a+");
    fprintf(fp_Medinf_drone_Prob_data, "%f\n", (double)Medinf_collect_count_dro / (double)Medinf_collect_count);
    fclose(fp_Medinf_drone_Prob_data);

    /******* 避難所から配送車（ドローン経由と配送車直接）への情報遅延時間　*******/
    count7 = 0; // 初期化
    sum7 = 0;
    fp_ETC_to_Vehicle = fopen(ETC_to_Vehicle_data_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_ETC_to_Vehicle == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_ETC_to_Vehicle, "%lf", &value7) == 1)
    {
        sum7 += value7;
        count7++;
    }
    fclose(fp_ETC_to_Vehicle); // 平均情報遅延時間ファイルクローズ

    if (count7 > 0)
    {
        average7 = sum7 / count7;
        printf("E(TC) to Vehicle：%f [hour]\n", average7 / 3600);
        // printf("sum7: %f, count7: %f\n", sum7, count7);
        //  各シミュレーションごとのMed_E(TD) のデータを格納する
        FILE *fp_Mean_ETC_to_Vehicle_data;
        char *Mean_ETC_to_Vehicle_file = "drone_datafile/txtfile/Mean_ETC_to_Vehicle.txt";
        fp_Mean_ETC_to_Vehicle_data = fopen(Mean_ETC_to_Vehicle_file, "a+");
        fprintf(fp_Mean_ETC_to_Vehicle_data, "%f\n", average7 / 3600);
        fclose(fp_Mean_ETC_to_Vehicle_data);
    }
    else
    {
        printf("E(TC) to Vehicle：データがありません\n");
    }

    /*************** ドローンによるバッテリー交換回数 *************/
    int sum_bat_swap_counter = 0;
    for (i = 0; i < S_C_D; i++)
    {
        sum_bat_swap_counter += infC_drone[i].bat_swap_counter;
    }
    printf("ドローンによるバッテリー総交換回数: %d\n", sum_bat_swap_counter);

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

    // 物資運搬車両の医療物資配送キューの表示
    for (i = 0; i < M; i++)
    {
        printf("配送車[%d]：", i);
        for (j = 0; j < v[i].queue_ptr; j++)
        {
            printf("%d ", v[i].Med_delivery_queue[j]);
        }
        printf("\n");
    }

    /******** 医療品の配送における配送車とドローンにおける配送の割合 *********/
    // 各シミュレーションごとのMed_E(TD) のデータを格納する
    FILE *fp_Mean_Med_re_DdeliveryCount_data;
    char *Mean_Med_re_DdeliveryCount_file = "drone_datafile/txtfile/Drone_deliveryProbability.txt";
    fp_Mean_Med_re_DdeliveryCount_data = fopen(Mean_Med_re_DdeliveryCount_file, "a+");
    fprintf(fp_Mean_Med_re_DdeliveryCount_data, "%f\n", (double)counter_Med_re_Drone_delivery / (double)counter_Med_re_delivery);
    fclose(fp_Mean_Med_re_DdeliveryCount_data);

    /******** 医療品情報の回収から配達における平均情報遅延時間 *********/
    value8 = 0;
    sum8 = 0;
    count8 = 0;
    average8 = 0;

    fp_infC_jyunkai_time = fopen(infC_jyunkai_time_file, "r"); // 平均情報遅延間隔ファイルのオープン
    if (fp_infC_jyunkai_time == NULL)
    {
        printf("ファイルを開くことができませんでした\n");
        return 1;
    }
    while (fscanf(fp_infC_jyunkai_time, "%lf", &value8) == 1)
    {
        sum8 += value8;
        count8++;
    }
    fclose(fp_infC_jyunkai_time); // 平均情報遅延時間ファイルクローズ

    if (count8 > 0)
    {
        average8 = sum8 / count8;
        printf("ドローンの一巡回にかかる時間（充電など含む）：%f [h]\n", average8 / 60);
        // データを格納する
        FILE *fp_Mean_InfCdrone_jyunkai_data;
        char *Mean_InfCdrone_jyunkai_file = "drone_datafile/txtfile/Mean_Jyunkai_time_InfCdrone.txt";
        fp_Mean_InfCdrone_jyunkai_data = fopen(Mean_InfCdrone_jyunkai_file, "a+");
        fprintf(fp_Mean_InfCdrone_jyunkai_data, "%f\n", average8 / 60);
        fclose(fp_Mean_InfCdrone_jyunkai_data);
    }
    else
    {
        printf("データがありません\n");
    }

    // 情報収集ドローンが避難所を訪れた回数の表示
    int sum_shelter_visit_counter = 0;
    for (i = 0; i < N; i++)
    {
        printf("避難所[%d] : %d\n", i, infC_drone[0].shelter_visit_counter[i]);
        sum_shelter_visit_counter += infC_drone[0].shelter_visit_counter[i];
    }
    printf("情報収集ドローンが訪れた避難所の合計回数: %d\n", sum_shelter_visit_counter);

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

    // ドローンの飛行可能半径を導出
    double capable_radius = (3600 / r_d_velo) * (capable_flight_time / 3600) / 2;

    // 飛行可能距離内の避難所数と距離外の避難所数のカウント
    int outer_capable_radius_count = 0;
    int inner_capable_radius_count = 0;
    for (i = 1; i < N; i++)
    {
        if (sqrt(pow(p[i].x - p[0].x, 2) + pow(p[i].y - p[0].y, 2)) > capable_radius)
        {
            outer_capable_radius_count++;
        }
        else
        {
            inner_capable_radius_count++;
        }
    }
    printf("配送センターからの飛行可能距離内の避難所数:%d\n", inner_capable_radius_count);
    printf("配送センターからの飛行可能距離外の避難所数:%d\n", outer_capable_radius_count);

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

    // fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", ad_file, data_file, R, R);
    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", ad_file, data_file, capable_radius, capable_radius);

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

    fprintf(gp, "plot \'%s\' u 1:2 with linespoints pt 7 lt rgbcolor'black',\'%s\' u 2:3 with points pt 7 lt rgbcolor'black',5 + %f*cos(t), 5 + %f*sin(t) lt rgbcolor'red'\n", ad_file, new_data_file, capable_radius, capable_radius);

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

    // 使用後にfree
    for (i = 0; i < M; i++)
    {
        free(reverse_cir[i]);
    }
    free(reverse_cir);

    return 0;
}