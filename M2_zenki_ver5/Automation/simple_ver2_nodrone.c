/**
 * @file vehicle_simulation_gif.c
 * @brief 1台の車両が円周上の複数の拠点を巡回する様子をシミュレートし、
 * gnuplotを用いてGIFアニメーションを生成するプログラム。
 *
 * - 半径rの円周上に、1つの集積所とNs個の避難所が等間隔に配置される。
 * - 車両は集積所（12時の位置）から出発し、一定速度vで時計回りに移動する。
 * - 各避難所および集積所に到着すると、荷物の積み下ろしのためにt秒間停止する。
 * - 指定された周回数Ntに達するとシミュレーションを終了する。
 * - 結果は 'simulation.gif' という名前のGIFファイルに出力される。
 *
 * コンパイル方法:
 * gcc vehicle_simulation_gif.c -o simulation_gif -lm
 *
 * 実行方法:
 * ./simulation_gif
 *
 * 必要なもの:
 * - gcc (Cコンパイラ)
 * - gnuplot (グラフ描画ツール)
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

// --- シミュレーションの基本設定 ---
#define PI 3.14159265358979323846

// 物理的なパラメータ
#define R 3000.0             // 円の半径 (m) | 3km
#define V (10000.0 / 3600.0) // 車両の速度 (m/s) | 10km/h を m/s に変換
#define T_STOP (30 * 60)     // 各拠点での停止時間 (s) | 10分

// シミュレーション設定
#define NS 10        // 避難所の数
#define NT 3         // シミュレーションの周回数
#define ENABLE_GIF 1 // GIF出力の有効/無効 (1:有効, 0:無効)

// 情報発生に関するパラメータ
#define LAMBDA 0.5    // ポアソン到着率 [/hour]
#define MAX_INFO 1000 // 最大情報数

// --- 派生的な定数 ---
#define TOTAL_STOPS (NS + 1) // 集積所を含めた拠点の総数
#define ANIMATION_FRAMES 20  // アニメーションの滑らかさ (1区間あたりの描画フレーム数) - 処理軽量化のため削減

// 情報管理用の構造体
typedef struct
{
    int shelter_id;         // 情報が発生した避難所ID (1〜NS)
    double generation_time; // 情報発生時刻
    double collection_time; // 情報回収時刻 (-1: 未回収)
    int collected;          // 回収済みフラグ
} Info;

/**
 * @brief gnuplotを初期化し、GIF出力のための設定を行う
 * @return gnuplotのパイプポインタ、失敗時はNULL
 */
FILE *init_gnuplot()
{
    // gnuplotをパイプモードで起動
    FILE *pipe = popen("gnuplot", "w");
    if (pipe == NULL)
    {
        fprintf(stderr, "エラー: gnuplotを起動できませんでした。gnuplotがインストールされているか確認してください。\n");
        return NULL;
    }

    // gnuplotの初期設定コマンドを送信
    // GIFアニメーション用の設定
    // delay: フレーム間の待機時間 (1/100秒単位) - 停止時間描画のため遅延を調整
    // size: 画像サイズ - GD Warningを避けるため適切なサイズに設定
    fprintf(pipe, "set terminal gif animate delay 20 size 600,600 crop\n");
    fprintf(pipe, "set output 'simulation.gif'\n");

    fprintf(pipe, "set title 'Vehicle Simulation (3km radius)'\n");
    fprintf(pipe, "set size square\n"); // 正方形に設定
    double plot_range = R * 1.2;        // プロット範囲を調整（3kmスケール対応）
    fprintf(pipe, "set xrange [%.1f:%.1f]\n", -plot_range, plot_range);
    fprintf(pipe, "set yrange [%.1f:%.1f]\n", -plot_range, plot_range);
    fprintf(pipe, "set xlabel 'X (m)'\n");
    fprintf(pipe, "set ylabel 'Y (m)'\n");
    fprintf(pipe, "unset key\n");                           // 凡例を非表示
    fprintf(pipe, "set grid lw 0.5 lc 'gray'\n");           // グリッドを灰色で細く表示
    fprintf(pipe, "set border lw 1\n");                     // 境界線を設定
    fprintf(pipe, "set style line 1 lc rgb 'gray' lw 2\n"); // 線のスタイルを明示的に設定

    return pipe;
}

/**
 * @brief 指数分布に従う乱数を生成（ポアソン過程の時間間隔）
 * @param lambda 到着率 [/hour]
 * @return 次の到着までの時間間隔 [秒]
 */
double generate_exponential_interval(double lambda)
{
    // (0,1)の一様乱数を生成
    double u = (double)rand() / RAND_MAX;
    // 指数分布の逆変換法: -ln(u) / lambda
    // lambdaは[/hour]なので、結果を秒に変換
    return -log(u) / (lambda / 3600.0); // 3600で割って[/秒]に変換
}

/**
 * @brief 1からNSまでの避難所IDをランダムに選択
 * @return 避難所ID (1〜NS)
 */
int generate_random_shelter()
{
    return (rand() % NS) + 1; // 1からNSまでの範囲
}

/**
 * @brief 毎フレームの描画処理を行う
 * @param pipe gnuplotのパイプポインタ
 * @param stop_coords 全ての拠点の座標配列
 * @param node_x 現在の車両のX座標
 * @param node_y 現在の車両のY座標
 * @param elapsed_time 経過時間（秒）
 * @param info_list 情報リスト
 * @param info_count 情報の総数
 */
void plot_frame(FILE *pipe, double stop_coords[][2], double node_x, double node_y, double elapsed_time, Info *info_list, int info_count)
{
    // GIF出力が無効の場合は何もしない
    if (!ENABLE_GIF || pipe == NULL)
        return;

    // 経過時間を時:分:秒の形式に変換
    int hours = (int)(elapsed_time / 3600);
    int minutes = (int)((elapsed_time - hours * 3600) / 60);
    int seconds = (int)(elapsed_time - hours * 3600 - minutes * 60);

    // 時間の表示形式を時と分のみに変更
    if (hours > 0)
    {
        fprintf(pipe, "set title 'Vehicle Simulation - Time: %02d:%02d hour'\n", hours, minutes);
    }
    else if (minutes > 0)
    {
        fprintf(pipe, "set title 'Vehicle Simulation - Time: %02d min'\n", minutes);
    }
    else
    {
        fprintf(pipe, "set title 'Vehicle Simulation - Time: 0 min'\n");
    }

    // 描画コマンドの送信
    // 1つ目: 経路（円）をグレーの線で描画
    // 2つ目: 拠点（集積所）を青い点で描画
    // 3つ目: 拠点（通常の避難所）を青い点で描画
    // 4つ目: 拠点（情報発生中の避難所）をオレンジ色で描画
    // 5つ目: 車両を赤い点で描画
    fprintf(pipe, "plot '-' with lines lc 'gray' lw 2 notitle, ");
    fprintf(pipe, "'-' with points pt 7 ps 1.5 lc 'blue' notitle, ");
    fprintf(pipe, "'-' with points pt 7 ps 1.5 lc 'blue' notitle, ");
    fprintf(pipe, "'-' with points pt 7 ps 1.8 lc 'orange' notitle, ");
    fprintf(pipe, "'-' with points pt 7 ps 2 lc 'red' notitle\n");

    // 1. 経路の座標データを送信（円を描画）
    for (int i = 0; i <= 360; i += 5) // 5度刻みで滑らかな円を描画
    {
        double angle = i * PI / 180.0;
        double x = R * cos(angle);
        double y = R * sin(angle);
        fprintf(pipe, "%.1f %.1f\n", x, y);
    }
    // 円を完全に閉じるため、最初の点を再度追加
    fprintf(pipe, "%.1f %.1f\n", R * cos(0), R * sin(0));
    fprintf(pipe, "e\n");

    // 2. 集積所の座標データを送信（青色）
    fprintf(pipe, "%.1f %.1f\n", stop_coords[0][0], stop_coords[0][1]);
    fprintf(pipe, "e\n");

    // 情報発生状況を確認
    int has_info[NS + 1] = {0}; // 各避難所の情報発生状況（0:なし、1:あり）
    for (int i = 0; i < info_count; i++)
    {
        if (!info_list[i].collected &&
            info_list[i].generation_time <= elapsed_time)
        {
            has_info[info_list[i].shelter_id] = 1;
        }
    }

    // 3. 通常の避難所の座標データを送信（青色）
    int normal_shelter_count = 0;
    for (int i = 1; i < TOTAL_STOPS; i++) // 集積所以外
    {
        if (!has_info[i]) // 情報が発生していない避難所
        {
            fprintf(pipe, "%.1f %.1f\n", stop_coords[i][0], stop_coords[i][1]);
            normal_shelter_count++;
        }
    }
    // 通常の避難所がない場合、範囲外のダミーデータを送信して警告を回避
    if (normal_shelter_count == 0)
    {
        fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0);
    }
    fprintf(pipe, "e\n");

    // 4. 情報発生中の避難所の座標データを送信（オレンジ色）
    int info_shelter_count = 0;
    for (int i = 1; i < TOTAL_STOPS; i++) // 集積所以外
    {
        if (has_info[i]) // 情報が発生している避難所
        {
            fprintf(pipe, "%.1f %.1f\n", stop_coords[i][0], stop_coords[i][1]);
            info_shelter_count++;
        }
    }
    // 情報発生中の避難所がない場合、範囲外のダミーデータを送信して警告を回避
    if (info_shelter_count == 0)
    {
        fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0);
    }
    fprintf(pipe, "e\n");

    // 5. 車両の現在位置を送信
    fprintf(pipe, "%.1f %.1f\n", node_x, node_y);
    fprintf(pipe, "e\n");

    // バッファをフラッシュしてgnuplotに送信
    fflush(pipe);
}

int main(void)
{
    // 乱数シードの初期化
    srand(time(NULL));

    // gnuplotパイプの初期化（GIF出力が有効な場合のみ）
    FILE *gnuplot_pipe = NULL;
    if (ENABLE_GIF)
    {
        gnuplot_pipe = init_gnuplot();
        if (gnuplot_pipe == NULL)
        {
            fprintf(stderr, "警告: gnuplotの初期化に失敗しました。GIF出力なしで続行します。\n");
        }
        else
        {
            printf("GIF出力が有効です。'simulation.gif' を生成します。\n");
        }
    }
    else
    {
        printf("GIF出力が無効です。処理を軽量化して実行します。\n");
    }

    // 拠点座標の初期化
    double stop_coords[TOTAL_STOPS][2];                    // 各拠点の座標配列 [拠点番号][x,y]
    const double angle_increment = 2.0 * PI / TOTAL_STOPS; // 隣接拠点間の角度差

    // 円周上に等間隔で拠点を配置
    // 集積所は12時の位置（PI/2）、避難所は時計回りに配置
    for (int i = 0; i < TOTAL_STOPS; i++)
    {
        double angle = PI / 2.0 - i * angle_increment; // 各拠点の角度
        stop_coords[i][0] = R * cos(angle);            // X座標
        stop_coords[i][1] = R * sin(angle);            // Y座標
    }

    // シミュレーション制御変数
    int current_stop_idx = 0;   // 現在の停止地点インデックス（0=集積所、1〜NS=避難所）
    int lap_count = 0;          // 完了した周回数
    double elapsed_time = 0.0;  // 経過時間（秒）
    int is_first_departure = 1; // 集積所からの初回出発フラグ

    // 情報管理変数
    Info info_list[MAX_INFO];                                      // 情報のリスト
    int info_count = 0;                                            // 発生した情報の総数
    double next_info_time = generate_exponential_interval(LAMBDA); // 次の情報発生時刻
    double total_tc = 0.0;                                         // 回収時間差の合計
    int collected_count = 0;                                       // 回収済み情報数

    // 移動時間計算用の定数
    const double segment_distance = (2.0 * PI * R) / TOTAL_STOPS; // 1区間の距離
    const double travel_time_per_segment = segment_distance / V;  // 1区間の移動時間

    // 実時間に基づいたフレーム数の計算
    const int stop_frames = ANIMATION_FRAMES;                                             // 停止時のフレーム数（基準）
    const int move_frames = (int)round((travel_time_per_segment / T_STOP) * stop_frames); // 移動時のフレーム数（実時間比例）

    printf("停止時間: %.1f秒, 移動時間: %.1f秒\n", (double)T_STOP, travel_time_per_segment);
    printf("停止フレーム数: %d, 移動フレーム数: %d\n", stop_frames, move_frames);

    if (ENABLE_GIF)
    {
        printf("シミュレーションを開始します。GIFファイル 'simulation.gif' を生成します...\n");
    }
    else
    {
        printf("シミュレーションを開始します（GIF出力なし）...\n");
    }
    printf("円の半径: %.1f m, 車両速度: %.2f m/s, 拠点数: %d\n", R, V, TOTAL_STOPS);

    /****** シミュレーション開始 ******/
    while (lap_count < NT)
    {
        // 情報発生処理（ポアソン到着）
        while (elapsed_time >= next_info_time && info_count < MAX_INFO)
        {
            // 新しい情報を発生させる
            info_list[info_count].shelter_id = generate_random_shelter();
            info_list[info_count].generation_time = next_info_time;
            info_list[info_count].collection_time = -1; // 未回収
            info_list[info_count].collected = 0;

            printf("情報発生: 時刻 %.1f秒 : 避難所 %d\n", next_info_time, info_list[info_count].shelter_id);

            info_count++;
            next_info_time += generate_exponential_interval(LAMBDA);
        }

        // 現在の停止地点の座標を取得
        double current_x = stop_coords[current_stop_idx][0];
        double current_y = stop_coords[current_stop_idx][1];

        // 停止中の描画（初回出発時は停止しない）
        if (!is_first_departure)
        {
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, elapsed_time, info_list, info_count);

            // 停止地点の情報を表示
            if (current_stop_idx == 0)
            {
                printf("周回: %d/%d | 集積所に到着。%d秒間停止します。\n", lap_count + 1, NT, T_STOP);
            }
            else
            {
                printf("周回: %d/%d | 避難所%dに到着。%d秒間停止します。\n", lap_count + 1, NT, current_stop_idx, T_STOP);

                // 避難所での情報回収処理
                for (int i = 0; i < info_count; i++)
                {
                    if (!info_list[i].collected && info_list[i].shelter_id == current_stop_idx)
                    {
                        // 情報を回収
                        info_list[i].collection_time = elapsed_time;
                        info_list[i].collected = 1;

                        double tc = elapsed_time - info_list[i].generation_time;
                        total_tc += tc;
                        collected_count++;

                        printf("  情報回収: 発生%.1f秒 → 回収%.1f秒 (Tc=%.1f秒)\n", info_list[i].generation_time, elapsed_time, tc);
                    }
                }
            }

            // 停止時間中の描画処理
            // 実時間に基づいたフレーム数で停止時間を表現
            const double time_per_stop_frame = T_STOP / stop_frames; // 1フレームあたりの時間

            for (int i = 0; i < stop_frames; ++i)
            {
                plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, elapsed_time, info_list, info_count);
                // GIF出力が有効な場合のみ待機時間を設ける
                if (ENABLE_GIF)
                {
                    usleep(40000); // 0.04秒の遅延
                }

                // 停止時間を徐々に経過時間に追加
                elapsed_time += time_per_stop_frame;
            }
        }
        else
        {
            // 初回出発時は停止せず、開始地点を描画
            printf("シミュレーション開始: 集積所から出発\n");
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, elapsed_time, info_list, info_count);
            is_first_departure = 0; // 初回出発フラグをリセット
        }

        // 次の目的地を計算
        int next_stop_idx = (current_stop_idx + 1) % TOTAL_STOPS;

        // 周回完了の判定（次の目的地が集積所なら1周完了）
        if (next_stop_idx == 0)
        {
            lap_count++;
        }

        printf("移動中: 拠点%d -> 拠点%d\n", current_stop_idx, next_stop_idx);

        // 移動アニメーション用の角度計算
        double start_angle = PI / 2.0 - current_stop_idx * angle_increment; // 開始角度
        double angle_change_per_frame = -angle_increment / move_frames;     // フレーム毎の角度変化（実時間比例）

        // 移動中のアニメーション描画
        for (int frame = 1; frame <= move_frames; frame++)
        {
            // 現在フレームの角度を計算
            double current_angle = start_angle + frame * angle_change_per_frame;
            current_x = R * cos(current_angle);
            current_y = R * sin(current_angle);

            // 移動中の車両位置を描画
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, elapsed_time, info_list, info_count);
            // GIF出力が有効な場合のみ待機時間を設ける
            if (ENABLE_GIF)
            {
                usleep(40000); // 0.04秒の遅延
            }

            // 移動時間を徐々に経過時間に追加（実時間比例）
            elapsed_time += travel_time_per_segment / move_frames;
        }

        // 次の停止地点へ移動
        current_stop_idx = next_stop_idx;
    }

    // シミュレーション終了処理
    printf("\nシミュレーション終了。%d周しました。\n", lap_count);
    printf("総経過時間: %.0f秒 (%.1f時間)\n", elapsed_time, elapsed_time / 3600.0);

    // 移動距離の計算と表示
    double total_distance = lap_count * 2.0 * PI * R;
    printf("総移動距離: %.1f m (%.2f km)\n", total_distance, total_distance / 1000.0);
    printf("平均速度: %.2f m/s (%.1f km/h)\n", V, V * 3.6);

    // 情報発生・回収の統計情報
    printf("\n=== 情報発生・回収統計 ===\n");
    printf("総情報発生数: %d件\n", info_count);
    printf("回収済み情報数: %d件\n", collected_count);
    printf("未回収情報数: %d件\n", info_count - collected_count);

    if (collected_count > 0)
    {
        double average_tc = total_tc / collected_count;
        printf("Tc平均値: %.2f秒 (%.2f分)\n", average_tc, average_tc / 60.0);
    }
    else
    {
        printf("Tc平均値: 情報回収なし\n");
    }

    if (ENABLE_GIF && gnuplot_pipe != NULL)
    {
        printf("'simulation.gif' がカレントディレクトリに生成されました。\n");

        // GIFファイルを適切に終了するための設定
        fprintf(gnuplot_pipe, "unset output\n");
        fflush(gnuplot_pipe);

        // gnuplotパイプを閉じる
        pclose(gnuplot_pipe);
    }
    else if (ENABLE_GIF)
    {
        printf("GIF出力が有効でしたが、gnuplotの初期化に失敗したため、GIFファイルは生成されませんでした。\n");
    }
    else
    {
        printf("GIF出力が無効のため、GIFファイルは生成されませんでした。\n");
    }

    return 0;
}
