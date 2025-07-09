/**
 * @file simple_ver2_drone.c
 * @brief 複数台ドローンと1台車両による円周巡回シミュレーション（情報回収システム）
 *
 * 【シミュレーション概要】
 * - 半径3kmの円周上に集積所1か所と避難所10か所が等間隔配置
 * - 車両：集積所（12時位置）から時計回りに巡回、各拠点で30分停止
 * - ドローン：車両の2倍速で反時計回りに連続巡回、台数は可変（ND設定）
 * - 情報：ポアソン過程で避難所にランダム発生、車両が回収してTc（回収遅延）を計測
 * - 可視化：gnuplotによるGIFアニメーション生成（2分間隔フレーム）
 *
 * 【座標系と角度定義】
 * - 集積所：12時方向（π/2ラジアン、90度）
 * - 車両移動：時計回り（角度減少方向）
 * - ドローン移動：反時計回り（角度増加方向）
 * - 拠点配置：集積所から時計回りに等間隔（2π/11ラジアン間隔）
 *
 * 【ドローン制御】
 * - 複数台対応：ND台のドローンが順次出発
 * - 出発間隔：1周時間÷ND（均等分散配置）
 * - 速度：車両の2倍（20km/h vs 10km/h）
 * - 動作：連続巡回（停止時間なし）
 *
 * コンパイル: gcc simple_ver2_drone.c -o simulation -lm
 * 実行: ./simulation
 * 必要環境: gcc, gnuplot
 *
 * 7/9：複数ドローンを周回させる処理追加、ドローンによる情報収集した場合の集積所への往復処理実装、
 * 　　避難所の回収済みフラグ有効化のタイミングはドローンが避難所に到着したタイミング
 * 　　TVとドローンによる物資運搬は未実装
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

// --- シミュレーションの基本設定 ---
#define PI 3.14159265358979323846

// === 物理的パラメータ ===
#define R 3000.0                   // 円の半径 (m) | 実スケール3km
#define V (10000.0 / 3600.0)       // 車両の速度 (m/s) | 10km/h → 2.78m/s
#define V_DRONE (20000.0 / 3600.0) // ドローンの速度 (m/s) | 20km/h → 5.56m/s（車両の2倍）
#define T_STOP (30 * 60)           // 各拠点での停止時間 (s) | 30分=1800秒

// === シミュレーション設定 ===
#define NS 10        // 避難所の数（集積所除く）
#define NT 3         // シミュレーションの周回数
#define ND 1         // ドローンの台数（0の場合はドローンなし、最大制限なし）
#define ENABLE_GIF 1 // GIF出力の有効/無効 (1:有効, 0:無効) | 処理軽量化用

// === 情報発生システム（ポアソン過程） ===
#define LAMBDA 0.6    // ポアソン到着率 [件/時間] | 1時間に平均0.5件の情報発生
#define MAX_INFO 1000 // 最大情報数（メモリ制限対策）

// === ドローン物資運搬システム ===
#define NR 2                   // ドローンの往復回数
#define T_DRONE_STOP (10 * 60) // ドローンの停止時間 (s) | 10分=600秒（集積所・避難所共通）

// === 派生的な定数（自動計算） ===
#define TOTAL_STOPS (NS + 1) // 集積所を含めた拠点の総数（11拠点）
#define DRAW_INTERVAL 120.0  // 描画間隔（秒）| 2分ごとにGIFフレーム生成

// === 情報管理用構造体 ===
/**
 * @brief 避難所で発生する情報を管理する構造体
 *
 * 情報のライフサイクル：
 * 1. ポアソン過程で避難所にランダム発生
 * 2. 車両が該当避難所を訪問時に回収
 * 3. Tc（発生から回収までの遅延時間）を計算
 */
typedef struct
{
    int shelter_id;         // 情報が発生した避難所ID (1〜NS) | 0は集積所なので除外
    double generation_time; // 情報発生時刻（秒）| シミュレーション開始からの経過時間
    double collection_time; // 情報回収時刻（秒）| -1の場合は未回収状態
    int collected;          // 回収済みフラグ（0:未回収, 1:回収済み）
} Info;

// === ドローン状態管理用構造体 ===
/**
 * @brief ドローンの動作状態を管理する構造体
 *
 * ドローンの動作モード：
 * 1. PATROL: 通常巡回（反時計回り）
 * 2. TO_DEPOT: 情報発見後、集積所への直線飛行
 * 3. AT_DEPOT: 集積所での物資積載（10分停止）
 * 4. TO_SHELTER: 集積所から避難所への直線飛行
 * 5. AT_SHELTER: 避難所での物資降ろし（10分停止）
 */
typedef enum
{
    DRONE_PATROL = 0,     // 通常巡回モード
    DRONE_TO_DEPOT = 1,   // 集積所への飛行中
    DRONE_AT_DEPOT = 2,   // 集積所で停止中
    DRONE_TO_SHELTER = 3, // 避難所への飛行中
    DRONE_AT_SHELTER = 4  // 避難所で停止中
} DroneState;

typedef struct
{
    double angle;      // 現在角度（ラジアン）
    double x, y;       // 現在位置（メートル）
    int active;        // 活動状態（0:未出発, 1:活動中）
    double start_time; // 出発時刻（秒）

    // 物資運搬関連
    DroneState state;          // 現在の動作状態
    int target_shelter;        // 目標避難所ID（物資運搬時）
    int current_trip;          // 現在の往復回数（1〜NR）
    double state_start_time;   // 現在状態の開始時刻
    double target_x, target_y; // 目標座標（直線飛行時）
} DroneInfo;

/**
 * @brief gnuplotを初期化し、GIFアニメーション出力のための設定を行う
 *
 * 【gnuplot設定詳細】
 * - terminal: GIFアニメーション形式、20/100秒間隔、600x600px
 * - plot範囲: 半径3kmの1.2倍（3.6km範囲）で余裕を持たせた表示
 * - 座標系: 数学座標系（右がX正、上がY正）
 * - グリッド: 位置把握用の薄いグレー格子
 *
 * @return gnuplotのパイプポインタ、失敗時はNULL
 */
FILE *init_gnuplot()
{
    // gnuplotをパイプモードで起動（双方向通信）
    FILE *pipe = popen("gnuplot", "w");
    if (pipe == NULL)
    {
        fprintf(stderr, "エラー: gnuplotを起動できませんでした。gnuplotがインストールされているか確認してください。\n");
        return NULL;
    }

    // === GIFアニメーション設定 ===
    // delay: フレーム間隔（1/100秒単位）→ 20 = 0.2秒間隔
    // size: 画像サイズ（600x600px）→ 正方形で円が歪まない
    // crop: 余白を自動トリミング
    fprintf(pipe, "set terminal gif animate delay 20 size 600,600 crop\n");
    fprintf(pipe, "set output 'simulation.gif'\n");

    // === 描画エリア設定 ===
    fprintf(pipe, "set title 'Vehicle Simulation (3km radius)'\n");
    fprintf(pipe, "set size square\n"); // アスペクト比1:1（円を正円で表示）
    double plot_range = R * 1.2;        // 表示範囲 = 半径×1.2（3.6km範囲）
    fprintf(pipe, "set xrange [%.1f:%.1f]\n", -plot_range, plot_range);
    fprintf(pipe, "set yrange [%.1f:%.1f]\n", -plot_range, plot_range);
    fprintf(pipe, "set xlabel 'X (m)'\n"); // X軸ラベル（メートル単位）
    fprintf(pipe, "set ylabel 'Y (m)'\n"); // Y軸ラベル（メートル単位）

    // === 表示スタイル設定 ===
    fprintf(pipe, "unset key\n");                           // 凡例を非表示（データ系列名を隠す）
    fprintf(pipe, "set grid lw 0.5 lc 'gray'\n");           // グリッド表示（線幅0.5、グレー色）
    fprintf(pipe, "set border lw 1\n");                     // 境界線の設定
    fprintf(pipe, "set style line 1 lc rgb 'gray' lw 2\n"); // 線スタイル定義（円周描画用）

    return pipe;
}

/**
 * @brief 指数分布に従う乱数を生成（ポアソン過程の時間間隔生成用）
 *
 * 【ポアソン過程の理論】
 * - 到着間隔は指数分布に従う
 * - 指数分布の逆変換法: X = -ln(U) / λ
 * - λ: 到着率[件/時間]、U: [0,1)の一様乱数
 *
 * 【実装詳細】
 * - λ=0.5[件/時間] → 平均間隔 = 1/0.5 = 2時間
 * - 秒単位変換: λ[件/時間] → λ/3600[件/秒]
 *
 * @param lambda 到着率 [件/時間]
 * @return 次の到着までの時間間隔 [秒]
 */
double generate_exponential_interval(double lambda)
{
    // (0,1)の一様乱数を生成（0は除外して数値安定性を確保）
    double u = (double)rand() / RAND_MAX;
    // 指数分布の逆変換法: -ln(u) / lambda
    // lambda[件/時間] → lambda/3600[件/秒] に変換
    return -log(u) / (lambda / 3600.0); // 結果は秒単位
}

/**
 * @brief ランダムな避難所IDを生成（情報発生場所の決定用）
 *
 * 【注意事項】
 * - 集積所（ID=0）では情報は発生しない
 * - 避難所のみ（ID=1〜NS）で情報が発生
 * - 一様分布でランダム選択（各避難所の発生確率は等しい）
 *
 * @return 避難所ID (1〜NS)
 */
int generate_random_shelter()
{
    return (rand() % NS) + 1; // 1からNSまでの一様分布
}

/**
 * @brief ドローンが情報発生避難所を検出するかチェック
 *
 * 【検出条件】
 * - ドローンが通常巡回モード（PATROL）である
 * - ドローンの現在位置が避難所近傍（距離閾値以内）
 * - その避難所に未回収の情報が存在する
 *
 * @param drone ドローン情報
 * @param stop_coords 拠点座標配列
 * @param info_list 情報リスト
 * @param info_count 情報総数
 * @param elapsed_time 現在時刻
 * @return 検出した避難所ID（検出なしの場合は0）
 */
int check_drone_info_detection(DroneInfo *drone, double stop_coords[][2], Info *info_list, int info_count, double elapsed_time)
{
    // 通常巡回モード以外では検出しない
    if (drone->state != DRONE_PATROL)
        return 0;

    const double DETECTION_RADIUS = 200.0; // 検出半径200m

    // 各避難所との距離をチェック
    for (int shelter_id = 1; shelter_id <= NS; shelter_id++)
    {
        double dx = drone->x - stop_coords[shelter_id][0];
        double dy = drone->y - stop_coords[shelter_id][1];
        double distance = sqrt(dx * dx + dy * dy);

        // 避難所近傍にいるかチェック
        if (distance <= DETECTION_RADIUS)
        {
            // この避難所に未回収情報があるかチェック
            for (int i = 0; i < info_count; i++)
            {
                if (!info_list[i].collected &&
                    info_list[i].shelter_id == shelter_id &&
                    info_list[i].generation_time <= elapsed_time)
                {
                    return shelter_id; // 情報発生避難所を検出
                }
            }
        }
    }

    return 0; // 検出なし
}

/**
 * @brief ドローンの状態更新処理
 *
 * @param drone ドローン情報
 * @param stop_coords 拠点座標配列
 * @param elapsed_time 現在時刻
 * @param time_step 時間ステップ
 * @param info_list 情報リスト（回収フラグ更新用）
 * @param info_count 情報総数
 */
void update_drone_state(DroneInfo *drone, double stop_coords[][2], double elapsed_time, double time_step, Info *info_list, int info_count)
{
    if (!drone->active)
        return;

    switch (drone->state)
    {
    case DRONE_PATROL:
        // 通常巡回（反時計回り）
        {
            double drone_angle_increment = (V_DRONE / R) * time_step;
            drone->angle += drone_angle_increment;

            // 角度正規化
            if (drone->angle > 2.0 * PI)
            {
                drone->angle -= 2.0 * PI;
            }

            // 座標更新
            drone->x = R * cos(drone->angle);
            drone->y = R * sin(drone->angle);
        }
        break;

    case DRONE_TO_DEPOT:
        // 集積所への直線飛行
        {
            double dx = drone->target_x - drone->x;
            double dy = drone->target_y - drone->y;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance <= V_DRONE * time_step)
            {
                // 集積所到着
                drone->x = drone->target_x;
                drone->y = drone->target_y;
                drone->state = DRONE_AT_DEPOT;
                drone->state_start_time = elapsed_time;
                printf("ドローン: 集積所到着（物資積載開始）\n");
            }
            else
            {
                // 直線移動継続
                double move_ratio = (V_DRONE * time_step) / distance;
                drone->x += dx * move_ratio;
                drone->y += dy * move_ratio;
            }
        }
        break;

    case DRONE_AT_DEPOT:
        // 集積所での停止（物資積載）
        if (elapsed_time >= drone->state_start_time + T_DRONE_STOP)
        {
            // 停止時間終了、避難所への飛行開始
            drone->target_x = stop_coords[drone->target_shelter][0];
            drone->target_y = stop_coords[drone->target_shelter][1];
            drone->state = DRONE_TO_SHELTER;
            printf("ドローン: 集積所出発（避難所%dへ）\n", drone->target_shelter);
        }
        break;

    case DRONE_TO_SHELTER:
        // 避難所への直線飛行
        {
            double dx = drone->target_x - drone->x;
            double dy = drone->target_y - drone->y;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance <= V_DRONE * time_step)
            {
                // 避難所到着
                drone->x = drone->target_x;
                drone->y = drone->target_y;
                drone->state = DRONE_AT_SHELTER;
                drone->state_start_time = elapsed_time;
                printf("ドローン: 避難所%d到着（物資降ろし開始）\n", drone->target_shelter);
            }
            else
            {
                // 直線移動継続
                double move_ratio = (V_DRONE * time_step) / distance;
                drone->x += dx * move_ratio;
                drone->y += dy * move_ratio;
            }
        }
        break;

    case DRONE_AT_SHELTER:
        // 避難所での停止（物資降ろし）
        if (elapsed_time >= drone->state_start_time + T_DRONE_STOP)
        {
            drone->current_trip++;

            if (drone->current_trip <= NR)
            {
                // 次の往復へ、集積所に戻る
                drone->target_x = stop_coords[0][0]; // 集積所座標
                drone->target_y = stop_coords[0][1];
                drone->state = DRONE_TO_DEPOT;
                printf("ドローン: 避難所%d出発（往復%d/%d回目）\n",
                       drone->target_shelter, drone->current_trip, NR);
            }
            else
            {
                // 往復完了、通常巡回に復帰
                drone->state = DRONE_PATROL;
                drone->current_trip = 0;

                // 現在位置から巡回角度を計算
                drone->angle = atan2(drone->y, drone->x);
                if (drone->angle < 0)
                    drone->angle += 2.0 * PI;

                printf("ドローン: 物資運搬完了、通常巡回に復帰\n");
            }
        }
        break;
    }
}

/**
 * @brief 毎フレームの描画処理（GIFアニメーション用フレーム生成）
 *
 * 【描画レイヤー構成】
 * 1. グレー円：移動経路（半径3kmの円周）
 * 2. 青点：集積所（12時方向の拠点）
 * 3. 青点：通常避難所（情報発生なし状態）
 * 4. オレンジ点：情報発生中避難所（未回収情報あり）
 * 5. 赤点：車両の現在位置
 * 6. 緑点：アクティブドローンの現在位置（ND>0時のみ）
 *
 * 【gnuplotデータ送信方式】
 * - plotコマンドで複数データ系列を指定
 * - 各系列のデータを'-'〜'e'区間で送信
 * - 空データの場合は範囲外座標でダミーデータ送信（警告回避）
 *
 * @param pipe gnuplotのパイプポインタ
 * @param stop_coords 全拠点の座標配列[拠点ID][x,y]
 * @param vehicle_x 車両の現在X座標
 * @param vehicle_y 車両の現在Y座標
 * @param drones ドローン情報配列[ドローンID]
 * @param elapsed_time 経過時間（秒）
 * @param info_list 情報リスト配列
 * @param info_count 発生済み情報の総数
 */
void plot_frame(FILE *pipe, double stop_coords[][2], double vehicle_x, double vehicle_y, DroneInfo drones[], double elapsed_time, Info *info_list, int info_count)
{
    // GIF出力が無効の場合は処理をスキップ（高速化）
    if (!ENABLE_GIF || pipe == NULL)
        return;

    // === 時刻表示の準備 ===
    // 経過時間を時:分:秒の形式に分解
    int hours = (int)(elapsed_time / 3600);
    int minutes = (int)((elapsed_time - hours * 3600) / 60);
    int seconds = (int)(elapsed_time - hours * 3600 - minutes * 60);

    // タイトルバーに経過時間を表示（秒は省略して見やすく）
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

    // === 描画コマンドの構築 ===
    // 基本レイヤー（円、集積所、避難所、車両）は常に描画
    fprintf(pipe, "plot '-' with lines lc 'gray' lw 2 notitle, ");      // 1. 円周経路
    fprintf(pipe, "'-' with points pt 7 ps 1.5 lc 'blue' notitle, ");   // 2. 集積所
    fprintf(pipe, "'-' with points pt 7 ps 1.5 lc 'blue' notitle, ");   // 3. 通常避難所
    fprintf(pipe, "'-' with points pt 7 ps 1.8 lc 'orange' notitle, "); // 4. 情報発生中避難所
    fprintf(pipe, "'-' with points pt 7 ps 2 lc 'red' notitle");        // 5. 車両

    // ドローンレイヤーは台数が1台以上の場合のみ追加
    if (ND > 0)
    {
        fprintf(pipe, ", '-' with points pt 5 ps 1.8 lc 'green' notitle"); // 6. ドローン群
    }
    fprintf(pipe, "\n");

    // === データ送信開始 ===

    // 1. 経路円の座標データ送信（5度刻みで滑らかな円を描画）
    for (int i = 0; i <= 360; i += 5)
    {
        double angle = i * PI / 180.0; // 度 → ラジアン変換
        double x = R * cos(angle);     // X座標 = R*cos(θ)
        double y = R * sin(angle);     // Y座標 = R*sin(θ)
        fprintf(pipe, "%.1f %.1f\n", x, y);
    }
    // 円を完全に閉じるため、開始点（0度）を再度送信
    fprintf(pipe, "%.1f %.1f\n", R * cos(0), R * sin(0));
    fprintf(pipe, "e\n"); // データ終了マーカー

    // 2. 集積所（拠点0）の座標データ送信
    fprintf(pipe, "%.1f %.1f\n", stop_coords[0][0], stop_coords[0][1]);
    fprintf(pipe, "e\n");

    // === 情報発生状況の解析 ===
    // 各避難所の情報発生状況を調査（描画色分け用）
    int has_info[NS + 1] = {0}; // 避難所別の情報フラグ（0:情報なし, 1:情報あり）
    for (int i = 0; i < info_count; i++)
    {
        // 未回収かつ既に発生済みの情報をチェック
        if (!info_list[i].collected &&
            info_list[i].generation_time <= elapsed_time)
        {
            has_info[info_list[i].shelter_id] = 1; // 該当避難所に情報ありマーク
        }
    }

    // 3. 通常避難所（情報なし）の座標データ送信（青色表示）
    int normal_shelter_count = 0;
    for (int i = 1; i < TOTAL_STOPS; i++) // 集積所（i=0）は除外
    {
        if (!has_info[i]) // 情報が発生していない避難所のみ
        {
            fprintf(pipe, "%.1f %.1f\n", stop_coords[i][0], stop_coords[i][1]);
            normal_shelter_count++;
        }
    }
    // 通常避難所が存在しない場合の対策（gnuplot警告回避）
    if (normal_shelter_count == 0)
    {
        fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0); // 画面外ダミー座標
    }
    fprintf(pipe, "e\n");

    // 4. 情報発生中避難所（情報あり）の座標データ送信（オレンジ色表示）
    int info_shelter_count = 0;
    for (int i = 1; i < TOTAL_STOPS; i++) // 集積所（i=0）は除外
    {
        if (has_info[i]) // 情報が発生している避難所のみ
        {
            fprintf(pipe, "%.1f %.1f\n", stop_coords[i][0], stop_coords[i][1]);
            info_shelter_count++;
        }
    }
    // 情報発生中避難所が存在しない場合の対策（gnuplot警告回避）
    if (info_shelter_count == 0)
    {
        fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0); // 画面外ダミー座標
    }
    fprintf(pipe, "e\n");

    // 5. 車両の現在位置を送信（赤色表示）
    fprintf(pipe, "%.1f %.1f\n", vehicle_x, vehicle_y);
    fprintf(pipe, "e\n");

    // 6. ドローン群の現在位置を送信（緑色表示）- ドローン台数>0の場合のみ
    if (ND > 0)
    {
        int active_drone_count = 0;
        for (int i = 0; i < ND; i++)
        {
            if (drones[i].active) // アクティブ状態のドローンのみ描画
            {
                fprintf(pipe, "%.1f %.1f\n", drones[i].x, drones[i].y);
                active_drone_count++;
            }
        }
        // アクティブドローンが存在しない場合の対策（gnuplot警告回避）
        if (active_drone_count == 0)
        {
            fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0); // 画面外ダミー座標
        }
        fprintf(pipe, "e\n");
    }

    // バッファをフラッシュしてgnuplotに確実に送信
    fflush(pipe);
}

/******************** メイン関数 *********************/
int main(void)
{
    // === 初期化処理 ===

    // 乱数シードの初期化（実行毎に異なる結果を得るため）
    srand(time(NULL));

    // === gnuplotパイプの初期化 ===
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

    // === 拠点座標の初期化 ===
    double stop_coords[TOTAL_STOPS][2];                    // 各拠点の座標配列 [拠点ID][x,y]
    const double angle_increment = 2.0 * PI / TOTAL_STOPS; // 隣接拠点間の角度差（ラジアン）

    // 円周上に拠点を等間隔配置
    // 【座標計算式】
    // - 集積所: 12時方向（π/2ラジアン）
    // - 避難所: 集積所から時計回りに等間隔配置
    // - 角度 = π/2 - i * (2π/拠点総数)
    for (int i = 0; i < TOTAL_STOPS; i++)
    {
        double angle = PI / 2.0 - i * angle_increment; // 各拠点の角度計算
        stop_coords[i][0] = R * cos(angle);            // X座標 = R*cos(θ)
        stop_coords[i][1] = R * sin(angle);            // Y座標 = R*sin(θ)
    }

    // === シミュレーション制御変数 ===
    int current_stop_idx = 0;   // 現在の停止地点インデックス（0=集積所、1〜NS=避難所）
    int lap_count = 0;          // 完了した周回数
    double elapsed_time = 0.0;  // 経過時間（秒）
    int is_first_departure = 1; // 集積所からの初回出発フラグ

    // === ドローン制御変数（複数台対応） ===
    DroneInfo drones[ND]; // ドローン情報配列

    // === 情報管理変数 ===
    Info info_list[MAX_INFO];                                      // 情報データベース
    int info_count = 0;                                            // 発生した情報の総数
    double next_info_time = generate_exponential_interval(LAMBDA); // 次の情報発生予定時刻
    double total_tc = 0.0;                                         // 回収時間差（Tc）の累積値
    int collected_count = 0;                                       // 回収済み情報数

    // === 移動時間計算 ===
    const double segment_distance = (2.0 * PI * R) / TOTAL_STOPS;            // 1区間の移動距離（メートル）
    const double travel_time_per_segment = segment_distance / V;             // 車両の1区間移動時間（秒）
    const double drone_travel_time_per_segment = segment_distance / V_DRONE; // ドローンの1区間移動時間（秒）

    // === 描画制御変数 ===
    double next_draw_time = 0.0; // 次の描画時刻（秒）

    // === ドローンシステムの初期化 ===
    double drone_lap_time = (2.0 * PI * R) / V_DRONE;                       // ドローンの1周時間（秒）
    double drone_departure_interval = (ND > 0) ? drone_lap_time / ND : 0.0; // ドローン出発間隔（秒）

    // === 各ドローンの初期化 ===
    // 全ドローンを集積所（12時位置）に配置し、順次出発スケジュールを設定
    for (int i = 0; i < ND; i++)
    {
        drones[i].angle = PI / 2.0;                          // 全て集積所（12時位置、π/2ラジアン）から開始
        drones[i].x = R * cos(drones[i].angle);              // 初期X座標
        drones[i].y = R * sin(drones[i].angle);              // 初期Y座標
        drones[i].active = 0;                                // 初期状態：未出発
        drones[i].start_time = i * drone_departure_interval; // 出発時刻（等間隔分散）

        // 物資運搬関連の初期化
        drones[i].state = DRONE_PATROL;                // 通常巡回モード
        drones[i].target_shelter = 0;                  // 目標避難所なし
        drones[i].current_trip = 0;                    // 往復回数0
        drones[i].state_start_time = 0.0;              // 状態開始時刻
        drones[i].target_x = drones[i].target_y = 0.0; // 目標座標
    }

    // === シミュレーション設定情報の表示 ===
    printf("=== シミュレーション設定 ===\n");
    printf("停止時間: %.1f秒, 車両1区間移動時間: %.1f秒\n", (double)T_STOP, travel_time_per_segment);
    printf("描画間隔: %.0f秒（%.0f分ごと）\n", DRAW_INTERVAL, DRAW_INTERVAL / 60.0);
    printf("車両速度: %.2f m/s (%.1f km/h), ドローン速度: %.2f m/s (%.1f km/h)\n",
           V, V * 3.6, V_DRONE, V_DRONE * 3.6);

    if (ND > 0)
    {
        printf("ドローン台数: %d台, 出発間隔: %.1f秒 (%.1f分)\n",
               ND, drone_departure_interval, drone_departure_interval / 60.0);
        printf("ドローン1周時間: %.1f秒 (%.1f分)\n", drone_lap_time, drone_lap_time / 60.0);
        printf("ドローン物資運搬: 往復%d回, 停止時間%.0f分（集積所・避難所共通）\n",
               NR, T_DRONE_STOP / 60.0);
    }
    else
    {
        printf("ドローンなし（ND=0）\n");
    }

    // === 実行開始メッセージ ===
    if (ENABLE_GIF)
    {
        printf("\nシミュレーションを開始します。GIFファイル 'simulation.gif' を生成します...\n");
    }
    else
    {
        printf("\nシミュレーションを開始します（GIF出力なし）...\n");
    }
    printf("円の半径: %.1f m, 車両速度: %.2f m/s, 拠点数: %d\n", R, V, TOTAL_STOPS);

    /****** シミュレーション開始 ******/
    while (lap_count < NT)
    {
        // === 情報発生処理（ポアソン過程） ===
        // 現在時刻が次の情報発生時刻に達している間、新しい情報を発生
        while (elapsed_time >= next_info_time && info_count < MAX_INFO)
        {
            // 新しい情報をランダムな避難所に発生
            info_list[info_count].shelter_id = generate_random_shelter(); // 1〜NSの避難所
            info_list[info_count].generation_time = next_info_time;       // 発生時刻
            info_list[info_count].collection_time = -1;                   // 未回収状態
            info_list[info_count].collected = 0;                          // 未回収フラグ

            printf("情報発生: 時刻 %.1f秒 (%.1f分) : 避難所 %d\n",
                   next_info_time, next_info_time / 60.0, info_list[info_count].shelter_id);

            info_count++;
            // 次の情報発生時刻を指数分布で決定
            next_info_time += generate_exponential_interval(LAMBDA);
        }

        // === 現在停止地点の座標取得 ===
        double current_x = stop_coords[current_stop_idx][0];
        double current_y = stop_coords[current_stop_idx][1];

        // === 停止中の処理 ===
        // 初回出発時以外は各拠点で停止時間を設ける
        if (!is_first_departure)
        {
            // 停止開始時の描画
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count);

            // === 停止地点情報の表示 ===
            if (current_stop_idx == 0)
            {
                printf("周回: %d/%d | 集積所に到着。%d秒間停止します。\n", lap_count + 1, NT, T_STOP);
            }
            else
            {
                printf("周回: %d/%d | 避難所%dに到着。%d秒間停止します。\n", lap_count + 1, NT, current_stop_idx, T_STOP);

                // === 避難所での情報回収処理 ===
                // 現在の避難所に発生している未回収情報を全て回収
                for (int i = 0; i < info_count; i++)
                {
                    if (!info_list[i].collected && info_list[i].shelter_id == current_stop_idx)
                    {
                        // 情報回収実行
                        info_list[i].collection_time = elapsed_time;
                        info_list[i].collected = 1;

                        // Tc（回収遅延時間）の計算と統計更新
                        double tc = elapsed_time - info_list[i].generation_time;
                        total_tc += tc;
                        collected_count++;

                        printf("  情報回収: 発生%.1f秒 → 回収%.1f秒 (Tc=%.1f秒=%.1f分)\n",
                               info_list[i].generation_time, elapsed_time, tc, tc / 60.0);
                    }
                }
            }

            // === 停止時間中の処理ループ ===
            // 停止期間中も時間を進めてドローンの位置更新と描画を実行
            double stop_start_time = elapsed_time;
            double stop_end_time = elapsed_time + T_STOP;

            while (elapsed_time < stop_end_time)
            {
                // === 描画タイミングの判定 ===
                // 指定間隔（2分）ごとに描画フレームを生成
                if (elapsed_time >= next_draw_time)
                {
                    plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count);
                    next_draw_time += DRAW_INTERVAL; // 次の描画時刻を更新
                }

                // === 時間ステップ進行 ===
                double time_step = 1.0; // 1秒刻みで時間を進める
                elapsed_time += time_step;

                // === ドローン群の状態更新 ===
                for (int i = 0; i < ND; i++)
                {
                    // ドローン出発判定（出発時刻に達したらアクティブ化）
                    if (!drones[i].active && elapsed_time >= drones[i].start_time)
                    {
                        drones[i].active = 1;
                        printf("ドローン%d: 出発（時刻%.1f秒）\n", i + 1, elapsed_time);
                    }

                    // アクティブドローンの処理
                    if (drones[i].active)
                    {
                        // 情報検出チェック（通常巡回モードのみ）
                        if (drones[i].state == DRONE_PATROL)
                        {
                            int detected_shelter = check_drone_info_detection(&drones[i], stop_coords, info_list, info_count, elapsed_time);
                            if (detected_shelter > 0)
                            {
                                // 情報を検出、物資運搬モードに移行
                                drones[i].target_shelter = detected_shelter;
                                drones[i].current_trip = 1;
                                drones[i].target_x = stop_coords[0][0]; // 集積所座標
                                drones[i].target_y = stop_coords[0][1];
                                drones[i].state = DRONE_TO_DEPOT;

                                // ドローンによる情報回収処理（検出と同時に回収）
                                for (int j = 0; j < info_count; j++)
                                {
                                    if (!info_list[j].collected &&
                                        info_list[j].shelter_id == detected_shelter &&
                                        info_list[j].generation_time <= elapsed_time)
                                    {
                                        info_list[j].collected = 1;
                                        printf("ドローン%d: 避難所%dで情報検出・回収しました\n", i + 1, detected_shelter);
                                        break; // 1つの情報のみ回収
                                    }
                                }

                                printf("ドローン%d: 集積所へ向かいます\n", i + 1);
                            }
                        }

                        // ドローンの状態更新
                        update_drone_state(&drones[i], stop_coords, elapsed_time, time_step, info_list, info_count);
                    }
                }
            }

            // 停止時間終了時の正確な時刻調整
            elapsed_time = stop_end_time;
        }
        else
        {
            // === 初回出発時の処理 ===
            // シミュレーション開始時は停止せず、開始地点を描画
            printf("シミュレーション開始: 集積所から出発\n");
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count);
            next_draw_time = DRAW_INTERVAL; // 最初の描画後、次の描画時刻を設定
            is_first_departure = 0;         // 初回出発フラグをリセット
        }

        // === 次の目的地の決定 ===
        int next_stop_idx = (current_stop_idx + 1) % TOTAL_STOPS;

        // === 周回完了判定 ===
        // 次の目的地が集積所（インデックス0）なら1周完了
        if (next_stop_idx == 0)
        {
            lap_count++;
            printf("--- %d周目完了 ---\n", lap_count);
        }

        printf("移動開始: 拠点%d → 拠点%d\n", current_stop_idx, next_stop_idx);

        // === 移動中の処理 ===
        double move_start_time = elapsed_time;
        double move_end_time = elapsed_time + travel_time_per_segment;

        // === 車両の移動角度計算 ===
        double start_angle = PI / 2.0 - current_stop_idx * angle_increment; // 出発地点の角度
        double end_angle = PI / 2.0 - next_stop_idx * angle_increment;      // 到着地点の角度

        // === 角度差の正規化（最短経路選択） ===
        // 時計回り移動のため、角度差を適切に計算
        double angle_diff = end_angle - start_angle;
        // 180度を超える場合は短い方向を選択（円周の性質を利用）
        if (angle_diff > PI)
        {
            angle_diff -= 2.0 * PI; // 360度減算で時計回り短縮
        }
        else if (angle_diff < -PI)
        {
            angle_diff += 2.0 * PI; // 360度加算で反時計回り短縮
        }

        // === 移動中の時間ループ ===
        while (elapsed_time < move_end_time)
        {
            // === 車両位置の計算 ===
            // 移動の進捗度を計算（0.0〜1.0の範囲）
            double progress = (elapsed_time - move_start_time) / travel_time_per_segment;
            if (progress > 1.0)
                progress = 1.0; // 進捗度の上限制限

            // 車両の現在角度を線形補間で計算
            double current_angle = start_angle + angle_diff * progress;
            current_x = R * cos(current_angle); // 現在のX座標
            current_y = R * sin(current_angle); // 現在のY座標

            // === 描画タイミングの判定 ===
            if (elapsed_time >= next_draw_time)
            {
                plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count);
                next_draw_time += DRAW_INTERVAL; // 次の描画時刻を更新
            }

            // === 時間ステップ進行 ===
            double time_step = 1.0; // 1秒刻みで時間を進める
            elapsed_time += time_step;

            // === ドローン群の状態更新（移動中も継続） ===
            for (int i = 0; i < ND; i++)
            {
                // ドローン出発判定
                if (!drones[i].active && elapsed_time >= drones[i].start_time)
                {
                    drones[i].active = 1;
                    printf("ドローン%d: 出発（時刻%.1f秒）\n", i + 1, elapsed_time);
                }

                // アクティブドローンの処理
                if (drones[i].active)
                {
                    // 情報検出チェック（通常巡回モードのみ）
                    if (drones[i].state == DRONE_PATROL)
                    {
                        int detected_shelter = check_drone_info_detection(&drones[i], stop_coords, info_list, info_count, elapsed_time);
                        if (detected_shelter > 0)
                        {
                            // 情報を検出、物資運搬モードに移行
                            drones[i].target_shelter = detected_shelter;
                            drones[i].current_trip = 1;
                            drones[i].target_x = stop_coords[0][0]; // 集積所座標
                            drones[i].target_y = stop_coords[0][1];
                            drones[i].state = DRONE_TO_DEPOT;

                            // ドローンによる情報回収処理（検出と同時に回収）
                            for (int j = 0; j < info_count; j++)
                            {
                                if (!info_list[j].collected &&
                                    info_list[j].shelter_id == detected_shelter &&
                                    info_list[j].generation_time <= elapsed_time)
                                {
                                    info_list[j].collected = 1;
                                    printf("ドローン%d: 避難所%dで情報検出・回収しました\n", i + 1, detected_shelter);
                                    break; // 1つの情報のみ回収
                                }
                            }

                            printf("ドローン%d: 集積所へ向かいます\n", i + 1);
                        }
                    }

                    // ドローンの状態更新
                    update_drone_state(&drones[i], stop_coords, elapsed_time, time_step, info_list, info_count);
                }
            }
        }

        // === 移動完了時の正確な位置調整 ===
        elapsed_time = move_end_time;

        // === 次の停止地点への移動 ===
        current_stop_idx = next_stop_idx;
    }

    // === シミュレーション終了処理 ===
    printf("\n=== シミュレーション終了 ===\n");
    printf("完了周回数: %d周\n", lap_count);
    printf("総経過時間: %.0f秒 (%.1f時間)\n", elapsed_time, elapsed_time / 3600.0);

    // === 移動距離の計算と表示 ===
    double total_distance = lap_count * 2.0 * PI * R; // 総移動距離 = 周回数 × 円周
    printf("車両総移動距離: %.1f m (%.2f km)\n", total_distance, total_distance / 1000.0);
    printf("車両平均速度: %.2f m/s (%.1f km/h)\n", V, V * 3.6);

    // === 情報発生・回収の統計情報 ===
    printf("\n=== 情報回収システム統計 ===\n");
    printf("総情報発生数: %d件\n", info_count);

    // 全ての情報の回収状況を再計算（車両＋ドローンによる回収）
    int total_collected_count = 0;
    for (int i = 0; i < info_count; i++)
    {
        if (info_list[i].collected)
        {
            total_collected_count++;
        }
    }

    printf("回収済み情報数: %d件（車両: %d件, ドローン: %d件）\n",
           total_collected_count, collected_count, total_collected_count - collected_count);
    printf("未回収情報数: %d件\n", info_count - total_collected_count);

    // === Tc（回収遅延時間）の統計 ===
    if (total_collected_count > 0)
    {
        double average_tc = total_tc / collected_count; // 車両による回収のTc平均（車両回収のみ記録されているため）
        printf("Tc平均値（車両回収分のみ）: %.2f秒 (%.2f分)\n", average_tc, average_tc / 60.0);
        printf("Tc総計（車両回収分のみ）: %.2f秒 (%.2f時間)\n", total_tc, total_tc / 3600.0);
        printf("注記: ドローン回収分のTc統計は現在未実装\n");
    }
    else
    {
        printf("Tc平均値: 計算不可（車両による回収情報なし）\n");
    }

    // === GIFファイル処理 ===
    if (ENABLE_GIF && gnuplot_pipe != NULL)
    {
        printf("\n=== GIF出力処理 ===\n");
        printf("'simulation.gif' がカレントディレクトリに生成されました。\n");

        // GIFファイルを適切に終了（重要：ファイル破損防止）
        fprintf(gnuplot_pipe, "unset output\n");
        fflush(gnuplot_pipe);

        // gnuplotパイプを閉じる
        pclose(gnuplot_pipe);
    }
    else if (ENABLE_GIF)
    {
        printf("\n=== GIF出力エラー ===\n");
        printf("GIF出力が有効でしたが、gnuplotの初期化に失敗したため、GIFファイルは生成されませんでした。\n");
    }
    else
    {
        printf("\n=== GIF出力設定 ===\n");
        printf("GIF出力が無効のため、GIFファイルは生成されませんでした。\n");
    }

    return 0;
}
