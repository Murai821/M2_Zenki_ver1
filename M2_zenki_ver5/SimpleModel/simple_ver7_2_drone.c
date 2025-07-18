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
 * - ドローン移動：DRONE_CLOCKWISE設定により選択可能（0=反時計回り、1=時計回り）
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
 * 　　TVとドローンによる物資運搬実装、避難所でランダムに余剰物資が発生し、ドローンとTVで運搬
 * 7/14：ドローンの巡回方向を設定可能にし、反時計回りと時計回りを選択できるように変更
 * 　　　諸々の統計処理追加(無駄な飛行時間、余剰物資の運搬量など)
 * 7/15：手法1：ドローンが余剰物資Bを運搬途中のときに避難所に車両が停止してもドローンは動作継続（情報を最初に回収したものが運搬をになう）
 *       手法2：ドローンが余剰物資Bを運搬途中のときに避難所に車両が停止した場合、車両も余剰物資を届ける（ドローンがすでに運搬しているものは差し引いて）
 * 7/18 :ver7の拡張：ドローンによる往復回数を算出するのではなく、その都度集積所で最大積載量を乗せるように変更した（例. 76kg -> 30, 30, 16）
 *
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
#define ND 2         // ドローンの台数（0の場合はドローンなし、最大制限なし）
#define ENABLE_GIF 1 // GIF出力の有効/無効 (1:有効, 0:無効) | 処理軽量化用

// === ドローン巡回方向設定 ===
#define DRONE_CLOCKWISE 0                                                    // ドローン巡回方向: 0=反時計回り, 1=時計回り
#define DRONE_DIRECTION_NAME ((DRONE_CLOCKWISE) ? "時計回り" : "反時計回り") // 表示用文字列

// === 情報発生システム（ポアソン過程） ===
#define LAMBDA 1.0    // ポアソン到着率 [件/時間] | 1時間に平均0.5件の情報発生
#define MAX_INFO 1000 // 最大情報数（メモリ制限対策）

// === ドローン物資運搬システム ===
#define NR 2                   // ドローンの往復回数（基本値、情報により動的変更）
#define T_DRONE_STOP (10 * 60) // ドローンの停止時間 (s) | 10分=600秒（集積所・避難所共通）
#define DRONE_MAX_CARRY 30.0   // ドローンの最大積載量 (kg)
#define MIN_EXTRA_DEMAND 0.0   // 余剰物資B需要量の最小値 (kg)
#define MAX_EXTRA_DEMAND 90.0  // 余剰物資B需要量の最大値 (kg)

// === 物資運搬車両システム ===
#define TOTAL_SUPPLY_WEIGHT 10000.0                                // 物資運搬車両の総積載量 (kg)
#define SUPPLY_A_RATIO 0.9                                         // 物資Aの割合（0.0〜1.0）
#define SUPPLY_B_RATIO 0.1                                         // 物資Bの割合（0.0〜1.0）
#define EXTRA_SUPPLY_B 1000.0                                      // 余剰分の物資B量 (kg)
#define SUPPLY_PER_SHELTER (TOTAL_SUPPLY_WEIGHT / NS)              // 1避難所あたりの物資量 (kg)
#define SUPPLY_A_PER_SHELTER (SUPPLY_PER_SHELTER * SUPPLY_A_RATIO) // 1避難所あたりの物資A量 (kg)
#define SUPPLY_B_PER_SHELTER (SUPPLY_PER_SHELTER * SUPPLY_B_RATIO) // 1避難所あたりの物資B量 (kg)

// === 余剰物資B配送手法の選択 ===
#define DELIVERY_METHOD_IGNORE 0     // 手法1：車両がドローンの運搬を無視
#define DELIVERY_METHOD_COORDINATE 1 // 手法2：車両がドローンの運搬状況を考慮
// #define DELIVERY_METHOD DELIVERY_METHOD_COORDINATE // 使用する配送手法を選択
#define DELIVERY_METHOD DELIVERY_METHOD_IGNORE // 使用する配送手法を選択

// === 派生的な定数（自動計算） ===
#define TOTAL_STOPS (NS + 1) // 集積所を含めた拠点の総数（11拠点）
#define DRAW_INTERVAL 120.0  // 描画間隔（秒）| 2分ごとにGIFフレーム生成

// === 回収主体の定数定義 ===
#define COLLECTED_BY_NONE 0    // 未回収
#define COLLECTED_BY_VEHICLE 1 // 車両による回収
#define COLLECTED_BY_DRONE 2   // ドローンによる回収

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
    int shelter_id;                // 情報が発生した避難所ID (1〜NS) | 0は集積所なので除外
    double generation_time;        // 情報発生時刻（秒）| シミュレーション開始からの経過時間
    double collection_time;        // 情報回収時刻（秒）| -1の場合は未回収状態
    int collected;                 // 回収済みフラグ（0:未回収, 1:回収済み）
    int collected_by;              // 回収主体（0:未回収, 1:車両, 2:ドローン）
    double extra_supply_demand;    // 余剰物資B需要量 (kg) | 0〜90kgの範囲
    double extra_supply_delivered; // 配送済み余剰物資B量 (kg)
    int delivery_completed;        // 余剰物資B配送完了フラグ（0:未完了, 1:完了）
} Info;

// === 避難所物資管理用構造体 ===
/**
 * @brief 各避難所の物資在庫を管理する構造体
 *
 * 物資運搬車両による物資配送管理：
 * - 物資A：主要物資（デフォルト90%）
 * - 物資B：補助物資（デフォルト10%）
 * - 車両が集積所に戻ると物資を補充して次の配送開始
 */
typedef struct
{
    double supply_a;       // 物資Aの在庫量 (kg)
    double supply_b;       // 物資Bの在庫量 (kg)
    double extra_supply_b; // 余剰物資Bの在庫量 (kg)
} ShelterSupply;

// === 物資運搬車両管理用構造体 ===
/**
 * @brief 物資運搬車両の状態と積載物資を管理する構造体
 */
typedef struct
{
    double remaining_supply_a;       // 積載中の物資A残量 (kg)
    double remaining_supply_b;       // 積載中の物資B残量 (kg)
    double remaining_extra_supply_b; // 積載中の余剰分物資B残量 (kg)
    int is_loaded;                   // 積載状態（0:空, 1:積載中）
} SupplyVehicle;

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
    DroneState state;             // 現在の動作状態
    int target_shelter;           // 目標避難所ID（物資運搬時）
    int current_trip;             // 現在の往復回数（1〜NR）
    int required_trips;           // 必要な往復回数（余剰物資B需要に基づく）
    double state_start_time;      // 現在状態の開始時刻
    double target_x, target_y;    // 目標座標（直線飛行時）
    double carrying_extra_supply; // 現在積載中の余剰物資B量 (kg)

    // 飛行時間統計用フィールド
    double total_flight_time;     // 総飛行時間（秒）
    double supply_transport_time; // 物資運搬飛行時間（秒）
    double last_state_time;       // 前回の状態変更時刻（秒）
} DroneInfo;

// === 関数プロトタイプ宣言 ===
double generate_extra_supply_demand();
int calculate_required_trips(double demand);
void update_drone_state(DroneInfo *drone, double stop_coords[][2], double elapsed_time, double time_step, Info *info_list, int info_count, ShelterSupply *shelter_supplies, double *total_extra_supply_by_drone, int *drone_delivery_count);
void update_drone_flight_time(DroneInfo *drone, double current_time, DroneState new_state);
double calculate_drone_transport_amount(DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);

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
void update_drone_state(DroneInfo *drone, double stop_coords[][2], double elapsed_time, double time_step, Info *info_list, int info_count, ShelterSupply *shelter_supplies, double *total_extra_supply_by_drone, int *drone_delivery_count)
{
    if (!drone->active)
        return;

    switch (drone->state)
    {
    case DRONE_PATROL:
        // 通常巡回（設定可能な方向）
        {
            // === ドローン角度増分の計算 ===
            double drone_angle_increment = (V_DRONE / R) * time_step;

            // === 巡回方向の適用 ===
            if (DRONE_CLOCKWISE)
            {
                // 時計回り：角度を減算（車両と同方向）
                drone->angle -= drone_angle_increment;

                // 角度正規化（負の値を正の値に変換）
                if (drone->angle < 0.0)
                {
                    drone->angle += 2.0 * PI;
                }
            }
            else
            {
                // 反時計回り：角度を加算（車両と逆方向、デフォルト）
                drone->angle += drone_angle_increment;

                // 角度正規化（2πを超えた値を正規化）
                if (drone->angle > 2.0 * PI)
                {
                    drone->angle -= 2.0 * PI;
                }
            }

            // === 座標更新 ===
            // 角度に基づいて円周上の座標を計算
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
                update_drone_flight_time(drone, elapsed_time, DRONE_AT_DEPOT);
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
        // 集積所での停止（余剰物資B積載）
        if (elapsed_time >= drone->state_start_time + T_DRONE_STOP)
        {
            // 今回の積載量を計算（残り需要量とドローン最大積載量の小さい方）
            double remaining_demand = 0;
            for (int i = 0; i < info_count; i++)
            {
                if (info_list[i].shelter_id == drone->target_shelter && !info_list[i].delivery_completed)
                {
                    remaining_demand = info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered;
                    break;
                }
            }

            drone->carrying_extra_supply = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;

            // 停止時間終了、避難所への飛行開始
            drone->target_x = stop_coords[drone->target_shelter][0];
            drone->target_y = stop_coords[drone->target_shelter][1];
            update_drone_flight_time(drone, elapsed_time, DRONE_TO_SHELTER);
            printf("ドローン: 集積所出発（避難所%dへ、余剰物資B %.0fkg積載）\n",
                   drone->target_shelter, drone->carrying_extra_supply);
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
                update_drone_flight_time(drone, elapsed_time, DRONE_AT_SHELTER);
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
        // 避難所での停止（余剰物資B降ろし）
        if (elapsed_time >= drone->state_start_time + T_DRONE_STOP)
        {
            // 余剰物資Bを配送
            for (int i = 0; i < info_count; i++)
            {
                if (info_list[i].shelter_id == drone->target_shelter && !info_list[i].delivery_completed)
                {
                    // 避難所の余剰物資B在庫を増加
                    // shelter_suppliesの配列インデックスは0ベースなので -1
                    int shelter_idx = drone->target_shelter - 1;
                    if (shelter_idx >= 0 && shelter_idx < NS)
                    {
                        shelter_supplies[shelter_idx].extra_supply_b += drone->carrying_extra_supply;
                    }

                    info_list[i].extra_supply_delivered += drone->carrying_extra_supply;
                    printf("ドローン: 避難所%dに余剰物資B %.0fkg配送（累計 %.0f/%.0fkg）\n",
                           drone->target_shelter, drone->carrying_extra_supply,
                           info_list[i].extra_supply_delivered, info_list[i].extra_supply_demand);

                    // === ドローンによる余剰物資B運搬統計の更新 ===
                    *total_extra_supply_by_drone += drone->carrying_extra_supply;
                    (*drone_delivery_count)++;

                    // 配送完了チェック
                    if (info_list[i].extra_supply_delivered >= info_list[i].extra_supply_demand)
                    {
                        info_list[i].delivery_completed = 1;
                        printf("ドローン: 避難所%dの余剰物資B配送完了, 避難所%d の Req:%f \n", drone->target_shelter, drone->target_shelter, info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered);
                        drone->current_trip = drone->required_trips + 1; // 往復完了フラグを設定（手法2によって,車両が避難所の余剰物資を運搬したことによるドローンの往復回数短縮を考慮して（車両によってドローンの往復必要回数がへる場合があるため））
                    }
                    break;
                }
            }

            drone->current_trip++;
            drone->carrying_extra_supply = 0; // 積載物資をリセット

            // 配送完了チェック（余剰物資B需要が満たされたか）
            int delivery_completed = 0;
            for (int i = 0; i < info_count; i++)
            {
                if (info_list[i].shelter_id == drone->target_shelter && info_list[i].delivery_completed)
                {
                    delivery_completed = 1;
                    break;
                }
            }

            if (!delivery_completed)
            {
                // まだ配送が必要、集積所に戻る
                drone->target_x = stop_coords[0][0]; // 集積所座標
                drone->target_y = stop_coords[0][1];
                update_drone_flight_time(drone, elapsed_time, DRONE_TO_DEPOT);
                printf("ドローン: 避難所%d出発（往復%d回目、配送継続）\n",
                       drone->target_shelter, drone->current_trip);
            }
            else
            {
                // 配送完了、通常巡回に復帰
                update_drone_flight_time(drone, elapsed_time, DRONE_PATROL);
                drone->current_trip = 0;
                drone->required_trips = 0;

                // 現在位置から巡回角度を計算
                drone->angle = atan2(drone->y, drone->x);
                if (drone->angle < 0)
                    drone->angle += 2.0 * PI;

                printf("ドローン: 余剰物資B運搬完了、通常巡回に復帰\n");
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
 * @param shelter_supplies 各避難所の物資在庫配列
 * @param supply_vehicle 物資運搬車両の状態
 */
void plot_frame(FILE *pipe, double stop_coords[][2], double vehicle_x, double vehicle_y, DroneInfo drones[], double elapsed_time, Info *info_list, int info_count, ShelterSupply shelter_supplies[], SupplyVehicle *supply_vehicle)
{
    // GIF出力が無効の場合は処理をスキップ（高速化）
    if (!ENABLE_GIF || pipe == NULL)
        return;

    // === 時刻表示の準備 ===
    // 経過時間を時:分:秒の形式に分解
    int hours = (int)(elapsed_time / 3600);
    int minutes = (int)((elapsed_time - hours * 3600) / 60);
    int seconds = (int)(elapsed_time - hours * 3600 - minutes * 60);

    // タイトルバーに経過時間と物資情報を表示
    if (hours > 0)
    {
        fprintf(pipe, "set title 'Vehicle Simulation - Time: %02d:%02d hour | A: %.0fkg B: %.0fkg ExtraB: %.0fkg'\n",
                hours, minutes, supply_vehicle->remaining_supply_a, supply_vehicle->remaining_supply_b, supply_vehicle->remaining_extra_supply_b);
    }
    else if (minutes > 0)
    {
        fprintf(pipe, "set title 'Vehicle Simulation - Time: %02d min | A: %.0fkg B: %.0fkg ExtraB: %.0fkg'\n",
                minutes, supply_vehicle->remaining_supply_a, supply_vehicle->remaining_supply_b, supply_vehicle->remaining_extra_supply_b);
    }
    else
    {
        fprintf(pipe, "set title 'Vehicle Simulation - Time: 0 min | A: %.0fkg B: %.0fkg ExtraB: %.0fkg'\n",
                supply_vehicle->remaining_supply_a, supply_vehicle->remaining_supply_b, supply_vehicle->remaining_extra_supply_b);
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
        fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0); // 画面外ダミーデータ
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
        fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0); // 画面外ダミーデータ
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
            fprintf(pipe, "%.1f %.1f\n", -999999.0, -999999.0); // 画面外ダミーデータ
        }
        fprintf(pipe, "e\n");
    }

    // === 物資量可視化処理 ===
    // 各避難所の近くに物資量を数値で表示
    // 物資A（青色テキスト）と物資B（赤色テキスト）の在庫量を数値で表現
    const double supply_offset = 20.0; // 避難所からの表示オフセット距離

    // 既存のlabelを削除してから新しいlabelを設定
    fprintf(pipe, "unset label\n");

    // 物資A・B・余剰B・要求余剰Bの数値表示
    for (int i = 1; i <= NS; i++) // 避難所のみ（集積所は除く）
    {
        // 各避難所の現在の要求余剰物資B量を計算
        double current_demand = 0.0;
        for (int j = 0; j < info_count; j++)
        {
            if (info_list[j].shelter_id == i && !info_list[j].delivery_completed &&
                info_list[j].generation_time <= elapsed_time)
            {
                current_demand += (info_list[j].extra_supply_demand - info_list[j].extra_supply_delivered);
            }
        }

        // 物資Aの表示位置（避難所の右上側）
        double display_x_a = stop_coords[i][0] + supply_offset;
        double display_y_a = stop_coords[i][1] + 300.0; // 最上段

        // 物資Bの表示位置（避難所の右上中央側）
        double display_x_b = stop_coords[i][0] + supply_offset;
        double display_y_b = stop_coords[i][1] + 100.0; // 上中段

        // 余剰物資Bの表示位置（避難所の右下中央側）
        double display_x_extra = stop_coords[i][0] + supply_offset;
        double display_y_extra = stop_coords[i][1] - 100.0; // 下中段

        // 要求余剰物資Bの表示位置（避難所の右下側）
        double display_x_demand = stop_coords[i][0] + supply_offset;
        double display_y_demand = stop_coords[i][1] - 300.0; // 最下段

        // 物資Aの数値表示（青色）- 常に表示
        fprintf(pipe, "set label 'A:%.0f' at %.1f,%.1f tc rgb '#0000FF' font ',11'\n",
                shelter_supplies[i - 1].supply_a, display_x_a, display_y_a);

        // 物資Bの数値表示（赤色）- 常に表示
        fprintf(pipe, "set label 'B:%.0f' at %.1f,%.1f tc rgb '#FF0000' font ',11'\n",
                shelter_supplies[i - 1].supply_b, display_x_b, display_y_b);

        // 余剰物資Bの数値表示（紫色）- 常に表示
        fprintf(pipe, "set label 'ExB:%.0f' at %.1f,%.1f tc rgb '#800080' font ',11'\n",
                shelter_supplies[i - 1].extra_supply_b, display_x_extra, display_y_extra);

        // 要求余剰物資Bの数値表示（オレンジ色）- 要求がある場合のみ表示
        if (current_demand > 0.0)
        {
            fprintf(pipe, "set label 'Req:%.0f' at %.1f,%.1f tc rgb '#FF8000' font ',11'\n",
                    current_demand, display_x_demand, display_y_demand);
        }
    }

    // バッファをフラッシュしてgnuplotに確実に送信
    fflush(pipe);
}

/**
 * @brief 余剰物資B需要量をランダムに生成
 *
 * 【生成ロジック】
 * - 一様分布により0〜90kgの範囲でランダム生成
 * - 実際の災害現場での緊急物資需要を模擬
 * - 各情報発生時に1回だけ生成され、配送完了まで固定値
 *
 * 【数学的背景】
 * - X = min + U * (max - min)
 * - U: [0,1)の一様乱数
 * - min: MIN_EXTRA_DEMAND (0kg)
 * - max: MAX_EXTRA_DEMAND (90kg)
 *
 * @return 余剰物資B需要量 (0〜90kg)
 */
double generate_extra_supply_demand()
{
    // 0〜90kgの範囲で一様分布による需要量生成
    // (double)rand() / RAND_MAX で [0,1) の一様乱数を生成
    // MIN_EXTRA_DEMAND + U * (MAX_EXTRA_DEMAND - MIN_EXTRA_DEMAND) で範囲を変換
    return MIN_EXTRA_DEMAND + ((double)rand() / RAND_MAX) * (MAX_EXTRA_DEMAND - MIN_EXTRA_DEMAND);
}

/**
 * @brief 余剰物資B需要に基づいてドローンの必要往復回数を計算
 *
 * 【計算ロジック】
 * - ドローンの最大積載量（30kg）で需要量を割って往復回数を算出
 * - 天井関数（ceil）により端数は1回分として計算
 * - 例：需要55kg → 55/30 = 1.83... → ceil(1.83) = 2往復
 *
 * 【制約条件】
 * - 需要量が0以下の場合は往復回数0
 * - 最大往復回数の制限なし（理論上MAX_EXTRA_DEMAND/DRONE_MAX_CARRYまで）
 * - 各往復で最大30kgまで運搬可能
 *
 * @param demand 余剰物資B需要量 (kg)
 * @return 必要往復回数（0以上の整数）
 */
int calculate_required_trips(double demand)
{
    // 需要量が0以下の場合は往復不要
    if (demand <= 0)
        return 0;

    // ceiling関数で端数切り上げにより必要往復回数を計算
    // 例: demand=55kg, DRONE_MAX_CARRY=30kg → 55/30=1.83 → ceil(1.83)=2往復
    return (int)ceil(demand / DRONE_MAX_CARRY);
}

/**
 * @brief ドローンの状態を変更・ドローンの飛行時間統計を更新する関数
 *
 * ドローンの状態変更時に呼び出され、前の状態での飛行時間を累積する。
 * 物資運搬に関連する状態（TO_DEPOT, AT_DEPOT, TO_SHELTER, AT_SHELTER）の時間を
 * supply_transport_timeに累積し、全ての活動時間をtotal_flight_timeに累積する。
 *
 * @param drone ドローン情報構造体
 * @param current_time 現在時刻（秒）
 * @param new_state 新しい状態
 */
void update_drone_flight_time(DroneInfo *drone, double current_time, DroneState new_state)
{
    if (!drone->active)
        return;

    // 前回の状態での経過時間を計算
    double state_duration = current_time - drone->last_state_time;

    // 総飛行時間に加算
    drone->total_flight_time += state_duration;

    // 物資運搬関連の状態の場合、物資運搬時間に加算：現在の状態に応じて物資運搬時刻を加算するか判断
    if (drone->state == DRONE_TO_DEPOT ||
        drone->state == DRONE_AT_DEPOT ||
        drone->state == DRONE_TO_SHELTER ||
        drone->state == DRONE_AT_SHELTER)
    {
        drone->supply_transport_time += state_duration;
    }

    // 次回の状態のために現在時刻を記録
    drone->last_state_time = current_time;
    drone->state = new_state; // ドローンの状態を更新
}

/**
 * @brief ドローンによる指定避難所への余剰物資B運搬量を計算する関数
 *
 * この関数は、指定された避難所に対してドローンが運搬中または運搬予定の
 * 余剰物資B量を計算します。車両が配送量を決定する際に使用します。
 *
 * @param drones ドローン配列
 * @param drone_count ドローン数
 * @param shelter_id 対象避難所ID
 * @param info_list 情報リスト
 * @param info_count 情報数
 * @return ドローンによる運搬量（運搬中+運搬予定）
 */
double calculate_drone_transport_amount(DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count)
{
    double drone_transport_amount = 0.0;

    // 指定避難所の情報を検索
    for (int i = 0; i < info_count; i++)
    {
        if (info_list[i].shelter_id == shelter_id && !info_list[i].delivery_completed)
        {
            // この避難所に対して運搬中のドローンを検索
            for (int j = 0; j < drone_count; j++)
            {
                if (drones[j].active && drones[j].target_shelter == shelter_id)
                {
                    // ドローンの状態に応じて運搬量を計算
                    switch (drones[j].state)
                    {
                    case DRONE_TO_DEPOT:
                    case DRONE_AT_DEPOT:
                        // 集積所に向かっている、または集積所にいる場合
                        // 次の往復で運搬予定の物資量を計算
                        {
                            double remaining_demand = info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered;
                            if (remaining_demand > 0)
                            {
                                // 次の往復で運搬する量 = min(DRONE_MAX_CARRY, 残り需要量)
                                double next_trip_amount = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;
                                drone_transport_amount += next_trip_amount;
                            }
                        }
                        break;
                    case DRONE_TO_SHELTER:
                        // 避難所に向かっている場合（積載中）
                        // 現在積載している物資量のみを計算
                        drone_transport_amount += drones[j].carrying_extra_supply;
                        break;
                    case DRONE_AT_SHELTER:
                        // 避難所にいる場合（降ろし中）
                        // 現在の往復が完了後、次の往復があるかチェック
                        {
                            double remaining_demand = info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered;
                            if (remaining_demand > 0 && !info_list[i].delivery_completed)
                            {
                                // 次の往復で運搬する量 = min(DRONE_MAX_CARRY, 残り需要量)
                                double next_trip_amount = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;
                                drone_transport_amount += next_trip_amount;
                            }
                        }
                        break;
                    default:
                        // その他の状態では運搬量はゼロ
                        break;
                    }
                }
            }
            break; // 該当する情報が見つかったので終了
        }
    }

    return drone_transport_amount;
}

/****************************************** メイン関数 ************************************************************/
int main(void)
{
    // === 初期化処理 ===

    // 乱数シードの初期化（実行毎に異なる結果を得るため）
    // srand(time(NULL));
    srand(12);

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

    // === 物資管理変数 ===
    ShelterSupply shelter_supplies[NS]; // 各避難所の物資在庫配列
    SupplyVehicle supply_vehicle;       // 物資運搬車両の状態

    // === 余剰物資B運搬統計変数 ===
    double total_extra_supply_by_vehicle = 0.0; // 車両による余剰物資B運搬総量 (kg)
    double total_extra_supply_by_drone = 0.0;   // ドローンによる余剰物資B運搬総量 (kg)
    int vehicle_delivery_count = 0;             // 車両による余剰物資B配送回数
    int drone_delivery_count = 0;               // ドローンによる余剰物資B配送回数(往復の回数も含む)

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
        drones[i].required_trips = 0;                  // 必要往復回数0
        drones[i].state_start_time = 0.0;              // 状態開始時刻
        drones[i].target_x = drones[i].target_y = 0.0; // 目標座標
        drones[i].carrying_extra_supply = 0.0;         // 積載中の余剰物資B量

        // 飛行時間統計用フィールドの初期化
        drones[i].total_flight_time = 0.0;     // 総飛行時間
        drones[i].supply_transport_time = 0.0; // 物資運搬飛行時間
        drones[i].last_state_time = 0.0;       // 前回の状態変更時刻
    }

    // === 【物資管理システムの初期化】 ===
    // 各避難所の物資在庫を初期化（空の状態から開始）
    // シミュレーション開始時点では、車両による配送前のため全避難所の在庫は0
    for (int i = 0; i < NS; i++)
    {
        shelter_supplies[i].supply_a = 0.0;       // 物資A在庫: 0kg
        shelter_supplies[i].supply_b = 0.0;       // 物資B在庫: 0kg
        shelter_supplies[i].extra_supply_b = 0.0; // 余剰物資B在庫: 0kg
    }

    // === 物資運搬車両の初期状態設定 ===
    // 車両は最初から満載状態でシミュレーション開始
    // 実際の災害対応では、事前準備された物資を積載して出発するシナリオを想定
    supply_vehicle.remaining_supply_a = TOTAL_SUPPLY_WEIGHT * SUPPLY_A_RATIO; // 物資A: 9000kg (90%)
    supply_vehicle.remaining_supply_b = TOTAL_SUPPLY_WEIGHT * SUPPLY_B_RATIO; // 物資B: 1000kg (10%)
    supply_vehicle.remaining_extra_supply_b = EXTRA_SUPPLY_B;                 // 余剰物資B: 1000kg
    supply_vehicle.is_loaded = 1;                                             // 積載状態フラグ: 積載中

    // === 【シミュレーション設定情報の表示】 ===
    printf("=== シミュレーション設定 ===\n");
    printf("停止時間: %.1f秒, 車両1区間移動時間: %.1f秒\n", (double)T_STOP, travel_time_per_segment);
    printf("描画間隔: %.0f秒（%.0f分ごと）\n", DRAW_INTERVAL, DRAW_INTERVAL / 60.0);
    printf("車両速度: %.2f m/s (%.1f km/h), ドローン速度: %.2f m/s (%.1f km/h)\n",
           V, V * 3.6, V_DRONE, V_DRONE * 3.6);

    // === 物資運搬システムの詳細設定表示 ===
    printf("物資運搬システム: 総積載量%.0fkg (A:%.0f%%, B:%.0f%%) + 余剰B:%.0fkg\n",
           TOTAL_SUPPLY_WEIGHT, SUPPLY_A_RATIO * 100, SUPPLY_B_RATIO * 100, EXTRA_SUPPLY_B);
    printf("1避難所あたり配送量: %.0fkg (A:%.0fkg, B:%.0fkg)\n",
           SUPPLY_PER_SHELTER, SUPPLY_A_PER_SHELTER, SUPPLY_B_PER_SHELTER);

    // === ドローンシステムの設定表示 ===
    if (ND > 0)
    {
        printf("ドローン台数: %d台, 出発間隔: %.1f秒 (%.1f分)\n",
               ND, drone_departure_interval, drone_departure_interval / 60.0);
        printf("ドローン1周時間: %.1f秒 (%.1f分), 巡回方向: %s\n",
               drone_lap_time, drone_lap_time / 60.0, DRONE_DIRECTION_NAME);
        printf("ドローン物資運搬: 往復%d回, 停止時間%.0f分（集積所・避難所共通）\n",
               NR, T_DRONE_STOP / 60.0);
        printf("余剰物資B配送手法: %s\n",
               (DELIVERY_METHOD == DELIVERY_METHOD_IGNORE) ? "手法1（ドローン運搬無視）" : "手法2（ドローン運搬考慮）");
    }
    else
    {
        printf("ドローンなし（ND=0）\n");
    }

    // === 【実行開始メッセージ】 ===
    if (ENABLE_GIF)
    {
        printf("\nシミュレーションを開始します。GIFファイル 'simulation.gif' を生成します...\n");
    }
    else
    {
        printf("\nシミュレーションを開始します（GIF出力なし）...\n");
    }
    printf("円の半径: %.1f m, 車両速度: %.2f m/s, 拠点数: %d\n", R, V, TOTAL_STOPS);

    /****** 【メインシミュレーションループ開始】 ******/
    // 指定された周回数（NT）まで車両の巡回を継続
    // 各周回で全拠点（集積所+10避難所）を訪問し、物資配送・情報回収を実行
    while (lap_count < NT)
    {
        // === 【情報発生処理】（ポアソン過程による確率的イベント生成） ===
        // 現在時刻が次の情報発生予定時刻に達している間、連続的に新情報を発生
        // ポアソン過程により、まれに短時間で複数情報が連続発生する場合もある
        while (elapsed_time >= next_info_time && info_count < MAX_INFO)
        {
            // === 新情報の生成と初期化 ===
            info_list[info_count].shelter_id = generate_random_shelter();               // ランダム避難所（1〜NS）を選択
            info_list[info_count].generation_time = next_info_time;                     // 発生時刻を記録
            info_list[info_count].collection_time = -1;                                 // 未回収状態（-1で表現）
            info_list[info_count].collected = 0;                                        // 回収フラグ: 未回収
            info_list[info_count].collected_by = COLLECTED_BY_NONE;                     // 回収主体: 未回収
            info_list[info_count].extra_supply_demand = generate_extra_supply_demand(); // 0〜90kgの需要量をランダム生成
            info_list[info_count].extra_supply_delivered = 0.0;                         // 配送済み量: 初期0kg
            info_list[info_count].delivery_completed = 0;                               // 配送完了フラグ: 未完了

            // 情報発生をログ出力（デバッグ・進捗確認用）
            printf("情報発生: 時刻 %.1f秒 (%.1f分) : 避難所 %d (余剰物資B需要: %.0fkg)\n",
                   next_info_time, next_info_time / 60.0, info_list[info_count].shelter_id,
                   info_list[info_count].extra_supply_demand);

            info_count++; // 情報総数をインクリメント

            // === 次回情報発生時刻の決定 ===
            // 指数分布により次の発生間隔を生成し、現在時刻に加算
            // ポアソン過程の性質により、間隔は指数分布に従う
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
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, shelter_supplies, &supply_vehicle);

            // === 停止地点情報の表示 ===
            // 集積所に停止する場合
            if (current_stop_idx == 0)
            {
                printf("周回: %d/%d | 集積所に到着。%d秒間停止します。\n", lap_count + 1, NT, T_STOP);

                // === 集積所での物資補充処理 ===
                if (!supply_vehicle.is_loaded)
                {
                    supply_vehicle.remaining_supply_a = TOTAL_SUPPLY_WEIGHT * SUPPLY_A_RATIO;
                    supply_vehicle.remaining_supply_b = TOTAL_SUPPLY_WEIGHT * SUPPLY_B_RATIO;
                    supply_vehicle.remaining_extra_supply_b = EXTRA_SUPPLY_B;
                    supply_vehicle.is_loaded = 1;
                    printf("  物資補充: A=%.0fkg, B=%.0fkg, 余剰B=%.0fkg を積載\n",
                           supply_vehicle.remaining_supply_a, supply_vehicle.remaining_supply_b,
                           supply_vehicle.remaining_extra_supply_b);
                }
            }
            else // 避難所に停止する場合
            {
                printf("周回: %d/%d | 避難所%dに到着。%d秒間停止します。\n", lap_count + 1, NT, current_stop_idx, T_STOP);

                // === 【避難所での物資配送処理】 ===
                // 車両が通常物資（A・B）を積載中で、両方とも残量がある場合のみ配送実行
                if (supply_vehicle.is_loaded && supply_vehicle.remaining_supply_a > 0 && supply_vehicle.remaining_supply_b > 0)
                {
                    // === 定量配送の実行 ===
                    // 各避難所に対して一定量（1000kg分）の物資A・Bを配送
                    // 物資A: 900kg (90%), 物資B: 100kg (10%) の比率で配送
                    shelter_supplies[current_stop_idx - 1].supply_a += SUPPLY_A_PER_SHELTER; // 避難所在庫増加
                    shelter_supplies[current_stop_idx - 1].supply_b += SUPPLY_B_PER_SHELTER;

                    // 車両積載量から配送分を減算
                    supply_vehicle.remaining_supply_a -= SUPPLY_A_PER_SHELTER;
                    supply_vehicle.remaining_supply_b -= SUPPLY_B_PER_SHELTER;

                    // 配送実行のログ出力
                    printf("  物資配送: A=%.0fkg, B=%.0fkg を配送\n", SUPPLY_A_PER_SHELTER, SUPPLY_B_PER_SHELTER);
                    printf("  避難所%d在庫: A=%.0fkg, B=%.0fkg\n",
                           current_stop_idx, shelter_supplies[current_stop_idx - 1].supply_a,
                           shelter_supplies[current_stop_idx - 1].supply_b);

                    // === 車両積載状態の判定 ===
                    // 通常物資A・Bが両方とも空になった場合、積載状態を解除
                    // 余剰物資Bは別途管理されるため、残量があっても積載状態は解除される
                    if (supply_vehicle.remaining_supply_a <= 0 && supply_vehicle.remaining_supply_b <= 0)
                    {
                        supply_vehicle.is_loaded = 0; // 空状態に変更
                        printf("  車両の通常物資が空になりました（余剰B: %.0fkg残存）\n",
                               supply_vehicle.remaining_extra_supply_b);
                    }
                }

                // === 【避難所での情報回収処理】 ===
                // 現在の避難所に発生している未回収情報を全て回収
                // 1つの避難所に複数情報が発生している場合も全て一括回収
                for (int i = 0; i < info_count; i++)
                {
                    // 対象条件: 未回収 かつ 現在の避難所で発生した情報
                    if (!info_list[i].collected && info_list[i].shelter_id == current_stop_idx)
                    {
                        // === 情報回収の実行 ===
                        info_list[i].collection_time = elapsed_time;      // 回収時刻を記録
                        info_list[i].collected = 1;                       // 回収済みフラグを設定
                        info_list[i].collected_by = COLLECTED_BY_VEHICLE; // 車両による回収を記録

                        // === Tc（回収遅延時間）の計算と統計更新 ===
                        // Tc = 回収時刻 - 発生時刻（災害情報システムの重要評価指標）
                        double tc = elapsed_time - info_list[i].generation_time;
                        total_tc += tc;    // 累積Tc（平均計算用）
                        collected_count++; // 回収件数カウント

                        // 回収実行のログ出力
                        printf("  情報回収: 発生%.1f秒 → 回収%.1f秒 (Tc=%.1f秒=%.1f分)\n",
                               info_list[i].generation_time, elapsed_time, tc, tc / 60.0);

                        // === 【車両による余剰物資B配送処理】 ===
                        // 情報回収と同時に、その情報に関連する余剰物資B需要に対応
                        // 車両に余剰物資Bの残量があり、配送が未完了の場合のみ実行（手法1と手法2においてどちらも実行される）
                        if (supply_vehicle.remaining_extra_supply_b > 0 && !info_list[i].delivery_completed)
                        {
                            double delivery_amount = 0.0;

                            delivery_amount = (info_list[i].extra_supply_demand > supply_vehicle.remaining_extra_supply_b) ? supply_vehicle.remaining_extra_supply_b : info_list[i].extra_supply_demand;

                            // === 配送の実行 ===
                            if (delivery_amount > 0)
                            {
                                shelter_supplies[current_stop_idx - 1].extra_supply_b += delivery_amount; // 避難所在庫増加
                                supply_vehicle.remaining_extra_supply_b -= delivery_amount;
                                info_list[i].extra_supply_delivered += delivery_amount;

                                // === 車両による余剰物資B運搬統計の更新 ===
                                total_extra_supply_by_vehicle += delivery_amount;
                                vehicle_delivery_count++;

                                if (info_list[i].extra_supply_delivered >= info_list[i].extra_supply_demand)
                                {
                                    info_list[i].delivery_completed = 1;
                                    printf("  余剰物資B配送: %.0fkg配送（需要完了）\n", delivery_amount);
                                }
                                else
                                {
                                    printf("  余剰物資B配送: %.0fkg配送（残り需要 %.0fkg）\n",
                                           delivery_amount, info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered);
                                }
                            }
                            else
                            {
                                printf("  余剰物資B配送: ドローンが運搬中のため車両配送スキップ\n");
                            }
                        }
                    }
                }

                // 避難所の情報は回収済みだが、ドローンによる余剰物資B配送が未完了のときに車両が到着した場合(手法2:すべてドローンに任せるのではなく車両がのこりの物資を運搬する)
                for (int i = 0; i < info_count; i++)
                {
                    // 現在の避難所において
                    if (info_list[i].shelter_id == current_stop_idx)
                    {
                        // === 【車両による余剰物資B配送処理】 ===
                        // 車両に余剰物資Bの残量があり、ドローンによる配送が未完了の場合のみ実行
                        if (supply_vehicle.remaining_extra_supply_b > 0 && !info_list[i].delivery_completed && info_list[i].extra_supply_demand > info_list[i].extra_supply_delivered)
                        {

                            double delivery_amount = 0.0;

                            if (DELIVERY_METHOD == DELIVERY_METHOD_COORDINATE)
                            {
                                // printf("aaaaaaaaaaaaaa\n");

                                // === 手法２：ドローンの運搬状況を考慮 ===
                                // ドローンによる運搬量（運搬中+運搬予定）を計算
                                double drone_transport_amount = calculate_drone_transport_amount(drones, ND, current_stop_idx, info_list, info_count);

                                // 車両が配送すべき量 = 需要量 - 既配送量 - ドローン運搬量
                                double remaining_demand = info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered - drone_transport_amount;

                                // 車両の配送量は、残り需要量と車両残量の小さい方
                                delivery_amount = (remaining_demand > 0) ? ((remaining_demand > supply_vehicle.remaining_extra_supply_b) ? supply_vehicle.remaining_extra_supply_b : remaining_demand) : 0.0;

                                printf("  配送量計算: 避難所%d, 需要%.0fkg - 既配送%.0fkg - ドローン運搬%.0fkg = 残り需要%.0fkg → 車両配送%.0fkg\n",
                                       info_list[i].shelter_id, info_list[i].extra_supply_demand, info_list[i].extra_supply_delivered, drone_transport_amount, remaining_demand, delivery_amount);

                                // === 配送の実行 ===
                                if (delivery_amount > 0)
                                {
                                    shelter_supplies[current_stop_idx - 1].extra_supply_b += delivery_amount; // 避難所在庫増加
                                    supply_vehicle.remaining_extra_supply_b -= delivery_amount;
                                    info_list[i].extra_supply_delivered += delivery_amount;

                                    // === 車両による余剰物資B運搬統計の更新 ===
                                    total_extra_supply_by_vehicle += delivery_amount;
                                    vehicle_delivery_count++;

                                    if (info_list[i].extra_supply_delivered >= info_list[i].extra_supply_demand)
                                    {
                                        info_list[i].delivery_completed = 1;
                                        printf("  余剰物資B配送: %.0fkg配送（需要完了）\n", delivery_amount);
                                    }
                                    else
                                    {
                                        printf("  余剰物資B配送: %.0fkg配送（残り需要 %.0fkg）\n",
                                               delivery_amount, info_list[i].extra_supply_demand - info_list[i].extra_supply_delivered);
                                    }

                                    break;
                                }
                                else
                                {
                                    printf("  余剰物資B配送: ドローンが運搬中のため車両配送スキップ\n");
                                }
                            }
                        }
                    }
                }
            }

            // === 停止時間中の処理ループ ===
            // 停止期間中も時間を進めてドローンの位置更新と描画を実行
            // 集積所、避難所問わず、車両が停止時間中はドローンの状態更新を行う
            double stop_start_time = elapsed_time;
            double stop_end_time = elapsed_time + T_STOP;

            // 停止時間が終了するまでのループ
            while (elapsed_time < stop_end_time)
            {
                // === 描画タイミングの判定 ===
                // 指定間隔（2分）ごとに描画フレームを生成
                if (elapsed_time >= next_draw_time)
                {
                    plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, shelter_supplies, &supply_vehicle);
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
                        drones[i].last_state_time = elapsed_time; // 飛行時間統計の基準時刻を設定
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
                                drones[i].target_y = stop_coords[0][1]; // 集積所座標
                                update_drone_flight_time(&drones[i], elapsed_time, DRONE_TO_DEPOT);

                                // ドローンによる情報回収処理（検出と同時に回収）
                                for (int j = 0; j < info_count; j++)
                                {
                                    if (!info_list[j].collected &&
                                        info_list[j].shelter_id == detected_shelter &&
                                        info_list[j].generation_time <= elapsed_time)
                                    {
                                        info_list[j].collected = 1;
                                        info_list[j].collection_time = elapsed_time;    // ドローン回収時刻を記録
                                        info_list[j].collected_by = COLLECTED_BY_DRONE; // ドローンによる回収を記録
                                        // 余剰物資B需要に基づいて運搬開始（往復回数は動的に決定）
                                        drones[i].required_trips = 999; // 大きな値に設定（実際は配送完了で終了）
                                        printf("ドローン%d: 避難所%dで情報検出・回収（余剰物資B需要: %.0fkg, 動的運搬開始）\n",
                                               i + 1, detected_shelter, info_list[j].extra_supply_demand);
                                        break; // 1つの情報のみ回収
                                    }
                                }

                                printf("ドローン%d: 集積所へ向かいます\n", i + 1);
                            }
                        }

                        // ドローンの状態更新
                        update_drone_state(&drones[i], stop_coords, elapsed_time, time_step, info_list, info_count, shelter_supplies, &total_extra_supply_by_drone, &drone_delivery_count);
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
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, shelter_supplies, &supply_vehicle);
            next_draw_time = DRAW_INTERVAL; // 最初の描画後、次の描画時刻を設定
            is_first_departure = 0;         // 初回出発フラグをリセット
        }

        // === 【次の目的地の決定】 ===
        // 車両の停止処理が完了した後、次の目的地を決定
        // 円周上の拠点を時計回りに順次訪問
        // インデックス0=集積所, 1〜NS=避難所1〜10
        int next_stop_idx = (current_stop_idx + 1) % TOTAL_STOPS;

        // === 【周回完了判定】 ===
        // 次の目的地が集積所（インデックス0）になった時点で1周完了
        // この時点でlap_countをインクリメントし、周回進捗を記録
        if (next_stop_idx == 0)
        {
            lap_count++; // 完了周回数を増加
            printf("--- %d周目完了 ---\n", lap_count);
        }

        // 移動開始のログ出力
        printf("移動開始: 拠点%d → 拠点%d\n", current_stop_idx, next_stop_idx);

        // === 【移動中の時間管理】 ===
        // 車両の1区間移動時間は固定値（segment_distance / V）
        double move_start_time = elapsed_time;                         // 移動開始時刻
        double move_end_time = elapsed_time + travel_time_per_segment; // 移動完了予定時刻

        // === 【車両の移動角度計算】 ===
        // 円周上の時計回り移動における角度変化を計算
        // 集積所=12時位置（π/2ラジアン）を基準とした角度座標系
        double start_angle = PI / 2.0 - current_stop_idx * angle_increment; // 出発地点の角度
        double end_angle = PI / 2.0 - next_stop_idx * angle_increment;      // 到着地点の角度

        // === 【角度差の正規化】（最短経路選択） ===
        // 時計回り移動のため、角度差を適切に計算
        // 2πを跨ぐ場合の処理も含む（例: 1時→11時への移動）
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
                plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, shelter_supplies, &supply_vehicle);
                next_draw_time += DRAW_INTERVAL; // 次の描画時刻を更新
            }

            // === 【時間ステップ進行】 ===
            // 1秒刻みできめ細かな状態更新を実行
            // 細かい時間刻みにより、ドローンの位置更新・情報検出・描画タイミングを正確に制御
            double time_step = 1.0; // 1秒ステップ（バランスの取れた精度と計算速度）
            elapsed_time += time_step;

            // === 【ドローン群の状態更新】（移動中も継続的に処理） ===
            // 車両の移動中も、ドローンは独立して動作を継続
            // 複数ドローンの並列処理により、効率的な情報回収・物資運搬を実現
            for (int i = 0; i < ND; i++)
            {
                // === ドローン出発判定 ===
                // 事前に設定された出発時刻に達したドローンをアクティブ化
                // 複数ドローンを時間差で出発させることで、円周上に均等分散配置
                if (!drones[i].active && elapsed_time >= drones[i].start_time)
                {
                    drones[i].active = 1;                     // アクティブ状態に変更
                    drones[i].last_state_time = elapsed_time; // 飛行時間統計の基準時刻を設定
                    printf("ドローン%d: 出発（時刻%.1f秒）\n", i + 1, elapsed_time);
                }

                // === アクティブドローンの処理 ===
                if (drones[i].active)
                {
                    // === 情報検出処理（通常巡回モードのみ） ===
                    // ドローンが通常巡回中に避難所近傍で情報を検出した場合の処理
                    // 物資運搬中や集積所移動中は検出を行わない（集中処理のため）
                    if (drones[i].state == DRONE_PATROL)
                    {
                        // 200m以内の避難所で未回収情報があるかチェック
                        int detected_shelter = check_drone_info_detection(&drones[i], stop_coords, info_list, info_count, elapsed_time);

                        if (detected_shelter > 0)
                        {
                            // === 物資運搬モードへの移行 ===
                            // 情報検出と同時に、ドローンを物資運搬モードに切り替え
                            drones[i].target_shelter = detected_shelter;                        // 配送先避難所を設定
                            drones[i].current_trip = 1;                                         // 往復回数カウンタを初期化
                            drones[i].target_x = stop_coords[0][0];                             // 集積所X座標を目標に設定
                            drones[i].target_y = stop_coords[0][1];                             // 集積所Y座標を目標に設定
                            update_drone_flight_time(&drones[i], elapsed_time, DRONE_TO_DEPOT); // 状態変更と飛行時間記録

                            // === ドローンによる情報回収処理（検出と同時に回収） ===
                            // 車両の巡回を待たずに、ドローンが即座に情報回収を実行
                            // これにより回収時間（Tc）の短縮が期待される
                            for (int j = 0; j < info_count; j++)
                            {
                                // 対象条件: 未回収 & 検出避難所 & 既発生済み
                                if (!info_list[j].collected &&
                                    info_list[j].shelter_id == detected_shelter &&
                                    info_list[j].generation_time <= elapsed_time)
                                {
                                    info_list[j].collected = 1;                     // 回収済みフラグを設定
                                    info_list[j].collection_time = elapsed_time;    // ドローン回収時刻を記録
                                    info_list[j].collected_by = COLLECTED_BY_DRONE; // ドローンによる回収を記録

                                    // === 動的運搬開始 ===
                                    // 余剰物資B需要量に応じて動的に運搬（往復回数は配送完了まで継続）
                                    drones[i].required_trips = 999; // 大きな値に設定（実際は配送完了で終了）

                                    printf("ドローン%d: 避難所%dで情報検出・回収（余剰物資B需要: %.0fkg, 動的運搬開始）\n",
                                           i + 1, detected_shelter, info_list[j].extra_supply_demand);
                                    break; // 1回の検出で1つの情報のみ処理
                                }
                            }

                            printf("ドローン%d: 集積所へ向かいます\n", i + 1);
                        }
                    }

                    // === ドローンの状態更新 ===
                    // 現在の状態（巡回・移動・停止）に応じた位置更新・状態遷移を実行
                    update_drone_state(&drones[i], stop_coords, elapsed_time, time_step, info_list, info_count, shelter_supplies, &total_extra_supply_by_drone, &drone_delivery_count);
                }
            }
        }

        // === 移動完了時の正確な位置調整 ===
        elapsed_time = move_end_time;

        // === 次の停止地点への移動 ===
        current_stop_idx = next_stop_idx;
    }

    // === 【シミュレーション終了処理】 ===
    printf("\n=== シミュレーション終了 ===\n");
    printf("完了周回数: %d周\n", lap_count);
    printf("総経過時間: %.0f秒 (%.1f時間)\n", elapsed_time, elapsed_time / 3600.0);

    // === 【車両移動距離の計算と表示】 ===
    // 物理的な移動実績を評価指標として算出
    double total_distance = lap_count * 2.0 * PI * R; // 総移動距離 = 周回数 × 円周（2πR）
    printf("車両総移動距離: %.1f m (%.2f km)\n", total_distance, total_distance / 1000.0);
    printf("車両平均速度: %.2f m/s (%.1f km/h)\n", V, V * 3.6); // m/s → km/h変換

    // === 【情報発生・回収の統計情報】 ===
    printf("\n=== 情報回収システム統計 ===\n");
    printf("情報発生率: %.2f [/hour]\n", LAMBDA); // 発生率
    printf("総情報発生数: %d件\n", info_count);

    // === 全情報の回収状況を再集計（車両・ドローン混合） ===
    // info_listを全走査して最終的な回収状況を正確に把握
    int total_collected_count = 0;
    for (int i = 0; i < info_count; i++)
    {
        if (info_list[i].collected)
        {
            total_collected_count++;
        }
    }

    // 回収実績の詳細表示
    // collected_count: 車両による回収数（Tc統計記録済み）
    // total_collected_count - collected_count: ドローンによる回収数
    printf("回収済み情報数: %d件（車両: %d件, ドローン: %d件）\n",
           total_collected_count, collected_count, total_collected_count - collected_count);
    printf("未回収情報数: %d件\n", info_count - total_collected_count);

    // === 【Tc（回収遅延時間）の統計解析】 ===
    // 災害情報システムの重要評価指標
    // Tc = 情報発生時刻から回収時刻までの遅延時間

    // collected_byフィールドに基づく正確な分類統計
    double vehicle_tc_total = 0.0;  // 車両回収分のTc総計
    double drone_tc_total = 0.0;    // ドローン回収分のTc総計
    double all_tc_total = 0.0;      // 全体のTc総計
    int vehicle_collected_calc = 0; // 車両回収数
    int drone_collected_calc = 0;   // ドローン回収数

    // 全回収済み情報を回収主体別に分類して統計計算
    for (int i = 0; i < info_count; i++)
    {
        if (info_list[i].collected && info_list[i].collection_time >= 0)
        {
            double tc = info_list[i].collection_time - info_list[i].generation_time;
            all_tc_total += tc;

            // 回収主体に基づく分類
            if (info_list[i].collected_by == COLLECTED_BY_VEHICLE)
            {
                vehicle_tc_total += tc;
                vehicle_collected_calc++;
            }
            else if (info_list[i].collected_by == COLLECTED_BY_DRONE)
            {
                drone_tc_total += tc;
                drone_collected_calc++;
            }
        }
    }

    if (total_collected_count > 0)
    {
        printf("=== Tc（回収遅延時間）統計 ===\n");

        // 車両回収分の統計
        if (vehicle_collected_calc > 0)
        {
            double vehicle_avg_tc = vehicle_tc_total / vehicle_collected_calc;
            printf("車両回収分 - 回収数: %d件, Tc平均: %.2f秒 (%.2f分)\n",
                   vehicle_collected_calc, vehicle_avg_tc, vehicle_avg_tc / 60.0);
        }
        else
        {
            printf("車両回収分 - 回収数: 0件\n");
        }

        // ドローン回収分の統計
        if (drone_collected_calc > 0)
        {
            double drone_avg_tc = drone_tc_total / drone_collected_calc;
            printf("ドローン回収分 - 回収数: %d件, Tc平均: %.2f秒 (%.2f分)\n",
                   drone_collected_calc, drone_avg_tc, drone_avg_tc / 60.0);
        }
        else
        {
            printf("ドローン回収分 - 回収数: 0件\n");
        }

        // 全体統計（車両＋ドローン）
        double all_avg_tc = all_tc_total / total_collected_count;
        printf("全体統計 - 回収数: %d件, Tc平均: %.2f秒 (%.2f分), Tc総計: %.2f秒 (%.2f時間)\n",
               total_collected_count, all_avg_tc, all_avg_tc / 60.0, all_tc_total, all_tc_total / 3600.0);
    }
    else
    {
        printf("Tc統計: 計算不可（回収済み情報なし）\n");
    }

    // === 【ドローン飛行時間統計（Pf: 物資運搬割合）】 ===
    if (ND > 0)
    {
        printf("\n=== ドローン飛行時間統計 ===\n");

        double total_flight_time_all = 0.0;       // 全ドローンの総飛行時間
        double total_supply_transport_time = 0.0; // 全ドローンの物資運搬時間
        int active_drones = 0;                    // 活動したドローン数

        for (int i = 0; i < ND; i++)
        {
            if (drones[i].active)
            {
                // シミュレーション終了時点での最終状態時間を更新
                update_drone_flight_time(&drones[i], elapsed_time, drones[i].state);

                total_flight_time_all += drones[i].total_flight_time;
                total_supply_transport_time += drones[i].supply_transport_time;
                active_drones++;

                // 個別ドローンの統計
                double pf_individual = (drones[i].total_flight_time > 0) ? (drones[i].supply_transport_time / drones[i].total_flight_time) * 100.0 : 0.0;

                printf("ドローン%d: 総飛行時間=%.1f秒 (%.1f分), 物資運搬時間=%.1f秒 (%.1f分), Pf=%.1f%%\n",
                       i + 1, drones[i].total_flight_time, drones[i].total_flight_time / 60.0,
                       drones[i].supply_transport_time, drones[i].supply_transport_time / 60.0,
                       pf_individual);
            }
        }

        // 全ドローン平均のPf計算
        if (active_drones > 0 && total_flight_time_all > 0)
        {
            double avg_flight_time = total_flight_time_all / active_drones;
            double avg_supply_transport_time = total_supply_transport_time / active_drones;
            double avg_pf = (avg_supply_transport_time / avg_flight_time) * 100.0;

            printf("全体平均: 飛行時間=%.1f秒 (%.1f分), 物資運搬時間=%.1f秒 (%.1f分), Pf=%.1f%%\n",
                   avg_flight_time, avg_flight_time / 60.0,
                   avg_supply_transport_time, avg_supply_transport_time / 60.0,
                   avg_pf);
        }
        else
        {
            printf("Pf統計: 計算不可（アクティブドローンなし）\n");
        }
    }

    // === 【余剰物資B運搬統計】 ===
    printf("\n=== 余剰物資B運搬統計 ===\n");
    double total_extra_supply = total_extra_supply_by_vehicle + total_extra_supply_by_drone;
    int total_delivery_count = vehicle_delivery_count + drone_delivery_count;

    printf("*車両による運搬: %.1fkg（配送回数: %d回）\n", total_extra_supply_by_vehicle, vehicle_delivery_count);
    printf("*ドローンによる運搬: %.1fkg（配送回数: %d回 (往復含む), %d回（往復含まない））\n", total_extra_supply_by_drone, drone_delivery_count, total_collected_count - collected_count);
    printf("*運搬総量: %.1fkg（総配送回数: %d回）\n", total_extra_supply, total_delivery_count);

    if (total_delivery_count > 0)
    {
        double avg_delivery_per_trip = total_extra_supply / total_delivery_count;
        printf("1回あたり平均配送量(車両＋ドローン): %.1fkg\n", avg_delivery_per_trip);

        if (vehicle_delivery_count > 0)
        {
            double avg_vehicle_delivery = total_extra_supply_by_vehicle / vehicle_delivery_count;
            printf("車両の1回あたり平均配送量: %.1fkg\n", avg_vehicle_delivery);
        }

        if (drone_delivery_count > 0)
        {
            double avg_drone_delivery = total_extra_supply_by_drone / drone_delivery_count;
            printf("ドローンの1回あたり平均配送量: %.1fkg\n", avg_drone_delivery);
        }

        // 運搬主体の割合表示（運搬総量のうち何kgを車両・ドローンが運搬したか）
        double vehicle_ratio = (total_extra_supply > 0) ? (total_extra_supply_by_vehicle / total_extra_supply) * 100.0 : 0.0;
        double drone_ratio = (total_extra_supply > 0) ? (total_extra_supply_by_drone / total_extra_supply) * 100.0 : 0.0;
        printf("運搬割合: 車両 %.1f%%, ドローン %.1f%%\n", vehicle_ratio, drone_ratio);

        // 一周あたりの車両の余剰物資配送量
        double vehicle_extra_supply_per_lap = total_extra_supply_by_vehicle / lap_count;
        printf("*車両の1周あたり余剰物資B配送量: %.1fkg\n", vehicle_extra_supply_per_lap);
    }
    else
    {
        printf("余剰物資Bの配送は発生しませんでした\n");
    }

    // === 【GIFファイル処理・クリーンアップ】 ===
    if (ENABLE_GIF && gnuplot_pipe != NULL)
    {
        printf("\n=== GIF出力処理 ===\n");
        printf("'simulation.gif' がカレントディレクトリに生成されました。\n");

        // === GIFファイルの適切な終了処理 ===
        // gnuplotに終了コマンドを送信してファイルを正常に閉じる
        // この処理を省略するとGIFファイルが破損する可能性がある
        fprintf(gnuplot_pipe, "unset output\n"); // 出力ファイルの明示的クローズ
        fflush(gnuplot_pipe);                    // バッファの強制フラッシュ

        // gnuplotプロセスの終了とパイプクローズ
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
