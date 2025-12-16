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
#define T_STOP (30 * 60)           // 各避難所での停止時間 (s) | 30分=1800秒
// #define T_STOP_DIS (30 * 60) / NDI // 各集積所での停止時間 (s) | 30分=1800秒/NDI
#define T_STOP_DIS (30 * 60)
#define DETECTION_RADIUS 10.0 // ドローンの検出半径 (m) | 避難所近傍での情報検出・協調運搬判定用

// === シミュレーション設定 ===
#define NS 12              // 避難所の数（集積所除く）
#define NDI 1              // 集積所の数(NV：物資運搬車両の台数と同じにする)
#define NV 1               // 車両の一台
#define NT 3               // シミュレーションの周回数
#define ND 3               // ドローンの台数（0の場合はドローンなし、最大制限なし）
#define ENABLE_GIF 1       // GIF出力の有効/無効 (1:有効, 0:無効) | 処理軽量化用
#define CLEAN_PNG_OUTPUT 1 // PNG出力時の軸ラベル・枠線削除 (1:削除, 0:通常表示)

// === ドローン巡回方向設定 ===
#define DRONE_CLOCKWISE 0                                                    // ドローン巡回方向: 0=反時計回り, 1=時計回り
#define DRONE_DIRECTION_NAME ((DRONE_CLOCKWISE) ? "時計回り" : "反時計回り") // 表示用文字列

// === 情報発生システム（ポアソン過程） ===
#define LAMBDA 1.0    // ポアソン到着率 [件/時間] | 1時間に平均0.5件の情報発生
#define MAX_INFO 5000 // 最大情報数（メモリ制限対策）

// === ドローン物資運搬システム ===
#define NR 2                      // ドローンの往復回数（基本値、情報により動的変更）
#define T_DRONE_STOP (10 * 60)    // ドローンの停止時間 (s) | 10分=600秒（集積所・避難所共通）
#define DRONE_MAX_CARRY 30.0      // ドローンの最大積載量 (kg)
#define FEW_SUPPLY_THRESHOLD 10.0 // 少量物資運搬の閾値 (kg)
#define MIN_EXTRA_DEMAND 30.0     // 余剰物資A需要量の最小値 (kg)
#define MAX_EXTRA_DEMAND 30.0     // 余剰物資A需要量の最大値 (kg)
#define THRESHOLD 0.70            // しきい値配送の閾値 (%)、需要量に対するドローン配送の保証割合

// === 物資運搬車両システム ===
#define TOTAL_SUPPLY_WEIGHT NS * 1000.0                              // 物資運搬車両の総積載量 (kg)
#define SUPPLY_GA_RATIO 0.9                                          // 通常物資G（A）の割合（0.0〜1.0）
#define SUPPLY_GB_RATIO 0.1                                          // 通常物資G（B）の割合（0.0〜1.0）
#define EXTRA_SUPPLY_B 99999.0                                       // 余剰分の物資B量 (kg): 物資運搬車両は余剰物資Bを無制限に積載している想定
#define SUPPLY_PER_SHELTER (TOTAL_SUPPLY_WEIGHT / NS)                // 1避難所あたりの物資量 (kg)
#define SUPPLY_GA_PER_SHELTER (SUPPLY_PER_SHELTER * SUPPLY_GA_RATIO) // 1避難所あたりの物資A量 (kg)
#define SUPPLY_GB_PER_SHELTER (SUPPLY_PER_SHELTER * SUPPLY_GB_RATIO) // 1避難所あたりの物資B量 (kg)

// === 余剰物資A配送手法の選択 ===
#define DELIVERY_METHOD_COORDINATE 1
#define DELIVERY_METHOD_MULTI_DRONE 2
#define DELIVERY_COORDINATATE_FLAG 0
#define DRONE_FLY_TO_NEAREST_DEPOT 1 // ドローンが最寄りの集積所に向かうかのフラグ（1:向かう, 0:向かわない）
#define DELIVERY_METHOD DELIVERY_METHOD_MULTI_DRONE
#define DRONE_TO_DEPOT_OPTION 2 //(1:最寄りの集積所に向かう, 2:次の物資運搬車両の集積所に向かう)

// === 派生的な定数（自動計算） ===
#define TOTAL_STOPS (NS + NDI) // 集積所を含めた拠点の総数（11拠点）
#define DRAW_INTERVAL 120.0    // 描画間隔（秒）| 2分ごとにGIFフレーム生成

// === 回収主体の定数定義 ===
#define COLLECTED_BY_NONE 0    // 未回収
#define COLLECTED_BY_VEHICLE 1 // 車両による回収
#define COLLECTED_BY_DRONE 2   // ドローンによる回収

// === GIFの出力時間範囲 ===
#define GIF_TIME_RANGE_FLAG 0         // Gif画層に時間指定を設けるかを示すフラグ（1：有効, 0：無効）
#define GIF_START_TIME_SEC (0 * 3600) // 40時間指定
#define GIF_END_TIME_SEC (45 * 3600)  // 45時間指定

// === 情報管理用構造体 ===
typedef struct
{
    int shelter_id;                          // 情報が発生した避難所ID (1〜NS+NDI-1) | dis_idx(0など)は集積所なので除外
    double generation_time;                  // 情報発生時刻（秒）| シミュレーション開始からの経過時間
    double collection_time;                  // 情報回収時刻（秒）| -1の場合は未回収状態
    double TV_collection_time;               // 物資運搬車両による情報回収時刻（秒）| -1の場合は未回収状態
    int collected;                           // 回収済みフラグ（0:未回収, 1:回収済み）
    int collected_by;                        // 回収主体（0:未回収, 1:車両, 2:ドローン）
    int collected_by_drone_later;            // 車両が発見し回収したが後にドローンがあとからやってきて情報を回収したフラグ（1:ドローンが後に発見）：ETc_dro算出用
    double drone_collection_time_later;      // ドローンが運搬車両のあとに情報を回収した時刻（秒）| -1の場合は未回収状態
    double A_extra_supply_demand;            // 余剰物資A需要量 (kg) | 0〜90kgの範囲
    double B_extra_supply_demand;            // 余剰物資B需要量 (kg) | 0〜90kgの範囲(Low priority：緊急度低い物資)
    double A_extra_supply_delivered;         // 配送済み余剰物資A量 (kg)
    double B_extra_supply_delivered;         // 配送済み余剰物資B量 (kg)
    int A_extra_delivery_completed;          // 余剰物資A配送完了フラグ（0:未完了, 1:完了）
    int B_extra_delivery_completed;          // 余剰物資B配送完了フラグ（0:未完了, 1:完了）
    double A_extra_delivery_completion_time; // 余剰物資A配送完了時刻（秒）| -1の場合は未完了
    double B_extra_delivery_completion_time; // 余剰物資Bの配送完了時刻（秒）| -1の場合は未完了
    int fast_detection_flag;                 // 迅速発見フラグ（1:車両が集積所で先に情報を発見, 0:遅れて発見）
    int responsible_TV_id;                   // （複数の運搬車両を想定した場合,初めて要求情報を回収したTVが運搬担当）
    int TV_inf_collect_at_depot;             // 物資運搬車両が集積所で情報を回収したフラグ（1:回収済み, 0:未回収）
} Info;

// === 避難所物資管理用構造体 ===
typedef struct
{
    double supply_ga;              // 物資Aの在庫量 (kg)
    double supply_gb;              // 物資Bの在庫量 (kg)
    double extra_supply_a;         // 余剰物資Aの在庫量 (kg)
    double extra_supply_b;         // 余剰物資Bの在庫量 (kg)
    int collect_info_id[MAX_INFO]; // 情報を回収した避難所のidに1をつけるリスト
    double tg_total_time;          // 物資G（通常物資）の運搬間隔時間合計
    double tg_count;               // 物資G（通常物資）の運搬回数
    double tg_last_time;           // 物資G（通常物資）の最後の運搬時間
} Facility;

// === 物資運搬車両管理用構造体 ===
/**
 * @brief 物資運搬車両の状態と積載物資を管理する構造体
 */
typedef struct // 物資運搬車両管理構造体
{
    double remaining_supply_ga;              // 積載中の物資G(A)残量 (kg)
    double remaining_supply_gb;              // 積載中の物資G(B)残量 (kg)
    double remaining_extra_supply_b;         // 積載中の余剰分物資B残量 (kg)
    int is_loaded;                           // 積載状態（0:空, 1:積載中）
    int collect_info_id[MAX_INFO];           // 情報を回収した避難所のidに1をつけるリスト
    int demand_supply_loaded_flag[MAX_INFO]; // 要求された物資を把握し、集積所で積載したかを示すフラグ(0:未積載, 1:積載済み)
    int next_depot_id;                       // 次に向かう集積所の番号（0,3,6など）
    int next_depot_idx;                      // 次に向かう集積所のインデックス（0〜NDI-1）:dis_idx[]のインデックス
    double last_depot_arrival_time;          // 最後に集積所に到着した時刻（秒）
} SupplyVehicle;

// === ドローン状態管理用構造体 ===
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
    DroneState state;               // 現在の動作状態
    int target_shelter;             // 目標避難所ID（物資運搬時）
    int target_depot;               // 目標集積所ID（物資運搬時）
    int current_trip;               // 現在の往復回数（1〜NR）
    int required_trips;             // 必要な往復回数（余剰物資A需要に基づく）
    double state_start_time;        // 現在状態の開始時刻
    double target_x, target_y;      // 目標座標（直線飛行時）
    double carrying_extra_supply;   // 現在積載中の余剰物資A量 (kg)
    int delivery_info_index;        // ドローンが物資運搬を行う情報のインデックス
    double supply_scheduled_amount; // ドローンが避難所に届ける予定の物資量 (kg)

    int collect_info_id[MAX_INFO]; // 情報を回収した避難所のidに1をつけるリスト

    // 飛行時間統計用フィールド
    double total_flight_time;         // 総飛行時間（秒）- 状態間遷移による累積
    double timestep_flight_time;      // time_stepによる累積飛行時間（秒）
    double supply_transport_time;     // 物資運搬飛行時間（秒）
    double few_supply_transport_time; // 少ない物資運搬飛行時間（秒）
    double last_state_time;           // 前回の状態変更時刻（秒）
} DroneInfo;                          // ドローン情報管理構造体終了

// === 関数プロトタイプ宣言 ===
/*
double generate_extra_supply_demand();
int calculate_required_trips(double demand);
void update_drone_state(DroneInfo *drone, DroneInfo drones[], int drone_count, double stop_coords[][2], double elapsed_time, double time_step, Info *info_list, int info_count, Facility *facilities, double *total_extra_supply_by_drone, int *drone_delivery_count, int drone_index, int dis_idx[NDI], SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], double vehicle_x[], double vehicle_y[]);
void update_drone_flight_time(DroneInfo *drone, double current_time, DroneState new_state);
double calculate_all_drones_transport_amount(DroneInfo *drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);
double get_other_drones_carrying_sum(DroneInfo *current_drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);
int should_drone_join_transport(DroneInfo *drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);
int check_drone_info_detection(DroneInfo *drone, double stop_coords[][2], Info *info_list, int info_count, double elapsed_time, int dis_idx[NDI]);
int check_drone_cooperative_transport(DroneInfo *drone, DroneInfo drones[], int drone_count, double stop_coords[][2], Info *info_list, int info_count, int dis_idx[NDI]);
int check_drone_depot_detection(DroneInfo *drone, double stop_coords[][2], int dis_idx[NDI]);
int find_nearest_depot(DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count);
int find_vehicle_next_depot(SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count, double vehicle_x[], double vehicle_y[], double ve_stop_end_time, int ve_stop_flag, int is_depot, double elapsed_time);
int find_vehicle_next_depot_V_1(SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count, double vehicle_x[], double vehicle_y[], double ve_stop_end_time, int ve_stop_flag, int is_depot, double elapsed_time);
void save_simulation_model_png(double stop_coords[][2], int dis_idx[NDI]);
*/
double generate_extra_supply_demand();
int calculate_required_trips(double demand);
FILE *init_gnuplot();
double generate_exponential_interval(double lambda);
int generate_random_shelter(int dis_idx[NDI]);
void plot_frame(FILE *pipe, double stop_coords[][2], double current_x[], double current_y[], DroneInfo drones[], double elapsed_time, Info *info_list, int info_count, Facility facilities[], SupplyVehicle supply_vehicle[], int dis_idx[NDI]);
void update_drone_state(DroneInfo *drone, DroneInfo drones[], int drone_count, double stop_coords[][2], double elapsed_time, double time_step, Info *info_list, int info_count, Facility *facilities, double *total_extra_supply_by_drone, int *drone_delivery_count, int drone_index, int dis_idx[NDI], SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], double vehicle_x[], double vehicle_y[]);
void update_drone_flight_time(DroneInfo *drone, double current_time, DroneState new_state);
double calculate_all_drones_transport_amount(DroneInfo *drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);
double get_other_drones_carrying_sum(DroneInfo *current_drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);
int should_drone_join_transport(DroneInfo *drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count);
int check_drone_info_detection(DroneInfo *drone, double stop_coords[][2], Info *info_list, int info_count, double elapsed_time, int dis_idx[NDI]);
int check_drone_cooperative_transport(DroneInfo *drone, DroneInfo drones[], int drone_count, double stop_coords[][2], Info *info_list, int info_count, int dis_idx[NDI]);
int check_drone_depot_detection(DroneInfo *drone, double stop_coords[][2], int dis_idx[NDI]);
int find_nearest_depot(DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count);
int find_vehicle_next_depot(SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count, double vehicle_x[], double vehicle_y[], double ve_stop_end_time, int ve_stop_flag, int is_depot, double elapsed_time);
int find_vehicle_next_depot_V_1(SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count, double vehicle_x[], double vehicle_y[], double ve_stop_end_time, int ve_stop_flag, int is_depot, double elapsed_time);
void save_simulation_model_png(double stop_coords[][2], int dis_idx[NDI]);

/**
 * @brief gnuplotを初期化し、GIFアニメーション出力のための設定を行う
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

    // === WSL環境での文字化け対策設定 ===
    fprintf(pipe, "set encoding utf8\n");
    fprintf(pipe, "set locale 'C'\n");

    fprintf(pipe, "set terminal gif animate delay 20 size 600,600 crop enhanced font 'DejaVu Sans,12'\n");
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
 */
double generate_exponential_interval(double lambda)
{
    double u = (double)rand() / RAND_MAX;
    return -log(u) / (lambda / 3600.0); // 結果は秒単位
}

/**
 * @brief ランダムな避難所IDを生成（情報発生場所の決定用）
 */
int generate_random_shelter(int dis_idx[NDI])
{
    int candidates[NS]; // 候補となる避難所IDの配列
    int candidate_count = 0;

    // 1からNS+NDI-1の範囲で、dis_idxに含まれない値を候補に追加
    for (int i = 1; i <= NS + NDI - 1; i++)
    {
        int is_depot = 0;
        // dis_idxに含まれているかチェック
        for (int j = 0; j < NDI; j++)
        {
            if (i == dis_idx[j])
            {
                is_depot = 1;
                break;
            }
        }
        // 集積所でない場合は候補に追加
        if (!is_depot)
        {
            candidates[candidate_count] = i;
            candidate_count++;
        }
    }

    // 候補からランダムに選択
    if (candidate_count > 0)
    {
        int random_index = rand() % candidate_count;
        return candidates[random_index];
    }

    return 1;
}

/**
 * @brief ドローンが情報発生避難所を検出するかチェック
 */
int check_drone_info_detection(DroneInfo *drone, double stop_coords[][2], Info *info_list, int info_count, double elapsed_time, int dis_idx[NDI])
{
    // 通常巡回モード以外では検出しない
    if (drone->state != DRONE_PATROL)
        return 0;

    // 各避難所との距離をチェック
    for (int shelter_id = 1; shelter_id <= NS + NDI - 1; shelter_id++)
    {
        // 集積所かどうかをチェック
        int is_depot = 0;
        for (int d = 0; d < NDI; d++)
        {
            if (shelter_id == dis_idx[d])
            {
                is_depot = 1;
                break; // 集積所と判明したら上のfor文ループを抜ける
            }
        }
        if (is_depot)
        {
            continue; // 集積所の場合、次のshelter_idに進む
        }

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
                    drone->delivery_info_index = i; // 物資運搬情報のインデックスを設定
                    return shelter_id;              // 情報発生避難所を検出
                }
            }
        }
    }

    return 0; // 検出なし
}

/**
 * @brief ドローンのDETECTION_RADIUS内に集積所があるかチェック
 */
int check_drone_depot_detection(DroneInfo *drone, double stop_coords[][2], int dis_idx[NDI])
{
    // 各集積所との距離をチェック
    for (int depot_idx = 0; depot_idx < NDI; depot_idx++)
    {
        int depot_id = dis_idx[depot_idx];
        double dx = drone->x - stop_coords[depot_id][0];
        double dy = drone->y - stop_coords[depot_id][1];
        double distance = sqrt(dx * dx + dy * dy);

        // 集積所近傍にいるかチェック
        if (distance <= DETECTION_RADIUS)
        {
            return depot_id; // dis_idx[depot_idx]集積所番号を返す
        }
    }

    return -1; // 検出なし
}

/**
 * @brief ドローンの協調運搬判定処理
 */
int check_drone_cooperative_transport(DroneInfo *drone, DroneInfo drones[], int drone_count, double stop_coords[][2], Info *info_list, int info_count, int dis_idx[NDI])
{
    // 手法3以外では協調運搬判定を行わない
    if (DELIVERY_METHOD != DELIVERY_METHOD_MULTI_DRONE)
        return 0;

    // 通常巡回モード以外では判定しない
    if (drone->state != DRONE_PATROL)
        return 0;

    // 各避難所との距離をチェック
    for (int shelter_id = 1; shelter_id <= NS + NDI - 1; shelter_id++)
    {
        // 集積所かどうかをチェック
        int is_depot = 0;
        for (int d = 0; d < NDI; d++)
        {
            if (shelter_id == dis_idx[d])
            {
                is_depot = 1;
                break; // 集積所と判明したらループを抜ける
            }
        }
        if (is_depot)
        {
            continue; // 集積所の場合、次のshelter_idに進む
        }

        double dx = drone->x - stop_coords[shelter_id][0];
        double dy = drone->y - stop_coords[shelter_id][1];
        double distance = sqrt(dx * dx + dy * dy);

        // 避難所近傍にいるかチェック
        if (distance <= DETECTION_RADIUS)
        {
            // 協調運搬の条件をチェック
            if (should_drone_join_transport(drone, drones, drone_count, shelter_id, info_list, info_count))
            {
                return shelter_id; // 協調運搬すべき避難所を検出
            }
        }
    }

    return 0; // 協調運搬不要
}

/**
 * @brief ドローンから最も近い集積所のインデックスを取得
 */
int find_nearest_depot(DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count)
{
    // NDI=1の場合,またはDRONE_FLY_TO_NEAREST_DEPOTが0のときは常に0を返す
    if (ndi_count <= 1 || DRONE_FLY_TO_NEAREST_DEPOT == 0)
    {
        return 0;
    }

    int nearest_depot_id = dis_idx[0]; // 最初の集積所IDで初期化
    double min_distance = -1.0;

    // 各集積所との距離を計算
    for (int depot = 0; depot < ndi_count; depot++)
    {
        int depot_id = dis_idx[depot];
        double dx = drones[i].x - stop_coords[depot_id][0];
        double dy = drones[i].y - stop_coords[depot_id][1];
        double distance = sqrt(dx * dx + dy * dy);

        // 最初の集積所、またはより近い集積所が見つかった場合
        if (min_distance < 0.0 || distance < min_distance)
        {
            min_distance = distance;
            nearest_depot_id = depot_id; // インデックスではなく実際の集積所IDを保存
        }
    }

    return nearest_depot_id; // 実際の集積所IDを返す
}

/**
 * @brief 運搬車両の次の集積所を決定する関数
 */
int find_vehicle_next_depot_V_1(SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count, double vehicle_x[], double vehicle_y[], double ve_stop_end_time, int ve_stop_flag, int is_depot, double elapsed_time)
{
    // 集積所が1つ以下の場合は処理不要
    if (ndi_count <= 1)
    {
        return dis_idx[0]; // 最初の集積所を返す
    }

    int ve_next_depot_id;
    // == ドローンが物資運搬車両が次に向かう集積所へ飛行するのに要する時間を算出
    if (is_depot == 1 && ve_stop_flag == 1 && ve_stop_end_time > elapsed_time) // 車両が集積所で停止中の場合
    {
        ve_next_depot_id = current_stop_idx[0];
    }
    else
    {
        ve_next_depot_id = supply_vehicle[0].next_depot_id; // それ以外の場合
    }

    // ドローンの現在座標と次の集積所との距離を計算
    double dx = drones[i].x - stop_coords[ve_next_depot_id][0];
    double dy = drones[i].y - stop_coords[ve_next_depot_id][1];
    double drone_distance = sqrt(dx * dx + dy * dy);
    // 距離を速度で割ってt1（時間）を計算
    double t1 = drone_distance / V_DRONE; // ドローンの速度で計算

    // == 物資運搬車両が次の集積所へ到達するまでの時間を算出（円周上移動）
    int ve_next_is_depot_flag = 0; // 次の目的地が集積所かどうかのフラグ
    double t2 = 999999.0;          // 車両の移動時間

    // 運搬車両の次の次の集積所IDを計算
    int next_next_depot_idx = -1;
    if (supply_vehicle[0].next_depot_idx + 1 >= NDI)
    {
        next_next_depot_idx = 0;
    }
    else
    {
        next_next_depot_idx = supply_vehicle[0].next_depot_idx + 1;
    }
    int ve_next_next_depot_id = dis_idx[next_next_depot_idx];

    // 車両が集積所で停止中の場合
    if (is_depot == 1 && ve_stop_flag == 1 && ve_stop_end_time > elapsed_time)
    {
        t2 = ve_stop_end_time - elapsed_time;
    }

    // ドローンと物資運搬車両の時間を比較
    // 集積所に飛行する時間 <= 車両の残りの停止時間
    if (t1 <= t2 && is_depot == 1)
    {
        return current_stop_idx[0];
    }
    else
    {
        return supply_vehicle[0].next_depot_id;
    }
}

/**
 * @brief 運搬車両の次の集積所を決定する関数
 */
int find_vehicle_next_depot(SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], DroneInfo drones[], int i, double stop_coords[][2], int dis_idx[NDI], int ndi_count, double vehicle_x[], double vehicle_y[], double ve_stop_end_time, int ve_stop_flag, int is_depot, double elapsed_time)
{
    // 集積所が1つ以下の場合は処理不要
    if (ndi_count <= 1)
    {
        return dis_idx[0]; // 最初の集積所を返す
    }

    // ========= すべての集積所とドローンの飛行元の避難所の角度を計算し、最も近い集積所のIDを取得（集積所から避難所へ時計回り）
    double min_angle = 2 * PI;           // 最小角度（初期値は360度）
    int min_angle_depot_id = dis_idx[0]; // 最小角度の集積所ID（初期値は最初の集積所）
    // 各集積所から避難所への時計回り角度を計算
    for (int j = 0; j < ndi_count; j++)
    {
        // 集積所の座標
        double depot_x = stop_coords[dis_idx[j]][0];
        double depot_y = stop_coords[dis_idx[j]][1];

        // ドローンの飛行元避難所の座標
        double shelter_x = stop_coords[drones[i].target_shelter][0];
        double shelter_y = stop_coords[drones[i].target_shelter][1];

        // (0,0)を中心とした時計回り角度を計算
        // 避難所の角度（(0,0)を中心とした時計回り）
        double shelter_angle = -atan2(shelter_y, shelter_x);
        if (shelter_angle < 0)
        {
            shelter_angle += 2 * PI;
        }

        // 集積所の角度（(0,0)を中心とした時計回り）
        double depot_angle = -atan2(depot_y, depot_x);
        if (depot_angle < 0)
        {
            depot_angle += 2 * PI;
        }

        // 集積所から避難所への時計回り角度差を計算
        double angle = shelter_angle - depot_angle;
        if (angle < 0)
        {
            angle += 2 * PI; // 負の角度を正の角度に変換
        }

        // 最小角度の集積所を更新
        if (angle < min_angle)
        {
            min_angle = angle;
            min_angle_depot_id = dis_idx[j];
        }
    }

    // ========= 上で求めた最近集積所と避難所の角度α、すべての運搬車両と避難所の角度βを比較し、α>βの運搬車両番号にフラグをたてる
    int is_not_shortend_area[NV];

    // 最近集積所から避難所への角度α（時計回り）を計算
    double depot_x = stop_coords[min_angle_depot_id][0];
    double depot_y = stop_coords[min_angle_depot_id][1];
    double shelter_x = stop_coords[drones[i].target_shelter][0];
    double shelter_y = stop_coords[drones[i].target_shelter][1];

    // (0,0)を中心とした角度α（最近集積所から避難所への時計回り角度差）を計算
    double shelter_angle_alpha = -atan2(shelter_y, shelter_x);
    if (shelter_angle_alpha < 0)
    {
        shelter_angle_alpha += 2 * PI;
    }

    double depot_angle_alpha = -atan2(depot_y, depot_x);
    if (depot_angle_alpha < 0)
    {
        depot_angle_alpha += 2 * PI;
    }

    double angle_alpha = shelter_angle_alpha - depot_angle_alpha;
    if (angle_alpha < 0)
    {
        angle_alpha += 2 * PI;
    }

    // 各運搬車両と避難所の角度βを計算し、α>βの場合フラグを立てる
    for (int v = 0; v < NV; v++)
    {
        // 運搬車両の現在位置
        double vehicle_pos_x = vehicle_x[v];
        double vehicle_pos_y = vehicle_y[v];

        // (0,0)を中心とした角度β（運搬車両から避難所への時計回り角度差）を計算
        double vehicle_angle_beta = -atan2(vehicle_pos_y, vehicle_pos_x);
        if (vehicle_angle_beta < 0)
        {
            vehicle_angle_beta += 2 * PI;
        }

        double angle_beta = shelter_angle_alpha - vehicle_angle_beta;
        if (angle_beta < 0)
        {
            angle_beta += 2 * PI;
        }

        // α > βの場合フラグを立てる
        if (angle_alpha > angle_beta)
        {
            is_not_shortend_area[v] = 1;
        }
        else
        {
            is_not_shortend_area[v] = 0;
        }
    }

    // ============== is_not_shortend_area[NV]フラグが立っていない運搬車両のうちから角度最小の運搬車両番号を選択
    int ve_num = -1;                   // 選択された運搬車両番号（初期値は-1）
    double min_vehicle_angle = 2 * PI; // 最小角度（初期値は360度）

    // フラグが立っていない運搬車両の中から角度最小の車両を選択
    for (int v = 0; v < NV; v++)
    {
        if (is_not_shortend_area[v] == 0)
        { // フラグが立っていない場合
            // 運搬車両の現在位置
            double vehicle_pos_x = vehicle_x[v];
            double vehicle_pos_y = vehicle_y[v];

            // (0,0)を中心とした時計回り角度を計算（運搬車両から避難所への角度差）
            double vehicle_center_angle = -atan2(vehicle_pos_y, vehicle_pos_x);
            if (vehicle_center_angle < 0)
            {
                vehicle_center_angle += 2 * PI;
            }

            double vehicle_angle = shelter_angle_alpha - vehicle_center_angle;
            if (vehicle_angle < 0)
            {
                vehicle_angle += 2 * PI;
            }

            // 最小角度の車両を更新
            if (vehicle_angle < min_vehicle_angle)
            {
                min_vehicle_angle = vehicle_angle;
                ve_num = v;
            }
        }
    }
    // 候補がない場合は最初の車両を選択（フォールバック）
    if (ve_num == -1)
    {
        ve_num = 0;
    }

    // ======================== 次に向かう集積所を決定
    int ve_next_depot_id;
    // == 選択された物資運搬車両が次に向かう集積所へ、ドローンが飛行するのに要する時間を算出
    if (is_depot == 1 && ve_stop_flag == 1 && ve_stop_end_time > elapsed_time) // 車両が集積所で停止中の場合
    {
        ve_next_depot_id = current_stop_idx[ve_num];
    }
    else
    {
        ve_next_depot_id = supply_vehicle[ve_num].next_depot_id; // それ以外の場合
    }

    // ドローンの現在座標と次の集積所との距離を計算
    double dx = drones[i].x - stop_coords[ve_next_depot_id][0];
    double dy = drones[i].y - stop_coords[ve_next_depot_id][1];
    double drone_distance = sqrt(dx * dx + dy * dy);
    // 距離を速度で割ってt1（時間）を計算
    double t1 = drone_distance / V_DRONE; // ドローンの速度で計算

    // == 物資運搬車両が次の集積所へ到達するまでの時間を算出（円周上移動）
    int ve_next_is_depot_flag = 0; // 次の目的地が集積所かどうかのフラグ
    double t2 = 999999.0;          // 車両の移動時間

    // 運搬車両の次の次の集積所IDを計算
    int next_next_depot_idx = -1;
    if (supply_vehicle[ve_num].next_depot_idx + 1 >= NDI)
    {
        next_next_depot_idx = 0;
    }
    else
    {
        next_next_depot_idx = supply_vehicle[ve_num].next_depot_idx + 1;
    }
    int ve_next_next_depot_id = dis_idx[next_next_depot_idx];

    // 車両が集積所で停止中の場合
    if (is_depot == 1 && ve_stop_flag == 1 && ve_stop_end_time > elapsed_time)
    {
        t2 = ve_stop_end_time - elapsed_time;
    }

    // ドローンと物資運搬車両の時間を比較
    // 集積所に飛行する時間 <= 車両の残りの停止時間 かつ 集積所に待機中
    if (t1 <= t2 && is_depot == 1)
    {
        // ドローンが集積所に到着するまでに車両が出発しない場合, current_stop_idxを返す
        return current_stop_idx[ve_num];
    }
    // 集積所に飛行する時間 > 車両の残りの停止時間 かつ 集積所に待機中
    else if (t1 > t2 && is_depot == 1)
    {
        // ドローンが集積所に到着するまでに車両が出発してしまう場合, min_angle_depot_idを返す
        // return supply_vehicle[ve_num].next_depot_id;
        return min_angle_depot_id;
    }
    else
    {
        // return supply_vehicle[ve_num].next_depot_id;
        return min_angle_depot_id;
    }
}

/**
 * @brief ドローンの状態更新処理
 */
void update_drone_state(DroneInfo *drone, DroneInfo drones[], int drone_count, double stop_coords[][2], double elapsed_time, double time_step, Info *info_list, int info_count, Facility *facilities, double *total_extra_supply_by_drone, int *drone_delivery_count, int drone_index, int dis_idx[NDI], SupplyVehicle supply_vehicle[], int current_stop_idx[], int next_stop_idx[], double vehicle_x[], double vehicle_y[])
{
    if (!drone->active)
        return;

    // ドローンが活動中の場合、time_stepごとに飛行時間を累積
    drone->timestep_flight_time += time_step;

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
        // 集積所での停止（余剰物資A積載）
        if (elapsed_time >= drone->state_start_time + T_DRONE_STOP)
        {
            // 集積所についたら集積所に避難所の情報を共有
            // == 情報リストの交換・情報回収フラグを更新
            for (int i = 0; i < info_count; i++)
            {
                // ドローン->集積所：ドローンが飛行してきた避難所で回収した情報について、集積所がその情報を保持していないなら
                if (drone->collect_info_id[i] == 1 && facilities[drone->target_depot].collect_info_id[i] == 0 && info_list[i].shelter_id == drone->target_shelter && drone->delivery_info_index == i)
                {
                    facilities[drone->target_depot].collect_info_id[i] = 1; // 集積所の情報リスト更新
                }

                // 集積所->ドローン：他のドローンが集めてきた情報を回収してリスト保持。他の集積所共有用
                if (drone->collect_info_id[i] == 0 && facilities[drone->target_depot].collect_info_id[i] == 1 && info_list[i].shelter_id == drone->target_shelter && drone->delivery_info_index == i)
                {
                    drone->collect_info_id[i] = 1; // ドローンの情報リスト更新
                }
            }

            // 今回の積載量を計算（残り需要量とドローン最大積載量の小さい方）
            double remaining_demand = 0;

            if (DELIVERY_METHOD == DELIVERY_METHOD_MULTI_DRONE)
            {
                // 他のドローンの積載量も考慮した需要量計算
                double other_drone_transport = get_other_drones_carrying_sum(drone, drones, ND, drone->target_shelter, info_list, info_count);

                for (int i = 0; i < info_count; i++)
                {
                    if (info_list[i].shelter_id == drone->target_shelter && !info_list[i].A_extra_delivery_completed)
                    {
                        // 残り需要量 = 総需要量 - 既配送量 - 他ドローン積載量
                        remaining_demand = info_list[i].A_extra_supply_demand - info_list[i].A_extra_supply_delivered - other_drone_transport;
                        break;
                    }
                }
            }
            else
            {
                for (int i = 0; i < info_count; i++)
                {
                    if (info_list[i].shelter_id == drone->target_shelter && !info_list[i].A_extra_delivery_completed)
                    {
                        remaining_demand = info_list[i].A_extra_supply_demand - info_list[i].A_extra_supply_delivered;
                        break;
                    }
                }
            }

            drone->carrying_extra_supply = drone->supply_scheduled_amount;

            // 停止時間終了、避難所への飛行開始
            drone->target_x = stop_coords[drone->target_shelter][0]; // 目標避難所のX座標
            drone->target_y = stop_coords[drone->target_shelter][1]; // 目標避難所のY座標
            update_drone_flight_time(drone, elapsed_time, DRONE_TO_SHELTER);
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
        // 避難所での停止（余剰物資A降ろし）
        if (elapsed_time >= drone->state_start_time + T_DRONE_STOP)
        {
            // 余剰物資Aを配送
            for (int i = 0; i < info_count; i++)
            {
                if (info_list[i].shelter_id == drone->target_shelter && !info_list[i].A_extra_delivery_completed && ((DELIVERY_METHOD != DELIVERY_METHOD_MULTI_DRONE && drone->delivery_info_index == i) || (DELIVERY_METHOD == DELIVERY_METHOD_MULTI_DRONE)) && drone->delivery_info_index == i) // A_extra_delivery_completedフラグが立っていない場合(手法2のみ情報のインデックス参照)
                {
                    drone->supply_scheduled_amount = 0; // 配送予定物資量リセット

                    // 避難所の余剰物資A在庫を増加
                    int shelter_idx = drone->target_shelter;
                    if (shelter_idx >= 1 && shelter_idx <= NS + NDI - 1)
                    {
                        facilities[shelter_idx].extra_supply_a += drone->carrying_extra_supply;
                    }

                    info_list[i].A_extra_supply_delivered += drone->carrying_extra_supply;

                    // === ドローンによる余剰物資A運搬統計の更新 ===
                    *total_extra_supply_by_drone += drone->carrying_extra_supply;
                    (*drone_delivery_count)++;

                    if (DELIVERY_METHOD == DELIVERY_METHOD_MULTI_DRONE)
                    {
                        double total_other_drone_transport = calculate_all_drones_transport_amount(drone, drones, ND, drone->target_shelter, info_list, info_count);

                        if (info_list[i].A_extra_supply_delivered + total_other_drone_transport >= info_list[i].A_extra_supply_demand) // 避難所に届けられた物資量と他のドローンの運搬中の物資量が需要量を超えた場合
                        {
                            if (total_other_drone_transport > 0) //
                            {
                                //
                            }
                            else // 自分が最後にとどけた場合
                            {
                                info_list[i].A_extra_delivery_completed = 1;
                                info_list[i].A_extra_delivery_completion_time = elapsed_time;
                            }

                            drone->current_trip = drone->required_trips + 999; // 往復完了フラグを設定（手法2によって,車両が避難所の余剰物資を運搬したことによるドローンの往復回数短縮を考慮して（車両によってドローンの往復必要回数がへる場合があるため））
                        }
                        else
                        {
                            // 配送完了していない場合、ドローンの次回配送予定量を設定
                            double remaining = info_list[i].A_extra_supply_demand - (info_list[i].A_extra_supply_delivered + total_other_drone_transport);
                            if (remaining < DRONE_MAX_CARRY)
                            {
                                drone->supply_scheduled_amount = remaining;
                            }
                            else
                            {
                                drone->supply_scheduled_amount = DRONE_MAX_CARRY;
                            }
                        }
                    }
                    else
                    {
                        if (info_list[i].A_extra_supply_delivered >= info_list[i].A_extra_supply_demand)
                        {
                            info_list[i].A_extra_delivery_completed = 1;
                            info_list[i].A_extra_delivery_completion_time = elapsed_time;
                            drone->current_trip = drone->required_trips + 999;
                        }
                    }
                }
            }

            drone->current_trip++;
            drone->carrying_extra_supply = 0; // 積載物資をリセット

            if (drone->current_trip <= drone->required_trips)
            {
                int target_depot_id;
                // 次の往復へ、車両の次の集積所に向かう
                if (DRONE_TO_DEPOT_OPTION == 1)
                {
                    target_depot_id = find_nearest_depot(drones, drone_index, stop_coords, dis_idx, NDI);
                }
                else if (DRONE_TO_DEPOT_OPTION == 2)
                {
                    if (NV == 1)
                    {
                        target_depot_id = find_vehicle_next_depot_V_1(supply_vehicle, current_stop_idx, next_stop_idx, drones, drone_index, stop_coords, dis_idx, NDI, vehicle_x, vehicle_y, 0.0, -1, -1, elapsed_time);
                    }
                    else
                    {
                        target_depot_id = find_vehicle_next_depot(supply_vehicle, current_stop_idx, next_stop_idx, drones, drone_index, stop_coords, dis_idx, NDI, vehicle_x, vehicle_y, 0.0, -1, -1, elapsed_time);
                    }
                }

                drone->target_depot = target_depot_id;             // 最寄り集積所IDを設定
                drone->target_x = stop_coords[target_depot_id][0]; // 最寄り集積所座標を目的として設定
                drone->target_y = stop_coords[target_depot_id][1];

                update_drone_flight_time(drone, elapsed_time, DRONE_TO_DEPOT);
            }
            else
            {
                // 往復完了、通常巡回に復帰
                update_drone_flight_time(drone, elapsed_time, DRONE_PATROL);
                drone->current_trip = 0;
                drone->required_trips = 0;

                // 現在位置から巡回角度を計算
                drone->angle = atan2(drone->y, drone->x);
                if (drone->angle < 0)
                    drone->angle += 2.0 * PI;
            }
        }
        break;
    }
}

/**
 * @brief 余剰物資A需要量をランダムに生成
 */
double generate_extra_supply_demand()
{
    if ((int)MAX_EXTRA_DEMAND - (int)MIN_EXTRA_DEMAND == 0) // MAX_EXTRA_DEMANDとMIN_EXTRA_DEMANDが同じ場合
    {
        return MIN_EXTRA_DEMAND;
    }
    else
    {
        return MIN_EXTRA_DEMAND + ((double)rand() / RAND_MAX) * (MAX_EXTRA_DEMAND - MIN_EXTRA_DEMAND);
    }
}

/**
 * @brief 余剰物資A需要に基づいてドローンの必要往復回数を計算
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

    // 物資運搬関連の状態の場合、物資運搬時間に加算：少ない物資を運搬している場合の
    if (drone->supply_scheduled_amount < FEW_SUPPLY_THRESHOLD &&
        drone->supply_scheduled_amount > 0 &&
        (drone->state == DRONE_TO_DEPOT ||
         drone->state == DRONE_AT_DEPOT ||
         drone->state == DRONE_TO_SHELTER ||
         drone->state == DRONE_AT_SHELTER))
    {
        drone->few_supply_transport_time += state_duration;
    }

    // 次回の状態のために現在時刻を記録
    drone->last_state_time = current_time;
    drone->state = new_state; // ドローンの状態を更新
}

/**
 * @brief ドローンが協調運搬に参加すべきかを判定する関数
 */
int should_drone_join_transport(DroneInfo *drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count)
{
    // 対象避難所の情報を検索（情報リスト全体をスキャン）
    for (int i = 0; i < info_count; i++)
    {
        // 指定避難所IDかつ未配送完了、かつ閾値以上の物資の配達を待機中でない情報を検索
        if (info_list[i].shelter_id == shelter_id && info_list[i].A_extra_delivery_completed == 0)
        {
            if (!info_list[i].collected)
                return 0; // 情報未回収なので協調運搬に参加しない

            double other_drones_transport = 0.0;
            double total_amount = 0.0;

            for (int j = 0; j < drone_count; j++)
            {
                if (drones[j].active && drones[j].target_shelter == shelter_id && &drones[j] != drone && drones[j].delivery_info_index == i)
                {
                    // ドローンの状態に応じて運搬量を計算
                    switch (drones[j].state)
                    {
                    case DRONE_TO_DEPOT: // 集積所への移動中
                    case DRONE_AT_DEPOT: // 集積所で物資積込中
                        // 次回往復で運搬予定の量を推定
                        {
                            double remaining_demand = info_list[i].A_extra_supply_demand - (info_list[i].A_extra_supply_delivered + total_amount);

                            if (remaining_demand > 0)
                            {
                                double next_trip_amount = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;
                                other_drones_transport += next_trip_amount; // 他ドローン運搬量に加算
                            }
                        }
                        break;
                    case DRONE_TO_SHELTER: // 避難所への移動中（物資運搬中）
                        other_drones_transport += drones[j].carrying_extra_supply;
                        break;
                    case DRONE_AT_SHELTER: // 避難所で荷下ろし中
                    {
                        other_drones_transport += drones[j].carrying_extra_supply; // 現在運搬中の物資量を加算

                        // 残り往復回数から今後の運搬量を推定
                        int remaining_trips = drones[j].required_trips - drones[j].current_trip;
                        if (remaining_trips > 0) // まだ往復が残っている場合
                        {
                            // 残り需要量を再計算
                            double remaining_demand = info_list[i].A_extra_supply_demand - (info_list[i].A_extra_supply_delivered + total_amount);
                            if (remaining_demand > 0)
                            {
                                // 次の往復で運搬する量 = min(ドローン積載容量, 残り需要量)
                                double next_trip_amount = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;
                                other_drones_transport += next_trip_amount; // 他ドローン運搬量に加算
                            }
                        }
                    }
                        total_amount += other_drones_transport;
                    default:
                        break;
                    }
                }
            }

            double total_covered = info_list[i].A_extra_supply_delivered + other_drones_transport;
            if (total_covered < info_list[i].A_extra_supply_demand) // ==== 手法３の場合 =======/
            {
                drone->delivery_info_index = i; // 物資運搬情報のインデックスを設定
                return 1;                       // 協調運搬に参加すべき
            }
            else
            {
                return 0; // 協調運搬に参加しない
            }
        }
    }

    return 0; // 該当する情報が見つからない場合は協調運搬に参加しない
}

/**
 * @brief 全ドローンによる指定避難所へのA運搬量を計算する関数（手法3用）
 */
double calculate_all_drones_transport_amount(DroneInfo *drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count)
{
    double total_transport_amount = 0.0;

    // 指定避難所の情報を検索（現在のドローンは除外）
    for (int i = 0; i < info_count; i++)
    {
        if (info_list[i].shelter_id == shelter_id && !info_list[i].A_extra_delivery_completed)
        {
            // この避難所に対して運搬中のドローンを検索
            for (int j = 0; j < drone_count; j++)
            {
                if (drones[j].active && drones[j].target_shelter == shelter_id && &drones[j] != drone && drones[j].delivery_info_index == i)
                {
                    // ドローンの状態に応じて運搬量を計算
                    switch (drones[j].state)
                    {
                    case DRONE_TO_DEPOT:
                    case DRONE_AT_DEPOT:
                    {
                        double remaining_demand = info_list[i].A_extra_supply_demand - info_list[i].A_extra_supply_delivered;

                        if (remaining_demand > 0)
                        {
                            // 次の往復で運搬する量 = min(DRONE_MAX_CARRY, 残り需要量)
                            double next_trip_amount = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;
                            total_transport_amount += next_trip_amount;
                        }
                    }
                    break;
                    case DRONE_TO_SHELTER:
                        // 避難所に向かっている場合（積載中）
                        total_transport_amount += drones[j].carrying_extra_supply;
                        break;
                    case DRONE_AT_SHELTER:
                    {
                        total_transport_amount += drones[j].carrying_extra_supply; // 現在積載している物資量を加算

                        int remaining_trips = drones[j].required_trips - drones[j].current_trip;
                        if (remaining_trips > 0)
                        {
                            double remaining_demand = info_list[i].A_extra_supply_demand - info_list[i].A_extra_supply_delivered;
                            if (remaining_demand > 0)
                            {
                                // 次の往復で運搬する量 = min(DRONE_MAX_CARRY, 残り需要量)
                                double next_trip_amount = (remaining_demand > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : remaining_demand;
                                total_transport_amount += next_trip_amount;
                            }
                        }
                    }
                    break;
                    default:
                        break;
                    }
                }
            }
            break; // 該当する情報が見つかったので終了
        }
    }

    return total_transport_amount;
}

/**
 * @brief 指定避難所への他ドローンの運搬量合計を計算する
 */
double get_other_drones_carrying_sum(DroneInfo *current_drone, DroneInfo drones[], int drone_count, int shelter_id, Info *info_list, int info_count)
{
    double total_carrying = 0.0;

    // 指定避難所の情報を検索
    for (int i = 0; i < info_count; i++)
    {
        if (info_list[i].shelter_id == shelter_id && !info_list[i].A_extra_delivery_completed)
        {
            // 全てのドローンをチェック
            for (int j = 0; j < drone_count; j++)
            {
                if (drones[j].delivery_info_index == i &&
                    drones[j].active && &drones[j] != current_drone &&
                    drones[j].target_shelter == shelter_id)
                {
                    total_carrying += drones[j].carrying_extra_supply;
                }
            }
            break; // 該当する情報が見つかったので終了
        }
    }

    return total_carrying;
}

/**
 * @brief 毎フレームの描画処理（GIFアニメーション用フレーム生成）
 */
void plot_frame(FILE *pipe, double stop_coords[][2], double vehicle_x[], double vehicle_y[], DroneInfo drones[], double elapsed_time, Info *info_list, int info_count, Facility facilities[], SupplyVehicle supply_vehicle[], int dis_idx[NDI])
{
    // GIF出力が無効、または指定時間範囲外の場合は処理をスキップ
    if (GIF_TIME_RANGE_FLAG == 1) // gifに出力する時間範囲が指定されている場合
    {
        if (!ENABLE_GIF || pipe == NULL || elapsed_time < GIF_START_TIME_SEC || elapsed_time > GIF_END_TIME_SEC)
            return;
    }
    else // gifに出力する時間範囲が指定されていない場合
    {
        if (!ENABLE_GIF || pipe == NULL)
            return;
    }

    // === 時刻表示の準備 ===
    // 経過時間を時:分:秒の形式に分解
    int hours = (int)(elapsed_time / 3600);
    int minutes = (int)((elapsed_time - hours * 3600) / 60);
    int seconds = (int)(elapsed_time - hours * 3600 - minutes * 60);

    // タイトルバーに経過時間のみ表示するバージョン
    if (hours > 0)
    {
        fprintf(pipe, "set title 'Simulation - Time: %02d:%02d hour'\n", hours, minutes);
    }
    else if (minutes > 0)
    {
        fprintf(pipe, "set title 'Simulation - Time: %02d min'\n", minutes);
    }
    else
    {
        fprintf(pipe, "set title 'Simulation - Time: 0 min'\n");
    }

    // 車両の右上に担当する避難所番号を表示
    for (int v = 0; v < NV; v++)
    {
        int label_count = 0; // ラベル数カウンター

        // まず、この車両が担当する避難所数を数える
        int responsible_count = 0;
        for (int i = 0; i < info_count; i++)
        {
            if (info_list[i].responsible_TV_id == v)
            {
                responsible_count++;
            }
        }

        // 担当避難所がある場合のみ表示
        if (responsible_count > 0)
        {
            for (int i = 0; i < info_count; i++)
            {
                if (info_list[i].responsible_TV_id == v) // 車両番号と一致する場合
                {
                    double display_x = vehicle_x[v] + 100 + (label_count * 400.0); // 横間隔200
                    double display_y = vehicle_y[v] - (label_count * 1.0);         // 縦間隔80（上から下へ）

                    fprintf(pipe, "set label '%d' at %.2f,%.2f tc rgb 'black' font ',15'\n",
                            info_list[i].shelter_id, display_x, display_y);

                    label_count += 1; // 次のラベル位置へ
                }
            }
        }
    }

    // === 描画コマンドの構築 ===
    // 基本レイヤー（円、集積所、避難所、車両）は常に描画
    fprintf(pipe, "plot '-' with lines lc 'gray' lw 2 notitle, ");                              // 1. 円周経路
    fprintf(pipe, "'-' with points pointtype 5 pointsize 3 linecolor rgb '#FF00FF' notitle, "); // 2. 集積所強調
    fprintf(pipe, "'-' with points pt 7 ps 1.5 lc 'gray' notitle, ");                           // 3. 通常避難所
    fprintf(pipe, "'-' with points pt 7 ps 2.2 lc 'orange' notitle, ");                         // 4. 情報発生中避難所
    fprintf(pipe, "'-' with points pt 7 ps 2.2 lc 'red' notitle");                              // 6. 車両

    // ドローンレイヤーは台数が1台以上の場合のみ追加
    if (ND > 0)
    {
        fprintf(pipe, ", '-' with points pt 5 ps 1.8 lc 'green' notitle"); // 6. ドローン群
    }
    fprintf(pipe, "\n");

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

    // 2. 集積所の座標データ送信
    for (int i = 0; i < NDI; i++)
    {
        fprintf(pipe, "%.1f %.1f\n", stop_coords[dis_idx[i]][0], stop_coords[dis_idx[i]][1]); // 集積所のインデックスを参照して強調
    }
    fprintf(pipe, "e\n");

    // === 情報発生状況の解析 ===
    // 各避難所の情報発生状況を調査（描画色分け用）
    int has_info[NS + NDI] = {0}; // 避難所別の情報フラグ（0:情報なし, 1:情報あり）
    for (int i = 0; i < info_count; i++)
    {
        // 未回収かつ既に発生済みの情報をチェック
        if (!info_list[i].collected &&
            info_list[i].generation_time <= elapsed_time)
        {
            has_info[info_list[i].shelter_id] = 1; // 該当避難所に情報ありマーク
        }
    }

    // 3. 通常避難所（情報なし）の座標データ送信（灰色表示）
    int normal_shelter_count = 0;
    for (int i = 1; i < TOTAL_STOPS; i++) // 集積所（i=0）は除外
    {
        // 集積所かどうかをチェック
        int is_depot = 0;
        for (int d = 0; d < NDI; d++)
        {
            if (i == dis_idx[d])
            {
                is_depot = 1;
                break; // 集積所と判明したらループを抜ける
            }
        }
        if (is_depot)
        {
            continue; // 集積所の場合、次のiに進む
        }

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

    // 4. 情報発生中避難所（情報ありかつ閾値配送待機なし）の座標データ送信（オレンジ色表示）
    int info_shelter_count = 0;
    for (int i = 1; i < TOTAL_STOPS; i++) // 集積所（i=0）は除外
    {
        if (has_info[i]) // 情報発生中かつ閾値配送待機中でない避難所
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

    // 6. 車両群の現在位置を送信（赤色表示） - NV台の車両すべて
    for (int v = 0; v < NV; v++) // 現在は vehicle[0]のみを表示（NV）
    {
        fprintf(pipe, "%.1f %.1f\n", vehicle_x[v], vehicle_y[v]);
    }
    fprintf(pipe, "e\n"); // 7. ドローン群の現在位置を送信（緑色表示）- ドローン台数>0の場合のみ
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
    // 物資GA（青色テキスト）と物資GB（赤色テキスト）の在庫量を数値で表現
    const double supply_offset = 20.0; // 避難所からの表示オフセット距離

    // 既存のlabelを削除してから新しいlabelを設定
    fprintf(pipe, "unset label\n");

    for (int i = 1; i <= NS + NDI - 1; i++) // 避難所のみ（集積所は除く）
    {
        // 集積所かどうかをチェック
        int is_depot = 0;
        for (int d = 0; d < NDI; d++)
        {
            if (i == dis_idx[d])
            {
                is_depot = 1;
                break; // 集積所と判明したらループを抜ける
            }
        }
        if (is_depot)
        {
            continue; // 集積所の場合、次のiに進む
        }

        // 各避難所の現在の要求余剰物資A量を計算
        double current_demand = 0.0;
        for (int j = 0; j < info_count; j++)
        {
            if (info_list[j].shelter_id == i && !info_list[j].A_extra_delivery_completed &&
                info_list[j].generation_time <= elapsed_time)
            {
                current_demand += (info_list[j].A_extra_supply_demand - info_list[j].A_extra_supply_delivered);
            }
        }

        // 物資GAの表示位置（避難所の右上側）
        double display_x_a = stop_coords[i][0] + supply_offset;
        double display_y_a = stop_coords[i][1] + 300.0; // 最上段

        // 物資GBの表示位置（避難所の右上中央側）
        double display_x_b = stop_coords[i][0] + supply_offset;
        double display_y_b = stop_coords[i][1] + 100.0; // 上中段

        // 余剰物資Aの表示位置（避難所の右下中央側）
        double display_x_extra = stop_coords[i][0] + supply_offset;
        double display_y_extra = stop_coords[i][1] - 100.0; // 下中段

        // 要求余剰物資Aの表示位置（避難所の右下側）
        double display_x_demand = stop_coords[i][0] + supply_offset;
        double display_y_demand = stop_coords[i][1] - 500.0; // 最下段

        // 施設インデックス表示（左、黒色）
        double display_x_index = stop_coords[i][0] - 400;
        double display_y_index = stop_coords[i][1] + 0.5;
        fprintf(pipe, "set label '%d' at %.1f,%.1f tc rgb '#000000' font ',14'\n",
                i, display_x_index, display_y_index);

        // 各避難所でそれぞれの物資の配送量と要求物資の数値を表示
        // 物資GAの数値表示（青色）- 常に表示
        fprintf(pipe, "set label 'GA:%.0f' at %.1f,%.1f tc rgb '#0000FF' font ',11'\n",
                facilities[i].supply_ga, display_x_a, display_y_a);

        // 物資GBの数値表示（赤色）- 常に表示
        fprintf(pipe, "set label 'GB:%.0f' at %.1f,%.1f tc rgb '#FF0000' font ',11'\n",
                facilities[i].supply_gb, display_x_b, display_y_b);

        // 余剰物資Aの数値表示（紫色）- 常に表示
        fprintf(pipe, "set label 'ExA:%.0f' at %.1f,%.1f tc rgb '#800080' font ',11'\n",
                facilities[i].extra_supply_a, display_x_extra, display_y_extra);

        // 余剰物資Bの数値表示（紫色）- 常に表示
        fprintf(pipe, "set label 'ExB:%.0f' at %.1f,%.1f tc rgb '#800080' font ',11'\n",
                facilities[i].extra_supply_b, display_x_extra, display_y_extra - 200.0);

        // 要求余剰物資Aの数値表示（オレンジ色）- 要求がある場合のみ表示
        if (current_demand > 0.0)
        {
            fprintf(pipe, "set label 'Req:%.0f' at %.1f,%.1f tc rgb '#FF8000' font ',11'\n",
                    current_demand, display_x_demand, display_y_demand);
        }
    }

    // === ドローンのsupply_scheduled_amountラベル表示 ===
    if (ND > 0)
    {
        const double drone_label_offset = 25.0; // ドローンから右上へのオフセット距離
        for (int i = 0; i < ND; i++)
        {
            if (drones[i].active && drones[i].supply_scheduled_amount > 0.0) // アクティブで予定物資量がある場合のみ表示
            {
                double label_x = drones[i].x + drone_label_offset;
                double label_y = drones[i].y + drone_label_offset;
                fprintf(pipe, "set label 'D%d:%.0fkg' at %.1f,%.1f tc rgb '#00AA00' font ',11'\n",
                        i + 1, drones[i].supply_scheduled_amount, label_x, label_y);
            }
        }
    }
    // バッファをフラッシュしてgnuplotに確実に送信
    fflush(pipe);
}

/**
 * @brief シミュレーションモデルの静的構造をPNGファイルとして保存
 *
 * 【表示内容】
 * - 集積所（中心の赤い四角）
 * - 避難所（円周上の青い円）
 * - 道路（円形の経路と集積所への接続線）
 * - 座標軸とラベル
 *
 * @param stop_coords 停止地点座標配列
 */
void save_simulation_model_png(double stop_coords[][2], int dis_idx[NDI])
{
    FILE *pipe = popen("gnuplot -persist", "w");
    if (pipe == NULL)
    {
        printf("エラー: gnuplotの起動に失敗しました\n");
        return;
    }

    double plot_range = R * 1.2;

    // PNG出力設定
    fprintf(pipe, "set terminal png size 800,600 font 'DejaVu Sans,12'\n");
    fprintf(pipe, "set output 'simulation_model.png'\n");

    // プロット範囲をGIFと同じに設定（600x600の範囲）
    fprintf(pipe, "set xrange [%.1f:%.1f]\n", -plot_range, plot_range);
    fprintf(pipe, "set yrange [%.1f:%.1f]\n", -plot_range, plot_range);

#if CLEAN_PNG_OUTPUT
    // クリーンな出力設定：軸ラベル・目盛り・枠線を削除
    fprintf(pipe, "unset xtics\n");  // x軸の目盛り削除
    fprintf(pipe, "unset ytics\n");  // y軸の目盛り削除
    fprintf(pipe, "unset xlabel\n"); // x軸ラベル削除
    fprintf(pipe, "unset ylabel\n"); // y軸ラベル削除
    fprintf(pipe, "unset border\n"); // 図を囲む枠線削除
#else
    // 通常の軸設定
    fprintf(pipe, "set xlabel 'X (m)'\n");
    fprintf(pipe, "set ylabel 'Y (m)'\n");
    fprintf(pipe, "set title 'Simulation Model'\n");
    fprintf(pipe, "set grid\n");
#endif

    fprintf(pipe, "set size ratio 1\n");

    // 凡例を無効にする
    fprintf(pipe, "unset key\n");

    // 道路（円形経路）の描画
    fprintf(pipe, "set parametric\n");
    fprintf(pipe, "set trange [0:2*pi]\n");
    fprintf(pipe, "set samples 100\n");

    // 複数のプロットを一度に描画
    fprintf(pipe, "plot ");

    // 1. 円形道路
    fprintf(pipe, "%.1f*cos(t), %.1f*sin(t) with lines linewidth 3 linecolor rgb '#808080' notitle, ",
            R, R);

    // 2. 集積所
    fprintf(pipe, "'-' with points pointtype 5 pointsize 3 linecolor rgb '#FF00FF' notitle, ");

    // 3. 避難所
    fprintf(pipe, "'-' with points pointtype 7 pointsize 2 linecolor rgb '#0000FF' notitle\n");

    // 集積所データ
    for (int i = 0; i < NDI; i++)
    {
        fprintf(pipe, "%.1f %.1f\n", stop_coords[dis_idx[i]][0], stop_coords[dis_idx[i]][1]); // 集積所のインデックスを参照して強調
    }
    fprintf(pipe, "e\n");

    // 避難所データ
    for (int i = 1; i <= NS + NDI - 1; i++)
    {
        // 集積所かどうかをチェック
        int is_depot = 0;
        for (int d = 0; d < NDI; d++)
        {
            if (i == dis_idx[d])
            {
                is_depot = 1;
                break; // 集積所と判明したらループを抜ける
            }
        }
        if (is_depot)
        {
            continue; // 集積所の場合、次のiに進む
        }

        fprintf(pipe, "%.1f %.1f\n", stop_coords[i][0], stop_coords[i][1]);
    }
    fprintf(pipe, "e\n");

    fflush(pipe);
    pclose(pipe);

    printf("シミュレーションモデル構造図を 'simulation_model.png' として保存しました\n");
}

/****************************************** メイン関数 ************************************************************/
int main(void)
{
    // === 初期化処理 ===
    srand(22); // 固定シードで再現性を確保

    // === gnuplotパイプの初期化 ===
    FILE *gnuplot_pipe = NULL;
    if (ENABLE_GIF)
    {
        gnuplot_pipe = init_gnuplot(); // GIFアニメーション用のgnuplotパイプを開く
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
    for (int i = 0; i < TOTAL_STOPS; i++)
    {
        double angle = PI / 2.0 - i * angle_increment; // 各拠点の角度計算
        stop_coords[i][0] = R * cos(angle);            // X座標 = R*cos(θ)
        stop_coords[i][1] = R * sin(angle);            // Y座標 = R*sin(θ)
    }

    // === シミュレーション制御変数 ===
    int current_stop_idx[NV];   // 各車両の現在の停止地点インデックス（0など=集積所、1〜NS+NDI-1=避難所）
    int next_stop_idx[NV];      // 各車両の次の停止地点インデックス
    int lap_count = 0;          // 完了した周回数
    double elapsed_time = 0.0;  // 経過時間（秒）
    int is_first_departure = 1; // 集積所からの初回出発フラグ

    // === 集積所のcurrent_stop_idx設定 ===
    int dis_idx[NDI] = {0}; // 集積所のcurrent_stop_idx
    if (NDI > 1)
    {
        double radix = (double)TOTAL_STOPS / (double)NDI; // 集積所配置の基数
        // printf("集積所インデックス配置(radix=%.2f): ", radix);
        for (int i = 0; i < NDI; i++)
        {
            dis_idx[i] = (int)((double)i * radix); // 集積所を均等配置(NSとNDIの関係で割り切れない場合は整数化)
        }
    }

    // === ドローン制御変数（複数台対応） ===
    DroneInfo drones[ND]; // ドローン情報配列

    // === 情報管理変数 ===
    Info info_list[MAX_INFO];                                      // 情報データベース
    int info_count = 0;                                            // 発生した情報の総数
    double next_info_time = generate_exponential_interval(LAMBDA); // 次の情報発生予定時刻
    double total_tc = 0.0;                                         // 回収時間差（Tc）の累積値
    int collected_count = 0;                                       // 回収済み情報数

    // === 物資管理変数 ===
    Facility facilities[NS + NDI];    // 各避難所の物資在庫配列（集積所のインデックスも含むが処理の際は除外する）
    SupplyVehicle supply_vehicle[NV]; // 物資運搬車両の状態配列（複数車両対応）

    // === 余剰物資運搬統計変数 ===
    double total_extra_supply_by_vehicle = 0.0; // 車両による余剰物資A運搬総量 (kg)
    double total_extra_supply_by_drone = 0.0;   // ドローンによる余剰物資A運搬総量 (kg)
    int vehicle_delivery_count = 0;             // 車両による余剰物資A配送回数
    int drone_delivery_count = 0;               // ドローンによる余剰物資A配送回数(往復の回数も含む)

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
        drones[i].state = DRONE_PATROL;                      // 通常巡回モード
        drones[i].target_shelter = 0;                        // 目標避難所なし
        drones[i].target_depot = -1;                         // 目標集積所なし
        drones[i].current_trip = 0;                          // 往復回数0
        drones[i].required_trips = 0;                        // 必要往復回数0
        drones[i].state_start_time = 0.0;                    // 状態開始時刻
        drones[i].target_x = drones[i].target_y = 0.0;       // 目標座標
        drones[i].carrying_extra_supply = 0.0;               // 積載中のA量
        drones[i].delivery_info_index = -1;                  // 物資運搬情報のインデックス（未設定）
        drones[i].supply_scheduled_amount = 0.0;             // 運搬予定物資量
        for (int j = 0; j < MAX_INFO; j++)
        {
            drones[i].collect_info_id[j] = 0; // 配送済み情報IDリスト初期化
        }
        drones[i].total_flight_time = 0.0;         // 総飛行時間
        drones[i].timestep_flight_time = 0.0;      // time_stepによる累積飛行時間
        drones[i].supply_transport_time = 0.0;     // 物資運搬飛行時間
        drones[i].few_supply_transport_time = 0.0; // 少ない物資運搬飛行時間
        drones[i].last_state_time = 0.0;           // 前回の状態変更時刻
    }

    // === 【物資管理システムの初期化】 ===
    for (int i = 0; i <= NS + NDI - 1; i++)
    {
        facilities[i].supply_ga = 0.0;      // 物資G(A)
        facilities[i].supply_gb = 0.0;      // 物資B在庫: 0kg
        facilities[i].extra_supply_a = 0.0; // 余剰物資A在庫: 0kg
        facilities[i].extra_supply_b = 0.0; // 余剰物資B在庫: 0kg
        for (int j = 0; j < MAX_INFO; j++)
        {
            facilities[i].collect_info_id[j] = 0; // 配送済み情報IDリスト初期化
        }
        facilities[i].tg_total_time = 0.0;
        facilities[i].tg_count = 0.0;
        facilities[i].tg_last_time = 0.0;
    }

    // === 物資運搬車両の初期状態設定 ===
    for (int v = 0; v < NV; v++)
    {
        supply_vehicle[v].remaining_supply_ga = TOTAL_SUPPLY_WEIGHT * SUPPLY_GA_RATIO; // 物資A: 9000kg (90%)
        supply_vehicle[v].remaining_supply_gb = TOTAL_SUPPLY_WEIGHT * SUPPLY_GB_RATIO; // 物資B: 1000kg (10%)
        supply_vehicle[v].remaining_extra_supply_b = EXTRA_SUPPLY_B;                   // 余剰物資: 1000kg
        supply_vehicle[v].is_loaded = 1;                                               // 積載状態フラグ: 積載中
        for (int i = 0; i < MAX_INFO; i++)
        {
            supply_vehicle[v].collect_info_id[i] = 0; // 配送済み情報IDリスト初期化
        }
        for (int i = 0; i < MAX_INFO; i++)
        {
            supply_vehicle[v].demand_supply_loaded_flag[i] = 0; // 要求物資積載フラグ初期化
        }
        supply_vehicle[v].next_depot_id = dis_idx[1]; // 次の集積所ID未設定
        supply_vehicle[v].next_depot_idx = 1;
        supply_vehicle[v].last_depot_arrival_time = 0.0; // 最後の集積所到着時刻初期化
    }

    // === シミュレーションモデル構造図をPNG出力 ===
    save_simulation_model_png(stop_coords, dis_idx);

    // === 車両配列の初期化（メインループ前） ===
    // 全車両を集積所から開始
    for (int v = 0; v < NV; v++)
    {
        current_stop_idx[v] = dis_idx[v]; // 全車両を集積所から開始

        next_stop_idx[v] = (current_stop_idx[v] + 1) % TOTAL_STOPS; // 次の停止地点を設定
    }

    /********************** 【メインシミュレーションループ開始】 *********************************************************************************************/
    // 指定された周回数（NT）まで車両の巡回を継続
    while (lap_count < NT)
    {
        // === 【情報発生処理】（ポアソン過程による確率的イベント生成） ===
        while (elapsed_time >= next_info_time && info_count < MAX_INFO)
        {
            // === 新情報の生成と初期化 ===
            info_list[info_count].shelter_id = generate_random_shelter(dis_idx);          // ランダム避難所（1〜NS+NDI-1において集積所を除くインデックス）を選択
            info_list[info_count].generation_time = next_info_time;                       // 発生時刻を記録
            info_list[info_count].collection_time = -1;                                   // 未回収状態（-1で表現）
            info_list[info_count].TV_collection_time = -1;                                // 未回収状態（-1で表現）
            info_list[info_count].collected = 0;                                          // 回収フラグ: 未回収
            info_list[info_count].collected_by = COLLECTED_BY_NONE;                       // 回収主体: 未回収
            info_list[info_count].collected_by_drone_later = 0;                           // 後にドローンが回収したフラグ: 未回収
            info_list[info_count].drone_collection_time_later = -1;                       // 後にドローンが回収した時刻: 未回収
            info_list[info_count].A_extra_supply_demand = generate_extra_supply_demand(); // 0〜90kgの需要量をランダム生成
            info_list[info_count].B_extra_supply_demand = generate_extra_supply_demand(); // 0〜90kgの需要量をランダム生成
            info_list[info_count].A_extra_supply_delivered = 0.0;                         // A 配送済み量: 初期0kg
            info_list[info_count].B_extra_supply_delivered = 0.0;                         // B 配送済み量: 初期0kg
            info_list[info_count].A_extra_delivery_completed = 0;                         // 配送完了フラグ: 未完了
            info_list[info_count].B_extra_delivery_completed = 0;                         // 余剰物資B配送完了フラグ: 未完了
            info_list[info_count].A_extra_delivery_completion_time = -1;                  // 配送完了時刻: 未完了
            info_list[info_count].B_extra_delivery_completion_time = -1;                  // 余剰物資Bの配送完了時刻: 未完了
            info_list[info_count].fast_detection_flag = 0;                                // 迅速発見フラグ: 未設定
            info_list[info_count].responsible_TV_id = -1;                                 // 最初に回収したTVドローンID: 未設定
            info_list[info_count].TV_inf_collect_at_depot = 0;                            // 集積所で情報がTVに回収されたフラグ: 未回収

            info_count++; // 情報総数をインクリメント

            next_info_time += generate_exponential_interval(LAMBDA);
        }

        // === 現在停止地点の座標取得 ===
        double current_x[NV]; // 車両座標配列（X座標）
        double current_y[NV]; // 車両座標配列（Y座標）

        // 現在は車両[0]のみを使用
        for (int v = 0; v < NV; v++)
        {
            current_x[v] = stop_coords[current_stop_idx[v]][0];
            current_y[v] = stop_coords[current_stop_idx[v]][1];
        }

        // === 車両停止中の処理 ===
        if (!is_first_departure)
        {
            for (int v = 0; v < NV; v++)
            {
                // 停止開始時の描画（現在位置をプロット）
                if (v == NV - 1) // 車両[NV-1]の処理のときのみ描画,最後の車両の運搬処理から描画
                {
                    plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, facilities, supply_vehicle, dis_idx);
                }

                // === 停止地点情報の表示 ===
                int is_depot = 0;
                int next_stop_depot = 0; // 物資運搬車両の次の集積所番号
                int next_d = 0;          // 物資運搬車両の次の集積所のインデックス

                for (int d = 0; d < NDI; d++)
                {
                    if (current_stop_idx[v] == dis_idx[d])
                    {
                        is_depot = 1; // 集積所であることを示すフラグを設定
                        next_d = d + 1;
                        if (next_d >= NDI)
                        {
                            next_d = 0; // 最後の集積所の場合、最初の集積所に戻る
                        }
                        break;
                    }
                }
                // 集積所に停止する場合
                if (is_depot) // current_stop_idx[v] == 0など集積所インデックスと一致したとき
                {
                    if (current_stop_idx[v] == 0)
                    {
                        supply_vehicle[v].last_depot_arrival_time = elapsed_time; // 最後の集積所到着時刻を更新
                    }
                    // == 物資運搬車両の次の集積所を設定 ==
                    supply_vehicle[v].next_depot_idx = next_d;
                    supply_vehicle[v].next_depot_id = dis_idx[next_d];

                    // === 【集積所での情報共有処理】 ===
                    // 車両が回収済みの情報を全て集積所共有
                    for (int i = 0; i < info_count; i++)
                    {
                        // 運搬車両-> 集積所：車両が情報を回収済みで、それを保持して集積所に到着した場合、集積所がその情報を保持していないと
                        if (supply_vehicle[v].collect_info_id[i] == 1 && facilities[current_stop_idx[v]].collect_info_id[i] == 0)
                        {
                            facilities[current_stop_idx[v]].collect_info_id[i] = 1; // 集積所での情報収集フラグを設定
                        }
                        // 集積所 -> 運搬車両 (運搬車両によってまだ一度も回収されたことのない情報(他のTVにより避難所で回収済みだがこのTVが集積所で回収する場合も）)
                        else if (supply_vehicle[v].collect_info_id[i] == 0 && facilities[current_stop_idx[v]].collect_info_id[i] == 1 && info_list[i].responsible_TV_id != v && info_list[i].TV_inf_collect_at_depot == 0)
                        {
                            supply_vehicle[v].collect_info_id[i] = 1; // 運搬車両での情報収集フラグを設定

                            info_list[i].TV_collection_time = elapsed_time; // 情報リストに運搬車両回収時刻を設定

                            info_list[i].fast_detection_flag = 1; // 迅速発見フラグを設定

                            info_list[i].responsible_TV_id = v; // 最初に回収した運搬車両IDを設定

                            info_list[i].TV_inf_collect_at_depot = 1; // 集積所でTVに回収されたフラグを設定
                        }
                    }

                    // 物資運搬車両の集積所での需要量把握した上での積載処理
                    for (int i = 0; i < info_count; i++)
                    {
                        if (facilities[current_stop_idx[v]].collect_info_id[i] == 1 && supply_vehicle[v].demand_supply_loaded_flag[i] == 0 && info_list[i].responsible_TV_id == v) // 集積所に物資運搬車両またはドローンによって共有された情報に基づき、要求の物資を積載
                        {
                            supply_vehicle[v].demand_supply_loaded_flag[i] = 1; // 要求物資積載フラグをセット
                        }
                    }

                    // === 集積所での物資補充処理 ===
                    // 車両が空の状態の場合、新たに物資を満載する
                    if (!supply_vehicle[v].is_loaded)
                    {
                        supply_vehicle[v].remaining_supply_ga = TOTAL_SUPPLY_WEIGHT * SUPPLY_GA_RATIO; // 物資A
                        supply_vehicle[v].remaining_supply_gb = TOTAL_SUPPLY_WEIGHT * SUPPLY_GB_RATIO; // 物資B
                        supply_vehicle[v].remaining_extra_supply_b = EXTRA_SUPPLY_B;                   // 余剰物資
                        supply_vehicle[v].is_loaded = 1;                                               // 積載状態フラグを設定
                    }
                }
                else /***************** 避難所に停止する場合（current_stop_idx[] が０でないとき）******************************************/
                {
                    // === 【避難所での物資配送処理】 ===
                    // 車両が通常物資（A・B）を積載中で、両方とも残量がある場合のみ配送実行
                    if (supply_vehicle[v].is_loaded && supply_vehicle[v].remaining_supply_ga > 0 && supply_vehicle[v].remaining_supply_gb > 0)
                    {
                        // === 定量配送の実行 ===
                        facilities[current_stop_idx[v]].supply_ga += SUPPLY_GA_PER_SHELTER; // 避難所在庫増加
                        facilities[current_stop_idx[v]].supply_gb += SUPPLY_GB_PER_SHELTER;
                        // === TG（通常物資運搬間隔）統計の更新 ===
                        if (facilities[current_stop_idx[v]].supply_ga > 0) // 一巡回目は除外
                        {
                            facilities[current_stop_idx[v]].tg_total_time += (elapsed_time - facilities[current_stop_idx[v]].tg_last_time); // 通常物資運搬間隔TG
                            facilities[current_stop_idx[v]].tg_count += 1.0;
                            facilities[current_stop_idx[v]].tg_last_time = elapsed_time;
                        }
                        supply_vehicle[v].remaining_supply_ga -= SUPPLY_GA_PER_SHELTER;
                        supply_vehicle[v].remaining_supply_gb -= SUPPLY_GB_PER_SHELTER;

                        // === 車両積載状態の判定 ===
                        if (supply_vehicle[v].remaining_supply_ga <= 0 && supply_vehicle[v].remaining_supply_gb <= 0)
                        {
                            supply_vehicle[v].is_loaded = 0; // 空状態に変更
                        }
                    }

                    // === 【避難所での情報回収処理】 ===
                    for (int i = 0; i < info_count; i++)
                    {
                        // 対象条件: 未回収 かつ 現在の避難所で発生した情報
                        if (!info_list[i].collected && info_list[i].shelter_id == current_stop_idx[v] && supply_vehicle[v].remaining_extra_supply_b > 0)
                        {
                            // === 情報回収の実行 ===
                            info_list[i].collection_time = elapsed_time;      // 回収時刻を記録
                            info_list[i].TV_collection_time = elapsed_time;   // 回収時刻を記録（物資運搬車両のみ）
                            info_list[i].collected = 1;                       // 回収済みフラグを設定
                            info_list[i].collected_by = COLLECTED_BY_VEHICLE; // 車両による回収を記録
                            info_list[i].responsible_TV_id = v;

                            supply_vehicle[v].collect_info_id[i] = 1; // 車両の回収情報IDリストに登録

                            // === Tc（回収遅延時間）の計算と統計更新 ===
                            double tc = elapsed_time - info_list[i].generation_time;
                            total_tc += tc;    // 累積Tc（平均計算用）
                            collected_count++; // 回収件数カウント
                            // === 【車両による余剰物資A配送処理】 ===
                            if (supply_vehicle[v].remaining_extra_supply_b > 0 && !info_list[i].A_extra_delivery_completed && supply_vehicle[v].demand_supply_loaded_flag[i] == 1)
                            {
                                double delivery_amount = 0.0;

                                delivery_amount = (info_list[i].A_extra_supply_demand > supply_vehicle[v].remaining_extra_supply_b) ? supply_vehicle[v].remaining_extra_supply_b : info_list[i].A_extra_supply_demand;

                                // === 配送の実行 ===
                                if (delivery_amount > 0)
                                {
                                    facilities[current_stop_idx[v]].extra_supply_a += delivery_amount; // 避難所在庫増加
                                    supply_vehicle[v].remaining_extra_supply_b -= delivery_amount;
                                    info_list[i].A_extra_supply_delivered += delivery_amount;

                                    // === 車両による余剰物資A運搬統計の更新 ===
                                    total_extra_supply_by_vehicle += delivery_amount;
                                    vehicle_delivery_count++;

                                    if (info_list[i].A_extra_supply_delivered >= info_list[i].A_extra_supply_demand)
                                    {
                                        info_list[i].A_extra_delivery_completed = 1;
                                        info_list[i].A_extra_delivery_completion_time = elapsed_time;
                                    }
                                }
                            }
                        }
                    }

                    // == すでにドローンによって情報が回収されているが、物資Bの要求を知るための処理
                    for (int i = 0; i < info_count; i++)
                    {
                        // 対象条件: ドローンによって既に情報が取得されている、物資Ｌの要求量情報を取得
                        if (info_list[i].shelter_id == current_stop_idx[v] && supply_vehicle[v].remaining_extra_supply_b > 0 && info_list[i].collected_by == COLLECTED_BY_DRONE)
                        {
                            supply_vehicle[v].collect_info_id[i] = 1; // 車両の回収情報IDリストに登録

                            info_list[i].responsible_TV_id = v; //
                        }
                    }

                    // === 【避難所での余剰物資A配送処理】一度集積所で情報を回収し需要に合ったものを積載してきてから運搬（自分で回収した情報で需要に合ったものを積載も含まれる） ===
                    for (int i = 0; i < info_count; i++)
                    {
                        if (ND >= 0) // ドローンを導入しない場合は余剰物資Aも物資運搬車両が全て配送する、また、ドローンを導入する場合もドローンで運搬しきれない物資Bも、集積所で所望の物資を積載した上で運搬
                        {
                            // 対象条件: 現在の避難所で発生した情報, 要求物資積載フラグが立っている, かつ配送未完了,余剰物資あり，運搬未達成
                            if (info_list[i].shelter_id == current_stop_idx[v] && supply_vehicle[v].demand_supply_loaded_flag[i] == 1 && supply_vehicle[v].remaining_extra_supply_b > 0 && !info_list[i].A_extra_delivery_completed)
                            {
                                double delivery_amount = 0.0;

                                delivery_amount = (info_list[i].A_extra_supply_demand > supply_vehicle[v].remaining_extra_supply_b) ? supply_vehicle[v].remaining_extra_supply_b : info_list[i].A_extra_supply_demand;

                                // === 配送の実行 ===
                                if (delivery_amount > 0)
                                {
                                    facilities[current_stop_idx[v]].extra_supply_a += delivery_amount; // 避難所在庫増加
                                    supply_vehicle[v].remaining_extra_supply_b -= delivery_amount;
                                    info_list[i].A_extra_supply_delivered += delivery_amount;

                                    // === 車両による余剰物資A運搬統計の更新 ===
                                    total_extra_supply_by_vehicle += delivery_amount;
                                    vehicle_delivery_count++;

                                    if (info_list[i].A_extra_supply_delivered >= info_list[i].A_extra_supply_demand) // 運搬完了したら
                                    {
                                        info_list[i].A_extra_delivery_completed = 1;
                                        info_list[i].A_extra_delivery_completion_time = elapsed_time;
                                    }
                                }
                            }
                        }
                    }

                    // === 【避難所での余剰物資B配送処理】一度集積所で情報を回収し需要に合ったものを積載してきてから（自分で回収した情報で重要に合ったものを積載も含まれる） ===
                    for (int i = 0; i < info_count; i++)
                    {
                        // 対象条件: 現在の避難所で発生した情報, 要求物資積載フラグが立っている, かつ配送未完了,余剰物資あり
                        if (info_list[i].shelter_id == current_stop_idx[v] && supply_vehicle[v].demand_supply_loaded_flag[i] == 1 && supply_vehicle[v].remaining_extra_supply_b > 0 && !info_list[i].B_extra_delivery_completed)
                        {
                            double delivery_amount = 0.0;

                            delivery_amount = (info_list[i].B_extra_supply_demand > supply_vehicle[v].remaining_extra_supply_b) ? supply_vehicle[v].remaining_extra_supply_b : info_list[i].B_extra_supply_demand;

                            // === 配送の実行 ===
                            if (delivery_amount > 0)
                            {
                                facilities[current_stop_idx[v]].extra_supply_b += delivery_amount; // 避難所在庫増加
                                supply_vehicle[v].remaining_extra_supply_b -= delivery_amount;
                                info_list[i].B_extra_supply_delivered += delivery_amount;

                                if (info_list[i].B_extra_supply_delivered >= info_list[i].B_extra_supply_demand) // 運搬完了したら
                                {
                                    info_list[i].B_extra_delivery_completed = 1;
                                    info_list[i].B_extra_delivery_completion_time = elapsed_time;
                                    info_list[i].responsible_TV_id = -1;
                                }
                            }
                        }
                    }
                }
                //************************************* 物資運搬車両の物資運搬処理ここまで ********************************************************************/

                // === 車両停止時間中のドローン処理ループ ===
                // 停止期間中も時間を進めてドローンの位置更新と描画を実行
                // 集積所、避難所問わず、車両が停止時間中はドローンの状態更新を行う
                double stop_start_time = elapsed_time;
                double stop_end_time = 0.0;

                // 集積所に停止する場合
                if (is_depot) // current_stop_idx[] == 0
                {
                    stop_end_time = elapsed_time + T_STOP_DIS;
                }
                else // 避難所に停止する場合
                {
                    stop_end_time = elapsed_time + T_STOP; //
                }

                // 停止時間が終了するまでのループ
                while (elapsed_time < stop_end_time)
                {
                    // === 描画タイミングの判定 ===
                    // 指定間隔（2分）ごとに描画フレームを生成
                    if (elapsed_time >= next_draw_time)
                    {
                        if (v == NV - 1) // 最後の車両の処理のときのみ描画
                        {
                            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, facilities, supply_vehicle, dis_idx);
                            next_draw_time += DRAW_INTERVAL; // 次の描画時刻を更新
                        }
                    }

                    /************************************************ドローンの処理 車両移動中と同様の処理******************************************************************/
                    // === 時間ステップ進行 ===
                    double time_step = 1.0; // 1秒刻みで時間を進める
                    elapsed_time += time_step;

                    // === ドローン群の状態更新 ===
                    for (int i = 0; i < ND; i++)
                    {
                        if (v == NV - 1) // 車両[NV-1]の処理のときのみ処理（最後の車両の物資運搬処理が終わってから）
                        {
                            // ドローン出発判定（出発時刻に達したらアクティブ化）
                            if (!drones[i].active && elapsed_time >= drones[i].start_time)
                            {
                                drones[i].active = 1;
                                drones[i].last_state_time = elapsed_time; // 飛行時間統計の基準時刻を設定
                            }

                            // アクティブドローンの処理
                            if (drones[i].active)
                            {
                                // 情報検出チェック（通常巡回モードのみ）
                                if (drones[i].state == DRONE_PATROL)
                                {
                                    int detected_shelter = check_drone_info_detection(&drones[i], stop_coords, info_list, info_count, elapsed_time, dis_idx);
                                    if (detected_shelter > 0) // 情報を検出した場合
                                    {
                                        // 情報を検出、物資運搬モードに移行
                                        drones[i].target_shelter = detected_shelter;
                                        drones[i].current_trip = 1;

                                        // ドローンの向かう集積所の決定
                                        int target_depot_id;
                                        if (DRONE_TO_DEPOT_OPTION == 1)
                                        {
                                            target_depot_id = find_nearest_depot(drones, i, stop_coords, dis_idx, NDI);
                                        }
                                        else if (DRONE_TO_DEPOT_OPTION == 2)
                                        {
                                            if (NV == 1)
                                            {
                                                target_depot_id = find_vehicle_next_depot_V_1(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, stop_end_time, 1, is_depot, elapsed_time);
                                            }
                                            else
                                            {
                                                target_depot_id = find_vehicle_next_depot(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, stop_end_time, 1, is_depot, elapsed_time);
                                            }
                                        }

                                        drones[i].target_depot = target_depot_id;             // 最寄り集積所IDを設定
                                        drones[i].target_x = stop_coords[target_depot_id][0]; // 最寄り集積所座標を目的として設定
                                        drones[i].target_y = stop_coords[target_depot_id][1];
                                        update_drone_flight_time(&drones[i], elapsed_time, DRONE_TO_DEPOT);

                                        // ドローンによる情報回収処理（検出と同時に回収）
                                        for (int j = 0; j < info_count; j++)
                                        {
                                            if (!info_list[j].collected &&
                                                info_list[j].shelter_id == detected_shelter &&
                                                info_list[j].generation_time <= elapsed_time)
                                            {
                                                drones[i].collect_info_id[j] = 1; // ドローンの回収情報IDリストに登録

                                                info_list[j].collected = 1;
                                                info_list[j].collection_time = elapsed_time;    // ドローン回収時刻を記録
                                                info_list[j].collected_by = COLLECTED_BY_DRONE; // ドローンによる回収を記録
                                                drones[i].required_trips = calculate_required_trips(info_list[j].A_extra_supply_demand);

                                                // 運搬予定物資量を設定
                                                if (info_list[j].A_extra_supply_demand < DRONE_MAX_CARRY)
                                                {
                                                    drones[i].supply_scheduled_amount = info_list[j].A_extra_supply_demand; // 運搬予定物資量を設定
                                                }
                                                else
                                                {
                                                    drones[i].supply_scheduled_amount = DRONE_MAX_CARRY; // 運搬予定物資量を設定（上限30kg）
                                                }
                                                break; // 1つの情報のみ回収
                                            }
                                        }
                                    }
                                    else // 情報を検出しなかった場合
                                    {
                                        if (DELIVERY_METHOD == DELIVERY_METHOD_MULTI_DRONE) // 手法3のとき
                                        {
                                            // 手法3: 協調運搬の判定
                                            int cooperative_shelter = check_drone_cooperative_transport(&drones[i], drones, ND, stop_coords, info_list, info_count, dis_idx);
                                            if (cooperative_shelter > 0)
                                            {
                                                drones[i].collect_info_id[drones[i].delivery_info_index] = 1; // ドローンの回収情報IDリストに登録

                                                info_list[drones[i].delivery_info_index].collected_by_drone_later = 1;               // ドローンによる後続運搬フラグを設定
                                                info_list[drones[i].delivery_info_index].drone_collection_time_later = elapsed_time; // ドローンが後に回収した時刻を設定

                                                drones[i].target_shelter = cooperative_shelter;
                                                drones[i].current_trip = 1;

                                                // ドローンの向かう集積所の決定
                                                int target_depot_id;
                                                if (DRONE_TO_DEPOT_OPTION == 1)
                                                {
                                                    target_depot_id = find_nearest_depot(drones, i, stop_coords, dis_idx, NDI);
                                                }
                                                else if (DRONE_TO_DEPOT_OPTION == 2)
                                                {
                                                    if (NV == 1)
                                                    {
                                                        target_depot_id = find_vehicle_next_depot_V_1(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, stop_end_time, 1, is_depot, elapsed_time);
                                                    }
                                                    else
                                                    {
                                                        target_depot_id = find_vehicle_next_depot(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, stop_end_time, 1, is_depot, elapsed_time);
                                                    }
                                                }

                                                drones[i].target_depot = target_depot_id;             // 最寄り集積所IDを設定
                                                drones[i].target_x = stop_coords[target_depot_id][0]; // 最寄り集積所座標を目的として設定
                                                drones[i].target_y = stop_coords[target_depot_id][1];
                                                // 近い集積所へ飛行する処理追加？
                                                update_drone_flight_time(&drones[i], elapsed_time, DRONE_TO_DEPOT);

                                                // 協調運搬の必要往復回数を計算
                                                for (int j = 0; j < info_count; j++)
                                                {
                                                    if (info_list[j].shelter_id == cooperative_shelter && !info_list[j].A_extra_delivery_completed)
                                                    {
                                                        double remaining_demand = info_list[j].A_extra_supply_demand - info_list[j].A_extra_supply_delivered;
                                                        double other_drones_transport = calculate_all_drones_transport_amount(&drones[i], drones, ND, cooperative_shelter, info_list, info_count);
                                                        double this_drone_responsibility = remaining_demand - other_drones_transport;                                                    // 他のドローンが運搬中の物資量を考慮して、自身が運搬する物資量を計算
                                                        drones[i].supply_scheduled_amount = (this_drone_responsibility > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : this_drone_responsibility; // 運搬予定物資量を設定

                                                        if (this_drone_responsibility > 0)
                                                        {
                                                            drones[i].required_trips = calculate_required_trips(this_drone_responsibility);
                                                        }
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                //== ドローンが巡回中に集積所を検出した場合は、その集積所とドローンの持つ情報リストを交換
                                int detected_depot = check_drone_depot_detection(&drones[i], stop_coords, dis_idx);
                                if (detected_depot > 0) // 集積所を検出した場合
                                {
                                    // ドローンが持つ情報リストを集積所に伝達
                                    for (int j = 0; j < info_count; j++)
                                    {
                                        // /ドローン -> 集積所
                                        if (drones[i].collect_info_id[j] == 1 && facilities[detected_depot].collect_info_id[j] == 0)
                                        {
                                            facilities[detected_depot].collect_info_id[j] = 1; // 集積所の回収情報リストを更新
                                        }
                                        // 集積所 -> ドローン
                                        else if (drones[i].collect_info_id[j] == 0 && facilities[detected_depot].collect_info_id[j] == 1)
                                        {
                                            drones[i].collect_info_id[j] = 1; // ドローンの回収情報IDリストを更新
                                        }
                                    }
                                }

                                // ドローンの状態更新
                                update_drone_state(&drones[i], drones, ND, stop_coords, elapsed_time, time_step, info_list, info_count, facilities, &total_extra_supply_by_drone, &drone_delivery_count, i, dis_idx, supply_vehicle, current_stop_idx, NULL, current_x, current_y);
                            }
                        }
                    }
                    /******************************************************************************************************************/

                    // 集積所に停止する場合
                    if (is_depot) // current_stop_idx[] == 0など集積所インデックスと一致したとき
                    {
                        // == 物資運搬車両の集積所での停止時間中の集積所との情報交換処理（停止中にあとからドローンが集積所にやってきた場合も）
                        for (int i = 0; i < info_count; i++)
                        {
                            // 集積所 -> 運搬車両
                            if (supply_vehicle[v].collect_info_id[i] == 0 && facilities[current_stop_idx[v]].collect_info_id[i] == 1 && info_list[i].responsible_TV_id != v && info_list[i].TV_inf_collect_at_depot == 0)
                            {
                                supply_vehicle[v].collect_info_id[i] = 1; // 運搬車両での情報収集フラグを設定

                                info_list[i].TV_collection_time = elapsed_time; // 情報リストに運搬車両回収時刻を設定

                                info_list[i].fast_detection_flag = 1; // 迅速発見フラグを設定

                                info_list[i].responsible_TV_id = v; // 最初に回収した運搬車両IDを設定

                                info_list[i].TV_inf_collect_at_depot = 1; // 集積所でTVに回収されたフラグを設定
                            }
                        }
                        // 物資運搬車両の集積所での需要量把握した上での積載処理
                        for (int i = 0; i < info_count; i++)
                        {
                            if (facilities[current_stop_idx[v]].collect_info_id[i] == 1 && supply_vehicle[v].demand_supply_loaded_flag[i] == 0 && info_list[i].responsible_TV_id == v) // 集積所に物資運搬車両またはドローンによって共有された情報に基づき、要求の物資を積載
                            {
                                supply_vehicle[v].demand_supply_loaded_flag[i] = 1; // 要求物資積載フラグをセット
                            }
                        }
                    }
                }

                // 停止時間終了時の正確な時刻調整
                elapsed_time = stop_end_time;

                if (NV != 1 && v != NV - 1)
                {
                    elapsed_time -= T_STOP; // 最後の車両以外は避難所停止時間分を戻し、次車両の処理へ
                }
            }
        }
        else // 初回出発時の処理
        {
            // === 初回出発時の処理 ===
            plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, facilities, supply_vehicle, dis_idx);
            next_draw_time = DRAW_INTERVAL; // 最初の描画後、次の描画時刻を設定
            is_first_departure = 0;         // 初回出発フラグをリセット
        }

        // === 【次の目的地の決定】 ===
        for (int v = 0; v < NV; v++)
        {
            next_stop_idx[v] = (current_stop_idx[v] + 1) % TOTAL_STOPS;
        }

        // === 【周回完了判定】 ===
        if (next_stop_idx[0] == 0)
        {
            lap_count++; // 完了周回数を増加
        }

        // 移動開始のログ出力
        printf("移動開始: 拠点%d → 拠点%d\n", current_stop_idx[0], next_stop_idx[0]);

        // === 【移動中の時間管理】 ===
        double move_start_time = elapsed_time;                         // 移動開始時刻
        double move_end_time = elapsed_time + travel_time_per_segment; // 移動完了予定時刻

        // === 【車両の移動角度計算】 ===
        double start_angle[NV]; // 各車両の出発地点の角度
        double end_angle[NV];   // 各車両の到着地点の角度
        double angle_diff[NV];  // 各車両の角度差

        // 全車両の角度計算をfor文で処理
        for (int v = 0; v < NV; v++)
        {
            start_angle[v] = PI / 2.0 - current_stop_idx[v] * angle_increment; // 出発地点の角度
            end_angle[v] = PI / 2.0 - next_stop_idx[v] * angle_increment;      // 到着地点の角度
        }

        // === 【角度差の正規化】（最短経路選択） ===
        for (int v = 0; v < NV; v++)
        {
            angle_diff[v] = end_angle[v] - start_angle[v];
            if (angle_diff[v] > PI)
            {
                angle_diff[v] -= 2.0 * PI; // 360度減算で時計回り短縮
            }
            else if (angle_diff[v] < -PI)
            {
                angle_diff[v] += 2.0 * PI; // 360度加算で反時計回り短縮
            }
        }

        // === 車両移動中の時間ループ ===
        while (elapsed_time < move_end_time)
        {
            // === 車両位置の計算 ===
            // 移動の進捗度を計算（0.0〜1.0の範囲）
            double progress = (elapsed_time - move_start_time) / travel_time_per_segment;
            if (progress > 1.0)
                progress = 1.0; // 進捗度の上限制限

            // 車両の現在角度を線形補間で計算
            double current_angle[NV]; // 各車両の現在角度

            // 全車両の位置計算をfor文で処理
            for (int v = 0; v < NV; v++)
            {
                current_angle[v] = start_angle[v] + angle_diff[v] * progress;
                current_x[v] = R * cos(current_angle[v]); // 現在のX座標
                current_y[v] = R * sin(current_angle[v]); // 現在のY座標
            }

            // === 描画タイミングの判定 ===
            if (elapsed_time >= next_draw_time)
            {
                plot_frame(gnuplot_pipe, stop_coords, current_x, current_y, drones, elapsed_time, info_list, info_count, facilities, supply_vehicle, dis_idx);
                next_draw_time += DRAW_INTERVAL; // 次の描画時刻を更新
            }

            /************************************************* 車両停止中と同様の処理 *****************************************************************/
            // === 【時間ステップ進行】 ===
            double time_step = 1.0; // 1秒ステップ（バランスの取れた精度と計算速度）
            elapsed_time += time_step;

            // === 【ドローン群の状態更新】（移動中も継続的に処理） ===
            for (int i = 0; i < ND; i++)
            {
                // === ドローン出発判定 ===
                if (!drones[i].active && elapsed_time >= drones[i].start_time)
                {
                    drones[i].active = 1;                     // アクティブ状態に変更
                    drones[i].last_state_time = elapsed_time; // 飛行時間統計の基準時刻を設定
                }

                // === アクティブドローンの処理 ===
                if (drones[i].active)
                {
                    // === 情報検出処理（通常巡回モードのみ） ===
                    if (drones[i].state == DRONE_PATROL)
                    {
                        // 検出半径以内の避難所で未回収情報があるかチェック
                        int detected_shelter = check_drone_info_detection(&drones[i], stop_coords, info_list, info_count, elapsed_time, dis_idx);

                        if (detected_shelter > 0) // 情報を検出した場合
                        {
                            // === 物資運搬モードへの移行 ===
                            // 情報検出と同時に、ドローンを物資運搬モードに切り替え
                            drones[i].target_shelter = detected_shelter; // 配送先避難所を設定
                            drones[i].current_trip = 1;                  // 往復回数カウンタを初期化

                            // ドローンの向かう集積所の決定
                            int target_depot_id;
                            if (DRONE_TO_DEPOT_OPTION == 1)
                            {
                                target_depot_id = find_nearest_depot(drones, i, stop_coords, dis_idx, NDI);
                            }
                            else if (DRONE_TO_DEPOT_OPTION == 2)
                            {
                                if (NV == 1)
                                {
                                    target_depot_id = find_vehicle_next_depot_V_1(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, 0.0, -1, -1, elapsed_time);
                                }
                                else
                                {
                                    target_depot_id = find_vehicle_next_depot(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, 0.0, -1, -1, elapsed_time);
                                }
                            }

                            drones[i].target_depot = target_depot_id;             // 最寄り集積所IDを設定
                            drones[i].target_x = stop_coords[target_depot_id][0]; // 最寄り集積所座標を目的として設定
                            drones[i].target_y = stop_coords[target_depot_id][1];
                            update_drone_flight_time(&drones[i], elapsed_time, DRONE_TO_DEPOT); // 状態変更と飛行時間記録

                            // === ドローンによる情報回収処理（検出と同時に回収） ===
                            for (int j = 0; j < info_count; j++)
                            {
                                // 対象条件: 未回収 & 検出避難所 & 既発生済み
                                if (!info_list[j].collected &&
                                    info_list[j].shelter_id == detected_shelter &&
                                    info_list[j].generation_time <= elapsed_time)
                                {
                                    drones[i].collect_info_id[j] = 1; // ドローンの回収情報IDリストに登録

                                    info_list[j].collected = 1;                     // 回収済みフラグを設定
                                    info_list[j].collection_time = elapsed_time;    // ドローン回収時刻を記録
                                    info_list[j].collected_by = COLLECTED_BY_DRONE; // ドローンによる回収を記録

                                    // === 必要往復回数の計算 ===
                                    drones[i].required_trips = calculate_required_trips(info_list[j].A_extra_supply_demand);

                                    if (info_list[j].A_extra_supply_demand < DRONE_MAX_CARRY)
                                    {
                                        drones[i].supply_scheduled_amount = info_list[j].A_extra_supply_demand; // 運搬予定物資量を設定
                                    }
                                    else
                                    {
                                        drones[i].supply_scheduled_amount = DRONE_MAX_CARRY; // 運搬予定物資量を設定（上限30kg）
                                    }
                                    break; // 1回の検出で1つの情報のみ処理
                                }
                            }
                        }
                        else // 情報を検出しなかった場合
                        {
                            if (DELIVERY_METHOD == DELIVERY_METHOD_MULTI_DRONE) // 手法3のとき
                            {
                                // 手法3: 協調運搬の判定
                                int cooperative_shelter = check_drone_cooperative_transport(&drones[i], drones, ND, stop_coords, info_list, info_count, dis_idx);
                                if (cooperative_shelter > 0)
                                {

                                    drones[i].collect_info_id[drones[i].delivery_info_index] = 1; // ドローンの回収情報IDリストに登録

                                    //  ETc_dro(先に運搬車両が回収済みであとからドローンがやってきた場合)
                                    info_list[drones[i].delivery_info_index].collected_by_drone_later = 1;               // ドローンによる後続運搬フラグを設定
                                    info_list[drones[i].delivery_info_index].drone_collection_time_later = elapsed_time; // ドローンが後に回収した時刻を設定

                                    // 協調運搬モードに移行
                                    drones[i].target_shelter = cooperative_shelter;
                                    drones[i].current_trip = 1;

                                    // ドローンの向かう集積所の決定
                                    int target_depot_id;
                                    if (DRONE_TO_DEPOT_OPTION == 1)
                                    {
                                        target_depot_id = find_nearest_depot(drones, i, stop_coords, dis_idx, NDI);
                                    }
                                    else if (DRONE_TO_DEPOT_OPTION == 2)
                                    {
                                        if (NV == 1)
                                        {
                                            target_depot_id = find_vehicle_next_depot_V_1(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, 0.0, -1, -1, elapsed_time);
                                        }
                                        else
                                        {
                                            target_depot_id = find_vehicle_next_depot(supply_vehicle, current_stop_idx, next_stop_idx, drones, i, stop_coords, dis_idx, NDI, current_x, current_y, 0.0, -1, -1, elapsed_time);
                                        }
                                    }

                                    drones[i].target_depot = target_depot_id;             // 最寄り集積所IDを設定
                                    drones[i].target_x = stop_coords[target_depot_id][0]; // 最寄り集積所座標を目的として設定
                                    drones[i].target_y = stop_coords[target_depot_id][1];
                                    update_drone_flight_time(&drones[i], elapsed_time, DRONE_TO_DEPOT);

                                    // 協調運搬の必要往復回数を計算
                                    for (int j = 0; j < info_count; j++)
                                    {
                                        if (info_list[j].shelter_id == cooperative_shelter && !info_list[j].A_extra_delivery_completed)
                                        {
                                            double remaining_demand = info_list[j].A_extra_supply_demand - info_list[j].A_extra_supply_delivered;
                                            double other_drones_transport = calculate_all_drones_transport_amount(&drones[i], drones, ND, cooperative_shelter, info_list, info_count);
                                            double this_drone_responsibility = remaining_demand - other_drones_transport;
                                            drones[i].supply_scheduled_amount = (this_drone_responsibility > DRONE_MAX_CARRY) ? DRONE_MAX_CARRY : this_drone_responsibility; // 運搬予定物資量を設定

                                            if (this_drone_responsibility > 0)
                                            {
                                                drones[i].required_trips = calculate_required_trips(this_drone_responsibility);
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    //== ドローンが巡回中に集積所を検出した場合は、その集積所とドローンの持つ情報リストを交換
                    int detected_depot = check_drone_depot_detection(&drones[i], stop_coords, dis_idx);
                    if (detected_depot > 0) // 集積所を検出した場合
                    {
                        // ドローンが持つ情報リストを集積所に伝達
                        for (int j = 0; j < info_count; j++)
                        {
                            // ドローン -> 集積所
                            if (drones[i].collect_info_id[j] == 1 && facilities[detected_depot].collect_info_id[j] == 0)
                            {
                                facilities[detected_depot].collect_info_id[j] = 1; // 集積所の回収情報リストを更新
                            }
                            // 集積所 -> ドローン
                            else if (drones[i].collect_info_id[j] == 0 && facilities[detected_depot].collect_info_id[j] == 1)
                            {
                                drones[i].collect_info_id[j] = 1; // ドローンの回収情報IDリストを更新
                            }
                        }
                    }

                    // === ドローンの状態更新 ===
                    update_drone_state(&drones[i], drones, ND, stop_coords, elapsed_time, time_step, info_list, info_count, facilities, &total_extra_supply_by_drone, &drone_delivery_count, i, dis_idx, supply_vehicle, current_stop_idx, next_stop_idx, current_x, current_y);
                }
            }
            /******************************************************************************************************************/
        }

        // === 移動完了時の正確な位置調整 ===
        elapsed_time = move_end_time;

        // === 次の停止地点への移動 ===
        for (int v = 0; v < NV; v++)
        {
            current_stop_idx[v] = next_stop_idx[v];
        }
    }
    return 0;
}
