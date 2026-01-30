#ifndef SYURONYOU_HEADER_H
#define SYURONYOU_HEADER_H

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
#define T_STOP_DIS (30 * 60)
#define DETECTION_RADIUS 10.0 // ドローンの検出半径 (m) | 避難所近傍での情報検出・協調運搬判定用

// === シミュレーション設定 ===
#define NS 12              // 避難所の数（集積所除く）
#define NDI 4              // 集積所の数(NV：物資運搬車両の台数と同じにする)
#define NV 1               // 車両の一台
#define NT 3               // シミュレーションの周回数
int ND = 0;                // ドローンの台数（0の場合はドローンなし、最大制限なし）
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

#endif // SYURONYOU_HEADER_H
