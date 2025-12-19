#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Syuronyou_header.h"

/************* メイン関数 ******/
int main(void)
{
    // === 初期化処理 ===
    srand(22); // 固定シードで再現性を確保

    // === gnuplotパイプの初期化 ===
    FILE *gnuplot_pipe = NULL;
    if (ENABLE_GIF)
    {
        gnuplot_pipe = init_gnuplot(); // GIFアニメーション用のgnuplotパイプを開く
    }
    else
    {
        printf("GIF出力が無効です。\n");
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

    // === 車両配列の初期化（メインループ前） ===
    // 全車両を集積所から開始
    for (int v = 0; v < NV; v++)
    {
        current_stop_idx[v] = dis_idx[v]; // 全車両を集積所から開始

        next_stop_idx[v] = (current_stop_idx[v] + 1) % TOTAL_STOPS; // 次の停止地点を設定
    }

    /********************** 【メインシミュレーションループ開始】 ****************/
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
                else /****** 避難所に停止する場合（current_stop_idx[] が０でないとき）*****/
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
                //***** 物資運搬車両の物資運搬処理ここまで ****/

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

                    /****ドローンの処理 車両移動中と同様の処理***/
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
                    /*********************************/

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

            /***** 車両停止中と同様の処理 *******/
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
                                // 協調運搬の判定
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
            /***************/
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