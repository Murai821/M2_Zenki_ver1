#include "Syuronyou_header.h"

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