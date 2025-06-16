#define _USE_MATH_DEFINES
#include "header.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define M_PI 3.14159265358979323846 // π の定義

/*配列において、ある値のインデックスを返す関数*/
int search_index(int arry[], int num)
{
    int n = 0;
    while (1)
    {
        if (arry[n] == num)
        {
            break;
        }
        n++;
    }

    return n;
}

/***************ダイクストラ法を行う関数の定義*************************/
/******ダイクストラ法********/
/*ダイクストラ法で用いる配列の宣言*/
double COST[N]; /*距離*/
int VIA[N];     /*経由点*/
char USED[N];   /*確定か未確定か*/

double dijkstra(int start, int goal, double weight[][N], int numline)
{

    // 初期化
    for (int i = 0; i < N; i++)
    {
        COST[i] = INF;
        USED[i] = FALSE;
        VIA[i] = -1;
    }

    int target;
    double min, result;

    COST[start] = 0;

    while (1)
    {
        /*未確定の中から距離が最も小さい地点を選び, その距離をその地点の最小距離として確定する*/
        min = INF;
        for (int i = 0; i < N; i++)
        {
            if (!USED[i] && min > COST[i])
            {
                min = COST[i];
                target = i;
            }
        }

        /*すべての地点の最短経路が確定*/
        if (target == goal)
        {
            result = COST[goal];
            return result;
        }

        /*今確定した場所から「直接つながっている」かつ「未確定」の地点に関して、今確定した場所を経由した場合の距離を計算し、今までの距離よりも小さければ書き直す*/
        for (int neighboring = 0; neighboring < N; neighboring++)
        {
            if (COST[neighboring] > weight[target][neighboring] + COST[target])
            {
                COST[neighboring] = weight[target][neighboring] + COST[target];
                VIA[neighboring] = target;
            }
        }
        USED[target] = TRUE;
    }
}

// RFCS法を行う関数
void RFCS_method(double weight[][N], int numline, int result_arry[], int *result_arry_size)
{
    double x[N][N] = {0};
    double y[N][N] = {0};
    double out = 0;

    // それぞれのi,jに対してダイクストラ関数を呼び出し値を格納 Xi,j
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (i != j)
            {
                out = dijkstra(i, j, weight, N);
                x[i][j] = out;
            }
        }
    }

    /****************** y_i,jの定義 ******************/

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (i >= j)
            {
                y[i][j] = INF;
            }
            else if (j - i > ceil((N - 1) / M))
            {
                y[i][j] = INF;
            }
            else
            {
                y[i][j] = cost_sum(i, j, x, N);
            }
        }
    }

    /****M個の回線に分割するため処理：y_ijにおいてa0からaNまでの最短経路を求める****/
    int start_point = 0;
    int goal_point = N - 1;

    int *path;       // サブ配列に追加する配列
    int path_length; // 追加する配列のサイズ

    double result = dijkstraVIA(start_point, goal_point, y, N, &path, &path_length);

    for (int i = 0; i < path_length; i++)
    {
        result_arry[i] = path[i]; // result_arryに結果を格納
    }

    *result_arry_size = path_length; // result_arryの要素数を格納
}

// ダイクストラ法により導出したスタート地点からゴール地点までの経由地点を格納して返す関数
double dijkstraVIA(int start, int goal, double weight[][N], int numline, int **path, int *path_length)
{
    int target;
    double min, result;

    // 初期化
    for (int i = 0; i < N; i++)
    {
        COST[i] = INF;
        USED[i] = FALSE;
        VIA[i] = -1;
    }
    COST[start] = 0;

    while (1)
    {
        // 未確定の中から距離が最も小さい地点を選び、確定する
        min = INF;
        target = -1;
        for (int i = 0; i < N; i++)
        {
            if (!USED[i] && min > COST[i])
            {
                min = COST[i];
                target = i;
            }
        }

        // すべての地点の最短経路が確定
        if (target == -1 || target == goal)
        {
            result = COST[goal];
            break;
        }

        // 経由地点を更新
        for (int neighboring = 0; neighboring < N; neighboring++)
        {
            if (weight[target][neighboring] != INF && COST[neighboring] > weight[target][neighboring] + COST[target])
            {
                COST[neighboring] = weight[target][neighboring] + COST[target];
                VIA[neighboring] = target;
            }
        }
        USED[target] = TRUE;
    }

    // 経路を動的に格納
    int current = goal;
    int count = 1; // スタート地点を含むので初期値を1に設定

    // 経由点数を数える (スタート地点を含む)
    while (current != start)
    {
        count++;
        current = VIA[current];
    }

    *path = (int *)malloc(count * sizeof(int));
    *path_length = count;

    current = goal;
    for (int i = count - 1; i >= 0; i--)
    {
        (*path)[i] = current;
        current = VIA[current];
    }
    (*path)[0] = start; // 最後にスタート地点を追加

    return result;
}

// ダイクストラ法を用いて、配列の後ろに追加する経由地点を導出する関数（追加する経由地点は：スタート地点を除いたゴール地点までの経由地点）
double dijkstraArrayAdd(int start, int goal, double weight[][N], int numline, int **path, int *path_length)
{
    int target;
    double min, result;

    // 初期化
    for (int i = 0; i < N; i++)
    {
        COST[i] = INF;
        USED[i] = FALSE;
        VIA[i] = -1;
    }
    COST[start] = 0;

    while (1)
    {
        // 未確定の中から距離が最も小さい地点を選び、確定する
        min = INF;
        for (int i = 0; i < N; i++)
        {
            if (!USED[i] && min > COST[i])
            {
                min = COST[i];
                target = i;
            }
        }

        // すべての地点の最短経路が確定
        if (target == goal)
        {
            result = COST[goal];
            break;
        }

        // 経由地点を更新
        for (int neighboring = 0; neighboring < N; neighboring++)
        {
            if (weight[target][neighboring] != INF && COST[neighboring] > weight[target][neighboring] + COST[target])
            {
                COST[neighboring] = weight[target][neighboring] + COST[target];
                VIA[neighboring] = target;
            }
        }
        USED[target] = TRUE;
    }

    // 経路を動的に格納
    int current = goal;
    int count = 0;

    // 経由点数を数える (スタート地点を含まない)
    while (current != start)
    {
        count++;
        current = VIA[current];
    }

    *path = (int *)malloc(count * sizeof(int));
    *path_length = count;

    current = goal;
    for (int i = count - 1; i >= 0; i--)
    {
        (*path)[i] = current;
        current = VIA[current];
    }

    return result;
}

double dijkstraArrayAppend(int start, int goal, double weight[N][N], int numline, int **path, int *path_length)
{
    int target;
    double min, result;

    // 初期化
    for (int i = 0; i < N; i++)
    {
        COST[i] = INF;
        USED[i] = FALSE;
        VIA[i] = -1;
    }
    COST[start] = 0;

    while (1)
    {
        // 未確定の中から距離が最も小さい地点を選び、確定する
        min = INF;
        for (int i = 0; i < N; i++)
        {
            if (!USED[i] && min > COST[i])
            {
                min = COST[i];
                target = i;
            }
        }

        // すべての地点の最短経路が確定
        if (target == goal)
        {
            result = COST[goal];
            break;
        }

        // 経由地点を更新
        for (int neighboring = 0; neighboring < N; neighboring++)
        {
            if (weight[target][neighboring] != INF && COST[neighboring] > weight[target][neighboring] + COST[target])
            {
                COST[neighboring] = weight[target][neighboring] + COST[target];
                VIA[neighboring] = target;
            }
        }
        USED[target] = TRUE;
    }

    // 経路を動的に格納
    int current = goal;
    int count = 0;

    // 経由点数を数える (スタート地点を含みゴール地点を含まない)
    while (current != start)
    {
        count++;
        current = VIA[current];
    }

    *path = (int *)malloc(count * sizeof(int));
    *path_length = count;

    current = goal;
    for (int i = count - 1; i >= 0; i--)
    {
        current = VIA[current];
        (*path)[i] = current;
    }

    return result;
}

/*RFCS法の「その他」におけるコストを求める関数*/
double cost_sum(int i, int j, double weight[][N], int numline)
{
    double cost = 0; // 総コスト

    cost += weight[0][i + 1] + weight[j][0];

    for (int k = i + 1; k <= j - 1; k++)
    {
        cost += weight[k][k + 1];
    }

    cost += CONST;

    return cost;
}

/*(0,1)の一様分布に従う乱数の生成関数*/
double Uniform(void)
{
    return ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
}

/*密度λ(lamda)の指数分布に従った乱数の生成関数*/
double rand_exp(double lambda)
{
    return -log(Uniform()) / lambda;
}

/*配列内の最小値を返す関数*/
double findMin(double arr[], int size)
{

    double min = arr[0]; // 配列の最初の要素を初期最小値として設定

    for (int i = 1; i < size; i++)
    {
        if (arr[i] < min)
        {
            min = arr[i]; // より小さい値が見つかった場合、最小値を更新
        }
    }

    return min;
}

/*配列内の最大値を返す関数*/
double findMax(double arr[], int size)
{

    double max = arr[0]; // 配列の最初の要素を初期最大値として設定

    for (int i = 1; i < size; i++)
    {
        if (arr[i] > max)
        {
            max = arr[i]; // より大きい値が見つかった場合、最大値を更新
        }
    }

    return max;
}

// ドローンと配送車の合流点を導出する関数
int solveSystem(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y)
{
    double a, b, c, d, e, f, g, h;
    double alpha, beta, gamma;
    double discriminant;
    double x1, y1;
    double x2, y2;
    double tan;
    double d_v_s; // 配送車-避難所間の距離
    double d_d_s; // ドローン-配送車間の距離
    tan = (xs - xv) / (ys - yv);
    int answer1 = TRUE; // x1,y1がただしい解であれば

    d_v_s = sqrt(pow(xs - xv, 2) + pow(ys - yv, 2)); // 避難所-配送車間
    d_d_s = sqrt(pow(xs - xd, 2) + pow(ys - yd, 2)); // ドローン-避難所間

    if (d_v_s < d_d_s / (vd / vv))
    {
        return -1; // ドローンが追いつけない
    }

    a = 1;
    b = -tan;
    c = -xv + tan * yv;
    d = vd * vd - vv * vv;
    e = -2 * (vd * vd * xv - vv * vv * xd);
    f = vd * vd - vv * vv;
    g = -2 * (vd * vd * yv - vv * vv * yd);
    h = vd * vd * xv * xv - vv * vv * xd * xd + vd * vd * yv * yv - vv * vv * yd * yd;

    alpha = d + ((a * a * f) / (b * b));
    beta = ((2 * a * c * f) / (b * b)) + e - ((a * g) / b);
    gamma = ((c * c * f) / (b * b)) + h - ((g * c) / b);

    discriminant = beta * beta - 4 * alpha * gamma; // 判別式

    if (discriminant < 0 || fabs(alpha) < 0.001) // D<0とゼロ除算
    {
        return -2; // 解がないエラー処理
    }
    else if (fabs(b) < 0.001)
    {
        return -2; // 解がないエラー処理
    }
    else if (beta > 0) // 解が一つ
    {
        x1 = (-beta + sqrt(discriminant)) / (2 * alpha);
        y1 = -(a * x1 + c) / b;

        if (xs > xv && ys > yv && (x1 < xv || x1 > xs))
        {
            return -2; // 解がないエラー処理
        }
        else if (xs > xv && ys < yv && (x1 < xv || x1 > xs))
        {
            return -2; // 解がないエラー処理
        }
        else if (xs < xv && ys > yv && (x1 < xs || x1 > xv))
        {
            return -2; // 解がないエラー処理
        }
        else if (xs < xv && ys < yv && (x1 < xs || x1 > xv))
        {
            return -2; // 解がないエラー処理
        }
        else
        {
            *x = x1;
            *y = y1;
        }
    }
    else if (beta < 0)
    {
        x1 = (-beta + sqrt(discriminant)) / (2 * alpha);
        y1 = -(a * x1 + c) / b;

        x2 = (-beta - sqrt(discriminant)) / (2 * alpha);
        y2 = -(a * x2 + c) / b;

        if (xs > xv && ys > yv && (x1 < xv || x1 > xs))
        {
            answer1 = FALSE; // x1,y1は不適
        }
        else if (xs > xv && ys < yv && (x1 < xv || x1 > xs))
        {
            answer1 = FALSE; // x1,y1は不適
        }
        else if (xs < xv && ys > yv && (x1 < xs || x1 > xv))
        {
            answer1 = FALSE; // x1,y1は不適
        }
        else if (xs < xv && ys < yv && (x1 < xs || x1 > xv))
        {
            answer1 = FALSE; // x1,y1は不適
        }
        else if (answer1 == TRUE) // x1,y1が適当なら
        {
            *x = x1;
            *y = y1;
        }

        if (answer1 == FALSE)
        {
            if (xs > xv && ys > yv && (x2 < xv || x2 > xs))
            {
                return -2; // 解がないエラー処理
            }
            else if (xs > xv && ys < yv && (x2 < xv || x2 > xs))
            {
                return -2; // 解がないエラー処理
            }
            else if (xs < xv && ys > yv && (x2 < xs || x2 > xv))
            {
                return -2; // 解がないエラー処理
            }
            else if (xs < xv && ys < yv && (x2 < xs || x2 > xv))
            {
                return -2; // 解がないエラー処理
            }
            else
            {
                *x = x2;
                *y = y2;
            }
        }
    }
}

// ２点間のcosを返す関数
double retCos(double xt, double yt, double xc, double yc)
{
    double d, cos;
    d = sqrt(pow(xt - xc, 2) + pow(yt - yc, 2));
    cos = (yt - yc) / d;
    return cos;
}

// ２点間のsinを返す関数
double retSin(double xt, double yt, double xc, double yc)
{
    double d, sin;
    d = sqrt(pow(xt - xc, 2) + pow(yt - yc, 2));
    sin = (xt - xc) / d;
    return sin;
}

/*２点間の距離を返す関数*/
double retDis(double xt, double yt, double xc, double yc)
{
    double d;
    d = sqrt(pow(xt - xc, 2) + pow(yt - yc, 2));
    return d;
}

// 引数の１の位を０にして返す関数
double removeOnePlace(double x)
{
    int x_dash, amari;
    double answer;
    x_dash = (int)x;
    amari = x_dash % 10;
    answer = (double)(x_dash - amari);
    return answer;
}

// 巡回路に応じてドローンと配送車の合流地点を導出する関数
int solveConfluence(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y, double v_d_ratio, double r_d_velo, double r_velo, double stay_t[], point *new_p, dro *drone, int dnum, vehicle *v, int vnum, int current[], int target[], int **cir, int **cir_flag, int ind[], int ind_last[], int ind_relief[], int *size)
{
    double d_sol = 0;                      // 避難所間で追いつけない場合の補正距離
    double d_com = 0;                      // ドローンが追いつけるか比較するための距離
    double v_com = 0;                      // 比較用（車）
    double d_d_v = retDis(xd, yd, xv, yv); // ドローン-目的の避難所間の距離
    double xs_dash, ys_dash, xs_dash_dash, ys_dash_dash, xs_dash_dash_dash, ys_dash_dash_dash, sin_sol, cos_sol;
    int answerFlag;

    if ((ind[drone[dnum].follow_num] > ind_last[drone[dnum].follow_num]) || ((ind[vnum] > ind_last[vnum]) && stay_t[vnum] == 0) || ((ind[vnum] - 1 == ind_last[vnum]) && stay_t[vnum] != 0 && (stay_t[vnum] < (d_d_v * r_d_velo)))) // ドローンの飛行をやめる条件(今の巡回路がすべての避難所を回り終わった、または次の巡回路の最後の避難所に追いつけない)
    {
        drone[dnum].free_mode = FALSE; // free_modeオフ
    }
    else
    {
        if (stay_t[vnum] != 0 && stay_t[vnum] > d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつける時
        {
            drone[dnum].xt = new_p[current[drone[dnum].target_num]].x;
            drone[dnum].yt = new_p[current[drone[dnum].target_num]].y;
        }
        else if (stay_t[vnum] != 0 && stay_t[vnum] < d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつけない時
        {

            d_sol = stay_t[vnum] / r_velo; // 戻す距離を算出

            xs = new_p[current[vnum]].x;
            ys = new_p[current[vnum]].y;
            xs_dash = new_p[target[vnum]].x;
            ys_dash = new_p[target[vnum]].y;

            sin_sol = retSin(xs_dash, ys_dash, xs, ys);
            cos_sol = retCos(xs_dash, ys_dash, xs, ys);

            answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

            if (answerFlag == -1)
            {
                if (cir_flag[vnum][ind[vnum]] == TRUE || (cir[vnum][ind[vnum]] == 0 && ind[vnum] == size[vnum] - 1)) // 配送車の目的避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                {
                    drone[dnum].xt = new_p[cir[vnum][ind[vnum]]].x;
                    drone[dnum].yt = new_p[cir[vnum][ind[vnum]]].y;
                }
                else
                {

                    xs = new_p[target[vnum]].x;
                    ys = new_p[target[vnum]].y;
                    xs_dash = new_p[cir[vnum][ind[vnum] + 1]].x;
                    ys_dash = new_p[cir[vnum][ind[vnum] + 1]].y;

                    d_sol = (stay_t[vnum] / r_velo) + retDis(xs, ys, v[vnum].x, v[vnum].y); // 戻す距離を算出

                    sin_sol = retSin(xs_dash, ys_dash, xs, ys);
                    cos_sol = retCos(xs_dash, ys_dash, xs, ys);

                    answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

                    if (answerFlag == -1)
                    {
                        if (cir_flag[vnum][ind[vnum] + 1] == TRUE || (cir[vnum][ind[vnum] + 1] == 0 && (ind[vnum] + 1) == size[vnum] - 1)) // 次の次の避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                        {
                            drone[dnum].xt = new_p[cir[vnum][ind[vnum] + 1]].x;
                            drone[dnum].yt = new_p[cir[vnum][ind[vnum] + 1]].y;
                        }
                        else
                        {
                            printf("エラー終了1\n");
                        }
                    }
                    else if (answerFlag == -2)
                    {
                        printf("エラー終了2\n");
                    }
                }
            }
            else if (answerFlag == -2)
            {
                printf("エラー終了3\n");
            }
        }
        else if (stay_t[vnum] == 0) // 目的の配送車が巡回路を巡回中の時
        {
            answerFlag = solveSystem(v[vnum].x, v[vnum].y, drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, new_p[target[vnum]].x, new_p[target[vnum]].y, &drone[dnum].xt, &drone[dnum].yt);

            if (answerFlag == -2) // 関数の導出エラー
            {
                printf("solveSystem関数のエラー\n");
                printf("0\n");
            }
            else if (answerFlag == -1) // ドローンが追いつけない時
            {

                if (cir_flag[vnum][ind[vnum]] == TRUE || (cir[vnum][ind[vnum]] == 0 && ind[vnum] == size[vnum] - 1)) // 配送車の目的避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                {
                    drone[dnum].xt = new_p[target[vnum]].x;
                    drone[dnum].yt = new_p[target[vnum]].y;
                }
                else
                {
                    xv = v[vnum].x;
                    yv = v[vnum].y;
                    xs = new_p[target[vnum]].x;
                    ys = new_p[target[vnum]].y;
                    xs_dash = new_p[cir[vnum][ind[vnum] + 1]].x;
                    ys_dash = new_p[cir[vnum][ind[vnum] + 1]].y;

                    d_sol = retDis(xs, ys, xv, yv);
                    sin_sol = retSin(xs_dash, ys_dash, xs, ys);
                    cos_sol = retCos(xs_dash, ys_dash, xs, ys);

                    answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

                    if (answerFlag == -1) // 更にドローンが追いつけない時
                    {

                        if (cir_flag[vnum][ind[vnum] + 1] == TRUE || (cir[vnum][ind[vnum] + 1] == 0 && (ind[vnum] + 1) == size[vnum] - 1)) // 次の次の避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                        {
                            drone[dnum].xt = new_p[cir[vnum][ind[vnum] + 1]].x;
                            drone[dnum].yt = new_p[cir[vnum][ind[vnum] + 1]].y;
                        }
                        else // 次の次の避難所が物資を下ろす避難所でないなら
                        {
                            // printf("エラー終了4\n");
                            // drone[dnum].xt = new_p[0].x;
                            // drone[dnum].yt = new_p[0].y;

                            xs_dash_dash = new_p[cir[vnum][ind[vnum] + 2]].x;
                            ys_dash_dash = new_p[cir[vnum][ind[vnum] + 2]].y;

                            d_sol = retDis(xs, ys, xv, yv) + retDis(xs_dash, ys_dash, xs, ys);
                            sin_sol = retSin(xs_dash_dash, ys_dash_dash, xs_dash, ys_dash);
                            cos_sol = retCos(xs_dash_dash, ys_dash_dash, xs_dash, ys_dash);

                            answerFlag = solveSystem(xs_dash - (d_sol * sin_sol), ys_dash - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash_dash, ys_dash_dash, &drone[dnum].xt, &drone[dnum].yt);

                            if (answerFlag == -1)
                            {
                                if (cir_flag[vnum][ind[vnum] + 2] == TRUE || (cir[vnum][ind[vnum] + 2] == 0 && (ind[vnum] + 2) == size[vnum] - 1)) // 次の次の次の避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                                {
                                    drone[dnum].xt = new_p[cir[vnum][ind[vnum] + 2]].x;
                                    drone[dnum].yt = new_p[cir[vnum][ind[vnum] + 2]].y;
                                }
                                else
                                {
                                    printf("エラー終了4\n");
                                    // xs_dash_dash_dash = new_p[cir[vnum][ind[vnum] + 3]].x;
                                    // ys_dash_dash_dash = new_p[cir[vnum][ind[vnum] + 3]].y;
                                    // printf("%d\n", ind[vnum] + 3);
                                }
                            }
                        }
                    }
                    else if (answerFlag == -2) // エラー処理
                    {
                        printf("エラー終了5\n");
                    }
                }
            }
        }
    }
}

// solveConfluence関数の修正版
#if 0
//  巡回路に応じてドローンと配送車の合流地点を導出する関数
int solveConfluence(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y, double v_d_ratio, double r_d_velo, double r_velo, double stay_t[], point *new_p, dro *drone, int dnum, vehicle *v, int vnum, int current[], int target[], int **cir, int **cir_flag, int ind[], int ind_last[], int ind_relief[], int *size)
{
    double d_sol = 0;                      // 避難所間で追いつけない場合の補正距離
    double d_com = 0;                      // ドローンが追いつけるか比較するための距離
    double v_com = 0;                      // 比較用（車）
    double d_d_v = retDis(xd, yd, xv, yv); // ドローン-目的の避難所間の距離
    double xs_dash, ys_dash, xs_dash_dash, ys_dash_dash, xs_dash_dash_dash, ys_dash_dash_dash, sin_sol, cos_sol;
    int answerFlag;

    if ((ind[drone[dnum].follow_num] > ind_last[drone[dnum].follow_num]) || ((ind[vnum] > ind_last[vnum]) && stay_t[vnum] == 0) || ((ind[vnum] - 1 == ind_last[vnum]) && stay_t[vnum] != 0 && (stay_t[vnum] < (d_d_v * r_d_velo)))) // ドローンの飛行をやめる条件(今の巡回路がすべての避難所を回り終わった、または次の巡回路の最後の避難所に追いつけない)
    {
        drone[dnum].free_mode = FALSE; // free_modeオフ
    }
    else
    {
        if (stay_t[vnum] != 0 && stay_t[vnum] > d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつける時
        {
            drone[dnum].xt = new_p[current[drone[dnum].target_num]].x;
            drone[dnum].yt = new_p[current[drone[dnum].target_num]].y;
        }
        else if (stay_t[vnum] != 0 && stay_t[vnum] < d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつけない時
        {
            d_sol = stay_t[vnum] / r_velo; // 戻す距離を算出

            xs = new_p[current[vnum]].x;
            ys = new_p[current[vnum]].y;

            int next_index = ind[vnum];
            while (1)
            {
                xs_dash = new_p[cir[vnum][next_index]].x;
                ys_dash = new_p[cir[vnum][next_index]].y;

                sin_sol = retSin(xs_dash, ys_dash, xs, ys);
                cos_sol = retCos(xs_dash, ys_dash, xs, ys);

                answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

                if (answerFlag == -1)
                {
                    if (cir_flag[vnum][next_index] == TRUE || (cir[vnum][next_index] == 0 && next_index == size[vnum] - 1))
                    {
                        drone[dnum].xt = new_p[cir[vnum][next_index]].x;
                        drone[dnum].yt = new_p[cir[vnum][next_index]].y;
                        break;
                    }
                }
                else if (answerFlag == -2)
                {
                    printf("エラー終了\n");
                    break;
                }
                else
                {
                    break;
                }

                // d_sol += retDis(xs, ys, v[vnum].x, v[vnum].y);
                /*
                if (next_index - ind[vnum] == 0)
                {
                    d_sol += retDis(xs, ys, v[vnum].x, v[vnum].y);
                }
                else
                {
                    d_sol += retDis(xs, ys, xs_dash, ys_dash);
                }
                    */
                d_sol += retDis(xs, ys, xs_dash, ys_dash);

                xs = xs_dash;
                ys = ys_dash;

                next_index++;
            }
        }
        else if (stay_t[vnum] == 0) // 目的の配送車が巡回路を巡回中の時
        {
            answerFlag = solveSystem(v[vnum].x, v[vnum].y, drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, new_p[target[vnum]].x, new_p[target[vnum]].y, &drone[dnum].xt, &drone[dnum].yt);

            if (answerFlag == -2) // 関数の導出エラー
            {
                printf("solveSystem関数のエラー\n");
                return -1;
            }
            else if (answerFlag == -1) // ドローンが追いつけない時
            {
                if (cir_flag[vnum][ind[vnum]] == TRUE || (cir[vnum][ind[vnum]] == 0 && ind[vnum] == size[vnum] - 1)) // 配送車の目的避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                {
                    drone[dnum].xt = new_p[cir[vnum][ind[vnum]]].x;
                    drone[dnum].yt = new_p[cir[vnum][ind[vnum]]].y;
                }
                else
                {
                    int next_index = ind[vnum] + 1;

                    xs = new_p[target[vnum]].x;
                    ys = new_p[target[vnum]].y;

                    while (next_index < size[vnum])
                    {
                        xs_dash = new_p[cir[vnum][next_index]].x;
                        ys_dash = new_p[cir[vnum][next_index]].y;

                        d_sol += (next_index == ind[vnum] + 1) ? retDis(xs, ys, v[vnum].x, v[vnum].y) : retDis(xs, ys, xs_dash, ys_dash);
                        sin_sol = retSin(xs_dash, ys_dash, xs, ys);
                        cos_sol = retCos(xs_dash, ys_dash, xs, ys);

                        answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

                        if (answerFlag == -1)
                        {
                            if (cir_flag[vnum][next_index - 1] == TRUE || (cir[vnum][next_index - 1] == 0 && next_index == size[vnum] - 1))
                            {
                                drone[dnum].xt = xs_dash;
                                drone[dnum].yt = ys_dash;
                                break;
                            }
                        }
                        else if (answerFlag == -2)
                        {
                            printf("エラー終了\n");
                            return -1;
                        }
                        else
                        {
                            break;
                        }

                        xs = xs_dash;
                        ys = ys_dash;
                        next_index++;
                    }
                }
            }
        }
    }
}
#endif

// solveConfluence2関数（バッテリー配布ドローンと帰還するTVとの合流地点導出）
int solveConfluenceVer2(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y, double v_d_ratio, double r_d_velo, double r_velo, double stay_t[], double dis_stay_t[], point *new_p, dro *drone, int dnum, vehicle *v, int vnum, int current[], int target[], int **cir, int **cir_flag, int ind[], int ind_last[], int ind_relief[], int *size)
{
    double d_sol = 0;                      // 避難所間で追いつけない場合の補正距離
    double d_com = 0;                      // ドローンが追いつけるか比較するための距離
    double v_com = 0;                      // 比較用（車）
    double d_d_v = retDis(xd, yd, xv, yv); // ドローン-目的の避難所間の距離
    double xs_dash, ys_dash, xs_dash_dash, ys_dash_dash, xs_dash_dash_dash, ys_dash_dash_dash, sin_sol, cos_sol;
    int answerFlag;

    /*
    if ((ind[drone[dnum].follow_num] > ind_last[drone[dnum].follow_num]) || ((ind[vnum] > ind_last[vnum]) && stay_t[vnum] == 0) || ((ind[vnum] - 1 == ind_last[vnum]) && stay_t[vnum] != 0 && (stay_t[vnum] < (d_d_v * r_d_velo)))) // ドローンの飛行をやめる条件(今の巡回路がすべての避難所を回り終わった、または次の巡回路の最後の避難所に追いつけない)
    {
        drone[dnum].free_mode = FALSE; // free_modeオフ
    }*/

    /**************目的TVが集積所にいる*************/
    if (dis_stay_t[vnum] != 0 && dis_stay_t[vnum] > d_d_v * r_d_velo) // 目的の配送車が集積所で物資を積載中で,かつ ドローンが追いつける時
    {
        drone[dnum].xt = new_p[0].x; // ドローンの目的地を集積所に設定
        drone[dnum].yt = new_p[0].y;
    }
    else if (dis_stay_t[vnum] != 0 && dis_stay_t[vnum] < d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつけない時
    {
        d_sol = dis_stay_t[vnum] / r_velo; // 戻す距離を算出

        xs = new_p[current[vnum]].x;
        ys = new_p[current[vnum]].y;

        int next_index = ind[vnum];
        while (1)
        {
            xs_dash = new_p[cir[vnum][next_index]].x;
            ys_dash = new_p[cir[vnum][next_index]].y;

            sin_sol = retSin(xs_dash, ys_dash, xs, ys);
            cos_sol = retCos(xs_dash, ys_dash, xs, ys);

            answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

            if (answerFlag == -1)
            {
                if (cir_flag[vnum][next_index] == TRUE || (cir[vnum][next_index] == 0 && next_index == size[vnum] - 1))
                {
                    drone[dnum].xt = new_p[cir[vnum][next_index]].x;
                    drone[dnum].yt = new_p[cir[vnum][next_index]].y;
                    break;
                }
            }
            else if (answerFlag == -2)
            {
                printf("エラー終了\n");
                break;
            }
            else
            {
                break;
            }

            d_sol += retDis(xs, ys, xs_dash, ys_dash);

            xs = xs_dash;
            ys = ys_dash;

            next_index++;
        }
    }
    else /******************************************* 目的TVが集積所にいない *****************************************************/
    {
        if (stay_t[vnum] != 0 && stay_t[vnum] > d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつける時
        {
            // drone[dnum].xt = new_p[current[drone[dnum].target_num]].x;
            // drone[dnum].yt = new_p[current[drone[dnum].target_num]].y;
            drone[dnum].xt = new_p[current[drone[dnum].follow_num]].x; // 追従する配送車の現在避難所に目的地を設定
            drone[dnum].yt = new_p[current[drone[dnum].follow_num]].y;
        }
        else if (stay_t[vnum] != 0 && stay_t[vnum] < d_d_v * r_d_velo) // 目的の配送車が避難所で物資をおろし中で,かつ ドローンが追いつけない時
        {
            d_sol = stay_t[vnum] / r_velo; // 戻す距離を算出

            xs = new_p[current[vnum]].x;
            ys = new_p[current[vnum]].y;

            int next_index = ind[vnum];
            while (1)
            {
                xs_dash = new_p[cir[vnum][next_index]].x;
                ys_dash = new_p[cir[vnum][next_index]].y;

                sin_sol = retSin(xs_dash, ys_dash, xs, ys);
                cos_sol = retCos(xs_dash, ys_dash, xs, ys);

                answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

                if (answerFlag == -1)
                {
                    if (cir_flag[vnum][next_index] == TRUE || (cir[vnum][next_index] == 0 && next_index == size[vnum] - 1))
                    {
                        drone[dnum].xt = new_p[cir[vnum][next_index]].x;
                        drone[dnum].yt = new_p[cir[vnum][next_index]].y;
                        break;
                    }
                }
                else if (answerFlag == -2)
                {
                    printf("エラー終了\n");
                    break;
                }
                else
                {
                    break;
                }

                // d_sol += retDis(xs, ys, v[vnum].x, v[vnum].y);
                /*
                if (next_index - ind[vnum] == 0)
                {
                    d_sol += retDis(xs, ys, v[vnum].x, v[vnum].y);
                }
                else
                {
                    d_sol += retDis(xs, ys, xs_dash, ys_dash);
                }
                    */
                d_sol += retDis(xs, ys, xs_dash, ys_dash);

                xs = xs_dash;
                ys = ys_dash;

                next_index++;
            }
        }
        else if (stay_t[vnum] == 0) // 目的の配送車が巡回路を巡回中の時
        {
            answerFlag = solveSystem(v[vnum].x, v[vnum].y, drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, new_p[target[vnum]].x, new_p[target[vnum]].y, &drone[dnum].xt, &drone[dnum].yt);

            if (answerFlag == -2) // 関数の導出エラー
            {
                printf("solveSystem関数のエラー\n");
                return -1;
            }
            else if (answerFlag == -1) // ドローンが追いつけない時
            {
                if (cir_flag[vnum][ind[vnum]] == TRUE || (cir[vnum][ind[vnum]] == 0 && ind[vnum] == size[vnum] - 1)) // 配送車の目的避難所が物資を下ろす避難所なら or 集積所（物資を補給するための帰還なら）
                {
                    drone[dnum].xt = new_p[cir[vnum][ind[vnum]]].x;
                    drone[dnum].yt = new_p[cir[vnum][ind[vnum]]].y;
                }
                else
                {
                    int next_index = ind[vnum] + 1;

                    xs = new_p[target[vnum]].x;
                    ys = new_p[target[vnum]].y;

                    while (next_index < size[vnum])
                    {
                        xs_dash = new_p[cir[vnum][next_index]].x;
                        ys_dash = new_p[cir[vnum][next_index]].y;

                        d_sol += (next_index == ind[vnum] + 1) ? retDis(xs, ys, v[vnum].x, v[vnum].y) : retDis(xs, ys, xs_dash, ys_dash);
                        sin_sol = retSin(xs_dash, ys_dash, xs, ys);
                        cos_sol = retCos(xs_dash, ys_dash, xs, ys);

                        answerFlag = solveSystem(xs - (d_sol * sin_sol), ys - (d_sol * cos_sol), drone[dnum].x, drone[dnum].y, 1.0, v_d_ratio, xs_dash, ys_dash, &drone[dnum].xt, &drone[dnum].yt);

                        if (answerFlag == -1)
                        {
                            if (cir_flag[vnum][next_index - 1] == TRUE || (cir[vnum][next_index - 1] == 0 && next_index == size[vnum] - 1))
                            {
                                drone[dnum].xt = xs_dash;
                                drone[dnum].yt = ys_dash;
                                break;
                            }
                        }
                        else if (answerFlag == -2)
                        {
                            printf("エラー終了\n");
                            return -1;
                        }
                        else
                        {
                            break;
                        }

                        xs = xs_dash;
                        ys = ys_dash;
                        next_index++;
                    }
                }
            }
        }
    }
}

// 配列を分割する関数
void split_array(int *new_jyunkai_keiro, int new_jyunkai_keiro_size, int **cir, int *size)
{

    int current_subarray = 0; // 現在のサブ配列のインデックス
    int current_index = 0;    // 現在のサブ配列内のインデックス
    int split_num = J_N;      // 巡回する避難所によって変更
    int split_judge = FALSE;

    // 元の配列を走査
    for (int i = 0; i < new_jyunkai_keiro_size; i++)
    {
        if (new_jyunkai_keiro[i] == split_num - 1)
        {
            split_judge = TRUE;
            // 現在のサブ配列に値を追加
            cir[current_subarray][current_index++] = new_jyunkai_keiro[i];
        }
        else if (new_jyunkai_keiro[i] == split_num && current_index != 0 && split_judge == TRUE)
        {

            if (split_num == N - 1) // 最後まで 50 行ったら
            {
                // 現在のサブ配列に値を追加
                cir[current_subarray][current_index++] = new_jyunkai_keiro[i];

                // 現在のサブ配列を終了し、次のサブ配列に移動
                size[current_subarray] = current_index + 1; // 現在のサブ配列のサイズを記録
                printf("subsize:%d\n", size[current_subarray]);
            }
            else
            {
                // 現在のサブ配列に値を追加
                cir[current_subarray][current_index++] = new_jyunkai_keiro[i];
                // 現在のサブ配列を終了し、次のサブ配列に移動
                size[current_subarray] = current_index; // 現在のサブ配列のサイズを記録
                printf("subsize:%d\n", size[current_subarray]);
                current_index = 0;     // 次のサブ配列のインデックスをリセット
                current_subarray += 1; // 現在のサブ配列から次のサブ配列へ
            }

            split_num += J_N;
            split_judge = FALSE;
        }
        else
        {
            // 現在のサブ配列に値を追加
            cir[current_subarray][current_index++] = new_jyunkai_keiro[i];
        }
    }
    // 最後のサブ配列のサイズを記録
    size[current_subarray] = current_index;
}

// 二次元配列の指定した行に一次元配列を追加する関数
int add_to_row(int ***array, int *row_sizes, int target_row, int *add_array, int add_size)
{
    // 新しい行のサイズ
    int new_row_size = row_sizes[target_row] + add_size;

    // 指定した行のメモリを再確保
    int *new_row = realloc((*array)[target_row], new_row_size * sizeof(int));
    if (new_row == NULL)
    {
        return -1; // メモリ再確保に失敗
    }

    // 新しい要素をコピー
    memcpy(new_row + row_sizes[target_row], add_array, add_size * sizeof(int));

    // 配列とサイズを更新
    (*array)[target_row] = new_row;
    row_sizes[target_row] = new_row_size;

    return 0; // 成功
}

// 二次元配列の指定した行の先頭に一次元配列を追加する関数
// 指定した行の先頭に一次元配列を追加する関数
int prepend_to_row(int ***array, int *row_sizes, int target_row, int *add_array, int add_size)
{
    // 新しい行のサイズ
    int new_row_size = row_sizes[target_row] + add_size;

    // 指定した行のメモリを再確保
    int *new_row = realloc((*array)[target_row], new_row_size * sizeof(int));
    if (new_row == NULL)
    {
        return -1; // メモリ再確保に失敗
    }

    // 既存の要素をシフト
    memmove(new_row + add_size, new_row, row_sizes[target_row] * sizeof(int));

    // 新しい要素を先頭にコピー
    memcpy(new_row, add_array, add_size * sizeof(int));

    // 配列とサイズを更新
    (*array)[target_row] = new_row;
    row_sizes[target_row] = new_row_size;

    return 0; // 成功
}

// connect配列に隣接行列のデータを格納する関数
void readAdjacencyMatrix(const char *filename, int connect[][N])
{
    FILE *file = fopen(filename, "r");
    if (file == NULL)
    {
        printf("ファイルを開くことができませんでした。\n");
        return;
    }

    // ファイルからデータを読み取り、connect配列に格納
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            fscanf(file, "%d", &connect[i][j]);
        }
    }

    fclose(file);
}

// Box-Muller法を用いた正規乱数生成
double generate_normal(double mean, double std_dev)
{
    double u1, u2, z;
    do
    {
        u1 = (double)rand() / RAND_MAX;
        u2 = (double)rand() / RAND_MAX;
        z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); // 標準正規分布 N(0,1)
        z = mean + std_dev * z;                          // 平均と標準偏差を適用
    } while (z < MIN_VAL || z > MAX_VAL); // 範囲外なら再生成

    return z;
}

// min から maxの範囲の整数値をランダムに返す関数
int get_random_int(int min, int max)
{
    return rand() % (max - min + 1) + min;
}