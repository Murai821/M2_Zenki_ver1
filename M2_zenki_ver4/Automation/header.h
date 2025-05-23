/*ヘッダファイル*/
#ifndef HEADER_H
#define HEADER_H

/**************************************** マクロ変数定義 ************************************************************************/
#define L 10.0      /*サービスエリアの大きさ*/
#define R 5.0       /*中心市街地の半径*/
#define A_R 1.0     /*中心市街地の半径 + A_R の範囲外に山間部の避難所を生成する*/
#define C_N 41      /*中心市街地の避難所数*/
#define N 51        /*避難所数+配送センター数*/
#define M_N N - C_N /* 山間部の避難所数 N - C_N*/
#define J_N 10      // TVが巡回する避難所数  山間部避難所数によって変化（山間部避難所10で中心市街地のみTVなら 40/5で 8 ）
#define HEN 11      /*hen[][]の要素数*/
#define M 5         /*配送車数*/
#define D 8         /*ドローン数*/
#define SD 5        /*シミュレーションで用いるドローン数*/
#define C_D 5       /* 要求情報を回収するドローンの台数 */
#define I_SIZE 150  /*情報配列の要素数*/
#define Y_SIZE 20   /*薬の情報配列の二次元要素数*/
#define Z_SIZE 4    /*薬の情報配列の三次元要素数*/
#define INF 9999    /*無限大*/
#define TRUE 1
#define FALSE 0
#define CONST 1000            /*定数*/
#define MAX_ELEMENTS 100      // pythonファイルから読み込む巡回セールスマン問題の近似解の巡回路の配列の最大要素数
#define CIR_SIZE 20           /*cir[M][] ,cir[M][]の要素数*/
#define MAX_SUBARRAYS 10      // 最大サブ配列数 RFCS法
#define MAX_SIZE 100          // 各サブ配列の最大サイズ RFCS法
#define MAX_charge_count 9999 // 各TVが一巡回する間にドローンを充電することができる回数
#define MAX_TVchargeable 300  // TVで一巡回する間にドローンを充電することができる総計（単位は[min]）
// Box-Muller法を用いた正規乱数生成関数に必要なパラメータ
#define MEAN 50.0      // 平均
#define STD_DEV 10.0   // 標準偏差
#define MIN_VAL 30.0   // 下限
#define MAX_VAL 70.0   // 上限
#define QUEUE_SIZE 100 // 医療品を配達する避難所番号を順に格納するキューのサイズ

/*************************************************** 構造体定義 *********************************************************************/
// 配送センターと避難所の構造体
typedef struct
{
    double x;                          // x座標
    double y;                          // y座標
    int re;                            // 物資数カウンター
    int re_req;                        // 物資要求量
    int re_req_sum;                    // 物資要求量の合計値
    int re_deli;                       // 一巡会の間に届けられた物資量
    int inf[N][I_SIZE];                // 情報配列
    int i_ptr[N];                      // 情報配列のポインタ
    double inf_med[N][Y_SIZE][Z_SIZE]; // 避難所の薬の情報配列三次元(避難所番号,情報配列のインデックス,（生成時間・緊急度（生成してから運搬までの目標時間）・要求情報が発生してから共有されるまでの時間・要求物資が運搬されたか(TRUE or FALSE)）
    int i_med_ptr[N];                  // 薬の情報配列のポインタ
} point;

// 配送車の構造体
typedef struct
{
    double x;
    double y;
    int re;     // 積載物資量
    int Med_re; // 医療品積載量
    int inf[N][I_SIZE];
    int i_ptr[N];
    double inf_med[N][Y_SIZE][Z_SIZE];  // 避難所の薬の情報配列三次元(避難所番号,情報配列のインデックス,（生成時間・緊急度（生成してから運搬までの目標時間）・要求情報が発生してから共有されるまでの時間・要求物資が運搬されたか(TRUE or FALSE)）
    int i_med_ptr[N];                   // 薬の情報配列のポインタ
    int next_wait_flag;                 // 次の避難所が荷降ろしのために30分要する避難所の場合にTRUE
    int drone_charge_count;             // TVでのドローンの最大充電可能台数
    double charge_amount;               // TVでのドローン総充電時間（一巡回中）
    double chargeable_flag;             // TVでドローンを充電可能かを表すフラグ（充電可能：１、不可能：０）
    int Med_delivery_queue[QUEUE_SIZE]; // 医療品を配達する避難所番号を順に格納するキュー
    int queue_ptr;                      // 医療品を配達する避難所番号を順に格納するキューのポインタ
    int queue_Notdelivery_ptr;          // キューのうち、医療品が未配達の避難所の戦闘のポインタ
} vehicle;

// ドローンの構造体
typedef struct
{
    double x;
    double y;
    double xt; // ドローンの飛行目標点
    double yt;
    int re;
    int Med_re; // 医療品積載量
    int inf[N][I_SIZE];
    int i_ptr[N];
    double inf_med[N][Y_SIZE][Z_SIZE]; // 避難所の薬の情報配列三次元(避難所番号,情報配列のインデックス,（生成時間・緊急度（生成してから運搬までの目標時間）・要求情報が発生してから共有されるまでの時間・要求物資が運搬されたか(TRUE or FALSE)）
    int i_med_ptr[N];                  // 薬の情報配列のポインタ
    int follow_num;                    // ドローンが従う配送車番号
    int target_num;                    // ドローンが巡回路をまたいで向かう巡回路番号
    int wait_flag;                     // ドローンが先回りして避難所で待機することを示すフラグ
    int free_mode;
    int charge_time;           // ドローンが飛行した分だけ充電する時間
    double flight_start_time;  // ドローンの飛行開始時間
    int FtoDiscenter_mode;     // ドローンが配送センターに向かうモード(避難所から集積所)（FALSE:配送車に従う、TRUE:ドローン単独で配送センターへ向かう）
    int delivery_mode;         // ドローンの配達モード(集積所から避難所)（FALSE:配送車に従う、TRUE:ドローン単独で配達)
    int target_shelter_num;    // ドローンの目標避難所番号
    double stay_Medload_time;  // ドローンが医療品を積載するために滞在する時間
    int cannot_fly_judge_flag; // ドローンの避難所への飛行不可能性を判断するフラグ（TRUE：飛行不可能 FALSE：飛行可能）
    int TV_wait_flag;          // ドローンが避難所でTVの到着を待つフラグ
} dro;

/********************************************** 関数定義 ******************************************************************/
/*配列において、ある値のインデックスを返す関数*/
int search_index(int arry[], int num);

/*ダイクストラ法を行う関数*/
double dijkstra(int start, int goal, double weight[][N], int numline);

// RFCS法を行いその結果導出された最短経路を配列に格納して返す関数
void RFCS_method(double weight[][N], int numline, int result_arry[], int *result_arry_size);

// ダイクストラ法により導出したスタート地点からゴール地点までの経由地点を格納して返す関数
double dijkstraVIA(int start, int goal, double weight[][N], int numline, int **path, int *path_length);

// ダイクストラ法を用いて、配列の後ろに追加する経由地点を導出する関数（追加する経由地点は：スタート地点を除いたゴール地点までの経由地点）
double dijkstraArrayAdd(int start, int goal, double weight[][N], int numline, int **path, int *path_length);

// ダイクストラ法を用いて、配列の前に追加する経由地点を導出する関数（追加する経由地点は：ゴール地点を除いた経由地点）
double dijkstraArrayAppend(int start, int goal, double weight[N][N], int numline, int **path, int *path_length);

/*RFCS法の「その他」におけるコストを求める関数*/
double cost_sum(int i, int j, double weight[][N], int numline);

/*(0,1)の一様分布に従う乱数の生成関数*/
double Uniform(void);

/*密度λ(lamda)の指数分布に従った乱数の生成関数*/
double rand_exp(double lambda);

/*配列内の最小値を返す関数*/
double findMin(double arr[], int size);

/*配列内の最大値を返す関数*/
double findMax(double arr[], int size);

// ドローンと配送車の合流点を導出する関数
int solveSystem(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y);

// ２点間のcosを返す関数
double retCos(double xt, double yt, double xc, double yc);

// ２点間のsinを返す関数
double retSin(double xt, double yt, double xc, double yc);

/*２点間の距離を返す関数*/
double retDis(double xt, double yt, double xc, double yc);

// 引数の１の位を０にして返す関数
double removeOnePlace(double x);

// 巡回路に応じてドローンと配送車の合流地点を導出する関数
int solveConfluence(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y, double v_d_ratio, double r_d_velo, double r_velo, double stay_t[], point *new_p, dro *drone, int dnum, vehicle *v, int vnum, int current[], int target[], int **cir, int **cir_flag, int ind[], int ind_last[], int ind_relief[], int *size);

// 配列を分割する関数
void split_array(int *new_jyunkai_keiro, int new_jyunkai_keiro_size, int **cir, int *size);

// 二次元配列の指定した行に一次元配列を追加する関数
int add_to_row(int ***array, int *row_sizes, int target_row, int *add_array, int add_size);

// 二次元配列の指定した行の先頭に一次元配列を追加する関数
int prepend_to_row(int ***array, int *row_sizes, int target_row, int *add_array, int add_size);

// connect配列に隣接行列のデータを格納する関数
void readAdjacencyMatrix(const char *filename, int connect[][N]);

// Box-Muller法を用いた正規乱数生成関数
double generate_normal(double mean, double std_dev);

// min から maxの範囲の整数値をランダムに返す関数
int get_random_int(int min, int max);

/*********************************************** 変数定義 ******************************************************************************/

#endif // HEADER_H
