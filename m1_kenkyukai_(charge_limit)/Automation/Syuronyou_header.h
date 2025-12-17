#ifndef HEADER_H
#define HEADER_H

// === マクロ変数定義 ===/
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
#define SD 6        /*シミュレーションで用いるドローン数*/
#define C_D 15      /* 要求情報を回収するドローンの初期化台数 */
#define S_C_D 1     /* 要求情報を回収するドローンのシミュレーションで用いる台数 */
#define S_N 16
#define B_D 20     /* バッテリー配布ドローン */
#define I_SIZE 100 /*情報配列の要素数*/
#define Y_SIZE 10  /*薬の情報配列の二次元要素数*/
#define Z_SIZE 6   /*薬の情報配列の三次元要素数*/
#define INF 9999   /*無限大*/
#define TRUE 1
#define FALSE 0
#define CONST 1000            /*定数*/
#define MAX_ELEMENTS 100      // pythonファイルから読み込む巡回セールスマン問題の近似解の巡回路の配列の最大要素数
#define CIR_SIZE 20           /*cir[M][] ,cir[M][]の要素数*/
#define MAX_SUBARRAYS 10      // 最大サブ配列数 RFCS法
#define MAX_SIZE 100          // 各サブ配列の最大サイズ RFCS法
#define MAX_charge_count 9999 // 各TVが一巡回する間にドローンを充電することができる回数
#define MAX_TVchargeable 300  // TVで一巡回する間にドローンを充電することができる総計（単位は[min]）

//=== 構造体定義 ===
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
    double inf_med[N][Y_SIZE][Z_SIZE]; // 避難所の薬の情報配列三次元(避難所番号,情報配列のインデックス,（生成時間・緊急度（生成してから運搬までの目標時間）・要求情報が発生してから共有されるまでの時間（避難所→TV）・要求物資が運搬されたか(TRUE or FALSE)・要求情報が発生してから共有されるまでの時間（避難所→ドローン）・要求情報がTVorドローンに既に共有されたか(TRUE or FALSE)）
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
    double inf_med[N][Y_SIZE][Z_SIZE]; // 避難所の薬の情報配列三次元(避難所番号,情報配列のインデックス,（生成時間・緊急度（生成してから運搬までの目標時間）・要求情報が発生してから共有されるまでの時間（避難所→TV）・要求物資が運搬されたか(TRUE or FALSE)・要求情報が発生してから共有されるまでの時間（避難所→ドローン）・要求情報がTVorドローンに既に共有されたか(TRUE or FALSE))
    int i_med_ptr[N];                  // 薬の情報配列のポインタ
    int next_wait_flag;                // 次の避難所が荷降ろしのために30分要する避難所の場合にTRUE
    int drone_charge_count;            // TVでのドローンの最大充電可能台数
    double charge_amount;              // TVでのドローン総充電時間（一巡回中）
    double chargeable_flag;            // TVでドローンを充電可能かを表すフラグ（充電可能：１、不可能：０）
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
    double inf_med[N][Y_SIZE][Z_SIZE]; // 避難所の薬の情報配列三次元(避難所番号,情報配列のインデックス,（生成時間・緊急度（生成してから運搬までの目標時間）・要求情報が発生してから共有されるまでの時間（避難所→TV）・要求物資が運搬されたか(TRUE or FALSE)・要求情報が発生してから共有されるまでの時間（避難所→ドローン）・要求情報がTVorドローンに既に共有されたか(TRUE or FALSE))
    int i_med_ptr[N];                  // 薬の情報配列のポインタ
    int follow_num;                    // ドローンが従う配送車番号
    int target_num;                    // ドローンが巡回路をまたいで向かう巡回路番号
    int wait_flag;                     // ドローンが先回りして避難所で待機することを示すフラグ
    int free_mode;
    int charge_time;          // ドローンが飛行した分だけ充電する時間
    double flight_start_time; // ドローンの飛行開始時間
} dro;

//=== 関数宣言 ===
int search_index(int arry[], int num);
double dijkstra(int start, int goal, double weight[][N], int numline);
void RFCS_method(double weight[][N], int numline, int result_arry[], int *result_arry_size);
double dijkstraVIA(int start, int goal, double weight[][N], int numline, int **path, int *path_length);
double dijkstraArrayAdd(int start, int goal, double weight[][N], int numline, int **path, int *path_length);
double dijkstraArrayAppend(int start, int goal, double weight[N][N], int numline, int **path, int *path_length);
double cost_sum(int i, int j, double weight[][N], int numline);
double Uniform(void);
double rand_exp(double lambda);
double findMin(double arr[], int size);
double findMax(double arr[], int size);
int solveSystem(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y);
double retCos(double xt, double yt, double xc, double yc);
double retSin(double xt, double yt, double xc, double yc);
double retDis(double xt, double yt, double xc, double yc);
double removeOnePlace(double x);
int solveConfluence(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y, double v_d_ratio, double r_d_velo, double r_velo, double stay_t[], point *new_p, dro *drone, int dnum, vehicle *v, int vnum, int current[], int target[], int **cir, int **cir_flag, int ind[], int ind_last[], int ind_relief[], int *size);
int solveConfluenceVer2(double xv, double yv, double xd, double yd, double vv, double vd, double xs, double ys, double *x, double *y, double v_d_ratio, double r_d_velo, double r_velo, double stay_t[], double dis_stay_t[], point *new_p, dro *drone, int dnum, vehicle *v, int vnum, int current[], int target[], int **cir, int **cir_flag, int ind[], int ind_last[], int ind_relief[], int *size);
void split_array(int *new_jyunkai_keiro, int new_jyunkai_keiro_size, int **cir, int *size);
int add_to_row(int ***array, int *row_sizes, int target_row, int *add_array, int add_size);
int prepend_to_row(int ***array, int *row_sizes, int target_row, int *add_array, int add_size);
void readAdjacencyMatrix(const char *filename, int connect[][N]);
int get_random_int(int min, int max);
void init_point(point *p, double x, double y);
void init_vehicle(vehicle *v, double x, double y);
void init_dro(dro *d, int follow_num, int target_num);
#endif // HEADER_H
