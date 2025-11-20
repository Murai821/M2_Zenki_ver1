import math
import random

# seed値の設定
random.seed(22)

def erlang_c(lam, mu, c):
    """
    Erlang C（待ち確率）を計算する関数
    lam : 到着率 λ
    mu  : サービス率 μ (=1/サービス時間)
    c   : サーバ数 ND
    """
    rho = lam / (c * mu)

    # 負荷率が1以上になると発散するので例外処理
    if rho >= 1.0:
        return 1.0

    # 分母
    sum_terms = sum((c * rho)**k / math.factorial(k) for k in range(c))
    last_term = (c * rho)**c / (math.factorial(c) * (1 - rho))
    denom = sum_terms + last_term

    # 分子
    numer = last_term

    return numer / denom


def average_delay(lam=1.0, service_time=0.7, c=1):
    """
    平均遅延時間（待ち時間＋サービス時間）を返す
    lam : 到着率 λ
    service_time : サービス時間 s
    c : 対応人数 ND
    """
    mu = 1.0 / service_time       # サービス率
    rho = lam / (c * mu)

    # Erlang C
    C = erlang_c(lam, mu, c)

    # 平均待ち時間 E[W]
    W = (C / (c * mu - lam)) * (1 / (2 * mu))

    # 平均遅延時間 E[T]
    T = W + service_time
    return W, T


R = 3  # 円の半径
N = 10  # 避難所数
NL = N-1  # 直線の本数
NT = (NL-1)//2 if NL%2 == 1 else NL//2  # NLが奇数なら(NL-1)/2、偶数ならNL/2
VD = 20  # ドローン速度
ND = 3  # ドローン台数
lam = 1.0  # 到着率 [件/時間]
service_T = 0.7  # サービス時間 [時間]

# x値を格納する配列
x = []

# i=0からNT未満まで計算
for i in range(NT+1):
    if i == 0:
        x_val = 2 * R
    else:
        x_val = 2 * R**2 * (1 - math.cos((2 * math.pi / N) * i))
    
    
    if i == 0:
        x_val = 2 * R

        if N % 2 == 0:  # Nが偶数のときのみ集積所と真反対の直線距離追加
            x.append(x_val)
    else:
        # 平方根を取る
        x_val = math.sqrt(x_val)
        # 同じ値を2つ格納
        x.append(round(x_val, 3))
        x.append(round(x_val, 3))
    

# x配列の全ての値を2倍にしたものをy配列とする
y = [round(val * 2, 3) for val in x]

# TD配列: yの各値をVDで割って1/3を足す
TD = [round(val / VD + 1/3, 3) for val in y]#ドローンの物資運搬時間

# TD配列の平均値を計算
TD_ave = round(sum(TD) / len(TD), 3) if len(TD) > 0 else 0

TA = (2*math.pi*R)/(ND*VD) #到着間隔の時間

print(f"R = {R}")
print(f"N = {N}")
print(f"NL = {NL}")
print(f"NT = {NT}")
print(f"VD = {VD}")
print(f"x配列: {x}")
print(f"y配列: {y}")
print(f"TD配列: {TD}")
print(f"TD_ave: {TD_ave}")
print(f"TA: {round(TA,3)}")

for c in range(1,5):
        W, T = average_delay(lam=lam, service_time=service_T, c=c)
        print(f"ND={c}:  平均待ち時間 W={W:.4f},  平均遅延時間 T={T:.4f}")