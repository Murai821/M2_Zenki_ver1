import math

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


# 使い方例
if __name__ == "__main__":
    for c in [1, 2, 3, 4]:
        W, T = average_delay(c=c)
        print(f"ND={c}:  平均待ち時間 W={W:.4f},  平均遅延時間 T={T:.4f}")
