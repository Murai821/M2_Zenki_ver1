def calculate_rsh(Np, Ns):
    """
    Rsh理論値を計算する関数
    
    Args:
        Np (int): 集積所数
        Ns (int): 避難所数
    
    Returns:
        float: Rsh理論値
    """
    # シグマ（Σ）の範囲を定義
    sigma_start_calc = Np + Ns - 1 - int(Ns/Np - 1)  # 計算値（小数点可能）
    sigma_start = int(sigma_start_calc)  # 整数に変換
    sigma_end = Np + Ns - 1  # シグマの終了値 (Np + Ns - 1)
    
    #print(f"集積所数 Np: {Np}, 避難所数 Ns: {Ns}")
    #print(f"シグマの開始値計算: {sigma_start_calc}")
    #print(f"シグマの範囲: k = {sigma_start} から {sigma_end}")
    #print(f"計算範囲: Σ(k={sigma_start} to {sigma_end})")
    
    denominator = Ns
    numerator = 0
    
    # シグマ計算: Σ(k=sigma_start to sigma_end) f(k)
    for k in range(sigma_start, sigma_end + 1):
        term = k * Np / (Np + Ns)
        numerator += term
    
    Rsh = numerator / denominator 
    
    #print(f"Rshの理論値: {Rsh}")
    return Rsh

# 使用例
if __name__ == "__main__":
    
    # パラメータ範囲での実行テスト
    print("\n=== パラメータ範囲実行 ===")
    print("i=1から6, j=1から10 (Np=i, Ns=j*Np)")
    print("格式: Np | Ns | Rsh")
    print("-" * 30)
    
    for i in range(1, 8):  # i = 1 to 7
        for j in range(1, 8):  # j = 1 to 7
            Np = i
            Ns = j * Np  # Ns = j * Np
            
            try:
                result = calculate_rsh(Np, Ns)
                print(f"{Np:2d} | {Ns:2d} | {result:.6f}")
            except Exception as e:
                print(f"{Np:2d} | {Ns:2d} | エラー: {e}")
    
    # デフォルト値での実行
    Np_default = 2  # 集積所数
    Ns_default = 10 # 避難所数
    
    result = calculate_rsh(Np_default, Ns_default)
    print(f"\n最終結果: Np={Np_default}, Ns={Ns_default} → Rsh={result}")
