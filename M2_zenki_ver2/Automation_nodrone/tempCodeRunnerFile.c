// new_p[i].re_req = generate_normal(MEAN, STD_DEV); // 各避難所の必要物資量をランダムに生成(正規分布)
                new_p[i].re_req += (int)rand_exp(lambda_re); // 指数分布による増加分