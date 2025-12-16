# Makefile for Syuronyou program

CC = gcc
CFLAGS = -Wall -Wextra -O2 -lm
TARGET = syuronyou_program
HEADER = Syuronyou_header.h
MODULE = Syuronyou_module.c
MAIN = Syuronyou_main.c

# デフォルトターゲット
all: $(TARGET)

# メインプログラムのビルド
$(TARGET): $(MODULE) $(MAIN) $(HEADER)
	$(CC) -o $(TARGET) $(MODULE) $(MAIN) $(CFLAGS)

# クリーンアップ
clean:
	rm -f $(TARGET) *.o *.png *.gif

# テスト実行
test: $(TARGET)
	./$(TARGET)

# ヘルプ
help:
	@echo "使用可能なターゲット:"
	@echo "  all     - プログラムをビルド"
	@echo "  clean   - 生成ファイルを削除"
	@echo "  test    - プログラムを実行"
	@echo "  help    - このヘルプを表示"

.PHONY: all clean test help
