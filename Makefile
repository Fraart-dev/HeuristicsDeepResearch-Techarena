# Режим по умолчанию: сдача (без INTERACTIVE_MODE / CLI_FILE_MODE).
# Читает instance.json, пишет solution.json, stdout щасорять нельзя
#
# make build      — собрать версию для сдачи (./solve)
# make run        — запустить версию для сдачи
# make debug      — собрать отладочную версию (./solve_dbg, с scanf/printf)
# make cli        — собрать file-mode версию (./solve_cli, путь из argv)
# make clean      — удалить бинарники

CXX        = g++
CXXFLAGS   = -O3 -std=c++17
TARGET     = solve
TARGET_DBG = solve_dbg
TARGET_CLI = solve_cli
SRC        = src/main.cpp

.PHONY: build run debug cli clean

build:
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

run:
	./$(TARGET)

debug:
	$(CXX) $(CXXFLAGS) -DINTERACTIVE_MODE -o $(TARGET_DBG) $(SRC)

cli:
	$(CXX) $(CXXFLAGS) -DCLI_FILE_MODE -o $(TARGET_CLI) $(SRC)

clean:
	rm -f $(TARGET) $(TARGET_DBG) $(TARGET_CLI)