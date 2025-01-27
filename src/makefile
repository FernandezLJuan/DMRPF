links := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -lconfig++
opts := -Wall -Wextra -Wpedantic -std=c++17 -march=native -Wno-unused-variable -Wno-unused-private-field
compiler := clang++

sim: main.cpp environment.cpp cell.cpp robot.cpp
	${compiler} $^ -o sim ${opts} ${links}

clean:
	rm sim
