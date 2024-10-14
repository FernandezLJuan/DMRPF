links := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -lconfig++
opts := -Wall -Wextra -Wpedantic
compiler := clang++

sim: main.cpp environment.cpp cell.cpp robot.cpp

	${compiler} $^ -o $@.o ${opts} ${links}