links := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -lconfig++
opts := -Wall -Wextra -Wpedantic -g
compiler := clang++

sim: main.cpp environment.cpp cell.cpp robot.cpp

	${compiler} $^ -o $@.o ${opts} ${links}

clean:
	rm *.o