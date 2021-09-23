.default: all

all: run

clean:
	rm -f run *.o

run: main.o Node.o NodeList.o PathSolver.o
	g++ -Wall -Werror -std=c++14 -O -o $@ $^

%.o: %.cpp
	g++ -Wall -Werror -std=c++14 -O -c $^
