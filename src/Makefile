.PHONY: all sfleta_graph.o sfleta_graph.a sfleta_graph_algorithms.o sfleta_graph_algorithms.a test gcov_report clang cppcheck leaks valgrind clean main
CC = g++ -std=c++17
CFLAGS = -Wall -Werror -Wextra
CPPCHECKFLAGS = --enable=all --language=c++ --std=c++17 --suppress=missingIncludeSystem --suppress=unusedFunction --suppress=missingInclude --suppress=unusedStructMember --suppress=unmatchedSuppression

SOURCES :=sfleta_*.cc
HEADERS :=*.h

SOURCES_ALGORITHMS :=sfleta_graph.cc \
					sfleta_queue.cc \
					sfleta_stack.cc \
					sfleta_base.cc \
					sfleta_graph_algorithms.cc

OBJECTS_ALGORITHMS=$(SOURCES_ALGORITHMS:.cc=.o)

OS = $(shell uname)
ifeq ($(OS),Darwin)
	CHECKFLAGS = -lgtest
	GCOVFLAGS = --coverage
else
	CHECKFLAGS = -lgtest -lpthread -lrt -lm
	GCOVFLAGS = -fprofile-arcs -ftest-coverage
endif

all: clean sfleta_graph.a sfleta_graph_algorithms.a

clean:
	rm -f *.o *.gcno *.gcda *.info *.out
	rm -rf ./report/ *.dSYM
	@clear

test: sfleta_graph_algorithms.a
	$(CC) $(CFLAGS) test.cc sfleta_graph_algorithms.a $(CHECKFLAGS) -o test.out
	./test.out

gcov_report:
	$(CC) $(CFLAGS) $(GCOVFLAGS) test.cc $(SOURCES_ALGORITHMS) $(CHECKFLAGS) -o gcov_rep.out
	./gcov_rep.out
	lcov -t "gcov_rep" --no-external --output-file=coverag.info --capture --directory .
	genhtml -o ./report coverag.info
	open ./report/index.html

sfleta_graph.o: sfleta_graph.cc
	$(CC) $(CFLAGS) -c $^
	
sfleta_graph.a: sfleta_graph.o
	ar rc sfleta_graph.a sfleta_graph.o
	ranlib sfleta_graph.a

sfleta_graph_algorithms.o: sfleta_graph_algorithms.cc sfleta_base.cc sfleta_queue.cc sfleta_stack.cc
	$(CC) $(CFLAGS) -c $^

sfleta_graph_algorithms.a: sfleta_graph_algorithms.o sfleta_base.o sfleta_queue.o sfleta_stack.o sfleta_graph.o
	ar rc sfleta_graph_algorithms.a sfleta_graph.o $^
	ranlib sfleta_graph_algorithms.a
	
main:
	@clear
	$(CC) main.cc $(SOURCES) -o main.out
	./main.out

clang:
	@cp ../materials/linters/.clang-format ./
	@clang-format -i *.cc
	@clang-format -i *.h
	clang-format -n *.cc
	clang-format -n *.h
	@rm .clang-format

cppcheck:
	@cppcheck $(CPPCHECKFLAGS) $(SOURCES)
	@cppcheck $(CPPCHECKFLAGS) $(HEADERS)
	@cppcheck $(CPPCHECKFLAGS) main.cc
	@cppcheck $(CPPCHECKFLAGS) test.cc

leaks:
	CK_FORK=no leaks --atExit -- ./test.out

valgrind:
	valgrind --leak-check=full --show-leak-kinds=all --trace-children=yes ./test.out

