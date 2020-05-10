make:
	mkdir -p bin
	g++ pathfinder.cpp -o ./bin/main -lsfml-graphics -lsfml-system -lsfml-window