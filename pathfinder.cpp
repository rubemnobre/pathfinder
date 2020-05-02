#include<SFML/Graphics.hpp>
#include<stdio.h>
#include<time.h>
#include<vector>
#include<cmath>
#include<fstream>

const float sq2 = sqrt(2);
const int size = 500; //w and h in pixels of the screen
const int block = 10; //size of a block in pixels
const int mapSize = size/block; //number of blocks that can fit in the screen. Will be the map matrix's dimention
const int startx = 0, starty = 0; //the start point's coordinates
const int endx = 49, endy = 49; //the goal's coordinates. Can't be in a pos object because the constructor needs it so it's a chicken and egg problem
const int htype = 0; //0-Euclidian h(n), 1-Manhattan, 2-h(n)=0, 3-Actual smallest distance considering 8-axis movement 
const int gtype = 0; //type 0->f(n) = g(n) + h(n), 1->f(n)=g(n)
const int real = 0; //indicates if the program should draw after each iteration of the algorithm
const bool Qaxis = false; //indicates if the program should use diagonal movement

bool isFree(int x, int y, short* map){
    if(x >= 0 && y >= 0 && x < mapSize && y < mapSize)
        return !(map[x * mapSize + y] == 1 || map[x * mapSize + y] == 3); //This function verifies if a position is available
    else return false;
}

float heuristic(int x, int y){
    if(htype == 1 || Qaxis)
        return abs(x-endx) + abs(y-endy); //manhattan distance

    if(htype == 2)
        return 0; //0. No heuristic.
    if(htype == 3){
        int dx = x - endx;
        dx = dx > 0 ? dx : -dx;
        int dy = y - endy;
        dy = dy > 0 ? dy : -dy;
        if(dx > dy){
            return (dx - dy) + dy*sq2;
        }else{
            return (dy - dx) + dx*sq2;
        }
    }
    return sqrt(pow(x - endx, 2) + pow(y - endy, 2)); //euclidian distance - default
}

int comp(float h1, float g1, float h2, float g2){ //compares f(n) of two blocks. it's for sorting the best blocks
    if(gtype == 1)
        return h1 <= h2; //compares only the h function. this makes the program behave like a Best First Searh algorithm

    return (h1 + g1) < (h2 + g2); //default f(n) comparison for A*
}

class pos{ //A class that holds a position's coordinates, the currently traveled distance, the heuristics value and its parent
    public:
        int x, y;
        int ind; //index of the node's parent on the done[] vector
        bool isEnd = 0; //indicates if the node is at goal
        float g, h;
        pos(int, int, float, int, short*); //constructor
        //void next(short* map, std::vector<this>*, std::vector<this>*);  //This function expands the position
};

typedef std::vector <pos> posv;

pos::pos(int nx, int ny, float ng, int nind, short *map){ //node constructor
    x = nx;
    y = ny;
    ind = nind;
    h = heuristic(x, y);
    g = ng;
    if(x == endx && y == endy) {
        isEnd = true;
        printf("Found end\n"); 
    }
    if(map[x * mapSize + y] == 0) map[x * mapSize + y] = 3; //indicates in the map array that there is a node
}

void next(pos current, short* map, posv *guesses, posv *done){
    bool success = 0;
    done->push_back((*guesses)[0]);
    int toerase = (*guesses)[0].ind;
    int a = done->size() - 2;
    int nx = current.x;
    int ny = current.y;
    float ng = current.g + 1; //the new g value for sideways moves
    float ng2 = current.g + sq2; // the new g value for diagonal values. sq2 is sqrt(2)

    if(isFree(nx + 1, ny, map)){ //check if the move is allowed 
        pos nxt(nx + 1, ny, ng, a+1, map); //creates position object in the new position
        int gsize = guesses->size();
        for(int i = 0; i < gsize; i++){
            if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                guesses->insert(guesses->begin() + i, nxt); //insert into a "sorted position"
                break;
            }else
            if(i == gsize - 1){
                guesses->push_back(nxt); //if the worst option, append to the end of the vector
            }
        }
        
    }
    

    if(isFree(nx, ny + 1, map)){
        pos nxt(nx, ny + 1, ng, a+1, map);
        int gsize = guesses->size();
        for(int i = 0; i < gsize; i++){
            if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                guesses->insert(guesses->begin() + i, nxt);
                break;
            }else
            if(i == gsize - 1){
                guesses->push_back(nxt);
            }
        }
        
    }
    if(isFree(nx - 1, ny, map)){
        pos nxt(nx - 1, ny, ng, a+1, map);
        int gsize = guesses->size();
        for(int i = 0; i < gsize; i++){
            if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                guesses->insert(guesses->begin() + i, nxt);
                break;
            }else
            if(i == gsize - 1){
                guesses->push_back(nxt);
            }
        }
        
    }
    if(isFree(nx, ny - 1, map)){
        pos nxt(nx, ny - 1, ng, a+1, map);
        int gsize = guesses->size();
        for(int i = 0; i < gsize; i++){
            if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                guesses->insert(guesses->begin() + i, nxt);
                break;
            }else
            if(i == gsize - 1){
                guesses->push_back(nxt);
            }
        }
        
    }
    if(!Qaxis){
        if(isFree(nx + 1, ny + 1, map)){
            pos nxt(nx + 1, ny + 1, ng2, a+1, map);
            int gsize = guesses->size();
            for(int i = 0; i < gsize; i++){
                if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                    guesses->insert(guesses->begin() + i, nxt);
                    break;
                }else
                if(i == gsize - 1){
                    guesses->push_back(nxt);
                }
            
            }
        }
        if(isFree(nx + 1, ny - 1, map)){
            pos nxt(nx + 1, ny - 1, ng2, a+1, map);
            int gsize = guesses->size();
            for(int i = 0; i < gsize; i++){
                if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                    guesses->insert(guesses->begin() + i, nxt);
                    break;
                }else
                if(i == gsize - 1){
                    guesses->push_back(nxt);
                }
            }
            
        }
        if(isFree(nx - 1, ny + 1, map)){
            pos nxt(nx - 1, ny + 1, ng2, a+1, map);
            int gsize = guesses->size();
            for(int i = 0; i < gsize; i++){
                if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                    guesses->insert(guesses->begin() + i, nxt);
                    break;
                }else
                if(i == gsize - 1){
                    guesses->push_back(nxt);
                }
            }
            
        }
        if(isFree(nx - 1, ny - 1, map)){
            pos nxt(nx - 1, ny - 1, ng2, a+1, map);
            int gsize = guesses->size();
            for(int i = 0; i < gsize; i++){
                if(comp(nxt.h, nxt.g, (*guesses)[i].g, (*guesses)[i].h)){
                    guesses->insert(guesses->begin() + i, nxt);
                    break;
                }else
                if(i == gsize - 1){
                    guesses->push_back(nxt);
                }
            }
            
        }
    }
    std::vector<pos>::iterator it = guesses->begin();
    int gsize = guesses->size();
    for(int i = 0; i < gsize; i++){
        if((*guesses)[i].ind == toerase){
            guesses->erase(it+i);
            break;
        }
    }
}

void drawMap(sf::RenderWindow *window, short *map){
    int i,j;
    sf::RectangleShape shape(sf::Vector2f(block, block));
    shape.setOutlineThickness(-0.5);
    for(i = 0; i <  mapSize; i++){
        for(j = 0; j < mapSize; j++){
            shape.setPosition(i * block, j * block);
            if(map[i * mapSize + j] == 4){
                shape.setFillColor(sf::Color::Green);
            }
            if(map[i * mapSize + j] == 3){
                shape.setFillColor(sf::Color::Red);
            }
            if(map[i * mapSize + j] == 2){
                shape.setFillColor(sf::Color::Blue);
            }
            if(map[i * mapSize + j] == 1){
                shape.setFillColor(sf::Color::White);
            }
            if(map[i * mapSize + j] == 0){
                shape.setFillColor(sf::Color::Black);
            }
            window->draw(shape);
        }
    }
}


void setup(bool reset, short* map, posv *guesses){ //initializes the variables used in the algorithm
    int i,j;
    for(i = 0; i < mapSize; i++){
        for(j = 0; j < mapSize; j++){
            if(reset && (map[i * mapSize + j] == 3 || map[i * mapSize + j] == 4))
                map[i * mapSize + j] = 0;
            if(!reset)
                map[i * mapSize + j] = 0;
        }
    }
    map[startx * mapSize + starty] = 2;
    map[endx * mapSize + endy] = 2;
    guesses->push_back(pos(startx, starty, 0.0, 0, map));
}

int main()
{   
    printf("This software was developed by Rubem Nobre @ https://github.com/rubemnobre/\nIt is based on the A* Pathfinding algorithm\nIt is free for all uses\n");
    short map[mapSize * mapSize]; //the map matrix, it holds every point's status. 0-Free 1-Wall 2-Start or End 3-Already considered 4-Chosen path
    int state = 0; //indicates which state the program is at. 0-setup 1-calculation 2-done
    posv guesses; //This vector holds the currently in consideration positions
    posv done;    //This vector holds the already expanded positions
    bool donePath; //Indicates if the back-tracing of the path is done
    clock_t startTime; //Clock time of when the expanding starts
    int finish = -1; //index of the current block in the back-tracing
    bool first; //indicates if this is the first cycle of the pathfinding to record the startTime variable only at the start
    //load font for the on-screen subtitles
    sf::Font font;
    font.loadFromFile(std::string("Vera.ttf"));

    setup(false, map, &guesses);

    sf::RenderWindow window(sf::VideoMode(size, size + 20), "Pathfinder");
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed){
                if(event.key.code == 72){ //right arrow
                    if(state <= 3)state++;
                }
                if(event.key.code == 71){ //left arrow
                    if(state > 0) state--;
                }
            }
        }

        window.clear();
        drawMap(&window, map); //draw the "map" of blocks

        if(state == 0){ //wall-drawing step

            if(sf::Mouse::isButtonPressed(sf::Mouse::Right)){
                sf::Vector2i pos = sf::Mouse::getPosition(window);
                int x, y;
                x = (int) (pos.x / block); //get the corrected coordinates of the mouse
                y = (int) (pos.y / block);
                if(x >= 0 && y >= 0 && x < mapSize && y < mapSize)
                    if(map[x * mapSize + y] == 1)
                        map[x * mapSize + y] = 0; //set the coordinates of the mouse as empty if there is a wall
            }
            if(sf::Mouse::isButtonPressed(sf::Mouse::Left)){
                sf::Vector2i pos = sf::Mouse::getPosition(window);
                int x, y;
                x = (int) (pos.x / block); 
                y = (int) (pos.y / block);
                if(x >= 0 && y >= 0 && x < mapSize && y < mapSize)
                    if(map[x * mapSize + y] == 0)
                        map[x * mapSize + y] = 1; //set the coordinates of the mouse as a wall if it is empty
            }
            sf::Text subtitle(sf::String("Draw map using the Mouse. Use the RMB to remove block. -- Right Arrow to start verification"), font, 10);
            subtitle.setPosition(0, size);
            subtitle.setFillColor(sf::Color::White);
            window.draw(subtitle);
        }

        if(state == 1){ //where the algorithm runs
            do{
                if(first){
                    startTime = clock();
                    first = false;
                }
                next(guesses[0], map, &guesses, &done); //the expanding function
                if(guesses[0].isEnd){ //checks if the best block is in the end position
                    printf("Clock ticks spent: %ld\n", (long)(clock() - startTime));
                    printf("Success! Total distance = %f vs h(0) = %f\n", guesses[0].g, heuristic(startx, starty));
                    finish = guesses[0].ind; //the path's last index, to track the smallest path
                    state = 2;
                    donePath = false;
                    break;
                }
                if(guesses.size() == 0){ //if there are no more blocks in consideration
                    printf("Failed to find a path\n");
                    state = 2;
                    finish = -1;
                    break;
                }
            }while(real); //only runs once if real == 0, else it runs until the break
            sf::Text subtitle(sf::String("Expanding... Right Arrow to Pause"), font, 10);
            subtitle.setPosition(0, size);
            subtitle.setFillColor(sf::Color::White);
            window.draw(subtitle);
        }
        
        if(state == 2){

            while(!donePath && finish != -1){ //"while the path-tracing isn't done" and only run if the finish index has been set
                int x, y;
                x = done[finish].x;
                y = done[finish].y;
                if(!(x == startx && y == starty))
                    map[x * mapSize + y] = 4; //draw block with the path code
                else {
                    printf("Done tracing path\n"); //if the start has been reached
                    donePath = true;
                    break;
                }
                finish = done[finish].ind; //get current block's parent
            }
            sf::Text subtitle(sf::String("Left arrow to unpause, Right arrow to go back to the beginning"), font, 10);
            subtitle.setPosition(0, size);
            subtitle.setFillColor(sf::Color::White);
            window.draw(subtitle);
        }
        if(state == 3){
            guesses.clear();
            finish = -1;
            donePath = false;
            state = 0;
            setup(true, map, &guesses);
        }
        window.display();
    }
    return 0;
}