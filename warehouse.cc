#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <climits>
#include <string>
#include <cstdlib>  
#include <ctime>    


const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const int GRID_SIZE = 40;
const int ROWS = SCREEN_HEIGHT / GRID_SIZE;
const int COLS = SCREEN_WIDTH / GRID_SIZE;
const int ROBOT_RADIUS = GRID_SIZE / 3;
const double ROBOT_SPEED = 2.0;

// Global SDL variables
SDL_Window* window = nullptr;
SDL_Renderer* renderer = nullptr;
TTF_Font* font = nullptr; 

// Toggle between BFS and A* pathfinding
bool useAStar = false;


bool avoidRobotCollisions = false;

// Structure definitions
struct Point {
    int x, y;
    Point(int a, int b) : x(a), y(b) {}
    Point() : x(0), y(0) {}
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    
    int heuristic(const Point& goal) const {
        return std::abs(x - goal.x) + std::abs(y - goal.y);
    }
};


std::vector<Point> findPath(Point start, Point end);
std::vector<Point> findPathA(Point start, Point end);

// Global warehouse grid: 0 = free, 1 = obstacle
std::vector<std::vector<int>> warehouseGrid(ROWS, std::vector<int>(COLS, 0));

// Enum for robot state
enum class RobotState { IDLE, TO_PICKUP, TO_DROPOFF };

// Product structure
struct Product {
    int id;
    Point location;
    bool pickedUp;
    bool delivered;
    Product(int id, int x, int y)
        : id(id), location(x, y), pickedUp(false), delivered(false) {}
};

// Robot structure with autonomous task fields
struct Robot {
    double x, y;           // Continuous position for smooth movement
    Point gridPos;         // Current grid cell
    RobotState state;      // Current task state
    int carryingProductId; // -1 if not carrying a product
    std::vector<Point> path; // Current computed path
    size_t currentPathIndex; // Index into the path vector
    Point destination;     // Current target destination

    Robot(int gridX, int gridY)
        : x(gridX * GRID_SIZE + GRID_SIZE / 2),
          y(gridY * GRID_SIZE + GRID_SIZE / 2),
          gridPos(gridX, gridY),
          state(RobotState::IDLE),
          carryingProductId(-1),
          currentPathIndex(0),
          destination(gridX, gridY)
    {}

    // Moves the robot toward the target cell at the given speed.
    void moveToward(Point target, double speed) {
        double targetX = target.x * GRID_SIZE + GRID_SIZE / 2;
        double targetY = target.y * GRID_SIZE + GRID_SIZE / 2;
        double dx = targetX - x;
        double dy = targetY - y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > speed) {
            x += speed * (dx / dist);
            y += speed * (dy / dist);
        } else {
            x = targetX;
            y = targetY;
            gridPos = target;
        }
    }
};

// Global simulation variables
std::vector<Robot> robots;
std::vector<Product> products;
Point dropOffPoint(COLS - 2, ROWS - 2); 

// Function prototypes
bool initSDL();
void destroySDL();
void renderGrid();
void renderObstacles();
void renderDropOffZone();
void renderRobots();
void renderProducts();
void renderPath(const std::vector<Point>& path, const SDL_Color& color);
void renderText(const std::string& message, int x, int y, SDL_Color color);
void renderInstructions();
void generateObstacles();
void clearObstacles();
Point getRandomFreeCell();
std::vector<Point> getPath(Point start, Point end);


void updateRobots() {
    // For each robot, update its task and movement.
    for (auto &robot : robots) {
        if (robot.state == RobotState::IDLE) {
            // Search for an available product.
            for (auto &prod : products) {
                if (!prod.pickedUp && !prod.delivered) {
                    robot.carryingProductId = prod.id;
                    prod.pickedUp = true;
                    robot.state = RobotState::TO_PICKUP;
                    robot.destination = prod.location;
                    robot.path = getPath(robot.gridPos, robot.destination);
                    robot.currentPathIndex = 0;
                    break;
                }
            }
        }
        else if (robot.state == RobotState::TO_PICKUP) {
            if (robot.path.empty()) {
                robot.path = getPath(robot.gridPos, robot.destination);
                robot.currentPathIndex = 0;
            }
            if (!robot.path.empty() && robot.currentPathIndex < robot.path.size()) {
                robot.moveToward(robot.path[robot.currentPathIndex], ROBOT_SPEED);
                if (robot.gridPos == robot.path[robot.currentPathIndex])
                    robot.currentPathIndex++;
            }
            // When reached the product's location, switch to drop-off task.
            if (robot.gridPos == robot.destination) {
                robot.state = RobotState::TO_DROPOFF;
                robot.destination = dropOffPoint;
                robot.path = getPath(robot.gridPos, robot.destination);
                robot.currentPathIndex = 0;
            }
        }
        else if (robot.state == RobotState::TO_DROPOFF) {
            if (robot.path.empty()) {
                robot.path = getPath(robot.gridPos, robot.destination);
                robot.currentPathIndex = 0;
            }
            if (!robot.path.empty() && robot.currentPathIndex < robot.path.size()) {
                robot.moveToward(robot.path[robot.currentPathIndex], ROBOT_SPEED);
                if (robot.gridPos == robot.path[robot.currentPathIndex])
                    robot.currentPathIndex++;
            }
            // When reached drop-off zone, deliver the product.
            if (robot.gridPos == robot.destination) {
                for (auto &prod : products) {
                    if (prod.id == robot.carryingProductId) {
                        prod.delivered = true;
                        break;
                    }
                }
                robot.carryingProductId = -1;
                robot.state = RobotState::IDLE;
                robot.path.clear();
                robot.currentPathIndex = 0;
            }
        }
    }
}

int main() {
    srand(static_cast<unsigned int>(time(0))); // Seed random number generator
    if (!initSDL()) return 1;


    robots.push_back(Robot(1, 1));
    robots.push_back(Robot(1, ROWS - 2));
    robots.push_back(Robot(COLS - 2, 1));


    SDL_Event e;
    bool quit = false;
    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT)
                quit = true;
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                int gridX = e.button.x / GRID_SIZE;
                int gridY = e.button.y / GRID_SIZE;
                if (e.button.button == SDL_BUTTON_RIGHT) {
                    // Manually toggle an obstacle
                    warehouseGrid[gridY][gridX] = 1 - warehouseGrid[gridY][gridX];
                }
            }
            else if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_t) {
                    useAStar = !useAStar;
                    // Recalculate paths for all robots.
                    for (auto &robot : robots) {
                        robot.path = getPath(robot.gridPos, robot.destination);
                        robot.currentPathIndex = 0;
                    }
                }
                else if (e.key.keysym.sym == SDLK_k) {
                    avoidRobotCollisions = !avoidRobotCollisions;
                    // Recalculate paths for all robots.
                    for (auto &robot : robots) {
                        if (robot.state != RobotState::IDLE) {
                            robot.path = getPath(robot.gridPos, robot.destination);
                            robot.currentPathIndex = 0;
                        }
                    }
                }
                else if (e.key.keysym.sym == SDLK_g) {
                    generateObstacles();
                    for (auto &robot : robots) {
                        if (robot.state != RobotState::IDLE) {
                            robot.path = getPath(robot.gridPos, robot.destination);
                            robot.currentPathIndex = 0;
                        }
                    }
                }
                else if (e.key.keysym.sym == SDLK_c) {
                    clearObstacles();
                    for (auto &robot : robots) {
                        if (robot.state != RobotState::IDLE) {
                            robot.path = getPath(robot.gridPos, robot.destination);
                            robot.currentPathIndex = 0;
                        }
                    }
                }
                else if (e.key.keysym.sym == SDLK_p) {
                    // Spawn a new product at a random free cell.
                    Point freeCell = getRandomFreeCell();
                    int newId = products.empty() ? 1 : products.back().id + 1;
                    products.push_back(Product(newId, freeCell.x, freeCell.y));
                }
                else if (e.key.keysym.sym == SDLK_m) {
                    // Spawn a new robot at a random free cell.
                    Point freeCell = getRandomFreeCell();
                    robots.push_back(Robot(freeCell.x, freeCell.y));
                }
            }
        }


        updateRobots();


        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);
        renderGrid();
        renderObstacles();
        renderDropOffZone();
        // Draw each robot's path.
        for (size_t i = 0; i < robots.size(); i++) {
            SDL_Color pathColor = {255, 215, 0, 255}; // Gold color
            renderPath(robots[i].path, pathColor);
        }
        renderProducts();
        renderRobots();
        renderInstructions();
        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }
    destroySDL();
    return 0;
}

bool initSDL() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " 
                  << SDL_GetError() << std::endl;
        return false;
    }
    if (TTF_Init() == -1) {
        std::cerr << "SDL_ttf could not initialize! TTF_Error: " 
                  << TTF_GetError() << std::endl;
        return false;
    }
    window = SDL_CreateWindow("Warehouse Robot Simulation", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
                              SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Window could not be created! SDL_Error: " 
                  << SDL_GetError() << std::endl;
        return false;
    }
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL_Error: " 
                  << SDL_GetError() << std::endl;
        return false;
    }
    font = TTF_OpenFont("ARIAL.TTF", 16);
    if (!font) {
        std::cerr << "Failed to load font! TTF_Error: " 
                  << TTF_GetError() << std::endl;
        return false;
    }
    return true;
}

void destroySDL() {
    TTF_CloseFont(font);
    font = nullptr;
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void renderGrid() {
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    for (int i = 0; i <= ROWS; ++i)
        SDL_RenderDrawLine(renderer, 0, i * GRID_SIZE, SCREEN_WIDTH, i * GRID_SIZE);
    for (int j = 0; j <= COLS; ++j)
        SDL_RenderDrawLine(renderer, j * GRID_SIZE, 0, j * GRID_SIZE, SCREEN_HEIGHT);
}

void renderObstacles() {
    SDL_SetRenderDrawColor(renderer, 200, 50, 50, 255);
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            if (warehouseGrid[i][j] == 1) {
                SDL_Rect rect = { j * GRID_SIZE, i * GRID_SIZE, GRID_SIZE, GRID_SIZE };
                SDL_RenderFillRect(renderer, &rect);
            }
        }
    }
}

void renderDropOffZone() {
    SDL_SetRenderDrawColor(renderer, 50, 50, 200, 255);
    SDL_Rect rect = { dropOffPoint.x * GRID_SIZE, dropOffPoint.y * GRID_SIZE, GRID_SIZE, GRID_SIZE };
    SDL_RenderFillRect(renderer, &rect);
}

void renderRobots() {
    for (size_t i = 0; i < robots.size(); i++) {
        SDL_Color color;
        switch(i % 3) {
            case 0: color = {50, 200, 50, 255}; break;
            case 1: color = {200, 200, 50, 255}; break;
            case 2: color = {50, 200, 200, 255}; break;
            default: color = {255, 255, 255, 255}; break;
        }
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_Rect rect = { static_cast<int>(robots[i].x - ROBOT_RADIUS),
                          static_cast<int>(robots[i].y - ROBOT_RADIUS),
                          ROBOT_RADIUS * 2, ROBOT_RADIUS * 2 };
        SDL_RenderFillRect(renderer, &rect);
        // Draw a small red indicator if carrying a product.
        if (robots[i].carryingProductId != -1) {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_Rect indicator = { static_cast<int>(robots[i].x) - 5,
                                   static_cast<int>(robots[i].y) - 5, 10, 10 };
            SDL_RenderFillRect(renderer, &indicator);
        }
    }
}

void SDL_RenderFillCircle(SDL_Renderer* renderer, int centerX, int centerY, int radius) {
  for (int w = 0; w < radius * 2; w++) {
      for (int h = 0; h < radius * 2; h++) {
          int dx = radius - w;
          int dy = radius - h;
          if ((dx * dx + dy * dy) <= (radius * radius)) {
              SDL_RenderDrawPoint(renderer, centerX + dx, centerY + dy);
          }
      }
  }
}

void renderProducts() {
    SDL_SetRenderDrawColor(renderer, 255, 140, 0, 255);
    for (const auto &prod : products) {
        if (!prod.pickedUp && !prod.delivered) {
            SDL_Rect rect = { prod.location.x * GRID_SIZE + GRID_SIZE/4,
                              prod.location.y * GRID_SIZE + GRID_SIZE/4,
                              GRID_SIZE/2, GRID_SIZE/2 };
            SDL_RenderFillCircle(renderer, prod.location.x * GRID_SIZE + GRID_SIZE/2,
                                 prod.location.y * GRID_SIZE + GRID_SIZE/2, GRID_SIZE/4);
        }
    }
}

void renderPath(const std::vector<Point>& path, const SDL_Color& color) {
    if (path.empty()) return;
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (const auto &p : path) {
        SDL_Rect rect = { p.x * GRID_SIZE + GRID_SIZE/3,
                          p.y * GRID_SIZE + GRID_SIZE/3,
                          GRID_SIZE/3, GRID_SIZE/3 };
        SDL_RenderFillRect(renderer, &rect);
    }
}

void renderText(const std::string& message, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Blended(font, message.c_str(), color);
    if (!surface) {
        std::cerr << "Failed to render text surface! TTF_Error: " 
                  << TTF_GetError() << std::endl;
        return;
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_Rect dstRect = { x, y, surface->w, surface->h };
    SDL_FreeSurface(surface);
    SDL_RenderCopy(renderer, texture, nullptr, &dstRect);
    SDL_DestroyTexture(texture);
}

void renderInstructions() {
    SDL_Color white = {255, 255, 255, 255};
    std::string algo = useAStar ? "A*" : "BFS";
    std::string collision = avoidRobotCollisions ? "ON" : "OFF";
    renderText("T: Toggle Algorithm (Current: " + algo + ")", 10, 5, white);
    renderText("K: Toggle Robot Collision Avoidance (Current: " + collision + ")", 10, 25, white);
    renderText("G: Generate Obstacles   C: Clear Obstacles", 10, 45, white);
    renderText("P: Spawn Product   M: Spawn Robot", 10, 65, white);
    // Display delivered products count.
    int deliveredCount = 0;
    for (const auto &prod : products)
        if (prod.delivered) deliveredCount++;
    renderText("Delivered Products: " + std::to_string(deliveredCount), 10, 85, white);
}

void generateObstacles() {
    // Randomly place obstacles in free cells (30% chance), leaving reserved cells free.
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            bool reserved = false;
            if (Point(j, i) == dropOffPoint) reserved = true;
            for (const auto &robot : robots) {
                if (robot.gridPos == Point(j, i)) { reserved = true; break; }
            }
            for (const auto &prod : products) {
                if (prod.location == Point(j, i)) { reserved = true; break; }
            }
            if (!reserved)
                warehouseGrid[i][j] = (rand() % 100 < 30) ? 1 : 0;
        }
    }
}

void clearObstacles() {
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++)
            warehouseGrid[i][j] = 0;
}

// Returns a random free cell 
Point getRandomFreeCell() {
    while (true) {
        int x = rand() % COLS;
        int y = rand() % ROWS;
        if (warehouseGrid[y][x] == 0) {
            if (Point(x, y) == dropOffPoint) continue;
            bool occupied = false;
            for (const auto &robot : robots)
                if (robot.gridPos == Point(x, y)) { occupied = true; break; }
            if (occupied) continue;
            return Point(x, y);
        }
    }
}

// Choose a path using the selected algorithm.
std::vector<Point> getPath(Point start, Point end) {
    return useAStar ? findPathA(start, end) : findPath(start, end);
}
std::vector<Point> findPath(Point start, Point end) {
  std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
  std::vector<std::vector<Point>> parents(ROWS, std::vector<Point>(COLS, {-1, -1}));
  std::queue<Point> queue;
  queue.push(start);
  visited[start.y][start.x] = true;
  const int dx[] = {0, 1, 0, -1};
  const int dy[] = {-1, 0, 1, 0};
  while (!queue.empty()) {
      Point current = queue.front();
      queue.pop();
      if (current == end) {
          std::vector<Point> path;
          while (!(current == start)) {
              path.push_back(current);
              current = parents[current.y][current.x];
          }
          std::reverse(path.begin(), path.end());
          return path;
      }
      for (int i = 0; i < 4; i++) {
          int nx = current.x + dx[i], ny = current.y + dy[i];
          if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS)
              continue;
          if (warehouseGrid[ny][nx] == 1)
              continue;
          bool skip = false;
          if (avoidRobotCollisions) {
              if (!(Point(nx, ny) == start) && !(Point(nx, ny) == end)) {
                  for (const auto &r : robots) {
                      if (r.state == RobotState::IDLE) continue;
                      // Check current grid position
                      if (r.gridPos.x == nx && r.gridPos.y == ny) {
                          skip = true;
                          break;
                      }
                      // Check if the cell is in the robot's path
                      for (const Point &p : r.path) {
                          if (p.x == nx && p.y == ny) {
                              skip = true;
                              break;
                          }
                      }
                      if (skip) break;
                  }
              }
          }
          if (skip)
              continue;
          if (!visited[ny][nx]) {
              queue.push(Point(nx, ny));
              visited[ny][nx] = true;
              parents[ny][nx] = current;
          }
      }
  }
  return {}; // No path found.
}

std::vector<Point> findPathA(Point start, Point end) {
  std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
  std::vector<std::vector<Point>> parents(ROWS, std::vector<Point>(COLS, {-1, -1}));
  auto comparator = [](const std::pair<Point, int>& a, const std::pair<Point, int>& b) {
      return a.second > b.second;
  };
  std::priority_queue<std::pair<Point, int>, std::vector<std::pair<Point, int>>, decltype(comparator)> openList(comparator);
  std::vector<std::vector<int>> gCost(ROWS, std::vector<int>(COLS, INT_MAX));
  openList.push({start, 0});
  gCost[start.y][start.x] = 0;
  const int dx[] = {0, 1, 0, -1};
  const int dy[] = {-1, 0, 1, 0};
  while (!openList.empty()) {
      Point current = openList.top().first;
      openList.pop();
      if (current == end) {
          std::vector<Point> path;
          while (!(current == start)) {
              path.push_back(current);
              current = parents[current.y][current.x];
          }
          std::reverse(path.begin(), path.end());
          return path;
      }
      visited[current.y][current.x] = true;
      for (int i = 0; i < 4; i++) {
          int nx = current.x + dx[i], ny = current.y + dy[i];
          if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS)
              continue;
          if (warehouseGrid[ny][nx] != 0)
              continue;
          bool skip = false;
          if (avoidRobotCollisions) {
              if (!(Point(nx, ny) == start) && !(Point(nx, ny) == end)) {
                  for (const auto &r : robots) {
                      if (r.state == RobotState::IDLE) continue;
                      // Check current grid position
                      if (r.gridPos.x == nx && r.gridPos.y == ny) {
                          skip = true;
                          break;
                      }
                      // Check if the cell is in the robot's path
                      for (const Point &p : r.path) {
                          if (p.x == nx && p.y == ny) {
                              skip = true;
                              break;
                          }
                      }
                      if (skip) break;
                  }
              }
          }
          if (skip)
              continue;
          int newGCost = gCost[current.y][current.x] + 1;
          int hCost = std::abs(nx - end.x) + std::abs(ny - end.y);
          int fCost = newGCost + hCost;
          if (newGCost < gCost[ny][nx]) {
              parents[ny][nx] = current;
              gCost[ny][nx] = newGCost;
              openList.push({Point(nx, ny), fCost});
          }
      }
  }
  return {}; 
}