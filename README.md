# Warehouse Robot Simulation

This project is a C++ based simulation of warehouse robots performing product delivery tasks within a grid-based warehouse environment. It utilizes SDL2 for graphics rendering to visualize the robots, products, obstacles, and pathfinding algorithms in action. The simulation supports pathfinding using both Breadth-First Search (BFS) and A* algorithms, obstacle generation and clearing, and dynamic robot collision avoidance.

## Getting Started

### Prerequisites
Ensure you have the following installed:
- C++ Compiler (e.g., g++)
- [SDL2](https://www.libsdl.org/)
- [SDL2_ttf](https://www.libsdl.org/projects/SDL_ttf/)


## Project Structure

*   `warehouse.cc`: Main C++ source file.
*   `Makefile`: Build the project.
*   `src/`
    *   `include/`: Header files.

### Installation
1. Clone the repository:
    ```bash
    git clone git@github.com:Luke23-45/Simulation-of-warehouse-working-robots-with-SDL2.git
    ```
## Building

This project uses `make` for building. To build the project, run the following command in your terminal:

1. Navigate to the project directory:
    ```bash
    cd Simulation-of-warehouse-working-robots-with-SDL2
    ```
3. Compile the code:
    ```bash
     make
    ```
4. Run the executable:
    ```bash
    ./main

    ```
5. In window:
    ```bash
    main.exe
    ```
6. To clean up the build artifacts
    ```bash
     make clean
    ```

## Features
- **Warehouse Simulation:** Visual representation of a warehouse environment using a grid.
- **Robot Agents:** Simulates multiple robot agents tasked with product delivery.
- **Product Delivery:** Products are spawned randomly and robots are assigned to pick them up and deliver them to a designated drop-off zone.
- **Pathfinding Algorithms:** Implements both Breadth-First Search (BFS) and A* search algorithms for robot path planning. Users can toggle between algorithms during runtime.
- **Obstacle Management:**
    - **Obstacle Generation:** Allows for random generation of obstacles within the warehouse.
    - **Obstacle Clearing:** Provides functionality to clear all generated obstacles.
    - **Manual Obstacle Placement:** Users can manually toggle obstacles on and off by right-clicking on grid cells.
- **Robot Collision Avoidance:** Features an option to enable robot collision avoidance during pathfinding, dynamically adjusting paths to prevent robots from occupying the same cell.
- **Real-time Visualization:** Utilizes SDL2 to render the warehouse grid, robots, products, obstacles, paths, and simulation status in real-time.
- **Informative UI:** Displays on-screen instructions, current pathfinding algorithm, collision avoidance status, and the count of delivered products.

## Key Controls

| Action              | Key       | Description                                                    |
| ------------------- | --------- | -------------------------------------------------------------- |
| Exit simulation     | `ESC` key | Closes the simulation window and terminates the program.      |
| Toggle Algorithm    | `T` key   | Switches between BFS and A* pathfinding algorithms.           |
| Toggle Collision Avoidance | `K` key   | Enables or disables robot collision avoidance during path planning. |
| Generate Obstacles  | `G` key   | Randomly generates obstacles within the warehouse grid.        |
| Clear Obstacles     | `C` key   | Removes all obstacles from the warehouse grid.                 |
| Spawn Product       | `P` key   | Creates a new product at a random free cell in the warehouse. |
| Spawn Robot         | `M` key   | Introduces a new robot into the warehouse at a random free cell.|
| Toggle Obstacle     | Right Mouse Click | Adds or removes an obstacle at the clicked grid cell.     |

## Code Structure
The project is structured as follows:

- **`warehouse.cc`**: Contains the main function and program loop, SDL initialization and destruction, event handling, game logic updates, and rendering functions.
- **Structures**: Defines key data structures:
    - `Point`: Represents grid coordinates.
    - `Product`: Stores information about products, including location and delivery status.
    - `Robot`: Manages robot properties such as position, state (IDLE, TO\_PICKUP, TO\_DROPOFF), path, and current task.
- **Enums**:
    - `RobotState`: Defines the possible states for robots in the simulation.
- **Global Variables**: Manages global simulation states, including:
    - `warehouseGrid`: 2D vector representing the warehouse layout (0: free, 1: obstacle).
    - `robots`: Vector of `Robot` objects in the simulation.
    - `products`: Vector of `Product` objects.
    - `dropOffPoint`: Destination point for product delivery.
    - `useAStar`: Boolean to toggle between BFS and A* pathfinding.
    - `avoidRobotCollisions`: Boolean to enable/disable robot collision avoidance.
- **Functions**:
    - **SDL Management**: `initSDL()`, `destroySDL()`: Initializes and quits SDL and SDL\_ttf libraries, and manages window and renderer creation/destruction.
    - **Rendering Functions**: `renderGrid()`, `renderObstacles()`, `renderDropOffZone()`, `renderRobots()`, `renderProducts()`, `renderPath()`, `renderText()`, `renderInstructions()`:  Handle all visual aspects of the simulation, drawing the grid, obstacles, robots, products, paths, text instructions, and status on the screen using SDL2.
    - **Pathfinding**: `findPath()`, `findPathA()`, `getPath()`: Implement BFS and A* pathfinding algorithms to calculate paths for robots, considering obstacles and optionally robot collision avoidance.
    - **Simulation Logic**: `updateRobots()`: Controls the behavior of each robot, including task assignment (pickup, drop-off), path planning, and movement.
    - **Obstacle and Product Management**: `generateObstacles()`, `clearObstacles()`, `getRandomFreeCell()`: Functions for generating random obstacles, clearing obstacles, and finding random free cells for product and robot spawning.
- **Pathfinding Algorithms**: Implements `findPath` (BFS) and `findPathA` (A*) for path calculation. These algorithms search for the shortest path from a robot's starting position to its destination, considering warehouse obstacles and, optionally, other robots to avoid collisions.

## Demo Video
Check out the project demo video on YouTube: [Project Demo Video](https://www.youtube.com/watch?v=McOcbGHyAWA)
## License

This project is licensed under the MIT License. Feel free to use, modify, and distribute the code.

## Acknowledgements

- SDL2 for graphics rendering.
- SDL2\_ttf for text rendering.
