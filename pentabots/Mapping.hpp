#pragma once

#include <Arduino.h>
#include "Driving.hpp"
#include "Turning.hpp"
#include "EncoderOdometry.hpp"

#define MAZE_SIZE 9
#define WALL_THRESHOLD 100 
#define UNVISITED_PENALTY 1000  

namespace mtrn3100 {

enum Direction { NORTH, EAST, SOUTH, WEST };

struct Cell {
    bool visited = false;
    bool walls[4] = {true, true, true, true};
    uint16_t distance = 0;
};

class MazeMap {
public:
    Cell grid[MAZE_SIZE][MAZE_SIZE];
    float completion = 0.0;
    uint8_t startRow = 0;
    uint8_t startCol = 0;
    uint8_t goalRow = 4;
    uint8_t goalCol = 7;
    
    void initialize() {
        for(int i=0; i<MAZE_SIZE; i++) {
            grid[0][i].walls[NORTH] = true;
            grid[MAZE_SIZE-1][i].walls[SOUTH] = true;
            grid[i][0].walls[WEST] = true;
            grid[i][MAZE_SIZE-1].walls[EAST] = true;
        }
    }
    
    void updateCompletion() {
        int visited = 0;
        for(int i=0; i<MAZE_SIZE; i++) {
            for(int j=0; j<MAZE_SIZE; j++) {
                if(grid[i][j].visited) visited++;
            }
        }
        completion = (visited * 100.0) / (MAZE_SIZE * MAZE_SIZE);
    }

    // void display() {
    //     Serial.print("Mapping: ");
    //     Serial.print(completion);
    //     Serial.println("% complete");
        
    //     // Simple ASCII visualization
    //     for(int i=0; i<MAZE_SIZE; i++) {
    //         // North walls
    //         for(int j=0; j<MAZE_SIZE; j++) {
    //             Serial.print("+");
    //             Serial.print(grid[i][j].walls[NORTH] ? "---" : "   ");
    //         }
    //         Serial.println("+");
            
    //         // West walls and cell content
    //         for(int j=0; j<MAZE_SIZE; j++) {
    //             Serial.print(grid[i][j].walls[WEST] ? "|" : " ");
    //             Serial.print(grid[i][j].visited ? " X " : " . ");
    //         }
    //         Serial.println(grid[i][MAZE_SIZE-1].walls[EAST] ? "|" : " ");
    //     }
        
    //     // Bottom walls
    //     for(int j=0; j<MAZE_SIZE; j++) {
    //         Serial.print("+");
    //         Serial.print(grid[MAZE_SIZE-1][j].walls[SOUTH] ? "---" : "   ");
    //     }
    //     Serial.println("+");
    // }
};

class AutonomousMapper {
public:
    AutonomousMapper(Driving& drive, Turning& turn, EncoderOdometry& odom, uint8_t startRow, uint8_t startCol, uint8_t goalRow, uint8_t goalCol) 
        : driver(drive), turner(turn), odometer(odom) {
        maze.startRow = startRow;
        maze.startCol = startCol;
        maze.goalRow = goalRow;
        maze.goalCol = goalCol;
        maze.initialize();
    }
    
    void exploreMaze() {
        currentRow = maze.startRow;
        currentCol = maze.startCol;
        currentDir = NORTH;
        
        while(maze.completion < 95.0) { // Until mostly mapped
            updateWalls();
            
            uint8_t nextDir = chooseNextDirection();
            moveToCell(nextDir);
            maze.updateCompletion();
            //maze.display();
            
            delay(500); /
        }
        
        returnToStart();
    }
    
    void solveShortestPath() {
        floodFill();
        followShortestPath();
    }

private:
    void updateWalls() {
        Cell& current = maze.grid[currentRow][currentCol];
        driver.updateLidar();
        
        // Front wall
        if(driver.getFrontDist() < WALL_THRESHOLD) {
            current.walls[currentDir] = true;
        } else {
            current.walls[currentDir] = false;
        }
        
        // Right wall
        if(driver.getRightDist() < WALL_THRESHOLD) {
            current.walls[(currentDir + 1) % 4] = true;
        } else {
            current.walls[(currentDir + 1) % 4] = false;
        }
        
        // Left wall
        if(driver.getLeftDist() < WALL_THRESHOLD) {
            current.walls[(currentDir + 3) % 4] = true;
        } else {
            current.walls[(currentDir + 3) % 4] = false;
        }
        
        current.visited = true;
    }
    
    uint8_t chooseNextDirection() {
        Cell& current = maze.grid[currentRow][currentCol];
        
        // Check adjacent cells
        bool canGoForward = !current.walls[currentDir];
        bool canGoRight = !current.walls[(currentDir + 1) % 4];
        bool canGoLeft = !current.walls[(currentDir + 3) % 4];
        
        // Prefer unvisited cells
        if(canGoForward && isCellUnvisited(currentDir)) return currentDir;
        if(canGoRight && isCellUnvisited((currentDir + 1) % 4)) return (currentDir + 1) % 4;
        if(canGoLeft && isCellUnvisited((currentDir + 3) % 4)) return (currentDir + 3) % 4;
        
        // Default to wall follower (right-hand rule)
        if(canGoRight) return (currentDir + 1) % 4;
        if(canGoForward) return currentDir;
        if(canGoLeft) return (currentDir + 3) % 4;
        
        // Dead end - turn around
        return (currentDir + 2) % 4;
    }
    
    bool isCellUnvisited(uint8_t dir) {
        int8_t newRow = currentRow;
        int8_t newCol = currentCol;
        
        switch(dir) {
            case NORTH: newRow--; break;
            case EAST: newCol++; break;
            case SOUTH: newRow++; break;
            case WEST: newCol--; break;
        }
        
        if(newRow < 0 || newRow >= MAZE_SIZE || newCol < 0 || newCol >= MAZE_SIZE) {
            return false;
        }
        
        return !maze.grid[newRow][newCol].visited;
    }
    
    void moveToCell(uint8_t dir) {
        int8_t turnAngle = (dir - currentDir) * 90;
        if(turnAngle > 180) turnAngle -= 360;
        if(turnAngle < -180) turnAngle += 360;
        
        if(turnAngle != 0) {
            turner.turn(turnAngle);
            currentDir = dir;
        }
        
        driver.drive(1);
        
        switch(currentDir) {
            case NORTH: currentRow--; break;
            case EAST: currentCol++; break;
            case SOUTH: currentRow++; break;
            case WEST: currentCol--; break;
        }
    }
    
    void floodFill() {
        // Initialize distances
        for(int i=0; i<MAZE_SIZE; i++) {
            for(int j=0; j<MAZE_SIZE; j++) {
                maze.grid[i][j].distance = UNVISITED_PENALTY;
            }
        }
        
        // Set goal cell distance to 0
        maze.grid[maze.goalRow][maze.goalCol].distance = 0;
        
        // Propagate distances
        bool changed;
        do {
            changed = false;
            for(int i=0; i<MAZE_SIZE; i++) {
                for(int j=0; j<MAZE_SIZE; j++) {
                    if(maze.grid[i][j].distance == UNVISITED_PENALTY) continue;
                    
                    // Check all neighbors
                    for(int d=0; d<4; d++) {
                        if(!maze.grid[i][j].walls[d]) { // No wall in this direction
                            int8_t ni = i, nj = j;
                            switch(d) {
                                case NORTH: ni--; break;
                                case EAST: nj++; break;
                                case SOUTH: ni++; break;
                                case WEST: nj--; break;
                            }
                            
                            if(ni >= 0 && ni < MAZE_SIZE && nj >= 0 && nj < MAZE_SIZE) {
                                if(maze.grid[ni][nj].distance > maze.grid[i][j].distance + 1) {
                                    maze.grid[ni][nj].distance = maze.grid[i][j].distance + 1;
                                    changed = true;
                                }
                            }
                        }
                    }
                }
            }
        } while(changed);
    }
    
    void followShortestPath() {
        currentRow = maze.startRow;
        currentCol = maze.startCol;
        currentDir = NORTH;
        
        while(currentRow != maze.goalRow || currentCol != maze.goalCol) {
            Cell& current = maze.grid[currentRow][currentCol];
            
            uint8_t bestDir = currentDir;
            uint16_t bestDist = current.distance;
            
            for(int d=0; d<4; d++) {
                if(!current.walls[d]) { 
                    int8_t ni = currentRow, nj = currentCol;
                    switch(d) {
                        case NORTH: ni--; break;
                        case EAST: nj++; break;
                        case SOUTH: ni++; break;
                        case WEST: nj--; break;
                    }
                    
                    if(ni >= 0 && ni < MAZE_SIZE && nj >= 0 && nj < MAZE_SIZE) {
                        if(maze.grid[ni][nj].distance < bestDist) {
                            bestDist = maze.grid[ni][nj].distance;
                            bestDir = d;
                        }
                    }
                }
            }
            
            moveToCell(bestDir);
            delay(500); 
        }
    }
    
    void returnToStart() {
        // Simple implementation - could use flood fill to find path back
        while(currentRow != maze.startRow || currentCol != maze.startCol) {
            //// Implement function
        }
    }

    MazeMap maze;
    Driving& driver;
    Turning& turner;
    EncoderOdometry& odometer;
    uint8_t currentRow;
    uint8_t currentCol;
    uint8_t currentDir;
};

} // namespace mtrn3100