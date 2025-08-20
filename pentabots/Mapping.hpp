#pragma once
#include <Arduino.h>
#include <VL6180X.h>
#include "Driving.hpp"
#include "Turning.hpp"

#define SIZE 9         
#define LARGEVAL 255

enum Side { TOP, BOTTOM, LEFT, RIGHT };

namespace mtrn3100 {

// --- OPTIMIZED NODE STRUCT ---
// The boolean flags have been packed into a single byte ('flags') to save SRAM.
// Helper functions are provided for easy access.
struct Node {
  uint8_t row, column;       // coordinates
  uint8_t floodval;          // floodfill value
  uint8_t flags;           // Holds visited, wallUp, wallDown, wallLeft, wallRight

  // --- Helper methods to get and set flags from the 'flags' byte ---
  
  // Bit 0: visited
  bool getVisited() const { return (flags >> 0) & 1; }
  void setVisited(bool val) { flags = (flags & ~0x01) | (val << 0); }

  // Bit 1: wallUp
  bool getWallUp() const { return (flags >> 1) & 1; }
  void setWallUp(bool val) { flags = (flags & ~0x02) | (val << 1); }

  // Bit 2: wallDown
  bool getWallDown() const { return (flags >> 2) & 1; }
  void setWallDown(bool val) { flags = (flags & ~0x04) | (val << 2); }
  
  // Bit 3: wallLeft
  bool getWallLeft() const { return (flags >> 3) & 1; }
  void setWallLeft(bool val) { flags = (flags & ~0x08) | (val << 3); }

  // Bit 4: wallRight
  bool getWallRight() const { return (flags >> 4) & 1; }
  void setWallRight(bool val) { flags = (flags & ~0x10) | (val << 4); }
};

struct Maze {
  Node map[SIZE][SIZE];
};

struct Position {
  uint8_t x;
  uint8_t y;

  bool operator==(const Position& other) const {
    return (x == other.x) && (y == other.y);
  }
};

class Mapping {
public:
  // CHANGE HERE FOR START AND GOAL POSITIONS
  uint8_t currentX;
  uint8_t currentY;
  uint8_t startX = 3;
  uint8_t startY = 3;
  uint8_t goalX  = 3;
  uint8_t goalY  = 8;

  Mapping(mtrn3100::Driving& drive, mtrn3100::Turning& turn)
    : driveController(drive), turnController(turn) {

    initialize_maze();

    currentX = startX;
    currentY = startY;

    // Remove the 4 corner cells
    remove_cell(0, 0);   
    remove_cell(1, 0);   
    remove_cell(0, 1);   

    remove_cell(7, 0);
    remove_cell(8, 0);   
    remove_cell(8, 1);  

    remove_cell(0, 7);   
    remove_cell(0, 8);   
    remove_cell(1, 8);  

    remove_cell(7, 8);   
    remove_cell(8, 7);   
    remove_cell(8, 8);

    remove_sides();

    // Initialize flood values
    for (uint8_t x = 0; x < SIZE; ++x) {
      for (uint8_t y = 0; y < SIZE; ++y) {
        maze.map[x][y].floodval = LARGEVAL;
      }
    }
    maze.map[goalX][goalY].floodval = 0;
  }

  bool reachedGoal() {
    return getCurrentPosition() == getGoalPosition();
  }
  void setVisited(uint8_t x, uint8_t y, bool val) {
    maze.map[x][y].setVisited(val);
  }

  bool getVisited(uint8_t x, uint8_t y) {
    return maze.map[x][y].getVisited();
  }

  void initialize_maze() {
    for (uint8_t x = 0; x < SIZE; ++x) {
      for (uint8_t y = 0; y < SIZE; ++y) {
        Node& n = maze.map[x][y];
        n.row = y;
        n.column = x;
        n.flags = 0; // This sets visited and all walls to false (0) at once.

        // initialize to Manhattan distance
        n.floodval = abs(x - goalX) + abs(y - goalY);
      }
    }
  }

  // Get the smallest floodval among valid neighbors
  uint8_t get_smallest_neighbor(uint8_t x, uint8_t y) {
    uint8_t smallest = LARGEVAL;
    
    // Updated to use getter methods
    if (x > 0 && !maze.map[x][y].getWallLeft()) 
      smallest = min(smallest, maze.map[x-1][y].floodval);

    if (x < SIZE-1 && !maze.map[x][y].getWallRight())
      smallest = min(smallest, maze.map[x+1][y].floodval);

    if (y > 0 && !maze.map[x][y].getWallDown())
      smallest = min(smallest, maze.map[x][y-1].floodval);

    if (y < SIZE-1 && !maze.map[x][y].getWallUp())
      smallest = min(smallest, maze.map[x][y+1].floodval);

    return smallest;
  }

  // Update walls using lidar
  // void update_walls(uint8_t x, uint8_t y) {
  //   Node& n = maze.map[x][y];
  //   driveController.updateLidar();

  //   // Updated to use setter methods
  //   if (driveController.getFrontDist() < 150) n.setWallUp(true);
  //   if (driveController.getLeftDist() < 100) n.setWallLeft(true);
  //   if (driveController.getRightDist() < 100) n.setWallRight(true);
  // }

  // void update_walls(uint8_t x, uint8_t y, char orientation) {
  //   Node& n = maze.map[x][y];
  //   driveController.updateLidar();

  //   bool frontWall = driveController.getFrontDist() < 150;
  //   bool leftWall = driveController.getLeftDist() < 100;
  //   bool rightWall = driveController.getRightDist() < 100;

  //   switch (orientation) {
  //       case 'U': // Facing Up (Positive Y)
  //           if (frontWall) n.setWallUp(true);
  //           if (leftWall) n.setWallLeft(true);
  //           if (rightWall) n.setWallRight(true);
  //           break;
  //       case 'R': // Facing Right (Positive X)
  //           if (frontWall) n.setWallRight(true);
  //           if (leftWall) n.setWallUp(true);
  //           if (rightWall) n.setWallDown(true);
  //           break;
  //       case 'D': // Facing Down (Negative Y)
  //           if (frontWall) n.setWallDown(true);
  //           if (leftWall) n.setWallRight(true);
  //           if (rightWall) n.setWallLeft(true);
  //           break;
  //       case 'L': // Facing Left (Negative X)
  //           if (frontWall) n.setWallLeft(true);
  //           if (leftWall) n.setWallDown(true);
  //           if (rightWall) n.setWallUp(true);
  //           break;
  //   }
  // }


  void update_walls(uint8_t x, uint8_t y, char orientation) {
    Node& n = maze.map[x][y];
    driveController.updateLidar();

    bool frontWall = driveController.getFrontDist() < 150;
    bool leftWall = driveController.getLeftDist() < 100;
    bool rightWall = driveController.getRightDist() < 100;

    switch (orientation) {
        case 'U': // Facing Up (Positive Y)
            if (frontWall) {
                n.setWallUp(true);
                if (y > 0) maze.map[x][y - 1].setWallDown(true); // Add this
            }
            if (leftWall) {
                n.setWallLeft(true);
                if (x > 0) maze.map[x - 1][y].setWallRight(true); // Add this
            }
            if (rightWall) {
                n.setWallRight(true);
                if (x < SIZE - 1) maze.map[x + 1][y].setWallLeft(true); // Add this
            }
            break;
        case 'R': // Facing Right (Positive X)
            if (frontWall) {
                n.setWallRight(true);
                if (x < SIZE - 1) maze.map[x + 1][y].setWallLeft(true); // Add this
            }
            if (leftWall) {
                n.setWallUp(true);
                if (y > 0) maze.map[x][y - 1].setWallDown(true); // Add this
            }
            if (rightWall) {
                n.setWallDown(true);
                if (y < SIZE - 1) maze.map[x][y + 1].setWallUp(true); // Add this
            }
            break;
        case 'D': // Facing Down (Negative Y)
            if (frontWall) {
                n.setWallDown(true);
                if (y < SIZE - 1) maze.map[x][y + 1].setWallUp(true); // Add this
            }
            if (leftWall) {
                n.setWallRight(true);
                if (x < SIZE - 1) maze.map[x + 1][y].setWallLeft(true); // Add this
            }
            if (rightWall) {
                n.setWallLeft(true);
                if (x > 0) maze.map[x - 1][y].setWallRight(true); // Add this
            }
            break;
        case 'L': // Facing Left (Negative X)
            if (frontWall) {
                n.setWallLeft(true);
                if (x > 0) maze.map[x - 1][y].setWallRight(true); // Add this
            }
            if (leftWall) {
                n.setWallDown(true);
                if (y < SIZE - 1) maze.map[x][y + 1].setWallUp(true); // Add this
            }
            if (rightWall) {
                n.setWallUp(true);
                if (y > 0) maze.map[x][y - 1].setWallDown(true); // Add this
            }
            break;
    }
}

  // Floodfill propagation
  void propagate_floodfill() {
    bool updated;
    do {
      updated = false;
      for (uint8_t x = 0; x < SIZE; ++x) {
        for (uint8_t y = 0; y < SIZE; ++y) {
          // The goal cell's value is always 0 and should never change.
          if (x == goalX && y == goalY) continue;

          uint8_t minNeighbor = get_smallest_neighbor(x, y);
          uint8_t newValue;

          // --- FIX IS HERE ---
          // If the smallest neighbor is LARGEVAL, this cell is unreachable.
          // Its value should also be LARGEVAL.
          if (minNeighbor == LARGEVAL) {
              newValue = LARGEVAL;
          } else {
              // Otherwise, its value is one more than its best neighbor.
              // This calculation is now safe from overflow.
              newValue = minNeighbor + 1;
          }

          // Only update and set the flag if the value has actually changed.
          if (maze.map[x][y].floodval != newValue) {
            maze.map[x][y].floodval = newValue;
            updated = true;
          }
        }
      }
    } while (updated);
  }
  // Decide the next move based on flood values
  char decide_next_move(uint8_t x, uint8_t y) {
    uint8_t smallest = get_smallest_neighbor(x, y);

    // 'U'p, 'L'eft, 'R'ight, 'D'own
    if (y < SIZE-1 && !maze.map[x][y].getWallUp() && maze.map[x][y-1].floodval == smallest) return 'U';
    if (x > 0 && !maze.map[x][y].getWallLeft() && maze.map[x-1][y].floodval == smallest) return 'L';
    if (x < SIZE-1 && !maze.map[x][y].getWallRight() && maze.map[x+1][y].floodval == smallest) return 'R';
    if (y > 0 && !maze.map[x][y].getWallDown() && maze.map[x][y+1].floodval == smallest) return 'D';

    return 'X'; // No valid move
  }

  // Update robot position after a move
  void update_position(char direction) {
    if (direction == 'U') {
      currentY -= 1; // Assuming Y increases upwards in your coordinate system
    } else if (direction == 'L') {
      currentX -= 1;
    } else if (direction == 'R') {
      currentX += 1;
    } else if (direction == 'D') {
      currentY += 1;
    }
    if (direction != 'X') {
        maze.map[currentX][currentY].setVisited(true);
    }
  }

  // Mark a cell as blocked
  void remove_cell(uint8_t x, uint8_t y) {
    if (x < 0 || x >= SIZE || y < 0 || y >= SIZE) return;

    Node& n = maze.map[x][y];
    // Updated to use setter methods
    n.setWallUp(true);
    n.setWallDown(true);
    n.setWallLeft(true);
    n.setWallRight(true);
    n.floodval = LARGEVAL;
  }

  void remove_sides() {
  for (int i = 0; i < SIZE; i++) {
    // Top side
    maze.map[i][0].setWallUp(true);
    // Bottom side
    maze.map[i][SIZE-1].setWallDown(true);
    // Left side
    maze.map[0][i].setWallLeft(true);
    // Right side
    maze.map[SIZE-1][i].setWallRight(true);
  }
}

  Position getStartPosition() { return {startX, startY}; }
  Position getGoalPosition()  { return {goalX, goalY}; }
  Position getCurrentPosition() { return {currentX, currentY}; }

  Maze* getMaze() { return &maze; } 
private:
  mtrn3100::Driving& driveController;
  mtrn3100::Turning& turnController;
  Maze maze;
};

}// namespace mtrn3100