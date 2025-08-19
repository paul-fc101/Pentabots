#pragma once
#include <Arduino.h>
#include <VL6180X.h>
#include "Driving.hpp"
#include "Turning.hpp"

#define SIZE 9         
#define LARGEVAL 255

namespace mtrn3100 {

// --- OPTIMIZED NODE STRUCT ---
// The boolean flags have been packed into a single byte ('flags') to save SRAM.
// Helper functions are provided for easy access.
struct Node {
  uint8_t row, column;       // coordinates
  short floodval;          // floodfill value
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
  short x;
  short y;

  bool operator==(const Position& other) const {
    return (x == other.x) && (y == other.y);
  }
};

class Mapping {
public:

  short currentX;
  short currentY;
  short startX = 0;
  short startY = 0;
  short goalX  = 4;   // Center cell for 9x9 maze
  short goalY  = 4;

  bool reachedGoal() {
    return getCurrentPosition() == getGoalPosition();
  }
  void setVisited(short x, short y, bool val) {
    maze.map[x][y].setVisited(val);
  }

  bool getVisited(short x, short y) {
    return maze.map[x][y].getVisited();
  }

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

    // Initialize flood values
    for (short x = 0; x < SIZE; ++x) {
      for (short y = 0; y < SIZE; ++y) {
        maze.map[x][y].floodval = LARGEVAL;
      }
    }
    maze.map[goalX][goalY].floodval = 0;
  }

  void initialize_maze() {
    for (short x = 0; x < SIZE; ++x) {
      for (short y = 0; y < SIZE; ++y) {
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
  short get_smallest_neighbor(short x, short y) {
    short smallest = LARGEVAL;
    
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
  void update_walls(short x, short y) {
    Node& n = maze.map[x][y];

    driveController.updateLidar();

    // Updated to use setter methods
    if (driveController.getFrontDist() < 150) n.setWallUp(true);
    if (driveController.getLeftDist() < 100) n.setWallLeft(true);
    if (driveController.getRightDist() < 100) n.setWallRight(true);
  }

  // Floodfill propagation
  void propagate_floodfill() {
    bool updated;
    do {
      updated = false;
      for (short x = 0; x < SIZE; ++x) {
        for (short y = 0; y < SIZE; ++y) {
          if (x == goalX && y == goalY) continue;

          short minNeighbor = get_smallest_neighbor(x, y);
          if (maze.map[x][y].floodval != minNeighbor + 1) {
            maze.map[x][y].floodval = minNeighbor + 1;
            updated = true;
          }
        }
      }
    } while (updated);
  }

  // Decide the next move based on flood values
  String decide_next_move(short x, short y) {
    short smallest = get_smallest_neighbor(x, y);

    // Updated to use getter methods
    if (y < SIZE-1 && !maze.map[x][y].getWallUp() && 
        maze.map[x][y+1].floodval == smallest && !maze.map[x][y+1].getVisited()) return "f";
    if (x > 0 && !maze.map[x][y].getWallLeft() && 
        maze.map[x-1][y].floodval == smallest && !maze.map[x-1][y].getVisited()) return "lf";
    if (x < SIZE-1 && !maze.map[x][y].getWallRight() && 
        maze.map[x+1][y].floodval == smallest && !maze.map[x+1][y].getVisited()) return "rf";
    if (y > 0 && !maze.map[x][y].getWallDown() && 
        maze.map[x][y-1].floodval == smallest && !maze.map[x][y-1].getVisited()) return "rrf";

    return "x"; // no valid move
  }

  // Update robot position after a move
  void update_position(const String& move) {
    if (move == "f") {
      currentY += 1;
    } else if (move == "lf") {
      currentX -= 1;
    } else if (move == "rf") {
      currentX += 1;
    } else if (move == "rrf") {
      currentY -= 1;
    }

    maze.map[currentX][currentY].setVisited(true);
}

  // Mark a cell as blocked
  void remove_cell(short x, short y) {
    if (x < 0 || x >= SIZE || y < 0 || y >= SIZE) return;

    Node& n = maze.map[x][y];
    // Updated to use setter methods
    n.setWallUp(true);
    n.setWallDown(true);
    n.setWallLeft(true);
    n.setWallRight(true);
    n.floodval = LARGEVAL;
  }

  Position getStartPosition() { return {startX, startY}; }
  Position getGoalPosition()  { return {goalX, goalY}; }
  Position getCurrentPosition() { return {currentX, currentY}; }

  Maze getMaze() { return maze; }

private:
  mtrn3100::Driving& driveController;
  mtrn3100::Turning& turnController;
  Maze maze;
};

} // namespace mtrn3100