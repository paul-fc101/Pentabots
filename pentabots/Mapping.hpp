#pragma once
#include <Arduino.h>
#include <VL6180X.h>
#include "Driving.hpp"
#include "Turning.hpp"

#define SIZE 9         
#define LARGEVAL 255

namespace mtrn3100 {

struct Node {
  short row, column;
  short floodval;
  bool visited;
  bool wallUp, wallDown, wallLeft, wallRight;
  Node *up, *down, *left, *right;
};

struct Maze {
  Node* map[SIZE][SIZE];
};

class Mapping {
public:

  short currentX;
  short currentY;
  short startX = 0;
  short startY = 0;
  short goalX  = 4;   // Center cell for 9x9 maze
  short goalY  = 4;
  
  Mapping(mtrn3100::Driving& drive, mtrn3100::Turning& turn)
    : driveController(drive), turnController(turn) {
    maze = new_Maze();

    currentX = startX;
    currentY = startY;

    // Remove cells
    // Bottom Left
    remove_cell(0, 0);   
    remove_cell(1, 0);   
    remove_cell(0, 1);   

    // Bottom Right
    remove_cell(7, 0);   
    remove_cell(8, 0);   
    remove_cell(8, 1);  

    // Top Left
    remove_cell(0, 7);   
    remove_cell(0, 8);   
    remove_cell(1, 8);  

    // Top Right
    remove_cell(7, 8);   
    remove_cell(8, 7);   
    remove_cell(8, 8);  


    // Initialize flood values: goal = 0, all others = LARGEVAL
    for (short x = 0; x < SIZE; ++x) {
      for (short y = 0; y < SIZE; ++y) {
        maze->map[x][y]->floodval = LARGEVAL;
      }
    }
    maze->map[goalX][goalY]->floodval = 0;
  }

  Maze* new_Maze() {
    Maze* m = new Maze;
    for (short x = 0; x < SIZE; ++x) {
      for (short y = 0; y < SIZE; ++y) {
        Node* n = new Node;
        n->row = y;
        n->column = x;
        n->visited = false;
        n->wallUp = n->wallDown = n->wallLeft = n->wallRight = false;
        n->up = n->down = n->left = n->right = nullptr;

        n->floodval = abs(x - goalX) + abs(y - goalY);

        m->map[x][y] = n;
      }
    }

// Link neighbors
    for (short x = 0; x < SIZE; ++x) {
      for (short y = 0; y < SIZE; ++y) {
        if (y > 0) m->map[x][y]->down = m->map[x][y - 1];
        if (y < SIZE - 1) m->map[x][y]->up = m->map[x][y + 1];
        if (x > 0) m->map[x][y]->left = m->map[x - 1][y];
        if (x < SIZE - 1) m->map[x][y]->right = m->map[x + 1][y];
      }
    }
    return m;
  }

// Looks at all accessible neighboring cells (no wall between). Finds the smallest floodval among them.
  short get_smallest_neighbor(Node* n) {
    short smallest = LARGEVAL;
    if (n->left && !n->wallLeft && n->left->floodval < smallest)
      smallest = n->left->floodval;
    if (n->right && !n->wallRight && n->right->floodval < smallest)
      smallest = n->right->floodval;
    if (n->down && !n->wallDown && n->down->floodval < smallest)
      smallest = n->down->floodval;
    if (n->up && !n->wallUp && n->up->floodval < smallest)
      smallest = n->up->floodval;
    return smallest;
  }

// If a sensor sees something within 50mm, marks that side as a wall.
  void update_walls(Node* n) {
    driveController.updateLidar();
    if (driveController.getFrontDist() < 50) n->wallUp = true;
    if (driveController.getLeftDist() < 50) n->wallLeft = true;
    if (driveController.getRightDist() < 50) n->wallRight = true;
  }

// loop Floodfill
  void propagate_floodfill() {
    bool updated;
    do {
      updated = false;
      for (short x = 0; x < SIZE; ++x) {
        for (short y = 0; y < SIZE; ++y) {
          Node* n = maze->map[x][y];
          if (x == goalX && y == goalY) continue; // goal always 0
          short minNeighbor = get_smallest_neighbor(n);
          if (n->floodval != minNeighbor + 1) {
            n->floodval = minNeighbor + 1;
            updated = true;
          }
        }
      }
    } while (updated);
  }

  char decide_next_move(Node* n) {
    short smallest = get_smallest_neighbor(n);
    if (n->up && !n->wallUp && n->up->floodval == smallest) return 'f';
    if (n->left && !n->wallLeft && n->left->floodval == smallest) return 'lf';
    if (n->right && !n->wallRight && n->right->floodval == smallest) return 'rf';
    if (n->down && !n->wallDown && n->down->floodval == smallest) return 'rrf'; // turn around
    return 'x';
  }

  void update_position(char move) {
    if (move == 'f') {        // forward
    currentY += 1;          
    } else if (move == 'lf') { // left
    currentX -= 1;
    } else if (move == 'rf') { // right
    currentX += 1;
    } else if (move == 'rrf') { // turn around/back
    currentY -= 1;
    }
  }

// Remove the cells that are not in the maze
  void remove_cell(short x, short y) {
    if (x < 0 || x >= SIZE || y < 0 || y >= SIZE) return; // safety check
    Node* n = maze->map[x][y];

    // Treat the cell as fully blocked
    n->wallUp = true;
    n->wallDown = true;
    n->wallLeft = true;
    n->wallRight = true;

    n->floodval = LARGEVAL;
}




  Maze* getMaze() { return maze; }

private:
  mtrn3100::Driving& driveController;
  mtrn3100::Turning& turnController;
  Maze* maze;
};

} 
