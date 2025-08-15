#pragma once
#include <Arduino.h>
#include <VL6180X.h>
#include "Driving.hpp"
#include "Turning.hpp"

#define SIZE 9          // 9x9 maze
#define LARGEVAL 255
#define VISITED_FLAG 1

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
  Mapping(Driving& drive, mtrn3100::Turning& turn): driveController(drive), turnController(turn) {
    maze = new_Maze();
  }

  // Create maze
  Maze* new_Maze() {
    Maze* m = new Maze;
    short halfsize = SIZE / 2;

    for (short x = 0; x < SIZE; ++x) {
      for (short y = 0; y < SIZE; ++y) {
        Node* n = new Node;
        n->row = y;
        n->column = x;
        n->visited = false;
        n->wallUp = n->wallDown = n->wallLeft = n->wallRight = false;
        n->up = n->down = n->left = n->right = nullptr;

        // Initial flood fill values (Manhattan distance to center)
        if (x < halfsize && y < halfsize)
          n->floodval = (halfsize - 1 - x) + (halfsize - 1 - y);
        else if (x < halfsize && y >= halfsize)
          n->floodval = (halfsize - 1 - x) + (y - halfsize);
        else if (x >= halfsize && y < halfsize)
          n->floodval = (x - halfsize) + (halfsize - 1 - y);
        else
          n->floodval = (x - halfsize) + (y - halfsize);

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

  // Get smallest neighbor value (ignores walls)
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

  void set_value(Node* n, short value) {
    n->floodval = value;
  }

  // Update walls based on LIDAR readings
  void update_walls(Node* n) {
    driveController.updateLidar();
    if (driveController.getFrontDist() < 50) n->wallUp = true;
    if (driveController.getLeftDist() < 50) n->wallLeft = true;
    if (driveController.getRightDist() < 50) n->wallRight = true;
    // wallDown can be set when robot moves into new cell and marks behind
  }

  // Simple flood fill propagation
  void propagate_floodfill() {
    bool updated;
    do {
      updated = false;
      for (short x = 0; x < SIZE; ++x) {
        for (short y = 0; y < SIZE; ++y) {
          Node* n = maze->map[x][y];
          short minNeighbor = get_smallest_neighbor(n);
          if (n->floodval != minNeighbor + 1) {
            n->floodval = minNeighbor + 1;
            updated = true;
          }
        }
      }
    } while (updated);
  }

  // Decide next move based on flood values
  char decide_next_move(Node* n) {
    short smallest = get_smallest_neighbor(n);
    if (n->up && !n->wallUp && n->up->floodval == smallest) return 'f';
    if (n->left && !n->wallLeft && n->left->floodval == smallest) return 'l';
    if (n->right && !n->wallRight && n->right->floodval == smallest) return 'r';
    if (n->down && !n->wallDown && n->down->floodval == smallest) return 'b';
    return 'x'; // no move
  }

  Maze* getMaze() { return maze; }

private:
  mtrn3100::Driving& driveController;
  mtrn3100::Turning& turnController;
  Maze* maze;
};
}