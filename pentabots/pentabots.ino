#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Turning.hpp"
#include "Driving.hpp"
#include "EncoderOdometry.hpp"
#include "Mapping.hpp"

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>
#include "string.h"
#include <U8g2lib.h>

#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

// These are the pins for the PCB encoder
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8

// Constants
#define FINAL_DIST 200
#define WHEEL_DIAM 32
#define WHEEL_BASE 91

#define RANGE 1

struct Cell {
  bool northWall = false;
  bool eastWall = false;
  bool southWall = false;
  bool westWall = false;
};

Cell grid[9][9];

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// Initalise
mtrn3100::Motor motor(MOT1PWM, MOT1DIR, 1);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR, 2);
MPU6050 mpu(Wire);

VL6180X FrontSensor, LeftSensor, RightSensor;

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometer(WHEEL_DIAM / 2, WHEEL_BASE);
mtrn3100::Driving driveController(motor, motor2, FrontSensor, LeftSensor, RightSensor, encoder, encoderOdometer);
mtrn3100::Turning turnController(motor, motor2, mpu, FrontSensor, LeftSensor, RightSensor);
//mtrn3100::Mapping mapping(driveController, turnController);
mtrn3100::Mapping mapper(driveController, turnController);

void MovementString(String command) {
  String newCmd = ""; 
  bool prevForward = false;
  int fwdCounter = 0;

  for (int i = 0; i < command.length(); i++) {
      char c = command[i];
      
      if (c == 'f' && !prevForward) {
          newCmd += 'F';
          prevForward = true;
          fwdCounter++;
      } else if (c == 'f' && prevForward) {
          fwdCounter++;
      } else {
          if (prevForward) {
              newCmd += String(fwdCounter);
          }
          newCmd += c;
          prevForward = false;
          fwdCounter = 0;
      }
  }
  if (prevForward) {
      newCmd += String(fwdCounter);
  }  

  bool movingForward = false;
  for (int i = 0; i < newCmd.length(); ++i) {
    if (movingForward) {
      movingForward = false;
      continue;
    }
    char c = newCmd.charAt(i);
    Serial.println(c);
    if (c == 'f') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(1);
    } else if (c == 'l') {
      // anticlockwise
      turnController.centerRobot();
      delay(100);
      turnController.turn(90);
    } else if (c == 'r') {
      // clockwise
      turnController.centerRobot();
      delay(100);
      turnController.turn(-90);
    } else if (c == 'a') {
      // diagonal right
      turnController.centerRobot();
      delay(100);
      turnController.turn(-45);
    } else if (c == 'b') {
      // diagonal left
      turnController.centerRobot();
      delay(100);
      turnController.turn(45);
    } else if (c == 'F') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(newCmd.charAt(i + 1) - '0');
      movingForward = true;
      delay(150);
      continue;
    } else if (c == 's') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(0.5);
    } else if (c == 'D') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(1.414);
    } else if (c == 'd') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(1.414 / 2);
    }
    movingForward = false;
    delay(150);
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(300);
    // No need to call Wire.begin() separately, U8g2 handles it.
  u8g2.begin(); // Initialize the U8g2 library
  // u8g2.clearBuffer();
  // u8g2.setFont(u8g2_font_ncenB08_tr);
  // u8g2.drawStr(0, 15, "Initializing...");
  // u8g2.sendBuffer();

  // // --- Show a startup message ---
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a font
    u8g2.drawStr(0, 10, "Micromouse Ready!"); // Draw a string
  } while (u8g2.nextPage());
  delay(1000);

  // --- Draw your 9x9 grid ---
  int gridSize = 9;
  int gridDimension = 63; // 63 is divisible by 9
  int cellSize = gridDimension / gridSize; // 7 pixels

  // displayMaze();
  // delay(1000);

  syncAndDrawMaze();
  delay(1000);

  turnController.attachMPU();

  turnController.setMPUInitRot();

  driveController.initalise_lidar();

  turnController.getLidar(FrontSensor, LeftSensor, RightSensor);

  //Uncomment for Task 3.2
  //turnController.turn(90);



  // Task 3.3
  //MovementString("ffffflfffbfbfafrfbfflff");
  //MovementString("fffff");
  //MovementString("ffff");
  //MovementString("ffffrfrfff");
  MovementString("s");
}

bool isDriving = false;
bool isTurning = true;

void loop() {
  // Task 3.1 - Driving
  //driveController.drivingCorrection(false);

  // Task 3.2 - Turning
  //turnController.turningCorrection(false);

}




void syncAndDrawMaze() {
  int gridSize = 9;
  int gridDimension = 63;
  int cellSize = gridDimension / gridSize;

  // --- 1. SYNC DATA from Mapping to local 'grid' ---
  // Get the maze data from your Mapping object
  mtrn3100::Maze* mazeData = mapper.getMaze();

  // Loop through every cell and copy the wall data
  for (int x = 0; x < gridSize; x++) {
    for (int y = 0; y < gridSize; y++) {
      mtrn3100::Node* node = mazeData->map[x][y];
      
      // Translate wall data from the Node to the Cell
      // Note the mapping: wallUp -> northWall, etc.
      grid[x][y].northWall = node->wallUp;
      grid[x][y].southWall = node->wallDown;
      grid[x][y].westWall  = node->wallLeft;
      grid[x][y].eastWall  = node->wallRight;
    }
  }

  // --- 2. DRAW EVERYTHING (this part is mostly the same) ---
  u8g2.firstPage();
  do {
    // Loop through our local 'grid' and draw the walls
    for (int x = 0; x < gridSize; x++) {
      for (int y = 0; y < gridSize; y++) {
        int pixelX = x * cellSize;
        int pixelY = y * cellSize;
        if (grid[x][y].northWall) { u8g2.drawLine(pixelX, pixelY, pixelX + cellSize, pixelY); }
        if (grid[x][y].eastWall)  { u8g2.drawLine(pixelX + cellSize, pixelY, pixelX + cellSize, pixelY + cellSize); }
        if (grid[x][y].southWall) { u8g2.drawLine(pixelX, pixelY + cellSize, pixelX + cellSize, pixelY + cellSize); }
        if (grid[x][y].westWall)  { u8g2.drawLine(pixelX, pixelY, pixelX, pixelY + cellSize); }
      }
    }

    // Draw the completion percentage text
    float maze_completion = 0.0; // You can update this variable as needed
    char completion_buffer[10];
    dtostrf(maze_completion, 4, 1, completion_buffer);

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(70, 15, "Complete:");
    u8g2.drawStr(70, 30, completion_buffer);
    u8g2.drawStr(70 + u8g2.getStrWidth(completion_buffer) + 1, 30, "%");

  } while (u8g2.nextPage());
}





// // This function handles all the logic for defining, building, and drawing the grid.
// void displayMaze() { 
//   int gridSize = 9; 
//   int gridDimension = 63; 
//   int cellSize = gridDimension / gridSize; 
//   // --- 1. DEFINE THE SHAPE --- 
//   bool isActiveCell[9][9]; 
//   for (int x = 0; x < gridSize; x++) { 
//     for (int y = 0; y < gridSize; y++) { 
//       isActiveCell[x][y] = true; 
//     } 
//   } 
//   // Deactivate the corner cells 
//   isActiveCell[0][0] = false; 
//   isActiveCell[0][1] = false; 
//   isActiveCell[1][0] = false; 
//   isActiveCell[8][0] = false; 
//   isActiveCell[8][1] = false; 
//   isActiveCell[7][0] = false; 
//   isActiveCell[0][8] = false; 
//   isActiveCell[1][8] = false; 
//   isActiveCell[0][7] = false; 
//   isActiveCell[8][8] = false; 
//   isActiveCell[8][7] = false; 
//   isActiveCell[7][8] = false; 
//   // --- 2. BUILD THE WALLS --- 
//   // Reset all walls to false before building 
//   for (int x = 0; x < gridSize; x++) { 
//     for (int y = 0; y < gridSize; y++) { 
//       grid[x][y].northWall = false; 
//       grid[x][y].eastWall = false; 
//       grid[x][y].southWall = false; 
//       grid[x][y].westWall = false; 
//     } 
//   } 
//   // Automatically generate the grid lines and the outer border 
//   for (int x = 0; x < gridSize; x++) { 
//     for (int y = 0; y < gridSize; y++) { 
//       if (isActiveCell[x][y]) { 
//         if (y == 0 || !isActiveCell[x][y - 1]) { 
//           grid[x][y].northWall = true; 
//         } 
//         if (y == gridSize - 1 || !isActiveCell[x][y + 1]) { 
//           grid[x][y].southWall = true; 
//         } 
//         if (x == 0 || !isActiveCell[x - 1][y]) { 
//           grid[x][y].westWall = true; 
//         }
//         if (x == gridSize - 1 || !isActiveCell[x + 1][y]) { 
//           grid[x][y].eastWall = true; 
//         }
//       } 
//     } 
//   } 
//   // --- 4. DRAW EVERYTHING --- 
//   u8g2.firstPage(); 
//   do { 
//     // Loop through our grid data structure and draw the walls 
//     for (int x = 0; x < gridSize; x++) {
//       for (int y = 0; y < gridSize; y++) {
//         int pixelX = x * cellSize; 
//         int pixelY = y * cellSize; 
//         if (grid[x][y].northWall) { 
//           u8g2.drawLine(pixelX, pixelY, pixelX + cellSize, pixelY); 
//         } 
//         if (grid[x][y].eastWall) { 
//           u8g2.drawLine(pixelX + cellSize, pixelY, pixelX + cellSize, pixelY + cellSize); 
//         } if (grid[x][y].southWall) { 
//           u8g2.drawLine(pixelX, pixelY + cellSize, pixelX + cellSize, pixelY + cellSize); 
//         } 
//         if (grid[x][y].westWall) { 
//           u8g2.drawLine(pixelX, pixelY, pixelX, pixelY + cellSize); 
//         }
//       } 
//     } 
//     // Draw the completion percentage text 
//     float maze_completion = 65.7; 
//     char completion_buffer[10]; 
//     dtostrf(maze_completion, 4, 1, completion_buffer); 
//     u8g2.setFont(u8g2_font_ncenB08_tr); 
//     u8g2.drawStr(70, 15, "Complete:"); 
//     u8g2.drawStr(70, 30, completion_buffer); 
//     u8g2.drawStr(70 + u8g2.getStrWidth(completion_buffer) + 1, 30, "%"); 
//     } 
//   while (u8g2.nextPage()); 
// }