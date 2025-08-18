#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Turning.hpp"
#include "Driving.hpp"
#include "EncoderOdometry.hpp"

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
      driveController.drive(1.414);
    } else if (c == 'b') {
      // diagonal left
      turnController.centerRobot();
      delay(100);
      turnController.turn(45);
      driveController.drive(1.414);
    } else if (c == 'F') {
      encoder.l_count = 0;
      encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(newCmd.charAt(i + 1) - '0');
      movingForward = true;
      delay(150);
      continue;
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

  // --- Show a startup message ---
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

  displayMaze();
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
}

bool isDriving = false;
bool isTurning = true;

void loop() {

  // Task 3.1 - Driving
  //driveController.drivingCorrection(false);

  // Task 3.2 - Turning
  //turnController.turningCorrection(false);

}
// This function handles all the logic for defining, building, and drawing the grid.
void displayMaze() {
  const int gridSize = 9;         // Maze dimensions (9x9 in your case)
  const int gridDimension = 63;   // Pixel size allocated to maze
  const int cellSize = gridDimension / gridSize;

  Maze maze = getMaze();

  // Get special positions
  auto startPos   = getStartPosition();
  auto goalPos    = getGoalPosition();
  auto currentPos = getCurrentPosition();

  // Calculate maze completion (visited cells / total active cells)
  int visitedCount = 0, activeCount = 0;
  for (int r = 0; r < gridSize; r++) {
    for (int c = 0; c < gridSize; c++) {
      Node* node = maze.map[r][c];
      if (node != nullptr) {
        activeCount++;
        if (node->visited) visitedCount++;
      }
    }
  }
  float maze_completion = (activeCount > 0) ? (100.0f * visitedCount / activeCount) : 0;

  // --- Draw Everything ---
  u8g2.firstPage();
  do {
    // 1. Draw maze walls
    for (int r = 0; r < gridSize; r++) {
      for (int c = 0; c < gridSize; c++) {
        Node* node = maze.map[r][c];
        if (node == nullptr) continue; // Skip unused cells

        int pixelX = c * cellSize;
        int pixelY = r * cellSize;

        if (node->wallUp)    u8g2.drawLine(pixelX, pixelY, pixelX + cellSize, pixelY);
        if (node->wallDown)  u8g2.drawLine(pixelX, pixelY + cellSize, pixelX + cellSize, pixelY + cellSize);
        if (node->wallLeft)  u8g2.drawLine(pixelX, pixelY, pixelX, pixelY + cellSize);
        if (node->wallRight) u8g2.drawLine(pixelX + cellSize, pixelY, pixelX + cellSize, pixelY + cellSize);
      }
    }

    // 2. Draw start (hollow triangle)
    {
      int cx = startPos.second * cellSize + cellSize/2;
      int cy = startPos.first  * cellSize + cellSize/2;
      u8g2.drawTriangle(cx, cy - 3, cx - 3, cy + 3, cx + 3, cy + 3);
    }

    // 3. Draw goal (hollow square)
    {
      int cx = goalPos.second * cellSize + cellSize/2;
      int cy = goalPos.first  * cellSize + cellSize/2;
      u8g2.drawFrame(cx - 3, cy - 3, 6, 6);
    }

    // 4. Draw current position (dot)
    {
      int cx = currentPos.second * cellSize + cellSize/2;
      int cy = currentPos.first  * cellSize + cellSize/2;
      u8g2.drawDisc(cx, cy, 2); // Small filled circle

      // If on start or goal, draw it inside the hollow
      if ((currentPos == startPos) || (currentPos == goalPos)) {
        u8g2.drawDisc(cx, cy, 2); // already fits inside
      }
    }

    // 5. Draw completion percentage
    char buffer[10];
    dtostrf(maze_completion, 4, 1, buffer);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(70, 15, "Complete:");
    u8g2.drawStr(70, 30, buffer);
    u8g2.drawStr(70 + u8g2.getStrWidth(buffer) + 1, 30, "%");

  } while (u8g2.nextPage());
}