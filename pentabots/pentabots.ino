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
#include "SimpleOLED.h"

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

// Initalise
mtrn3100::Motor motor(MOT1PWM, MOT1DIR, 1);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR, 2);
MPU6050 mpu(Wire);
SimpleOLED oled(0x3C, 128, 64);
VL6180X FrontSensor, LeftSensor, RightSensor;

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometer(WHEEL_DIAM / 2, WHEEL_BASE);
mtrn3100::Driving driveController(motor, motor2, FrontSensor, LeftSensor, RightSensor, encoder, encoderOdometer);
mtrn3100::Turning turnController(motor, motor2, mpu, FrontSensor, LeftSensor, RightSensor);
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
    // Serial.println(c);
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
  Serial.println("hi");
  Wire.begin();
  delay(300);
  if (!oled.begin()) {
    Serial.println("Failed to start OLED");
    while (1); // Don't continue if it failed
  }

  // displayMaze();
  // delay(1000);
  // oled.clear(); // Start with a clear buffer
  // oled.setCursor(0, 0);
  // oled.print("Hello World!");
  // oled.drawCircle(100, 40, 15, 1);
  // oled.display(); 
  searchMaze();
  String path = generatePath();
  MovementString(ReversePath(path));
  MovementString(path);


  //drawMazeFromMapping();


  //syncAndDrawMaze();
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


// void syncAndDrawMaze() {
//   const int gridSize = 9;
//   const int gridDimension = 63;
//   const int cellSize = gridDimension / gridSize;

//   // --- 1. SYNC DATA from Mapping to local 'grid' ---
//   mtrn3100::Maze mazeData = mapper.getMaze();
//   for (int x = 0; x < gridSize; x++) {
//     for (int y = 0; y < gridSize; y++) {
//       mtrn3100::Node node = mazeData.map[x][y];
//       grid[x][y].northWall = node.getWallUp();
//       grid[x][y].southWall = node.getWallDown();
//       grid[x][y].westWall  = node.getWallLeft();
//       grid[x][y].eastWall  = node.getWallRight();
//     }
//   }

//   // --- 2. DRAW EVERYTHING ---
//   oled.clear();
//   for (int x = 0; x < gridSize; x++) {
//     for (int y = 0; y < gridSize; y++) {
//       int pixelX = x * cellSize;
//       int pixelY = y * cellSize;

//       if (grid[x][y].northWall) oled.drawLine(pixelX, pixelY, pixelX + cellSize, pixelY);
//       if (grid[x][y].eastWall)  oled.drawLine(pixelX + cellSize, pixelY, pixelX + cellSize, pixelY + cellSize);
//       if (grid[x][y].southWall) oled.drawLine(pixelX, pixelY + cellSize, pixelX + cellSize, pixelY + cellSize);
//       if (grid[x][y].westWall)  oled.drawLine(pixelX, pixelY, pixelX, pixelY + cellSize);
//     }
//   }

//   // Draw completion text
//   float maze_completion = 0.0; // Update dynamically if needed
//   oled.setCursor(70, 0);
//   oled.print("Complete:");
//   oled.setCursor(70, 10);
//   oled.print(maze_completion, 1);
//   oled.print("%");

//   oled.display(); // Show everything
// }
void searchMaze() {
  Serial.println("hello");
  Serial.println(mapper.reachedGoal());
  while (!mapper.reachedGoal()) {
    //Serial.println("yes");
    auto pos = mapper.getCurrentPosition();
    mapper.setVisited(pos.x, pos.y, true);
          //Serial.println("yes1");
    // 1. Update walls using lidar
    mapper.update_walls(pos.x, pos.y);
          //Serial.println("yes2");

    // 2. Decide next move 
    String move = mapper.decide_next_move(pos.x, pos.y);
    Serial.println("no");

    // 3. Update position
    mapper.update_position(move);

    // 4. Execute movement
    MovementString(move);

    // 5. Propagate flood fill
    mapper.propagate_floodfill();

  }
}

String generatePath() {
  String path = "";
  auto start = mapper.getStartPosition();
  auto goal  = mapper.getGoalPosition();

  // Temporary variables for simulation
  short simX = start.x;
  short simY = start.y;

  while (!(simX == goal.x && simY == goal.y)) {
    String move = mapper.decide_next_move(simX, simY);
    path += move;

    // simulate updating position
    if (move == "f") simY++;
    else if (move == "lf") simX--;
    else if (move == "rf") simX++;
    else if (move == "rrf") simY--;
  }

  return path;
}

String ReversePath(String path) {
  String reversed = "rr";

  // Work backwards through the string
  for (int i = path.length() - 1; i >= 0; --i) {
    char c = path.charAt(i);

    if (c == 'f') {
      // Forward is still forward when retracing
      reversed.concat('f');
    } 
    else if (c == 'l') {
      // Left turn becomes right turn
      reversed.concat('r');
    } 
    else if (c == 'r') {
      // Right turn becomes left turn
      reversed.concat('l');
    } 
  }
  reversed.concat("rr");
  return reversed;
}




void drawMazeFromMapping() {
    const int gridSize = 9;
    const int gridDimension = 63;
    const int cellSize = gridDimension / gridSize;

    // Get the maze from Mapping
    mtrn3100::Maze mazeData = mapper.getMaze();

    oled.clear();

    // Loop through the maze and draw walls directly
    for (int x = 0; x < gridSize; x++) {
        for (int y = 0; y < gridSize; y++) {
            mtrn3100::Node& node = mazeData.map[x][y]; // reference to avoid copying
            int pixelX = x * cellSize;
            int pixelY = y * cellSize;

            if (node.getWallUp())    oled.drawLine(pixelX, pixelY, pixelX + cellSize, pixelY);
            if (node.getWallRight()) oled.drawLine(pixelX + cellSize, pixelY, pixelX + cellSize, pixelY + cellSize);
            if (node.getWallDown())  oled.drawLine(pixelX, pixelY + cellSize, pixelX + cellSize, pixelY + cellSize);
            if (node.getWallLeft())  oled.drawLine(pixelX, pixelY, pixelX, pixelY + cellSize);
        }
    }

    // Draw completion text
    float maze_completion = 0.0; // update dynamically
    oled.setCursor(70, 0);
    oled.print("Complete:");
    oled.setCursor(70, 10);
    oled.print(maze_completion, 1);
    oled.print("%");

    oled.display();
}