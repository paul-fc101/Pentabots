#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Turning.hpp"
#include "Driving.hpp"
#include "EncoderOdometry.hpp"
#include "Mapping.hpp"
#include "String.h"

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>
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
// #define FINAL_DIST 200
#define WHEEL_DIAM 32
#define WHEEL_BASE 91

#define MAX_PATH_LENGTH 100 
char current_orientation = 'U'; // Start facing Up ('U', 'R', 'D', 'L')

// #define RANGE 1

// Initalise
mtrn3100::Motor motor(MOT1PWM, MOT1DIR, 1);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR, 2);
MPU6050 mpu(Wire);
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);
VL6180X FrontSensor, LeftSensor, RightSensor;

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometer(WHEEL_DIAM / 2, WHEEL_BASE);
mtrn3100::Driving driveController(motor, motor2, FrontSensor, LeftSensor, RightSensor, encoder, encoderOdometer);
mtrn3100::Turning turnController(motor, motor2, mpu, FrontSensor, LeftSensor, RightSensor);
mtrn3100::Mapping mapper(driveController, turnController);


void MovementString(const char *command) {
  // New command buffer - size depends on expected complexity
  // 64 chars should be plenty for university tasks; enlarge if necessary
  char newCmd[64];
  size_t nc = 0;

  bool prevForward = false;
  int fwdCounter = 0;

  for (const char *p = command; *p && nc + 4 < sizeof(newCmd); ++p) {
    char c = *p;
    if (c == 'f') {
      if (!prevForward) { // start a run
        if (nc + 1 < sizeof(newCmd)) newCmd[nc++] = 'F'; // marker for run
        prevForward = true;
        fwdCounter = 1;
      } else {
        ++fwdCounter;
      }
    } else {
      if (prevForward) {
        // append run length as single char digit(s)
        // if count >9, we clamp or add multi-digit (here we support up to 99)
        int tens = fwdCounter / 10;
        int ones = fwdCounter % 10;
        if (tens) {
          newCmd[nc++] = '0' + tens;
        }
        newCmd[nc++] = '0' + ones;
      }
      // add this non-'f' command
      newCmd[nc++] = c;
      prevForward = false;
      fwdCounter = 0;
    }
  }
  if (prevForward && nc + 3 < sizeof(newCmd)) {
    int tens = fwdCounter / 10;
    int ones = fwdCounter % 10;
    if (tens) newCmd[nc++] = '0' + tens;
    newCmd[nc++] = '0' + ones;
  }
  newCmd[nc] = '\0';

  // parse newCmd
  for (size_t i = 0; i < nc; ++i) {
    char c = newCmd[i];
    if (c == 'F') {
      // next char should be a digit
      int val = 1;
      if (i + 1 < nc && newCmd[i+1] >= '0' && newCmd[i+1] <= '9') {
        val = newCmd[++i] - '0';
      }
      encoder.l_count = encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive((float)val);
      delay(150);
    } else if (c == 'f') {
      encoder.l_count = encoder.r_count = 0;
      encoderOdometer.reset();
      driveController.drive(1.0);
      delay(150);
    } else if (c == 'l') { turnController.centerRobot(); delay(100); turnController.turn(90); }
    else if (c == 'r') { turnController.centerRobot(); delay(100); turnController.turn(-90); }
    else if (c == 'a') { turnController.centerRobot(); delay(100); turnController.turn(-45); }
    else if (c == 'b') { turnController.centerRobot(); delay(100); turnController.turn(45); }
    else if (c == 's') { encoder.l_count = encoder.r_count = 0; encoderOdometer.reset(); driveController.drive(0.5); delay(150); }
    else if (c == 'D') { encoder.l_count = encoder.r_count = 0; encoderOdometer.reset(); driveController.drive(1.414); delay(150); }
    else if (c == 'd') { encoder.l_count = encoder.r_count = 0; encoderOdometer.reset(); driveController.drive(1.414/2); delay(150); }
  }
}

// void MovementString(String command) {
//   String newCmd = ""; 
//   bool prevForward = false;
//   int fwdCounter = 0;

//   for (int i = 0; i < command.length(); i++) {
//       char c = command[i];
      
//       if (c == 'f' && !prevForward) {
//           newCmd += 'F';
//           prevForward = true;
//           fwdCounter++;
//       } else if (c == 'f' && prevForward) {
//           fwdCounter++;
//       } else {
//           if (prevForward) {
//               newCmd += String(fwdCounter);
//           }
//           newCmd += c;
//           prevForward = false;
//           fwdCounter = 0;
//       }
//   }
//   if (prevForward) {
//       newCmd += String(fwdCounter);
//   }  

//   bool movingForward = false;
//   for (int i = 0; i < newCmd.length(); ++i) {
//     if (movingForward) {
//       movingForward = false;
//       continue;
//     }
//     char c = newCmd.charAt(i);
//     // Serial.println(c);
//     if (c == 'f') {
//       encoder.l_count = 0;
//       encoder.r_count = 0;
//       encoderOdometer.reset();
//       driveController.drive(1);
//     } else if (c == 'l') {
//       // anticlockwise
//       turnController.centerRobot();
//       delay(100);
//       turnController.turn(90);
//     } else if (c == 'r') {
//       // clockwise
//       turnController.centerRobot();
//       delay(100);
//       turnController.turn(-90);
//     } else if (c == 'a') {
//       // diagonal right
//       turnController.centerRobot();
//       delay(100);
//       turnController.turn(-45);
//     } else if (c == 'b') {
//       // diagonal left
//       turnController.centerRobot();
//       delay(100);
//       turnController.turn(45);
//     } else if (c == 'F') {
//       encoder.l_count = 0;
//       encoder.r_count = 0;
//       encoderOdometer.reset();
//       driveController.drive(newCmd.charAt(i + 1) - '0');
//       movingForward = true;
//       delay(150);
//       continue;
//     } else if (c == 's') {
//       encoder.l_count = 0;
//       encoder.r_count = 0;
//       encoderOdometer.reset();
//       driveController.drive(0.5);
//     } else if (c == 'D') {
//       encoder.l_count = 0;
//       encoder.r_count = 0;
//       encoderOdometer.reset();
//       driveController.drive(1.414);
//     } else if (c == 'd') {
//       encoder.l_count = 0;
//       encoder.r_count = 0;
//       encoderOdometer.reset();
//       driveController.drive(1.414 / 2);
//     }
//     movingForward = false;
//     delay(150);
//   }
// }


void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(300);
  Serial.println("1");
  //u8g2.begin();
  Serial.println("2");
  delay(300);
  turnController.attachMPU();
  Serial.println("3");

  turnController.setMPUInitRot();
  Serial.println("4");

  driveController.initalise_lidar();

  Serial.println("5");
  turnController.getLidar(FrontSensor, LeftSensor, RightSensor);

  // Task 3.3
  //MovementString("ffffflfffbfbfafrfbfflff");
  //MovementString("fffff");
  //MovementString("ffff");
  //MovementString("ffffrfrfff");


  // Task 4.3
  Serial.println("6");
  searchMaze();
  // String path = generatePath();
  // MovementString(ReversePath(path));
  // MovementString(path);
  // char path[10];
  // char reversed[10];
  // generatePath(path, 10);
  
  // ReversePath(path, reversed, 10);
  // MovementString(reversed);
  // MovementString(path);

  char path[MAX_PATH_LENGTH];
  char reversed_path[MAX_PATH_LENGTH];

  generatePath(path, MAX_PATH_LENGTH);
  ReversePath(path, reversed_path, MAX_PATH_LENGTH);

  MovementString(reversed_path); // Go to start
  MovementString(path);          // Go to goal
}

bool isDriving = false;
bool isTurning = true;

void loop() {
  // Task 3.1 - Driving
  //driveController.drivingCorrection(false);

  // Task 3.2 - Turning
  //turnController.turningCorrection(false);

}


// void drawMazeFromMapping() {
//   const int gridSize = 9;
//   const int gridDimension = 63;   // pixels used for maze
//   const int cellSize = gridDimension / gridSize; // expected 7

//   mtrn3100::Maze* mazePtr = mapper.getMaze();
//   if (!mazePtr) return;

//   mtrn3100::Position startPos = mapper.getStartPosition();
//   mtrn3100::Position currPos  = mapper.getCurrentPosition();
//   mtrn3100::Position goalPos  = mapper.getGoalPosition();

//   // pixel centers of cells
//   auto cellCenterX = [&](int col){ return col * cellSize + (cellSize / 2); };
//   auto cellCenterY = [&](int row){ return row * cellSize + (cellSize / 2); };

//   int start_cx = cellCenterX(startPos.x);
//   int start_cy = cellCenterY(startPos.y);

//   int curr_cx  = cellCenterX(currPos.x);
//   int curr_cy  = cellCenterY(currPos.y);

//   int goal_cx  = cellCenterX(goalPos.x);
//   int goal_cy  = cellCenterY(goalPos.y);

//   // Sizes: keep dot small so it fits inside circles
//   const int dotHalf = 1; // 3x3 filled box
//   // circle radii: ensure >= dotHalf+1 and >=1
//   int maxRadius = (cellSize / 2) - 1;
//   if (maxRadius < 2) maxRadius = 2;

//   u8g2.firstPage();
//   do {
//     // Draw maze walls
//     for (int row = 0; row < gridSize; ++row) {
//       for (int col = 0; col < gridSize; ++col) {
//         mtrn3100::Node &node = mazePtr->map[col][row];

//         int x0 = col * cellSize;
//         int y0 = row * cellSize;
//         int x1 = x0 + cellSize;
//         int y1 = y0 + cellSize;

//         if (node.getWallUp()) u8g2.drawLine(x0, y0, x1, y0); // top
//         if (node.getWallRight()) u8g2.drawLine(x1, y0, x1, y1); // right
//         if (node.getWallDown())  u8g2.drawLine(x0, y1, x1, y1); // bottom
//         if (node.getWallLeft())  u8g2.drawLine(x0, y0, x0, y1); // left
//       }
//     }

//     // Start: hollow circle
//     u8g2.drawCircle(start_cx, start_cy, maxRadius, U8G2_DRAW_ALL);

//     // Current: small filled dot (so it is visible and fits inside circles)
//     u8g2.drawBox(curr_cx - dotHalf, curr_cy - dotHalf, dotHalf * 2 + 1, dotHalf * 2 + 1);

//     // Goal: hollow circle
//     u8g2.drawCircle(goal_cx, goal_cy, maxRadius, U8G2_DRAW_ALL);

//     // Completion text
//     float maze_completion = 1/69;
//     char completion_buffer[8];
//     dtostrf(maze_completion, 4, 1, completion_buffer);

//     u8g2.setFont(u8g2_font_ncenB08_tr);
//     u8g2.drawStr(gridDimension + 4, 10, "Complete:");
//     u8g2.drawStr(gridDimension + 4, 24, completion_buffer);
//     u8g2.drawStr(gridDimension + 4 + u8g2.getStrWidth(completion_buffer) + 2, 24, "%");

//   } while (u8g2.nextPage());
// }


void searchMaze() {
  while (!mapper.reachedGoal()) {
    //drawMazeFromMapping();
    //Serial.println("yes");
    auto pos = mapper.getCurrentPosition();
    mapper.setVisited(pos.x, pos.y, true);
          //Serial.println("yes1");
    // 1. Update walls using lidar
    mapper.update_walls(pos.x, pos.y);
          //Serial.println("yes2");

    // 2. Decide next move 
    char move = mapper.decide_next_move(pos.x, pos.y);

    // 3. Update position
    mapper.update_position(move);
    Serial.println(move);
    if (move == 'U') {
      MovementString("f");
    } else if (move == 'L') {
      MovementString("lf");
    } else if (move == 'R') {
      MovementString("rf");
    } else if (move == 'D') {
      MovementString("rrf");
    }



    // 5. Propagate flood fill
    mapper.propagate_floodfill();

  }
}

// String generatePath() {
//   String path = "";
//   auto start = mapper.getStartPosition();
//   auto goal  = mapper.getGoalPosition();

//   // Temporary variables for simulation
//   short simX = start.x;
//   short simY = start.y;

//   while (!(simX == goal.x && simY == goal.y)) {
//     String move = mapper.decide_next_move(simX, simY);
//     path += move;

//     // simulate updating position
//     if (move == "f") simY++;
//     else if (move == "lf") simX--;
//     else if (move == "rf") simX++;
//     else if (move == "rrf") simY--;
//   }

//   return path;
// }

// String ReversePath(String path) {
//   String reversed = "rr";

//   // Work backwards through the string
//   for (int i = path.length() - 1; i >= 0; --i) {
//     char c = path.charAt(i);

//     if (c == 'f') {
//       // Forward is still forward when retracing
//       reversed.concat('f');
//     } 
//     else if (c == 'l') {
//       // Left turn becomes right turn
//       reversed.concat('r');
//     } 
//     else if (c == 'r') {
//       // Right turn becomes left turn
//       reversed.concat('l');
//     } 
//   }
//   reversed.concat("rr");
//   return reversed;
// }

void generatePath(char* path_buffer, int buffer_size) {
  path_buffer[0] = '\0'; // Start with an empty string
  current_orientation = 'U'; // Assume path generation starts facing Up

  auto start = mapper.getStartPosition();
  auto goal  = mapper.getGoalPosition();
  short simX = start.x;
  short simY = start.y;

  while (!(simX == goal.x && simY == goal.y)) {
    char next_direction = mapper.decide_next_move(simX, simY);
    if (next_direction == 'X') break;

    char command[4] = "";
    // Logic to convert direction to command relative to current orientation
    if (current_orientation == 'U') {
        if (next_direction == 'U') strcat(command, "f");
        else if (next_direction == 'L') strcat(command, "lf");
        else if (next_direction == 'R') strcat(command, "rf");
        else if (next_direction == 'D') strcat(command, "rrf");
    } // ... add else-if blocks for 'R', 'D', 'L' orientations as in searchMaze ...

    // Check for buffer overflow before appending
    if (strlen(path_buffer) + strlen(command) < buffer_size) {
        strcat(path_buffer, command);
    } else {
        // Buffer is full, stop.
        break;
    }

    current_orientation = next_direction;

    // Simulate updating position
    if (next_direction == 'U') simY++;
    else if (next_direction == 'L') simX--;
    else if (next_direction == 'R') simX++;
    else if (next_direction == 'D') simY--;
  }
}


void ReversePath(const char* original_path, char* reversed_path, int buffer_size) {
    // Start with a 180-degree turn
    strcpy(reversed_path, "rr");

    int len = strlen(original_path);
    for (int i = len - 1; i >= 0; --i) {
        char c = original_path[i];
        char to_add[2] = {0}; // char array to hold one character

        if (c == 'f') {
            to_add[0] = 'f';
        } else if (c == 'l') {
            to_add[0] = 'r';
        } else if (c == 'r') {
            to_add[0] = 'l';
        } else {
            continue; // Skip non-movement characters
        }

        if (strlen(reversed_path) + 1 < buffer_size) {
            strcat(reversed_path, to_add);
        } else {
            break; // Buffer full
        }
    }
    
    // End with a final 180-degree turn to face the goal
    if (strlen(reversed_path) + 2 < buffer_size) {
        strcat(reversed_path, "rr");
    }
}



// void generatePath(char* pathBuffer, int bufferSize) {
//   int index = 0;
//   auto start = mapper.getStartPosition();
//   auto goal  = mapper.getGoalPosition();
//   short simX = start.x;
//   short simY = start.y;

//   while (!(simX == goal.x && simY == goal.y) && index < bufferSize - 4) { 
//       // Get the next move as a C-style string (char pointer)
//       const char* move = mapper.decide_next_move(simX, simY);

//       if (strcmp(move, "f") == 0) {
//           pathBuffer[index++] = 'f';
//           simY++;
//       } else if (strcmp(move, "lf") == 0) {
//           pathBuffer[index++] = 'l';
//           pathBuffer[index++] = 'f';
//           simX--;
//       } else if (strcmp(move, "rf") == 0) {
//           pathBuffer[index++] = 'r';
//           pathBuffer[index++] = 'f';
//           simX++;
//       } else if (strcmp(move, "rrf") == 0) {
//           pathBuffer[index++] = 'r';
//           pathBuffer[index++] = 'r';
//           pathBuffer[index++] = 'f';
//           simY--;
//       } else {
//           // No valid move, break loop
//           break;
//       }
//   }
//   pathBuffer[index] = '\0'; // Null terminate string
// }


// void ReversePath(const char* path, char* reversed, int bufferSize) {
//   int index = 0;

//   // Add "rr" at start
//   if (index + 2 < bufferSize) {
//     reversed[index++] = 'r';
//     reversed[index++] = 'r';
//   }

//   // Work backwards through path
//   int len = strlen(path);
//   for (int i = len - 1; i >= 0 && index < bufferSize - 1; --i) {
//     char c = path[i];

//     if (c == 'f') {
//       reversed[index++] = 'f';  // forward stays forward
//     } else if (c == 'l') {
//       reversed[index++] = 'r';  // left becomes right
//     } else if (c == 'r') {
//       reversed[index++] = 'l';  // right becomes left
//     }
//   }

//   // Add "rr" at end
//   if (index + 2 < bufferSize) {
//     reversed[index++] = 'r';
//     reversed[index++] = 'r';
//   }

//   reversed[index] = '\0'; // Null terminate
// }
