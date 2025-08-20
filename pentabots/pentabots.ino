#include "Motor.hpp"
#include "DualEncoder.hpp"
#include "Turning.hpp"
#include "Driving.hpp"
#include "EncoderOdometry.hpp"
#include "Mapping.hpp"

#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <U8g2lib.h>

#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

// Constants
// #define FINAL_DIST 200
#define WHEEL_RAD 16
#define WHEEL_BASE 91

#define MAX_PATH_LENGTH 15
char current_orientation = 'U'; // Start facing Up ('U', 'R', 'D', 'L')

// #define RANGE 1

// Initalise
mtrn3100::Motor motor(MOT1PWM, MOT1DIR, 1);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR, 2);
MPU6050 mpu(Wire);
VL6180X FrontSensor, LeftSensor, RightSensor;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// These are the pins for the PCB encoder
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8



mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometer(WHEEL_RAD, WHEEL_BASE);
mtrn3100::Driving driveController(motor, motor2, FrontSensor, LeftSensor, RightSensor, encoder, encoderOdometer, WHEEL_RAD, WHEEL_BASE);
mtrn3100::Turning turnController(motor, motor2, mpu, FrontSensor, LeftSensor, RightSensor);
mtrn3100::Mapping mapper(driveController, turnController);

float numVisited = 0;

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
      setDrive((float)val);
    } else if (c == 'f') {
      setDrive(1.0);
    } else if (c == 'l') { turnController.centerRobot(); delay(100); turnController.turn(90); }
    else if (c == 'r') { turnController.centerRobot(); delay(100); turnController.turn(-90); }
    else if (c == 'a') { turnController.centerRobot(); delay(100); turnController.turn(-45); }
    else if (c == 'b') { turnController.centerRobot(); delay(100); turnController.turn(45); }
    else if (c == 's') { setDrive(0.5); }
    else if (c == 'D') { setDrive(1.414); }
    else if (c == 'd') { setDrive(1.414/2); }
  }
}

void setDrive(float val) {
  encoder.l_count = encoder.r_count = 0; 
  encoderOdometer.reset(); 
  driveController.drive(val);
  delay(150);
}

void setup() {
  Serial.begin(9600);
  delay(300);       
  // Serial.println("0");
  Wire.begin();
  delay(300);
  // Serial.println("1");
  driveController.initalise_lidar();
  delay(50);
  // Serial.println("2");
  u8g2.begin();
  delay(50);
  // Serial.println("3");
  delay(300);
  turnController.attachMPU();
  delay(50);
  // Serial.println("4");
  turnController.setMPUInitRot();
  delay(50);
  // Serial.println("5");
  turnController.getLidar(FrontSensor, LeftSensor, RightSensor);
  // Serial.println("6");
  // // Task 3.3
  // // //MovementString("ffffflfffbfbfafrfbfflff");
  // // //MovementString("fffff");
  // // //MovementString("ffff");
  // // //MovementString("ffffrfrfff");


  // // // Task 4.3
  drawMazeFromMapping();
  searchMaze();
  // Serial.println("7");

  char path[MAX_PATH_LENGTH];
  char reversed_path[MAX_PATH_LENGTH];

  generatePath(path, MAX_PATH_LENGTH);
  ReversePath(path, reversed_path, MAX_PATH_LENGTH);
  
  MovementString(reversed_path);
  MovementString(path);
}


// bool isDriving = false;
// bool isTurning = true;

void loop() {
  // Task 3.1 - Driving
  //driveController.drivingCorrection(false);

  // Task 3.2 - Turning
  //turnController.turningCorrection(false);

  //   byte error, address;
  // int nDevices;
  // Serial.println("Scanning...");
  // nDevices = 0;
  // for (address = 1; address < 127; address++) {
  //   // The i2c_scanner uses the return value of
  //   // the Write.endTransmisstion to see if
  //   // a device did acknowledge to the address.
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();
  //   if (error == 0) {
  //     Serial.print("I2C device found at address 0x");
  //     if (address < 16)
  //       Serial.print("0");
  //     Serial.print(address, HEX);
  //     Serial.println("  !");
  //     nDevices++;
  //   } else if (error == 4) {
  //     Serial.print("Unknown error at address 0x");
  //     if (address < 16)
  //       Serial.print("0");
  //     Serial.println(address, HEX);
  //   }
  // }
  // if (nDevices == 0)
  //   Serial.println("No I2C devices found\n");
  // else
  //   Serial.println("done\n");
  // delay(5000); // wait 5 seconds for next scan

}


void drawMazeFromMapping() {
  const int gridSize = 9;
  const int gridDimension = 63;   // pixels used for maze
  const int cellSize = gridDimension / gridSize; // expected 7

  mtrn3100::Maze* mazePtr = mapper.getMaze();
  if (!mazePtr) return;

  mtrn3100::Position startPos = mapper.getStartPosition();
  mtrn3100::Position currPos  = mapper.getCurrentPosition();
  mtrn3100::Position goalPos  = mapper.getGoalPosition();

  // pixel centers of cells
  auto cellCenterX = [&](int col){ return col * cellSize + (cellSize / 2); };
  auto cellCenterY = [&](int row){ return row * cellSize + (cellSize / 2); };

  int start_cx = cellCenterX(startPos.x);
  int start_cy = cellCenterY(startPos.y);

  int curr_cx  = cellCenterX(currPos.x);
  int curr_cy  = cellCenterY(currPos.y);

  int goal_cx  = cellCenterX(goalPos.x);
  int goal_cy  = cellCenterY(goalPos.y);

  // Sizes: keep dot small so it fits inside circles
  const int dotHalf = 1; // 3x3 filled box
  // circle radii: ensure >= dotHalf+1 and >=1
  int maxRadius = (cellSize / 2) - 1;
  if (maxRadius < 2) maxRadius = 2;

  u8g2.firstPage();
  do {
    // Draw maze walls
    for (int row = 0; row < gridSize; ++row) {
      for (int col = 0; col < gridSize; ++col) {
        mtrn3100::Node &node = mazePtr->map[col][row];

        int x0 = col * cellSize;
        int y0 = row * cellSize;
        int x1 = x0 + cellSize;
        int y1 = y0 + cellSize;

        if (node.getWallUp()) u8g2.drawLine(x0, y0, x1, y0); // top
        if (node.getWallRight()) u8g2.drawLine(x1, y0, x1, y1); // right
        if (node.getWallDown())  u8g2.drawLine(x0, y1, x1, y1); // bottom
        if (node.getWallLeft())  u8g2.drawLine(x0, y0, x0, y1); // left
        if (node.getVisited()) {
            u8g2.drawPixel(x0 + cellSize / 2, y0 + cellSize / 2);
        }
      }
    }

    // // Start: hollow circle
    // u8g2.drawCircle(start_cx, start_cy, maxRadius, U8G2_DRAW_ALL);

    // Current: small filled dot (so it is visible and fits inside circles)
    u8g2.drawBox(curr_cx - dotHalf, curr_cy - dotHalf, dotHalf * 2 + 1, dotHalf * 2 + 1);

    // // Goal: hollow circle
    // u8g2.drawCircle(goal_cx, goal_cy, maxRadius, U8G2_DRAW_ALL);

    // Completion text
    char completion_buffer[8];
    long percentage = (long)numVisited * 100 / 69; 
    itoa(percentage, completion_buffer, 10);

    // u8g2.setFont(u8g2_font_5x7_tn);
    u8g2.setFont(u8g2_font_5x7_tn);
    // u8g2.drawStr(gridDimension + 4, 10, "Complete:");
    u8g2.drawStr(gridDimension + 4, 24, completion_buffer);
    // u8g2.drawStr(gridDimension + 4 + u8g2.getStrWidth(completion_buffer) + 2, 24, "%");

  } while (u8g2.nextPage());
}


void searchMaze() {
  while (!mapper.reachedGoal()) {
    mtrn3100::Position pos = mapper.getCurrentPosition();
    // Serial.println(pos.x);
    // Serial.println(pos.y);
    numVisited++;
    mapper.update_walls(pos.x, pos.y, current_orientation);
    drawMazeFromMapping();
    mapper.propagate_floodfill();
    char next_move = mapper.decide_next_move(pos.x, pos.y);
    // Serial.println(next_move);
    const char* command = getMovementCommand(current_orientation, next_move);
    // Serial.println(command);
    MovementString(command);
    mapper.update_position(next_move);
    current_orientation = next_move;
    drawMazeFromMapping();
  }
}

// cur
void generatePath(char* path_buffer, int buffer_size) {
  path_buffer[0] = '\0'; // Start with an empty string
  char sim_orientation = 'U'; // Simulate starting at 'U'

  mtrn3100::Position start = mapper.getStartPosition();
  mtrn3100::Position goal  = mapper.getGoalPosition();
  uint8_t simX = start.x;
  uint8_t simY = start.y;

  while (!(simX == goal.x && simY == goal.y)) {
    char next_direction = mapper.decide_next_move(simX, simY);
    if (next_direction == 'X') break; // Path is blocked

    // Use the helper function to get the command string
    const char* command = getMovementCommand(sim_orientation, next_direction);

    // Append the command to the path buffer, checking for overflow
    if (strlen(path_buffer) + strlen(command) < buffer_size) {
        strcat(path_buffer, command);
    } else {
        break; // Stop if buffer is full
    }

    // Update the simulated state for the next iteration
    sim_orientation = next_direction;
    if (next_direction == 'U') simY++;
    else if (next_direction == 'L') simX--;
    else if (next_direction == 'R') simX++;
    else if (next_direction == 'D') simY--;
    // Serial.println(path_buffer);
  }
}

// cur
void ReversePath(const char* original_path, char* reversed_path, int buffer_size) {
  // Serial.println(original_path);
  // Serial.println("");
  // Start with a 180-degree turn to face back
  strncpy(reversed_path, "rr", buffer_size - 1);
  reversed_path[buffer_size - 1] = '\0';

  int len = strlen(original_path);
  for (int i = len - 1; i >= 0; --i) {
    char to_add = '\0';
    switch (original_path[i]) {
      case 'f': to_add = 'f'; break;
      case 'l': to_add = 'r'; break; // Reverse left is right
      case 'r': to_add = 'l'; break; // Reverse right is left
    }

    if (to_add && (strlen(reversed_path) + 1 < buffer_size)) {
        size_t current_len = strlen(reversed_path);
        reversed_path[current_len] = to_add;
        reversed_path[current_len + 1] = '\0';
    }
  }
    
  // End with a final 180-degree turn to face the goal from the start
  if (strlen(reversed_path) + 2 < buffer_size) {
      strcat(reversed_path, "rr");
  }
}


uint8_t orientationToIndex(char orientation) {
    switch (orientation) {
        case 'U': return 0;
        case 'R': return 1;
        case 'D': return 2;
        case 'L': return 3;
    }
    return 4; // Should not happen
}

const char* getMovementCommand(char current, char target) {
    if (current == target) {
        return "f"; // No turn needed
    }
    // Serial.println(current);
    // Serial.println(target);

    uint8_t currentIndex = orientationToIndex(current);
    uint8_t targetIndex = orientationToIndex(target);

    // Using modulo arithmetic to check for turns
    // (currentIndex + 1) % 4 corresponds to a right turn
    if ((currentIndex + 1) % 4 == targetIndex) {
        return "rf"; // Turn right, then forward
    }
    // (currentIndex + 4 - 1) % 4 corresponds to a left turn
    else if ((currentIndex + 3) % 4 == targetIndex) {
        return "lf"; // Turn left, then forward
    }
    // Any other case must be a 180-degree turn
    else {
        return "rrf"; // Turn 180, then forward
    }
}
