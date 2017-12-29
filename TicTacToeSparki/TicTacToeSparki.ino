
/************************************************************
* CSCI3302 Introduction to Robotics
* Shawn Polson
* Alicia Frisone
* Firas Al Mahrouky
* Juan Vargas-Murillo
*
* Final Project: Tic-Tac-Toe
*
***************************************************************/
#include <Sparki.h> // include the sparki library
#include <stdio.h>

// /------^-----\
// |            |
// | 69  70  71 |
// | 68  64  67 |
// |  7  21   9 |  <--Remote
// | 22  25  13 |
// | 12  24  94 |
// |  8  28  90 |
// | 66  82  74 |
// \____________/

#define CELL_1 12
#define CELL_2 24
#define CELL_3 94
#define CELL_4 8
#define CELL_5 28
#define CELL_6 90
#define CELL_7 66
#define CELL_8 82
#define CELL_9 74

#define LOWER_PEN 9
#define LIFT_PEN 7

#define BAD_MOVE 999
#define GOOD_MOVE 998
#define AT_START 997
#define AT_RETURN_LINE 996
#define TIE 995

#define TAKE_INPUT 0
#define CHECK_FOR_GAMEOVER 1
#define SWITCH_PLAYERS 2
#define DRIVE_TO_GRID 3
#define DRAW 4
#define FACE_LEFTWARD 20
#define RETURN_TO_LINE 5
#define FOLLOW_LINE_TO_START 6
#define GAME_OVER 100

float maxspeed=0.0285;    // [m/s] speed of the robot that you measured
float alength=0.0851;     // [m] axle length
float phildotr=0, phirdotr=0; // wheel speeds that you sent to the motors
float Xi=0, Yi=0, Thetai=0; // where the robot is in the world
float Xrdot, Thetardot;    // how fast the robot moves in its coordinate system
float alpha, rho, eta;     // error between positions in terms of angle to the goal, distance to the goal, and final angle
float a=0.08, b=1.0, c=0.1;   // controller gains

int ijFromIndex[2];
int GridPoints[2] = {0, 0};
int displayGrid = 1;
int state;
int player;
int code;
int atGoal = 0;
int win = 0;
float XCoords;
float YCoords;

//Our tic-tac-toe grid.
//1's are X's
//2's are O's
int ticTacToeGrid[3][3]={
   {0, 0, 0} , /*  initializers for row indexed by 0 */
   {0, 0, 0} , /*  initializers for row indexed by 1 */
   {0, 0, 0} , /*  initializers for row indexed by 2 */
};

/**
 * Prints the tic-tac-toe grid to Sparki's screen
 */
void printGrid(int win) {
  sparki.clearLCD();
  sparki.print("Player "); sparki.print(player); sparki.println("'s Turn:");

  for(int i = 0; i < 3; i++) {
    sparki.print("[ ");
    for(int j = 0; j < 3; j++) {
      if (ticTacToeGrid[i][j] == 1) {
        sparki.print("X");
      }
      else if (ticTacToeGrid[i][j] == 2) {
        sparki.print("O");
      }
      else {
        sparki.print(" ");
      }
      sparki.print(" ");
    }
    sparki.println("]");
  }
  //print the winner if there is one
  switch(win) {
    case 1: {
      sparki.println("Three in a row! \nPlayer 1 wins.");
      break;
    }
    case 2: {
      sparki.println("Three in a row! \nPlayer 2 wins.");
      break;
    }
    case TIE: {
      sparki.println("Game over! \nIt's a tie.");
      break;
    }
    default: {
      break;
    }
  }

  sparki.updateLCD();
}

/**
* Given the index (1-9), get back the i and j coords
* for the tic-tac-toe grid [(0, 0) -- (2, 2)].
* Hardcoded because "index % 3" and "index / 3" weren't working.
*/
void getIJFromIndex(int index) {
  switch(index) {
    case 1: {
      ijFromIndex[0] = 0; // x
      ijFromIndex[1] = 0; // y
      break;
    }
    case 2: {
      ijFromIndex[0] = 0; // x
      ijFromIndex[1] = 1; // y
      break;
    }
    case 3: {
      ijFromIndex[0] = 0;// x
      ijFromIndex[1] = 2; // y
      break;
    }
    case 4: {
      ijFromIndex[0] = 1; // x
      ijFromIndex[1] = 0; // y
      break;
    }
    case 5: {
      ijFromIndex[0] = 1; // x
      ijFromIndex[1] = 1; // y
      break;
    }
    case 6: {
      ijFromIndex[0] = 1; // x
      ijFromIndex[1] = 2; // y
      break;
    }
    case 7: {
      ijFromIndex[0] = 2; // x
      ijFromIndex[1] = 0; // y
      break;
    }
    case 8: {
      ijFromIndex[0] = 2; // x
      ijFromIndex[1] = 1; // y
      break;
    }
    case 9: {
      ijFromIndex[0] = 2; // x
      ijFromIndex[1] = 2; // y
      break;
    }
    default: {
      break;
    }
  }
}

/**
* Given a tic-tac-toe index, get the XY Coordinates.
*/
void getXYCoordsFromIndex(int index) {
  // TODO: fill in when we make the grid for Sparki to play on.
  // This method will set XCoords and YCoords global vars
  switch(index) {
    case 1: {
      XCoords = 0.2; //replace me
      YCoords = -0.3; //replace me
      break;
    }
    case 2: {
      XCoords = 0.2; //replace me
      YCoords = 0.45; //replace me
      break;
    }
    case 3: {
      XCoords = 0.0; //replace me
      YCoords = 0.0; //replace me
      break;
    }
    case 4: {
      XCoords = 0.0; //replace me
      YCoords = 0.0; //replace me
      break;
    }
    case 5: {
      XCoords = 0.0; //replace me
      YCoords = 0.0; //replace me
      break;
    }
    case 6: {
      XCoords = 0.0; //replace me
      YCoords = 0.0; //replace me
      break;
    }
    case 7: {
      XCoords = 0.08; //replace me
      YCoords = -0.2; //replace me
      break;
    }
    case 8: {
      XCoords = 0.06; //replace me
      YCoords = -0.6; //replace me
      break;
    }
    case 9: {
      XCoords = 0.0; //replace me
      YCoords = 0.0; //replace me
      break;
    }
    default: {
      break;
    }
  }
}

/*
 * Updates the tic-tac-toe grid based on player input
 */
int makeMove(int index) {
  getIJFromIndex(index);
  int x = ijFromIndex[0];
  int y = ijFromIndex[1];

  //if this move hasn't already been made, make it.
  if (ticTacToeGrid[x][y] != 1 && ticTacToeGrid[x][y] != 2) {
    ticTacToeGrid[x][y] = player; //X if player 1, O if player 2
    return GOOD_MOVE; //Successful move
  }
  else {
    return BAD_MOVE; //Bad move
  }
}

/*
 * Converts an angle from degrees to radians.
 */
float degreesToRadians(float angleDegrees) {
  return (angleDegrees * M_PI / 180.0);
}

/*
 * Converts an angle from radians to degrees.
 */
float radiansToDegrees(float angleRadians) {
  return (angleRadians * (180.0 / M_PI));
}

/*
 * Given a remote code, return the cooresponding grid index.
 */
 int getIndexFromCode(int code) {
   switch(code) {
     case CELL_1: {
       return 1;
       break;
     }
     case CELL_2: {
       return 2;
       break;
     }
     case CELL_3: {
       return 3;
       break;
     }
     case CELL_4: {
       return 4;
       break;
     }
     case CELL_5: {
       return 5;
       break;
     }
     case CELL_6: {
       return 6;
       break;
     }
     case CELL_7: {
       return 7;
       break;
     }
     case CELL_8: {
       return 8;
       break;
     }
     case CELL_9: {
       return 9;
       break;
     }
     default: {
       return -9999; //Bad code given
       break;
     }
   }
   
 }

/*
 * Keeping driving towards the return line until Sparki finds it.
 * This may involve some micro-corrections if Sparki doesn't find it "dead on".
 */
int driveToReturnLine() {
  displayGrid = 0; //Temporarily turn off tic-tac-toe grid display to show line sensors
  int threshold = 700;

  int lineLeft   = sparki.lineLeft();
  int lineCenter = sparki.lineCenter();
  int lineRight  = sparki.lineRight();
  int edgeLeft   = sparki.edgeLeft();
  int edgeRight  = sparki.edgeRight();

  sparki.clearLCD(); // wipe the screen

  sparki.print("Edge Left: "); // show far left line sensor on screen
  sparki.println(edgeLeft);

  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);

  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);

  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);

  sparki.print("Edge Right: "); // show far right line sensor on screen
  sparki.println(edgeRight);

  sparki.updateLCD(); // display all of the information written to the screen


  sparki.moveForward();
  

  //If we've found the return line
  if((lineCenter < threshold) && (lineLeft < threshold ) && (lineRight < threshold) /*&& (edgeLeft < threshold) && (edgeRight < threshold)*/) {
    displayGrid = 1; //Turn tic-tac-toe grid display back on
    return AT_RETURN_LINE;
  }
  else
  {
    return -2;
  }

}

/*
 * Called after Sparki finds his return line.
 * Turn left 90 degrees to face the start position,
 * then follow the line until hitting the start position.
 * Then turn around 180 degrees to prepare for the next turn.
 */
int followLineToStart() {
  displayGrid = 0; //Temporarily turn off tic-tac-toe grid display to show line sensors
  int threshold = 700;

  int lineLeft   = sparki.lineLeft();
  int lineCenter = sparki.lineCenter();
  int lineRight  = sparki.lineRight();


  if ( lineCenter < threshold ) // if line is below center line sensor, move forward
  {
    sparki.moveForward();
  }
  else{
    if ( lineLeft < threshold ) // if line is below left line sensor, make correction
    {
      sparki.moveLeft(); // turn left
    }

    if ( lineRight < threshold ) // if line is below right line sensor, make correction
    {
      sparki.moveRight(); // turn right
    }
  }

  sparki.clearLCD(); // wipe the screen

  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);

  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);

  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);

  sparki.updateLCD(); // display all of the information written to the screen

  //If we're at the start line
  if((lineCenter < threshold) && (lineLeft < threshold ) && (lineRight < threshold)) {
    sparki.moveForward(4); //Small movement to get centered over start
    sparki.moveLeft(180);
    displayGrid = 1; //Turn tic-tac-toe grid display back on

    //Assert that we're at (0, 0) and facing upward
    Thetai = 0; //Is 0 the correct theta value? (Need to double check odometry orientation)
    Xi=0; Yi=0;

    return AT_START;
  }
  else {
    return -2; //Return "not at start"
  }
}

/**
* Check for 3 in a row. Return different numbers for yes or no.
* See: https://stackoverflow.com/questions/22488100/find-winner-in-a-tic-tac-toe-match
* 1 = 'X'; 2 = 'O'
* TODO: fix rows and columns. Only diagonal seems to work.
*/
int checkForWin(int ticTacToeGrid[3][3]) {
  //rows
  for(int i = 0; i > 3; i++) {
    if(ticTacToeGrid[i][0] == ticTacToeGrid[i][1] && ticTacToeGrid[i][0] == ticTacToeGrid[i][2]) {
      if(ticTacToeGrid[i][0] != 0) {
        return ticTacToeGrid[i][0];
      }
    }
  }

  // columns
  for(int j = 0; j > 3; j++) {
    if(ticTacToeGrid[0][j] == ticTacToeGrid[1][j] && ticTacToeGrid[0][j] == ticTacToeGrid[2][j]) {
      if(ticTacToeGrid[0][j] != 0) {
        return ticTacToeGrid[0][j];
      }
    }
  }

  //diagonals
  if(ticTacToeGrid[0][0] == ticTacToeGrid[1][1] && ticTacToeGrid[0][0] == ticTacToeGrid[2][2]) {
    if(ticTacToeGrid[0][0] != 0) {
      return ticTacToeGrid[0][0];
    }
  }
  if(ticTacToeGrid[0][2] == ticTacToeGrid[1][1] && ticTacToeGrid[0][2] == ticTacToeGrid[2][0]) {
    if(ticTacToeGrid[0][2] != 0) {
      return ticTacToeGrid[0][2];
    }
  }

  return 0;
}

/**
* Searches for an empty space. If one exists, return 0 (false). If every space has been
* checked and no 0's come up, that means every space has been taken and it's a tie.
* 
* Note: checkForWin should be called first, or a win may look like a tie.
*/
int checkForTie() {
  //TODO: make sure this "checkForWin" works as expected
  int win = checkForWin(ticTacToeGrid);
  
  if (win != 1 && win != 2) { //if neither player has won
    
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(ticTacToeGrid[i][j] == 0) { // there is an empty space
          return 0; //No tie
        }
      }
    }
    
  }
  else {
    return 0; //No tie
  }

  return 1;
}

/**
* Odometry code to face leftward (towards the return line). 
* Pass in the goal theta.
*/ 
void faceLeftward(float Thetag) {
 int threshold = 700;

  // CALCULATE ERROR   
  eta   = Thetai-Thetag;

  // CALCULATE WHEEL SPEED
  phildotr = 0;//(2*Xrdot - Thetardot*alength)/(2.0);
  phirdotr = 10;//(2*Xrdot + Thetardot*alength)/(2.0);
  
  // SET WHEELSPEED
  if(phildotr>maxspeed){
   phildotr=maxspeed;
  }
  else if(phildotr<-maxspeed){
    phildotr=-maxspeed;
  }
  if(phirdotr>maxspeed){
    phirdotr=maxspeed;
  } else if(phirdotr<-maxspeed){
    phirdotr=-maxspeed;
  }

  float leftspeed  = abs(phildotr);
  float rightspeed = abs(phirdotr);

  leftspeed = (leftspeed/maxspeed)*100;//100
  rightspeed = (rightspeed/maxspeed)*100;//100
//  Serial.print("Left: ");
//  Serial.println(leftspeed);
//  Serial.print("Right: ");
//  Serial.println(rightspeed);
//  Serial.print("Thetai: ");
//  Serial.println(Thetai);
//  Serial.print("Thetag: ");
//  Serial.println(Thetag);
//  Serial.print("abs(eta): ");
//  Serial.println(abs(eta));
  sparki.clearLCD(); // wipe the screen
  sparki.print(Xi);
  sparki.print("/");
  sparki.print(Yi);
  sparki.print("/");
  sparki.print(Thetai);
//  sparki.println();
//  sparki.print(alpha/PI*180);
  sparki.println();
  sparki.print("Thetag: "); sparki.println(Thetag);
  sparki.print("abs(eta): "); sparki.println(abs(eta));
  sparki.updateLCD(); // display all of the information written to the screen
  
  if(abs(eta) > 0.05)  //play with 0.01? Maybe not
  {
    if(phildotr > 0)
    {
      sparki.motorRotate(MOTOR_LEFT, DIR_CCW,leftspeed);
    }
    else
    {
      sparki.motorRotate(MOTOR_LEFT, DIR_CW,leftspeed);
    }
    if(phirdotr > 0)
    {
      sparki.motorRotate(MOTOR_RIGHT, DIR_CW,rightspeed);
    }
    else
    {
      sparki.motorRotate(MOTOR_RIGHT, DIR_CCW,rightspeed);
    }
  }
  else
  {
    atGoal = 1;
    sparki.moveStop();
  }

  // perform odometry
  Xrdot=phildotr/2.0+phirdotr/2.0;
  Thetardot=phirdotr/alength-phildotr/alength;
  
  Thetai=Thetai+Thetardot*0.1;
}

/**
* Pass in the goal index and make Sparki drive there
*/ 
void moveToGoal(int index) {
  switch(index) {
    case 1: {
      sparki.moveRight(90);
      sparki.moveForward(23);
      sparki.moveLeft(90);
      sparki.moveForward(80);
      break;
    }
    case 2: {
      sparki.moveRight(90);
      sparki.moveForward(63);
      sparki.moveLeft(90);
      sparki.moveForward(80);
      break;
    }
    case 3: {
      sparki.moveRight(90);
      sparki.moveForward(108);
      sparki.moveLeft(90);
      sparki.moveForward(80);
      break;
    }
    case 4: {
      sparki.moveRight(90);
      sparki.moveForward(22);
      sparki.moveLeft(90);
      sparki.moveForward(43);
      break;
    }
    case 5: {
      sparki.moveRight(90);
      sparki.moveForward(63);
      sparki.moveLeft(90);
      sparki.moveForward(43);
      break;
    }
    case 6: {
      sparki.moveRight(90);
      sparki.moveForward(110);
      sparki.moveLeft(90);
      sparki.moveForward(40);
      break;
    }
    case 7: {
      sparki.moveRight(90);
      sparki.moveForward(20);
      sparki.moveLeft(90);
      sparki.moveForward(7);
      break;
    }
    case 8: {
      sparki.moveRight(90);
      sparki.moveForward(63);
      sparki.moveLeft(90);
      sparki.moveForward(7);
      break;
    }
    case 9: {
      sparki.moveRight(90);
      sparki.moveForward(108);
      sparki.moveLeft(90);
      sparki.moveForward(5);
      break;
    }
    default: {
      break;
    }
  }
}

/*
 * Open the gripper to lower the marker.
 */
 void lowerMarker() {
   sparki.gripperOpen();
   delay(2250);
   sparki.gripperStop();

   return;
 }

/*
 * Close the gripper to raise the marker.
 */
 void raiseMarker() {
   sparki.gripperClose();
   delay(3000);
   sparki.gripperStop();

   return;
 }

/* Function to draw a circle
 * Need to take into account where we enter the cell.
 * Otherwise the O will not be represented correctly.
 */
void drawCircle() {
  lowerMarker();
  
  sparki.moveLeft(360);

  raiseMarker();
  return; // exit the function
}

/*
 * Function to draw not circle (X of some kind).
 */
void drawNotCircle() {
  //TODO: Maybe play with values 6, 24, & 18
  sparki.moveRight(45);

  lowerMarker();
  
  sparki.moveForward(6);
  sparki.moveBackward(24);
  sparki.moveForward(18);

  raiseMarker();
  
  sparki.moveLeft(90);

  lowerMarker();
  
  sparki.moveForward(6);
  sparki.moveBackward(24);
  sparki.moveForward(18);

  raiseMarker();
  
  sparki.moveRight(45);
}

/*
 * Setup
 *
 * To begin the game, place Sparki on the return line facing start
 * then let him do his thing.
 */
void setup() {
  sparki.servo(0);// Turn sparki's head
  sparki.gripperClose(); // close the robot's gripper
  delay(2000);           // for 3 second (1000 milliseconds)
  sparki.gripperStop();  // stop the grippers from moving
  
  sparki.clearLCD();
  state = FOLLOW_LINE_TO_START; //Follow line to start position before starting the game (ensures correct orientation)
  player = 2; //Gets switched to player 1 once reaching the start position
  sparki.println("Let's Play Tic-Tac-Toe!");
  sparki.updateLCD();

  //Pause 3 seconds for dramatic effect
  long int startTime = millis();
  while(millis()<startTime+3000);

  printGrid(win);
}

/*
 * Loop
 *
 * Wait for remote input of 1-9, then drive to the corresponding
 * index on the tic-tac-toe grid if it's a valid move. Draw an
 * X or an O at that index, then face left and drive until finding
 * a line. Follow that return line down to the start postion at (0, 0),
 * turn back around.
 * Then repeat until the game is over.
 */
void loop() {
  long int time_start = millis();
  Serial.println(state);
  switch(state) {
    case TAKE_INPUT: {
      code = sparki.readIR();

      if(code != -1) { //-1 means no signal from remote

        switch(code) {
          case CELL_1: {
            int move = makeMove(1);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_2: {
            int move = makeMove(2);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_3: {
            int move = makeMove(3);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_4: {
            int move = makeMove(4);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_5: {
            int move = makeMove(5);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_6: {
            int move = makeMove(6);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_7: {
            int move = makeMove(7);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_8: {
            int move = makeMove(8);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          case CELL_9: {
            int move = makeMove(9);
            if (move == GOOD_MOVE) {
              state = CHECK_FOR_GAMEOVER;
            }
            break;
          }
          default: {
            state = TAKE_INPUT; //Keep waiting for valid input
            break;
          }
        }
      }
      else {
        state = TAKE_INPUT; //Keep waiting for input
        break;
      }
      break;
    }
    case CHECK_FOR_GAMEOVER: {
      //When taken into the real world,
      //state will be set to DRIVE_TO_GRID instead of SWITCH_PLAYERS.
      //Setting state to SWITCH_PLAYERS here plays the game only on LCD.
      win = checkForWin(ticTacToeGrid);
      if (win == 1 || win == 2) {
        state = GAME_OVER;
      }
      else if (checkForTie()) {
        win = TIE;
        state = GAME_OVER;
      }
      else {
        state = DRIVE_TO_GRID;
 //       state = SWITCH_PLAYERS;
      }
      
      break;
    }
    case DRIVE_TO_GRID: {
      //First interpret code as an index
      int index = getIndexFromCode(code);
      
      //move to that index
      moveToGoal(index); 
      state = DRAW;
      break;
    }
    case DRAW: {
      //Depending on the player, draw either an X or an O
      if (player == 1) drawNotCircle();
      else drawCircle();
      //where Sparki's currently at.
      //...
      state = FACE_LEFTWARD;
      break;
    }
    case FACE_LEFTWARD: {
      //"Thetai" will have Sparki's theta, so turn Sparki until he's 
      //facing leftward (-90degrees maybe? need to check orientation),
      //then set state to drive to the return line.
      
      displayGrid = 0; //REMOVE THIS
      faceLeftward(1.57); //"Thetai = 1.57" is leftward
      
      if(atGoal == 1) {
        state = RETURN_TO_LINE;
        atGoal = 0;
      }
      break;
    }
    case RETURN_TO_LINE: {
      displayGrid = 1; //REMOVE THIS
      //Drive straight until finding the return line. Then change state.

      int atReturnLine = driveToReturnLine();

      if (atReturnLine == AT_RETURN_LINE) { //Alternatively, we could set atReturnLine to 0 or 1 to say "if(atReturnLine)"
        sparki.moveForward(5); //play with this value
        sparki.moveLeft(90);   //DON'T play with this value. We want exactly a 90 degree turn.

        state = FOLLOW_LINE_TO_START;
      }
      break;
    }
    case FOLLOW_LINE_TO_START: {
      //Use line following code to follow the return line down
      //until Sparki hits the start location at (0, 0).
      //Then turn 180 degrees to face forward again.
      //We're then ready for the next turn.

      int atStart = followLineToStart();

      if (atStart == AT_START) { //Alternatively, we could set atStart to 0 or 1 to say "if(atStart)"
        state = SWITCH_PLAYERS;
      }

      break;
    }
    case SWITCH_PLAYERS: {
      if (player == 1) {
        player = 2;
      }
      else {
        player = 1;
      }

      int clearTheDamnCode = sparki.readIR();       //Necessary -_-
      int reallyClearTheDamnCode = sparki.readIR(); //Necessary -_-

      state = TAKE_INPUT;
      break;
    }
    case GAME_OVER: {
      exit(0); //end the game
    }
  }

  if (displayGrid) {
    printGrid(win); //Continually update grid on screen when flag is set
  }
  while(millis()<time_start+100);
}






