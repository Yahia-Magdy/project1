#define IR1 16
#define IR2 17
#define IR3 18
#define IR4 19
#define IR5 21

int tryNumber = 1;
String path = "";
String simplePath = "";

void setup() {
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  Serial.begin(9600);

//  //UNCOMMENT TO TEST
//  String path_ = "";
//  String simplePath_ = "";
//  path_ = "LLBRLLLLBLSRBLLRLLLBSBLLLR";
//  int len_ = path_.length() + 1;
//  char buff_[len_];
//  path.toCharArray(buff_, len_);
//  simplePath_ = simplify(buff_);
//  Serial.println(simplePath_); //correct answer is "SLLSRRLSLR"
}

void loop() {
  if (tryNumber == 1) { //check for the try number, then run the appropriate code
    firstTry();
  }
  else {
    secondTry();
  }
}

void firstTry() {
  String state = lineState();
  if (state ==  "offLEFT") {
    //((correct course to the right))
  }
  else if (state ==  "offRIGHT") {
    //((correct course to the left))
  }
  else if (state ==  "NONE") {
    //((turn back because dead end reached))
    path += "B";
  }
  else if (state ==  "LEFTandRIGHT") {
    //((move a little bit forward to check for special cases))
    String state2 = lineState();
    if (state2 == "NONE") { //T-Intersection
      //((turn left as priority is to the left way))
      path += "L";
    }
    else if (state2 == "STRAIGHT") { //Cross Intersection
      //((turn left as priority is to the left way))
      path += "L";
    }
    else if (state2 == "LEFTandRIGHT") { //Solid Rectangle (end of the maze)
      //first try finished, now to the second try
      tryNumber = 2;
      int len = path.length() + 1;
      char buff[len];
      path.toCharArray(buff, len);
      simplePath = simplify(buff);
    }
  }
  else if (state ==  "LEFT") {
    //((move a little bit forward to check for special cases))
    String state2 = lineState();
    if (state2 == "NONE") { //90-Degrees Turn
      //((turn left as there are no other way))
      path += "L";
    }
    else if (state2 == "STRAIGHT") { //T-Intersection
      //((turn left as priority is to the left way))
      path += "L";
    }
  }
  else if (state ==  "RIGHT") {
    //((move a little bit forward to check for special cases))
    String state2 = lineState();
    if (state2 == "NONE") { //90-Degrees Turn
      //((turn right as there are no other way))
      path += "R";
    }
    else if (state2 == "STRAIGHT") { //T-Intersection
      //((keep moving forward as priority is to the left way))
      path += "S";
    }
  }
  else if (state ==  "STRAIGHT") {
    //((keep moving forward))
  }
}

void secondTry() {
  //((TODO))
}

//returns the state of the line that is currently followed
String lineState() {
  //reading ir sensor array
  int ir1 = digitalRead(IR1);
  int ir2 = digitalRead(IR2);
  int ir3 = digitalRead(IR3);
  int ir4 = digitalRead(IR4);
  int ir5 = digitalRead(IR5);

  //if only the left-most sensor detects blackness, then the robot is going off-course to the left & must be corrected
  if (!ir2 && !ir3 && !ir4 && ir1) {
    return "offLEFT";
  }
  //if only the right-most sensor detects blackness, then the robot is going off-course to the right & must be corrected
  if (!ir2 && !ir3 && !ir4 && ir5) {
    return "offRIGHT";
  }
  //if no sensors detect blackness, then the robot has either reached a dead end and must go back, or has gone off-course and can no longer be corrected
  //SPECIAL CASE: if this was the case right after "RIGHT" or "LEFT" is returned, then the robot is at a 90-degrees turn
  //SPECIAL CASE: if this was the case right after "LEFTandRIGHT" is returned, then the robot is at a T-intersection
  if (!ir2 && !ir3 && !ir4) {
    return "NONE";
  }
  //if both the left-most & right-most sensors detect blackness, then then are branches to the left & to the right
  //IMPORTANT: must move a little forward and call the function again to check for T-intersection if "NONE" is returned
  //SPECIAL CASE: if this was the case right after "LEFTandRIGHT", then the robot is at a solid rectangle & has reached the end of the maze
  if (ir1 && ir5) {
    return "LEFTandRIGHT";
  }
  //if the left-most detects blackness, then then is a branch to the left
  //IMPORTANT: must move a little forward and call the function again to check for 90-degrees turn if "NONE" is returned
  if (ir1) {
    return "LEFT";
  }
  //if the right-most detects blackness, then then is a branch to the right
  //IMPORTANT: must move a little forward and call the function again to check for 90-degrees turn if "NONE" is returned
  if (ir5) {
    return "RIGHT";
  }
  //if none of the above cases are satisfied, then the robot is moving correctly along a straight line
  return "STRAIGHT";
}

//takes an string of a sequence of movements and returns an string with a simplified sequence, convert string to char array before passing to the function
char* simplify(char* sequence) {
  //calculate size of the string
  int n = 0;
  while (sequence[n] != '\0') {
    n++;
  }

  //new string variable for simplified sequence
  char* simple = (char*) malloc(n);
  int c = 0; //increment for the new string

  int done = 1; //will stay = 1 if the sequence cannot be simplified more, else = 0
  int i; //increment variable for the old string
  for (i = 1; i < n - 1; i++) { //start from 1 till n-1 because a 'B' cannot be the first or last movement
    if (sequence[i] == 'B') {
      done = 0; //simplifications exist for the sequence
      if (sequence[i - 1] == 'L' && sequence[i + 1] == 'L') { //replace "LBL" with 'S'
        simple[c] = 'S';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'L' && sequence[i + 1] == 'R') { //replace "LBR" with 'B'
        simple[c] = 'B';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'L' && sequence[i + 1] == 'S') { //replace "LBS" with 'R'
        simple[c] = 'R';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'R' && sequence[i + 1] == 'L') { //replace "RBL" with 'B'
        simple[c] = 'B';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'S' && sequence[i + 1] == 'L') { //replace "SBL" with 'R'
        simple[c] = 'R';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'S' && sequence[i + 1] == 'S') { //replace "SBS" with 'B'
        simple[c] = 'B';
        i += 2;
        c++;
        break;
      }
      else { //add the original movement if cannot simplify this segment
        simple[c] = sequence[i - 1];
        c++;
      }
    }
    else { //add the original movement if cannot simplify this segment
      simple[c] = sequence[i - 1];
      c++;
    }
  }

  //return old string if it was not simplified
  if (done == 1) {
    return sequence;
  }

  //add trailing original movements
  for (; i < n; i++) {
    simple[c] = sequence[i];
    c++;
  }
  simple[c] = '\0'; //end the string with a null character

  return simplify(simple);
}
