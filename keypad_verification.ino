  //Using defines 
  #define ROW_NUM 4
  #define COL_NUM 3
  #define DELAY_T 5
  #define SERIAL_B 9600
 int pinRows[ROW_NUM]= {2, 3, 4, 5}; //Setting up pins to be used for the columns
 int pinCols[COL_NUM]= {6, 7, 8}; //Setting up pins to be used for the rows

 char KEYMAP[ROW_NUM][COL_NUM] = {
                        {'1', '2', '3'}, //main matrix used to return values to the keypad
                        {'4', '5', '6'}, 
                        {'7', '8', '9'}, 
                        {'*', '0', '#'} 
                        };


char scan_keypad_once(){
  //row = i
  //col = j
  for (int i=0; i < ROW_NUM; ++i){
    digitalWrite(pinRows[i], HIGH); //Setting all the row pins to HIGH
  }

  for (int i = 0; i < ROW_NUM; ++i){ //Start a scan for each row
    digitalWrite(pinRows[i],LOW); //Start every row set at 0
    delayMicroseconds(DELAY_T); //CHECK
    for (int j = 0; j < COL_NUM; j++){
      if(digitalRead(pinCols[j]) == LOW){ //if 0,
        digitalWrite(pinRows[i], HIGH); //Then set pin to HIGH (1)
        return KEYMAP[i][j];
      } //end of IF loop
    } //end of FOR LOOP 
    digitalWrite(pinRows[i],HIGH); //Restoring row to HIGH
  }//end of FOR loop
return '\0'; 
}//end of function loop

char get_key(){
  static bool buttonPressed = false; //Boolean variable to determine if a button is pressed
  char scan = scan_keypad_once();
  if(scan != '\0' && !buttonPressed){
    buttonPressed = true;
    delay(20);
    if(scan_keypad_once() == scan){
      return scan;
    }
  }

  if(scan == '\0'){ //if the keypad returns NULL character
    buttonPressed = false; //This means button is released
  }

return '\0';

} //end of function loop 

void setup(){ //This function sets the rows to HIGH inputs and the columns to outputs
    // put your setup code here, to run once:
  Serial.begin(SERIAL_B); //Using a baud rate of 9600 bps

  for (int row = 0; row < ROW_NUM; ++row){
    pinMode(pinRows[row], OUTPUT); //Setting the rows to become outputs
    digitalWrite(pinRows[row], HIGH); //Making these rows high
  }

  for(int col = 0; col < COL_NUM; ++col){
    pinMode(pinCols[col], INPUT_PULLUP);//Setting the columns to becomes 
  }
  Serial.println("Keypad ready:");
}

void loop() {
  // put your main code here, to run repeatedly:
char key = get_key();
if(key != '\0'){
  Serial.print("Key: " );
  Serial.println(key);
}
}
