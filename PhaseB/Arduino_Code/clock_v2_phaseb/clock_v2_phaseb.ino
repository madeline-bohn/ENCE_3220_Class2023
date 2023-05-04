#define BUTTON_2  2  
#define BUTTON_1  3  
#define GREEN_LED 4
#define RED_LED   5
#define BUZZER    6

#define DATA      9 // DS
#define LATCH     8 // ST_CP
#define CLOCK     7 // SH_CP

#define DIGIT_4   10
#define DIGIT_3   11
#define DIGIT_2   12
#define DIGIT_1   13

#define BUFF_SIZE 20

#define DEFAULT_COUNT 30 // default value is 30secs

// 7-Seg Display Variables
unsigned char gtable[]=
{0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c
,0x39,0x5e,0x79,0x71,0x00};
byte gCurrentDigit;

// Volatile Variables
volatile unsigned char gISRFlag1 = 0;
volatile unsigned char gBuzzerFlag = 0;
volatile unsigned char gISRFlag2 = 0;
volatile unsigned char gbuttonISR1 = 0;
volatile unsigned char gbuttonISR2 = 0;
volatile int  gCount = DEFAULT_COUNT;

// Timer Variables
unsigned char gTimerRunning = 0; 
unsigned int gReloadTimer1 = 62500; // corresponds to 1 second
unsigned int gReloadTimer2 = 100;
byte gReloadTimerDisplay = 10;  // display refresh time
char gIncomingChar;
char gCommMsg[BUFF_SIZE];
int buff = 0;
byte gPackageFlag = 0;
byte gProcessFlag = 0;


void setup() {
   // LEDs Pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // LEDs -> Timer Stopped
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  // Button Pins
  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), gbuttonISR1, RISING);
  pinMode(BUTTON_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), gbuttonISR2, RISING);

  // Buzer Pins
  pinMode(BUZZER, OUTPUT);

  // Buzzer -> Off
  digitalWrite(BUZZER,LOW);

  // 7-Seg Display
  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  pinMode(DIGIT_4, OUTPUT);

  // Shift Register Pins
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, OUTPUT);

  dispOff();  // turn off the display
  pinMode(LED, OUTPUT); //declare led as an output
  Serial.begin(9600); //baud rate of 9600

  // Initialize Timer1
  // Speed of Timer1 = 16MHz/64 = 250 KHz
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = gReloadTimer1;            // max value 2^16 - 1 = 65535
  TCCR1A |= (1<<WGM11);
  TCCR1B = (1<<CS11) | (1<<CS10);   // 64 prescaler
  TIMSK1 |= (1<<OCIE1A);
  interrupts(); 
}
void display(unsigned char num, unsigned char dp){
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, gtable[num] | (dp<<7));
  digitalWrite(LATCH, HIGH);
}
//used to compare two arrays of chars to see if equal
char compareMsg(char a[], char b[], int size){
  int i; 
  char result;

  for(i = 0;  i < size; i++){
    if(a[i] != b[i]){
      result = 0; //arrays not equal
    }else{
      result = 1; //arrays are equal
    }
  } 
  return result;
} 
//interrupt routine to read data from ESP
ISR(TIMER_COMPA_vect){ 
  if(Serial.available()>0){
    gISRFlag2 = 1;
  }
}
void dispOff(){
   digitalWrite(DIGIT_1, HIGH);
   digitalWrite(DIGIT_2, HIGH);
   digitalWrite(DIGIT_3, HIGH);
   digitalWrite(DIGIT_4, HIGH);
}
void buttonISR2(){
  //increment clock
  gCount++;
}
void buttonISR1(){ 
  // Set ISR Flag
  gISRFlag1 = 1;
}

// Timer2 interrupt service routine (ISR)
ISR(TIMER2_COMPA_vect) {
  dispOff();  // turn off the display
  OCR2A = gReloadTimer2;  // load timer
 
  switch (gCurrentDigit) {
    case 1: //0x:xx
      display( int((gCount/60) / 10) % 6, 0 );   // prepare to display digit 1 (most left)
      digitalWrite(DIGIT_1, LOW);  // turn on digit 1
      break;
 
    case 2: //x0:xx
      display( int(gCount / 60) % 10, 1 );   // prepare to display digit 2
      digitalWrite(DIGIT_2, LOW);     // turn on digit 2
      break;
 
    case 3: //xx:0x
      display( (gCount / 10) % 6, 0 );   // prepare to display digit 3
      digitalWrite(DIGIT_3, LOW);    // turn on digit 3
      break;
 
    case 4: //xx:x0
      display(gCount % 10, 0); // prepare to display digit 4 (most right)
      digitalWrite(DIGIT_4, LOW);  // turn on digit 4
      break;

    default:
      break;
  }
  gCurrentDigit = (gCurrentDigit % 4) + 1;
}

// Timer1 interrupt service routine (ISR)
ISR(TIMER1_COMPA_vect){
  gCount--;
  OCR1A = gReloadTimer1;

  if(gCount == 0) {
      // Stop Timer
      stopTimer1();
      
      // Raise Alarm
      gBuzzerFlag = 1;
      gTimerRunning = 0;
  }
}
void stopTimer1(){
  // Stop Timer
  TCCR1B = 0; // stop clock
  TIMSK1 = 0; // cancel clock timer interrupt
}
void startTimer1(){
  // Initialize Timer1 (16bit) -> Used for clock
  // Speed of Timer1 = 16MHz/256 = 62.5 KHz
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = gReloadTimer1; // compare match register 16MHz/256
  TCCR1B |= (1<<WGM12);   // CTC mode
  TCCR1B |= (1<<CS12);    // 256 prescaler 
  TIMSK1 |= (1<<OCIE1A);  // enable timer compare interrupt
  interrupts();
}
void activeBuzzer(){
  unsigned char i;
  unsigned char sleepTime = 1; // ms
  
  for(i=0;i<100;i++){
    digitalWrite(BUZZER,HIGH);
    delay(sleepTime);//wait for 1ms
    digitalWrite(BUZZER,LOW);
    delay(sleepTime);//wait for 1ms
  }
}
void loop() {
  char messageBuff[BUFF_SIZE];
  int auxCount = 0;
  unsigned char auxDigit = '0';


  if(gISRFlag1 == 1 || gISRFlag2 == 1){
    // Reset ISR Flag
    gISRFlag1 = 0;
    gISRFlag2 = 0;

    gIncomingChar = Serial.read(); //read incoming data

    if(gPackageFlag == 1){
      gCommMsg[buff] = gIncomingChar;  //if new message add to buffer and increment size
      buff++; 

      if(buff == BUFF_SIZE){
        gPackageFlag = 0; //if at max size reset start flag and raise processor flag
        gProcessFlag = 1;
      }
    }
    if(gIncomingChar == '$'){
      gPackageFlag = 1; //if start of msg -> raise flag

      int i;
      for(int i = 0; i<BUFF_SIZE; i++){
        gCommMsg[i] == 0; //set message to 0
      }
      buff = 0; //set msg index to 0
    }

    if((gIncomingChar == '\n') && (gPackageFlag == 1)){
      gPackageFlag = 0; //signal end of msg
      gProcessFlag == 1;
    }
    if(gTimerRunning == 0){
      // Start Timer
      gTimerRunning = 1;

      if(gCount == 0){
        gCount = DEFAULT_COUNT;
      }      
      if(gBuzzerFlag == 1){
        gBuzzerFlag = 0;

        // LEDs -> Timer Stopped
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
      }
      else
      {
        startTimer1();
        // LEDs -> Timer Running
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
      }
    }
    else
    {
      // Stop Timer
      stopTimer1();
      gTimerRunning = 0;

      // LEDs -> Timer Running
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, HIGH);
    }
  }
  if(gProcessFlag == 1){
      gProcessFlag = 0; //reset flag

      if(compareMsg(gCommMsg, "STR", 3) == 1){
        digitalWrite(GREEN_LED, HIGH); //if start signal -> turn LED on
      }
      if(compareMsg(gCommMsg, "STP",3) == 1){
        digitalWrite(RED_LED, LOW); //if stop signal -> turn LED off
      }
      if(compareMsg(gCommMsg, "GET", 3) == 1){
        Serial.print("$00:01\n"); //if get signal -> send clock status
      }
    }


  // Attend gBuzzerFlag
  if(gBuzzerFlag == 1){
    // Make Noise...
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    activeBuzzer();
  }
}