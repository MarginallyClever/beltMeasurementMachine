//-----------------------------------------------------------
// Belt measurement machine
// dan@marginallyclever.com 2018-06-14
//-----------------------------------------------------------
#include <LiquidCrystal.h>

// motor pins
#define DIR_PIN           (16)
#define STEP_PIN          (17)
#define ENABLE_PIN        (48)

// microstepping
#define MICROSTEPS           (16.0)  // change this.  microstepping on this microcontroller
#define DEGREES_PER_STEP     ( 1.8)  // change this.  as advertised by the stepper motor maker

#define NORMAL_MOTOR_STEPS   (360.0/DEGREES_PER_STEP)  // 360/0.9=400.  360/1.8=200.
#define STEPS_PER_TURN       (NORMAL_MOTOR_STEPS * MICROSTEPS)  // default number of steps per turn * microsteps
#define PULLEY_PITCH         (2*20.0) // 2mm per tooth, 20 teeth.
#define MM_PER_STEP          (PULLEY_PITCH/STEPS_PER_TURN)
#define STEPS_PER_MM         (STEPS_PER_TURN/PULLEY_PITCH)

#define DEFAULT_MM_TO_PUSH 1000

// Smart controller settings
#define BEEPER             44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40

// lcd size
#define LCD_HEIGHT         4
#define LCD_WIDTH          20

// Encoder rotation values
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43

#define BLEN_C             2
#define BLEN_B             1
#define BLEN_A             0
#define encrot0            0
#define encrot1            2
#define encrot2            3
#define encrot3            1
//-----------------------------------------------------------


// lcd
LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7);
int lcd_rot_old  = 0;
int lcd_turn     = 0;
char lcd_click_old = HIGH;
char lcd_click_now = false;
boolean odds=0;


// menu
int menu_pos;
boolean menu_active;
char lcd_message[LCD_WIDTH*LCD_HEIGHT];
long lcd_update;

// belt
int millimeters;
long soFar;
boolean goOut;

//-----------------------------------------------------------


void setup() {
  // stepper motor
  pinMode(DIR_PIN,OUTPUT);
  pinMode(STEP_PIN,OUTPUT);
  pinMode(ENABLE_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,LOW);  // enable

  // lcd
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  pinMode(BEEPER,OUTPUT);
  pinMode(BTN_EN1, INPUT);
  pinMode(BTN_EN2, INPUT);
  pinMode(BTN_ENC, INPUT);
  digitalWrite(BTN_EN1, HIGH);
  digitalWrite(BTN_EN2, HIGH);
  digitalWrite(BTN_ENC, HIGH);

  setup_belts();
  setup_menu();

  lcd_draw();
  
  // serial
  Serial.begin(57600);
  Serial.println("\n** READY **");
}

void setup_belts() {
  millimeters=DEFAULT_MM_TO_PUSH;
  soFar=0;
}

void setup_menu() {
  menu_pos=4;
  menu_active=false;
  lcd_update=0;
  goOut=true;
}

void change_mm(int scale) {
  if(lcd_turn == 0) return;
  
  int adjust = millimeters + scale * lcd_turn;
  if(adjust<=50000 && adjust>0) {
    millimeters = adjust;
  }
}

void loop() {
  // detect & respond to user behavior
  LCD_read();
  
  if(!menu_active) {
    // move between menus
    menu_pos = (menu_pos + lcd_turn + 6) % 6;
    // enter position
    if(lcd_click_now) {
      menu_active=true;
      if(menu_pos==4) {
        // in our out
        goOut = !goOut;
        menu_active=false;
      }
    }
  } else {
    // change / activate menu
    switch(menu_pos) {
      case 0:  change_mm(1000);  break;
      case 1:  change_mm( 100);  break;
      case 2:  change_mm(  10);  break;
      case 3:  change_mm(   1);  break;
      case 4:  menu_active=false;  break;
      case 5:  countOutNow();  menu_active=false;  break;
    }
  
    // exit menu
    if(lcd_click_now) {
      menu_active=false;
    }
  }

  // update LCD panel
  if(lcd_click_now || lcd_turn != 0) {
    lcd_draw();
  }

  switch(menu_pos) {
    case 0: lcd.setCursor(1,0);  break;
    case 1: lcd.setCursor(2,0);  break;
    case 2: lcd.setCursor(3,0);  break;
    case 3: lcd.setCursor(4,0);  break;
    case 4: lcd.setCursor(8,0);  break;
    case 5: lcd.setCursor(12,0);  break;
  }
  // blink cursor
  if(millis() % 600 < 300) {
    lcd.cursor();
  } else {
    lcd.noCursor();
  }
  
  lcd_turn=0;
  lcd_click_now=false;

  delay(20);
}


void lcd_draw() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print((millimeters       ) / 10000);
  lcd.print((millimeters %10000) /  1000);
  lcd.print((millimeters % 1000) /   100);
  lcd.print((millimeters %  100) /    10);
  lcd.print((millimeters %   10)        );
  lcd.print("mm ");
  lcd.print(goOut?"OUT ":"IN  ");
  lcd.print("GO");
  
  lcd.setCursor(0,1);
  lcd.print(soFar * MM_PER_STEP);
  lcd.print("mm so far");

  char c = (menu_active?'*':'_');
  Serial.print((millimeters       ) / 10000);
  Serial.print((millimeters %10000) /  1000);  if(menu_pos==0) Serial.print(c);
  Serial.print((millimeters % 1000) /   100);  if(menu_pos==1) Serial.print(c);
  Serial.print((millimeters %  100) /    10);  if(menu_pos==2) Serial.print(c);
  Serial.print((millimeters %   10)        );  if(menu_pos==3) Serial.print(c);
  Serial.print(goOut?"OUT ":"IN  ");           if(menu_pos==4) Serial.print(c);
  Serial.print(                       " GO");  if(menu_pos==5) Serial.print(c);

  Serial.print("\n");
}

void countOutNow() {
  Serial.print("Counting ");
  Serial.print(goOut?"out ":"in ");
  Serial.print(millimeters);
  Serial.println("mm of belt");

  Serial.print(MM_PER_STEP);
  Serial.println(" mm/steps");
  
  long stepsRemaining = (float)millimeters * STEPS_PER_MM;

  digitalWrite(DIR_PIN,goOut?LOW:HIGH);
  int add = goOut?1:-1;
  
  Serial.print(stepsRemaining);
  Serial.println(" steps");

  while(stepsRemaining>0) {
    digitalWrite(STEP_PIN,HIGH);
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(150);
    if((stepsRemaining%(int)STEPS_PER_MM)==0) {
      lcd_draw();
    }
    stepsRemaining--;
    soFar+=add;
  }
  Serial.println("Done.");
}


void LCD_read() {
  // detect pot turns
  int rot = ((digitalRead(BTN_EN1) == LOW) << BLEN_A)
          | ((digitalRead(BTN_EN2) == LOW) << BLEN_B);
  switch (rot) {
    case encrot0:
      if ( lcd_rot_old == encrot3 ) lcd_turn++;
      if ( lcd_rot_old == encrot1 ) lcd_turn--;
      break;
    case encrot1:
      if ( lcd_rot_old == encrot0 ) lcd_turn++;
      if ( lcd_rot_old == encrot2 ) lcd_turn--;
      break;
    case encrot2:
      if ( lcd_rot_old == encrot1 ) lcd_turn++;
      if ( lcd_rot_old == encrot3 ) lcd_turn--;
      break;
    case encrot3:
      if ( lcd_rot_old == encrot2 ) lcd_turn++;
      if ( lcd_rot_old == encrot0 ) lcd_turn--;
      break;
  }
  lcd_rot_old = rot;

  if(odds) {
    lcd_turn=0;
    odds=false;
  } else {
    odds=true;
  }

  // find click state
  int btn = digitalRead(BTN_ENC);
  if ( btn != lcd_click_old && btn == HIGH ) {
    lcd_click_now = true;
  }
  lcd_click_old = btn;
}
