#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

int counter = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial bt_serial(8, 9); // for bluetooth

int motorSpeed = 200;
int turnSpeed = 180;
int direction = 0;

String receivedData = "";

#define TRIGGER_PIN_R 12
#define ECHO_PIN_R 13
#define TRIGGER_PIN_L 12
#define ECHO_PIN_L 10

#define LINE_LOST_THRESHOLD 1000

#define ENA 6 // S6
#define IN1 7 // S7
#define IN2 3 // S3

#define ENB 5 // S5
#define IN3 4 // S4
#define IN4 2 // S2

#define LT_LEFT A2
#define LT_FORWARD A0
#define LT_RIGHT A1

#define Light_Sensor A3
#define LED 11

#define Forward 1
#define Backward 2
#define TurnRight 3
#define TurnLeft 4
#define Stop 5

void setup()
{

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(TRIGGER_PIN_R, OUTPUT);
    pinMode(ECHO_PIN_R, INPUT);
    pinMode(TRIGGER_PIN_L, OUTPUT);
    pinMode(ECHO_PIN_L, INPUT);

    pinMode(LED, OUTPUT);
    pinMode(Light_Sensor, INPUT);

    // lcd setup
    lcd.init();
    lcd.backlight();

    Serial.begin(9600);
    bt_serial.begin(9600);
}

long getObstacleDistance(String s)
{
    long duration, cm;
    if (s == "r")
    {
        digitalWrite(TRIGGER_PIN_R, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN_R, LOW);
        duration = pulseIn(ECHO_PIN_R, HIGH);
    }
    else if (s == "l")
    {
        digitalWrite(TRIGGER_PIN_L, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN_L, LOW);
        duration = pulseIn(ECHO_PIN_L, HIGH);
    }
    else
    {
        return 0;
    }
    return microseconds_to_cm(duration);
}

///////////// Line Tracer Functions //////////////////
bool lt_isLeft()
{
    int ret = analogRead(LT_LEFT);
    // Serial.println("Left");
    // Serial.println(ret);
    return ret > 200 ? true : false;
}

bool lt_isRight()
{
    int ret = analogRead(LT_RIGHT);
    // Serial.println("Right");
    // Serial.println(ret);
    return ret > 200 ? true : false;
}

bool lt_isForward()
{
    int ret = analogRead(LT_FORWARD);
    // Serial.println("Forward");
    // Serial.println(ret);
    return ret > 200 ? true : false;
}
///////////////////////////////////////////////
unsigned long lineLostStartTime = 0; // 선을 놓친 시점을 기록
bool isLineLost = false;

void lt_mode_update()
{
    bool ll = lt_isLeft();
    bool ff = lt_isForward();
    bool rr = lt_isRight();

    if (ll || ff || rr)
    {
        isLineLost = false;
        if (ll || (ll && ff) || (ll && ff && rr))
        {
            direction = TurnLeft; // Left sensor detected - turn left immediately
            Serial.println("Left detected");
        }
        else if (ff && !rr)
        {
            direction = Forward; // Only forward sensor detected - go straight
            Serial.println("Forward detected");
        }
        else if (rr && !ff)
        {
            direction = TurnRight; // Only right sensor detected - turn right to get back on line
            Serial.println("Right detected");
        }
        // else if (ff && rr)
        // {
        //     direction = TurnLeft; // Both forward and right detected - turn left (left first drive)
        // }
        else
        {
            direction = Forward; // Default case
        }
    }
    else
    {
        // No line detected at all
        if (!isLineLost)
        {
            lineLostStartTime = millis();
            isLineLost = true;
            direction = Forward; // Keep moving forward briefly when line is first lost
        }
        else if (isLineLost && millis() - lineLostStartTime > LINE_LOST_THRESHOLD)
        {
            Uturn(); // After 1 second, perform U-turn to find line
            isLineLost = false;
        }
        else
        {
            direction = Forward; // Continue forward while waiting for timeout
        }
    }
}

long microseconds_to_cm(long microseconds)
{
    return microseconds / 29 / 2;
}

void change(char myDirection = direction)
{
    direction = myDirection;
    if (myDirection == Forward)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, motorSpeed);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, motorSpeed);
    }
    else if (myDirection == TurnRight)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, motorSpeed);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, turnSpeed);
    }
    else if (myDirection == TurnLeft)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, motorSpeed);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, turnSpeed);
    }
    else if (myDirection == Backward)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, motorSpeed);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, motorSpeed);
    }
    else if (myDirection == Stop)
    {
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
    }
}

void Uturn()
{
    // Left First Drive U-turn: Always turn left to find the line
    change(TurnLeft);
    // delay(50); // Give initial turn time

    // Keep turning left until any sensor detects the line
    while (!lt_isForward())
    {
        if (lt_isRight())
        {
            direction = TurnRight;
            break;
        }

        Serial.println("Making U-turn");
        change(TurnLeft);
        delay(100);
        change(Stop);
        delay(50);
        // If right has detected during the Uturn, get back in line
    }

    // Once line is found, continue with normal line following
    // The lt_mode_update() will handle the rest
}

void printToLCD(String toPrint)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(toPrint);
}

void process_BT_commands()
{
    // char c = bt_serial.read(); // Read the incoming character

    // // Check if the character is the end of the message
    // if (c == '\n') {
    //   // Message is complete!
    //   Serial.print("Received Message: ");
    //   Serial.println(receivedData);

    // }  else{
    //   receivedData += c;
    // }

    // bt_serial.print("Tick #");
    bt_serial.print("Left: ");
    bt_serial.print(String(getObstacleDistance("l")));
    bt_serial.print("   Right: ");
    bt_serial.print(String(getObstacleDistance("r"))); // print counter and increment 1
    bt_serial.print("\n");
}

void loop()
{
    process_BT_commands();

    digitalWrite(LED, HIGH);
    printToLCD("Ready to run!");

    // Update line following logic and move smoothly
    change(Stop);
    delay(120);

    lt_mode_update();
    change();

    // Short delay for responsive control without jerky movement
    delay(100); // Much shorter delay for smooth operation
}