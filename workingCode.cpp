#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// --- [핀 설정] ---
#define TRIGGER_PIN_R 12
#define ECHO_PIN_R 13
#define TRIGGER_PIN_L 12 // (주의: 사용자가 제공한 핀 번호 유지, 충돌 확인 필요)
#define ECHO_PIN_L 10

#define ENA 6
#define IN1 7
#define IN2 3

#define ENB 5
#define IN3 4
#define IN4 2

#define LT_LEFT A2
#define LT_FORWARD A0
#define LT_RIGHT A1

#define Light_Sensor A3
#define LED 11

// --- [방향 상수] ---
#define Forward 1
#define Backward 2
#define TurnRight 3
#define TurnLeft 4
#define Stop 5

// --- [설정 값] ---
String runningMode = "lfs"; // lfs (Line Following System), stop, exp, ttt

int motorSpeed = 200;
int turnSpeed = 180;
int direction = 0;

byte buffer[512];
int buffer_index;
String receivedData;

// --- [타이머 및 상태 변수 (Non-blocking)] ---
unsigned long previousMillis = 0; // 일반 주행 펄스 타이머
const long intervalRun = 100;     // 주행 시간 (ON)
const long intervalStop = 120;    // 정지 시간 (OFF)

unsigned long uTurnTimer = 0;  // U-turn 펄스 타이머
const long uTurnRunTime = 100; // U-turn 회전 시간
const long uTurnStopTime = 50; // U-turn 정지 시간

// --- [플래그 변수] ---
bool isMotorRunning = false;  // 현재 모터가 도는 중인지 (일반 주행용)
bool isUturning = false;      // 현재 U-turn 모드인지
bool uTurnMotorState = false; // U-turn 중 모터 ON/OFF 상태

// --- [라인 이탈 및 스마트 U-turn 변수] ---
unsigned long lineLostStartTime = 0;
bool isLineLost = false;
#define LINE_LOST_THRESHOLD 1000 // 1초 이상 선 없으면 U-turn

// ★ [핵심] 마지막으로 감지된 방향 기억 (기본값: Left)
int lastSeenDirection = TurnLeft;

// --- [객체 초기화] ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial bt_serial(8, 9);

// --- [함수 선언] ---
void startUturn();
void change(char myDirection);

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

    lcd.init();
    lcd.backlight();

    Serial.begin(9600);
    bt_serial.begin(9600);

    buffer_index = 0;
}

long microseconds_to_cm(long microseconds)
{
    return microseconds / 29 / 2;
}

long getObstacleDistance(String s)
{
    long duration;
    // PulseIn에 타임아웃(5000us = 5ms)을 주어 블로킹 방지
    if (s == "r")
    {
        digitalWrite(TRIGGER_PIN_R, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN_R, LOW);
        duration = pulseIn(ECHO_PIN_R, HIGH, 5000);
    }
    else if (s == "l")
    {
        digitalWrite(TRIGGER_PIN_L, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN_L, LOW);
        duration = pulseIn(ECHO_PIN_L, HIGH, 5000);
    }
    else
    {
        return 0;
    }
    return microseconds_to_cm(duration);
}

// --- [센서 함수] ---
bool lt_isLeft() { return analogRead(LT_LEFT) > 200; }
bool lt_isRight() { return analogRead(LT_RIGHT) > 200; }
bool lt_isForward() { return analogRead(LT_FORWARD) > 200; }

// --- [핵심 로직: 센서 판단 및 방향 기억] ---
void lt_mode_update()
{
    bool ll = lt_isLeft();
    bool ff = lt_isForward();
    bool rr = lt_isRight();

    if (ll || ff || rr)
    {
        isLineLost = false;

        // ★ [Memory Logic] 마지막으로 감지한 코너 방향 업데이트
        // ff만 보일 때는 업데이트 하지 않아 직전 경향성 유지
        if (ll)
            lastSeenDirection = TurnLeft;
        else if (rr)
            lastSeenDirection = TurnRight;

        // 방향 결정 로직
        if (ll || (ll && ff) || (ll && ff && rr))
        {
            direction = TurnLeft;
        }
        else if (ff && !rr)
        {
            direction = Forward;
        }
        else if (rr && !ff)
        {
            direction = TurnRight;
        }
        else
        {
            direction = Forward;
        }
    }
    else
    {
        // 라인 감지 안됨 (All Low)
        if (!isLineLost)
        {
            // 처음 놓친 순간
            lineLostStartTime = millis();
            isLineLost = true;
            direction = Forward; // 관성 주행 (잠시 직진)
        }
        else if (isLineLost && millis() - lineLostStartTime > LINE_LOST_THRESHOLD)
        {
            // 일정 시간(1초) 지남 -> U-turn 시작
            startUturn();
            isLineLost = false;
        }
        else
        {
            // 아직 시간 안됨 -> 관성 주행 유지
            direction = Forward;
        }
    }
}

// --- [모터 제어 함수] ---
void change(char myDirection = direction)
{
    if (myDirection != 0)
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
    else if (myDirection == Stop)
    {
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
    }
}

// --- [U-turn 시작 설정] ---
void startUturn()
{
    isUturning = true;
    uTurnMotorState = true; // 바로 회전 시작
    uTurnTimer = millis();

    // ★ [스마트 U-turn] 마지막으로 본 방향으로 회전 시작
    // 오른쪽 보다가 놓쳤으면 오른쪽으로 돌고, 왼쪽이면 왼쪽으로 돔
    change(lastSeenDirection);

    // 디버깅용 (필요시 주석 해제)
    // if(lastSeenDirection == TurnRight) Serial.println("U-Turn Right!");
    // else Serial.println("U-Turn Left!");
}

// --- [U-turn 처리 프로세스 (Non-blocking)] ---
void processUturn()
{
    // 1. 종료 조건: 라인을 다시 찾았는가?
    if (lt_isForward() || lt_isLeft() || lt_isRight())
    {
        isUturning = false;

        // 찾은 즉시 lastSeenDirection 최신화
        if (lt_isLeft())
            lastSeenDirection = TurnLeft;
        if (lt_isRight())
            lastSeenDirection = TurnRight;

        return;
    }

    unsigned long currentMillis = millis();

    // 2. 펄스 구동 (회전 <-> 정지 반복)
    if (uTurnMotorState)
    {
        // [회전 중]
        change(lastSeenDirection); // 기억해둔 방향으로 계속 회전

        if (currentMillis - uTurnTimer >= uTurnRunTime)
        {
            uTurnTimer = currentMillis;
            uTurnMotorState = false; // -> 정지로 전환
        }
    }
    else
    {
        // [정지 중]
        change(Stop);

        if (currentMillis - uTurnTimer >= uTurnStopTime)
        {
            uTurnTimer = currentMillis;
            uTurnMotorState = true; // -> 회전으로 전환
        }
    }
}

void printToLCD(String toPrint)
{
    static unsigned long lcdTimer = 0;
    if (millis() - lcdTimer > 500)
    { // 0.5초마다 갱신
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(toPrint);
        lcdTimer = millis();
    }
}

void process_BT_commands()
{
    if (bt_serial.available())
    {
        byte data = bt_serial.read();
        buffer[buffer_index++] = data;

        if (data == '\n' || buffer_index >= sizeof(buffer) - 1)
        {
            buffer[buffer_index] = '\0'; // Null-terminate the string
            receivedData = String((char *)buffer);

            // Process the received data: trim and take only the first word
            receivedData.trim(); // remove leading/trailing whitespace/newlines
            receivedData.toLowerCase();
            int sep = receivedData.indexOf(' ');
            if (sep != -1)
            {
                receivedData = receivedData.substring(0, sep);
            }

            Serial.print("Received via BT: ");
            Serial.println(receivedData);

            // Update runningMode based on received command
            // Set runningMode accordingly coz only these 4 modes are valid
            if (receivedData == "stop")
            {
                runningMode = "stop";
            }
            else if (receivedData == "lfs")
            {
                runningMode = "lfs";
            }
            else if (receivedData == "exp" || receivedData == "start")
            {
                runningMode = "exp";
            }
            else if (receivedData == "ttt")
            {
                runningMode = "ttt";
            }

            // Reset buffer index for next message
            buffer_index = 0;
        }
    }
}

// --- [메인 루프] ---
void loop()
{
    process_BT_commands();
    printToLCD("Running...");

    digitalWrite(LED, LOW);

    unsigned long currentMillis = millis();

    if (runningMode == "stop")
    {
        change(Stop);
        digitalWrite(LED, HIGH);
        return;
    }

    if (runningMode == "lfs")
    {
        if (analogRead(Light_Sensor) < 920)
        {
            change(Stop);
            digitalWrite(LED, HIGH);
            return;
        }

        // 1. U-turn 상황 (최우선 처리)
        if (isUturning)
        {
            processUturn();
        }

        // 2. 일반 주행 상황 (Pulse 구동)
        else
        {
            if (isMotorRunning)
            {
                // [RUN 상태: 100ms]
                lt_mode_update(); // 센서 확인
                change();         // 설정된 direction으로 이동

                if (currentMillis - previousMillis >= intervalRun)
                {
                    previousMillis = currentMillis;
                    isMotorRunning = false; // STOP 전환
                }
            }
            else
            {
                // [STOP 상태: 120ms]
                change(Stop); // 정지

                if (currentMillis - previousMillis >= intervalStop)
                {
                    previousMillis = currentMillis;
                    isMotorRunning = true; // RUN 전환
                }
            }
        }
    }

    if (runningMode == "exp")
    {
        // 실험 모드 코드 작성 가능
        return;
    }

    if (runningMode == "ttt")
    {
        // TTT 모드 코드 작성 가능
        return;
    }
}