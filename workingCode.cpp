#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// --- [핀 설정] ---
#define TRIGGER_PIN_R 12
#define ECHO_PIN_R 13
#define TRIGGER_PIN_L 11
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
#define LED 1

// --- [방향 상수] ---
#define Forward 1
#define Backward 2
#define TurnRight 3
#define TurnLeft 4
#define Stop 5

// --- [설정 값] ---
String runningMode; // lfs, stop, exp, ttt

int motorSpeed = 200;
int turnSpeed = 180;
int direction = 0;

byte buffer[512];
int buffer_index;
String receivedData;

// --- [타이머 및 상태 변수] ---
unsigned long previousMillis = 0;
const long intervalRun = 100;
const long intervalStop = 120;

unsigned long uTurnTimer = 0;
const long uTurnRunTime = 100;
const long uTurnStopTime = 50;

// --- [플래그 변수] ---
bool isMotorRunning = false;
bool isUturning = false;
bool uTurnMotorState = false;

// --- [라인 이탈 변수] ---
unsigned long lineLostStartTime = 0;
bool isLineLost = false;
#define LINE_LOST_THRESHOLD 1000

int lastSeenDirection = TurnLeft;

// --- [Exploration 변수] ---
int pathStage = 0;
bool passingIntersection = false;
unsigned long turnTimer = 0;
bool isTurningExp = false;
bool leftDetectorTriggered = false;
bool rightDetectorTriggered = false;
int positions[9];
int positions_index = 0;
bool obstacleAtTurn = false;
int coneMin = 5;
int coneMax = 40;
unsigned long intersectionTimer = 0;

// --- [탐지 자격 부여(Debouncing) 변수] ---
int leftClearCount = 0;
int rightClearCount = 0;
const int CLEAR_THRESHOLD = 5;

bool leftSensorReady = true;
bool rightSensorReady = true;

// --- [Tic-Tac-Toe 변수] ---
int ttt_board[10];
bool waitingForTTTInput = false;

const int winCombos[8][3] = {
    {1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {1, 4, 7}, {2, 5, 8}, {3, 6, 9}, {1, 5, 9}, {3, 5, 7}};

// --- [객체 초기화] ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial bt_serial(8, 9);

// --- [함수 선언] ---
void startUturn();
void change(char myDirection);
void printToLCD(String line1, String line2);

// --- [구현부] ---

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

    bt_serial.begin(9600);

    buffer_index = 0;
    runningMode = "stop";

    printToLCD("Ready...", "");
}

long microseconds_to_cm(long microseconds)
{
    return microseconds / 29 / 2;
}

long getObstacleDistance(String s)
{
    long duration;
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
bool lt_isLeft() { return analogRead(LT_LEFT) > 150; }
bool lt_isRight() { return analogRead(LT_RIGHT) > 150; }
bool lt_isForward() { return analogRead(LT_FORWARD) > 150; }

// --- [LFS Logic] ---
void lfs_mode_update()
{
    bool ll = lt_isLeft();
    bool ff = lt_isForward();
    bool rr = lt_isRight();

    if (ll || ff || rr)
    {
        isLineLost = false;
        if (ll)
            lastSeenDirection = TurnLeft;
        else if (rr)
            lastSeenDirection = TurnRight;

        if (ll || (ll && ff))
            direction = TurnLeft;
        else if (ff && !rr)
            direction = Forward;
        else if (rr && !ff)
            direction = TurnRight;
        else if (ll && ff && rr)
            direction = lastSeenDirection;
        else
            direction = Forward;
    }
    else
    {
        if (!isLineLost)
        {
            lineLostStartTime = millis();
            isLineLost = true;
            direction = Forward;
        }
        else if (isLineLost && millis() - lineLostStartTime > LINE_LOST_THRESHOLD)
        {
            startUturn();
            isLineLost = false;
        }
        else
        {
            direction = Forward;
        }
    }
}

void addPositionIfUnique(int pos)
{
    for (int i = 0; i < positions_index; i++)
    {
        if (positions[i] == pos)
            return;
    }
    if (positions_index < (sizeof(positions) / sizeof(positions[0])))
    {
        positions[positions_index++] = pos;
    }
}

void exp_mode_update()
{
    if (isTurningExp)
    {
        handle_turn_sequence();
        return;
    }

    long distL = getObstacleDistance("l");
    long distR = getObstacleDistance("r");

    if (distL <= coneMin || distL >= coneMax)
    {
        leftClearCount++;
        if (leftClearCount >= CLEAR_THRESHOLD)
            leftSensorReady = true;
    }
    else
        leftClearCount = 0;

    if (distR <= coneMin || distR >= coneMax)
    {
        rightClearCount++;
        if (rightClearCount >= CLEAR_THRESHOLD)
            rightSensorReady = true;
    }
    else
        rightClearCount = 0;

    if (!rightDetectorTriggered && (distR > coneMin && distR < coneMax) && rightSensorReady)
    {
        bt_serial.print("R Obs: ");
        bt_serial.println(distR);
        rightDetectorTriggered = true;
        rightSensorReady = false;
    }

    if (!leftDetectorTriggered && (distL > coneMin && distL < coneMax) && leftSensorReady)
    {
        bt_serial.print("L Obs: ");
        bt_serial.println(distL);
        leftDetectorTriggered = true;
        leftSensorReady = false;
    }

    bool ll = lt_isLeft();
    bool ff = lt_isForward();
    bool rr = lt_isRight();

    if (ff)
        direction = Forward;
    else if (ll)
        direction = TurnLeft;
    else if (rr)
        direction = TurnRight;
    else
        direction = Forward;

    bool isIntersection = (ll && rr) && ff;

    if (isIntersection)
    {
        if (!passingIntersection)
        {
            bt_serial.print("Intersection Stage: ");
            bt_serial.println(pathStage);
            execute_path_logic();
            passingIntersection = true;
        }
    }
    else
    {
        if (!ll && !rr && ff)
            passingIntersection = false;
    }
}

void execute_path_logic()
{
    switch (pathStage)
    {
    case 0:
        direction = Forward;
        if (leftDetectorTriggered)
        {
            addPositionIfUnique(9);
            leftDetectorTriggered = false;
        }
        if (rightDetectorTriggered)
        {
            addPositionIfUnique(6);
            rightDetectorTriggered = false;
        }
        break;

    case 1:
        change(Forward);
        delay(250);
        initiate_turn_right();
        if (leftDetectorTriggered)
        {
            addPositionIfUnique(8);
            leftDetectorTriggered = false;
        }
        if (rightDetectorTriggered)
        {
            addPositionIfUnique(5);
            rightDetectorTriggered = false;
        }
        break;

    case 2:
        change(Forward);
        delay(250);
        initiate_turn_right();
        if (leftDetectorTriggered)
        {
            addPositionIfUnique(4);
            leftDetectorTriggered = false;
        }
        break;

    case 3:
        direction = Forward;
        if (leftDetectorTriggered)
        {
            addPositionIfUnique(2);
            leftDetectorTriggered = false;
        }
        break;

    default:
        direction = Stop;
        break;
    }
    pathStage++;
}

void initiate_turn_right()
{
    isTurningExp = true;
    turnTimer = millis();
    change(TurnRight);
}

void handle_turn_sequence()
{
    unsigned long currentTurnTime = millis() - turnTimer;
    long distL = getObstacleDistance("l");

    if (currentTurnTime < (intervalRun + intervalStop) * 8)
    {
        change(TurnRight);

        if ((pathStage == 2) && !obstacleAtTurn && (distL > coneMin && distL < coneMax))
        {
            obstacleAtTurn = true;
            leftDetectorTriggered = false;
            addPositionIfUnique(7);
            leftSensorReady = false;
            leftClearCount = 0;
        }

        if ((pathStage == 3) && !obstacleAtTurn && (distL > coneMin && distL < coneMax))
        {
            obstacleAtTurn = true;
            leftDetectorTriggered = false;
            addPositionIfUnique(1);
            leftSensorReady = false;
            leftClearCount = 0;
        }
    }
    else
    {
        obstacleAtTurn = false;
        if (lt_isForward())
        {
            isTurningExp = false;
            change(Forward);
            passingIntersection = true;
            intersectionTimer = millis();
        }
        else
        {
            change(TurnRight);
        }
    }
}

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

void startUturn()
{
    isUturning = true;
    uTurnMotorState = true;
    uTurnTimer = millis();
    change(lastSeenDirection);
}

void processUturn()
{
    if (lt_isForward() || lt_isLeft() || lt_isRight())
    {
        isUturning = false;
        if (lt_isLeft())
            lastSeenDirection = TurnLeft;
        if (lt_isRight())
            lastSeenDirection = TurnRight;
        return;
    }

    unsigned long currentMillis = millis();
    if (uTurnMotorState)
    {
        change(lastSeenDirection);
        if (currentMillis - uTurnTimer >= uTurnRunTime)
        {
            uTurnTimer = currentMillis;
            uTurnMotorState = false;
        }
    }
    else
    {
        change(Stop);
        if (currentMillis - uTurnTimer >= uTurnStopTime)
        {
            uTurnTimer = currentMillis;
            uTurnMotorState = true;
        }
    }
}

void printToLCD(String line1, String line2)
{
    static String prevLine1 = "";
    static String prevLine2 = "";

    if (line1 != prevLine1 || line2 != prevLine2)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(line1);
        if (line2 != "")
        {
            lcd.setCursor(0, 1);
            lcd.print(line2);
        }
        prevLine1 = line1;
        prevLine2 = line2;
    }
}

// --- [Tic-Tac-Toe Minimax Algorithm] ---
void clearBoard()
{
    for (int i = 1; i <= 9; i++)
        ttt_board[i] = 0;
}

bool checkWin(int player)
{
    for (int i = 0; i < 8; i++)
    {
        if (ttt_board[winCombos[i][0]] == player &&
            ttt_board[winCombos[i][1]] == player &&
            ttt_board[winCombos[i][2]] == player)
            return true;
    }
    return false;
}

bool isBoardFull()
{
    for (int i = 1; i <= 9; i++)
        if (ttt_board[i] == 0)
            return false;
    return true;
}

// Minimax 재귀 함수
// isMaximizing: True면 Player A(1)의 턴(이득 극대화), False면 Player B(2)의 턴(손실 극대화)
int minimax(int depth, bool isMaximizing)
{
    if (checkWin(1))
        return 10 - depth; // A 승리 (빨리 이길수록 점수 높음)
    if (checkWin(2))
        return depth - 10; // B 승리 (A 패배)
    if (isBoardFull())
        return 0; // 무승부

    if (isMaximizing)
    {
        int bestScore = -1000;
        for (int i = 1; i <= 9; i++)
        {
            if (ttt_board[i] == 0)
            {
                ttt_board[i] = 1; // A가 둬봄
                int score = minimax(depth + 1, false);
                ttt_board[i] = 0; // 원상복구
                if (score > bestScore)
                    bestScore = score;
            }
        }
        return bestScore;
    }
    else
    {
        int bestScore = 1000;
        for (int i = 1; i <= 9; i++)
        {
            if (ttt_board[i] == 0)
            {
                ttt_board[i] = 2; // B가 둬봄
                int score = minimax(depth + 1, true);
                ttt_board[i] = 0; // 원상복구
                if (score < bestScore)
                    bestScore = score;
            }
        }
        return bestScore;
    }
}

// 최적의 수 찾기 메인 함수
int findWinningMove()
{
    int bestVal = -1000;
    int bestMove = 0;

    // 빈칸을 모두 순회하며 Minimax 점수 계산
    for (int i = 1; i <= 9; i++)
    {
        if (ttt_board[i] == 0)
        {
            ttt_board[i] = 1;                // A가 여기에 둔다면?
            int moveVal = minimax(0, false); // 그 다음은 B 차례
            ttt_board[i] = 0;                // 원상복구

            if (moveVal > bestVal)
            {
                bestMove = i;
                bestVal = moveVal;
            }
        }
    }
    return bestMove; // 최적의 위치 반환
}

void processTTTLogic(String inputStr)
{
    clearBoard();
    inputStr += ",";
    int startIndex = 0;
    int commaIndex = inputStr.indexOf(',');

    while (commaIndex != -1)
    {
        String numStr = inputStr.substring(startIndex, commaIndex);
        numStr.trim();
        int pos = numStr.toInt();
        if (pos >= 1 && pos <= 9)
            ttt_board[pos] = 1; // Player A
        startIndex = commaIndex + 1;
        commaIndex = inputStr.indexOf(',', startIndex);
    }

    for (int i = 0; i < positions_index; i++)
    {
        int detectedPos = positions[i];
        if (ttt_board[detectedPos] != 1)
            ttt_board[detectedPos] = 2; // Player B
    }

    int nextMove = findWinningMove();
    bt_serial.print("Rec Move A: ");
    bt_serial.println(nextMove);
    printToLCD("TTT Win Move:", String(nextMove));
}

void process_BT_commands()
{
    if (bt_serial.available())
    {
        byte data = bt_serial.read();
        buffer[buffer_index++] = data;

        if (data == '\n' || buffer_index >= sizeof(buffer) - 1)
        {
            buffer[buffer_index] = '\0';
            receivedData = String((char *)buffer);
            receivedData.trim();
            buffer_index = 0;

            if (waitingForTTTInput)
            {
                processTTTLogic(receivedData);
                waitingForTTTInput = false;
                runningMode = "stop";
                return;
            }

            receivedData.toLowerCase();
            String command = receivedData;
            int sep = receivedData.indexOf(' ');
            if (sep != -1)
                command = receivedData.substring(0, sep);

            if (command == "stop")
            {
                if (runningMode == "exp" && pathStage >= 4)
                {
                    if (leftDetectorTriggered)
                    {
                        addPositionIfUnique(3);
                        leftDetectorTriggered = false;
                    }

                    for (int i = 0; i < positions_index - 1; i++)
                    {
                        for (int j = 0; j < positions_index - i - 1; j++)
                        {
                            if (positions[j] > positions[j + 1])
                            {
                                int temp = positions[j];
                                positions[j] = positions[j + 1];
                                positions[j + 1] = temp;
                            }
                        }
                    }

                    String posStr = "";
                    for (int i = 0; i < positions_index; i++)
                    {
                        posStr += String(positions[i]);
                        if (i < positions_index - 1)
                            posStr += ",";
                    }
                    bt_serial.print("Positions: ");
                    bt_serial.println(posStr);
                    printToLCD("Found Obs:", posStr);
                }
                runningMode = "stop";
            }
            else if (command == "lfs")
                runningMode = "lfs";
            else if (command == "exp" || command == "start")
            {
                runningMode = "exp";
                bt_serial.println("Exploration Mode Started");
            }
            else if (command == "ttt")
            {
                runningMode = "ttt";
                waitingForTTTInput = true;
                bt_serial.println("Enter Player A nums:");
                printToLCD("Mode: TTT Input", "Waiting...");
            }
        }
    }
}

void loop()
{
    process_BT_commands();
    digitalWrite(LED, LOW);

    unsigned long currentMillis = millis();

    if (runningMode != "stop" && runningMode != "ttt")
    {
        printToLCD("Running Mode:", runningMode);
    }

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
        if (isUturning)
            processUturn();
        else
        {
            if (isMotorRunning)
            {
                lfs_mode_update();
                change();
                if (currentMillis - previousMillis >= intervalRun)
                {
                    previousMillis = currentMillis;
                    isMotorRunning = false;
                }
            }
            else
            {
                change(Stop);
                if (currentMillis - previousMillis >= intervalStop)
                {
                    previousMillis = currentMillis;
                    isMotorRunning = true;
                }
            }
        }
    }

    if (runningMode == "exp")
    {
        if (isMotorRunning)
        {
            exp_mode_update();
            change();
            if (currentMillis - previousMillis >= intervalRun)
            {
                previousMillis = currentMillis;
                isMotorRunning = false;
            }
        }
        else
        {
            change(Stop);
            if (currentMillis - previousMillis >= intervalStop)
            {
                previousMillis = currentMillis;
                isMotorRunning = true;
            }
        }
        return;
    }
}