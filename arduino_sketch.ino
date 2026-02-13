#define MAX_COMMANDS 32
#define LINE_BUFFER_SIZE 64

struct Command {
  float angle;
  float speed;
  float duration;
};

Command commandList[MAX_COMMANDS];
int commandCount = 0;
int currentCommandIndex = 0;

unsigned long commandStartTime = 0;

bool receivingPacket = false;

char lineBuffer[LINE_BUFFER_SIZE];
int lineIndex = 0;

// ===== STATE MACHINE =====

enum DroneState {
  STATE_IDLE,
  STATE_READY,
  STATE_EXECUTING,
  STATE_COMPLETE,
  STATE_ERROR
};

DroneState currentState = STATE_IDLE;

// Forward declaration
void setState(DroneState newState);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  setState(STATE_IDLE);
}

void loop() {
  handleSerial();
  updateStateMachine();
}

// =============================
// SERIAL HANDLING
// =============================

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      lineBuffer[lineIndex] = '\0';
      processLine(lineBuffer);
      lineIndex = 0;
    }
    else if (lineIndex < LINE_BUFFER_SIZE - 1) {
      lineBuffer[lineIndex++] = c;
    }
  }
}

void processLine(char* line) {

  if (strcmp(line, "CMD_START") == 0) {
    receivingPacket = true;
    commandCount = 0;
    return;
  }

  if (strcmp(line, "CMD_END") == 0) {
    receivingPacket = false;
    setState(STATE_READY);
    return;
  }

  if (strcmp(line, "EXECUTE") == 0) {
    if (currentState == STATE_READY) {
      currentCommandIndex = 0;
      commandStartTime = millis();
      setState(STATE_EXECUTING);
    }
    return;
  }

  if (!receivingPacket) return;

  if (commandCount >= MAX_COMMANDS) {
    setState(STATE_ERROR);
    return;
  }

  float angle, speed, duration;
  if (sscanf(line, "%f,%f,%f", &angle, &speed, &duration) == 3) {
    commandList[commandCount++] = {angle, speed, duration};
  } else {
    setState(STATE_ERROR);
  }
}

// =============================
// STATE MACHINE LOGIC
// =============================

void updateStateMachine() {

  if (currentState == STATE_EXECUTING) {

    Command currentCmd = commandList[currentCommandIndex];

    unsigned long elapsed = millis() - commandStartTime;

    if (elapsed >= (unsigned long)(currentCmd.duration * 1000)) {
      currentCommandIndex++;

      if (currentCommandIndex >= commandCount) {
        setState(STATE_COMPLETE);
      } else {
        commandStartTime = millis();
      }
    }
  }
}

// =============================
// STATE HANDLING
// =============================

void setState(DroneState newState) {

  if (currentState == newState) return;

  currentState = newState;

  Serial.print("STATE:");

  switch (currentState) {
    case STATE_IDLE:
      Serial.println("IDLE");
      break;
    case STATE_READY:
      Serial.println("READY");
      break;
    case STATE_EXECUTING:
      Serial.println("EXECUTING");
      break;
    case STATE_COMPLETE:
      Serial.println("COMPLETE");
      break;
    case STATE_ERROR:
      Serial.println("ERROR");
      break;
  }
}
