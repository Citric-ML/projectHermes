#define MAX_COMMANDS 32
#define LINE_BUFFER_SIZE 64

struct Command {
  float angle;
  float speed;
  float duration;
};

Command commandList[MAX_COMMANDS];
int commandCount = 0;

bool receivingPacket = false;

char lineBuffer[LINE_BUFFER_SIZE];
int lineIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Safe on Nano

  Serial.println("Arduino ready");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    // Line ending detected
    if (c == '\n') {
      lineBuffer[lineIndex] = '\0';
      processLine(lineBuffer);
      lineIndex = 0;
    }
    // Prevent buffer overflow
    else if (lineIndex < LINE_BUFFER_SIZE - 1) {
      lineBuffer[lineIndex++] = c;
    }
  }
}

void processLine(char* line) {
  if (strcmp(line, "CMD_START") == 0) {
    receivingPacket = true;
    commandCount = 0;
    Serial.println("Packet start");
    return;
  }

  if (strcmp(line, "CMD_END") == 0) {
    receivingPacket = false;
    Serial.print("Packet complete. Commands received: ");
    Serial.println(commandCount);
    return;
  }

  if (!receivingPacket) {
    return; // Ignore stray data
  }

  if (commandCount >= MAX_COMMANDS) {
    Serial.println("Command buffer full!");
    return;
  }

  // Parse CSV: angle,speed,duration
  float angle, speed, duration;
  if (sscanf(line, "%f,%f,%f", &angle, &speed, &duration) == 3) {
    commandList[commandCount++] = {angle, speed, duration};
    Serial.print("Stored command ");
    Serial.println(commandCount);
  } else {
    Serial.print("Parse error: ");
    Serial.println(line);
  }
}
