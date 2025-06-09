#define ENCODER_RIGHT_A 36
#define ENCODER_RIGHT_B 39
#define ENCODER_LEFT_A  34
#define ENCODER_LEFT_B  35

volatile long stepsRight = 0;
volatile long stepsLeft = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Прерывание для правого энкодера
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_A) == HIGH) {
    int b = digitalRead(ENCODER_RIGHT_B);
    portENTER_CRITICAL_ISR(&mux);
    if (b == LOW) stepsRight++;
    else stepsRight--;
    portEXIT_CRITICAL_ISR(&mux);
  }
}

// Прерывание для левого энкодера
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_A) == HIGH) {
    int b = digitalRead(ENCODER_LEFT_B);
    portENTER_CRITICAL_ISR(&mux);
    if (b == LOW) stepsLeft++;
    else stepsLeft--;
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);

  // Прерывания только по фронту (RISING) канала A каждого энкодера
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
}

void loop() {
  static long lastRight = 0, lastLeft = 0;
  long right, left;

  portENTER_CRITICAL(&mux);
  right = stepsRight;
  left = stepsLeft;
  portEXIT_CRITICAL(&mux);

  if (right != lastRight || left != lastLeft) {
    Serial.print("Right steps: ");
    Serial.print(right);
    Serial.print(" | Left steps: ");
    Serial.println(left);
    lastRight = right;
    lastLeft = left;
  }
  delay(20);
}