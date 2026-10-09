/*
 * Blue Pill alive test. No DWM3000, no radio.
 * PC13 blinks continuously. USART1 PA9 prints ALIVE at 115200.
 * BOOT0 jumper must sit at 0.
 */

static Uart BoardUart(USART1);

void setup() {
  pinMode(PC13, OUTPUT);
  BoardUart.setTx(PA9);
  BoardUart.setRx(PA10);
  BoardUart.begin(115200);
  BoardUart.println("ALIVE");
}

void loop() {
  digitalWrite(PC13, LOW);
  delay(200);
  digitalWrite(PC13, HIGH);
  delay(200);
  BoardUart.println("ALIVE");
}
