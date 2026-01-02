#define UART_IMU Serial2   

void setup() {
    // Debug serial (USB)
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("================================");
    Serial.println(" ESP32-CAM UART TEST START ");
    Serial.println("================================");

    // UART2 for STM32 communication
    // RX = GPIO3, TX = not used (-1)
    UART_IMU.begin(115200, SERIAL_8N1, 3, -1);

    Serial.println("UART1 initialized (RX=GPIO3)");
    Serial.println("Waiting for data from STM32...");
}

void loop() {
    // Check if data available from STM32
    while (UART_IMU.available()) {
        String line = UART_IMU.readStringUntil('\n');
        line.replace("\r", "");   // clean CR

        Serial.print("RX from STM32: ");
        Serial.println(line);
    }
}
