#pragma once
#include <Arduino.h>

#define UART_SYNC0 0xAA
#define UART_SYNC1 0x55

void uartInit() {
    Serial2.begin(115200, SERIAL_8N1, 16,17); // Nastavení UART2
    Serial.println("UART initialized");
}

template<typename T>
bool uartReceiveStruct(T& outStruct, HardwareSerial& uart = Serial2) {
    enum State { WAIT_SYNC0, WAIT_SYNC1, READ_PAYLOAD };
    static State state = WAIT_SYNC0;
    static uint8_t buf[sizeof(T)];
    static size_t idx = 0;

    while (uart.available()) {
        uint8_t c = uart.read();
        Serial.print("0x"); Serial.print(c, HEX); Serial.print(" ");
        switch (state) {
            case WAIT_SYNC0:
                if (c == UART_SYNC0) state = WAIT_SYNC1;
                break;
            case WAIT_SYNC1:
                if (c == UART_SYNC1) {
                    state = READ_PAYLOAD;
                    idx = 0;
                } else {
                    state = (c == UART_SYNC0) ? WAIT_SYNC1 : WAIT_SYNC0;
                }
                break;
            case READ_PAYLOAD:
                buf[idx++] = c;
                if (idx >= sizeof(T)) {
                    memcpy(&outStruct, buf, sizeof(T));
                    state = WAIT_SYNC0;
                    Serial.println("\nStruct received.");
                    return true;
                }
                break;
        }
    }
    return false;
}