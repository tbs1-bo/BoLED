/**
 * BoLED - 7x120 LED-Matrix mit Multiplexing für ESP32
 * V1 from 39c3
 * 
 * Hardware:
 * - 7 Zeilen x 120 Spalten LED-Matrix
 * - Multiplexing über 3 Row-Address-Pins
 * - 8 Column-Data-Pins + 5 Column-Address-Pins
 * - UART2 für serielle Steuerung (TX=GPIO17, RX=GPIO16)
 */

#include <HardwareSerial.h>

// ==================== Konstanten ====================
const int ROWS = 7;
const int COLS = 120;
const int FRAMEBUFFER_SIZE = ROWS * COLS / 8;  // 105 Bytes
const int EXPECTED_IMAGE_BYTES = 120;           // 120*7/7 = 120

// Mapping der Zeilenadressen (Hardware-bedingt)
const uint8_t ROW_ADDRESSES[8] = {1, 5, 4, 6, 7, 2, 3, 0};

// Pin-Definitionen
const int ROW_PINS[3] = {4, 5, 2};
const int COL_DATA_PINS[8] = {21, 22, 23, 25, 26, 27, 32, 33};
const int COL_ADDRESS_PINS[5] = {18, 15, 14, 12, 13};
const int ENABLE_PIN = 19;

// Befehls-Bytes für serielle Kommunikation
const uint8_t CMD_PIXEL_ON = 0x83;   // 131 dezimal
const uint8_t CMD_PIXEL_OFF = 0x82;  // 130 dezimal
const uint8_t CMD_IMAGE = 0x81;      // 129 dezimal

// Parser-Status
enum ParserState {
    STATE_IDLE,
    STATE_WAIT_X,
    STATE_WAIT_Y,
    STATE_WAIT_IMAGE
};

// ==================== Globale Variablen ====================
uint8_t framebuffer[FRAMEBUFFER_SIZE];      // Buffer mit 8 Datenbits pro Byte (Spalten*Zeilen/8 Byte)
int currentRow = 0;
bool running = false;

uint8_t imageBuffer[EXPECTED_IMAGE_BYTES];      // Buffer mit 7 Datenbits pro Byte (Spalten*Zeilen/7 Byte)


// Statistik
int counter = 0;
unsigned long sum = 0;

// ==================== Display-Funktionen ====================

void initPins() {
    // Row-Pins als Output
    for (int i = 0; i < 3; i++) {
        pinMode(ROW_PINS[i], OUTPUT);
        digitalWrite(ROW_PINS[i], LOW);
    }
    
    // Column-Data-Pins als Output
    for (int i = 0; i < 8; i++) {
        pinMode(COL_DATA_PINS[i], OUTPUT);
        digitalWrite(COL_DATA_PINS[i], LOW);
    }
    
    // Column-Address-Pins als Output
    for (int i = 0; i < 5; i++) {
        pinMode(COL_ADDRESS_PINS[i], OUTPUT);
        digitalWrite(COL_ADDRESS_PINS[i], LOW);
    }
    
    // Enable-Pin
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // Invertiert: HIGH = disabled
}

void setRow(int rowNumber) {
    uint8_t addr = ROW_ADDRESSES[rowNumber];
    for (int i = 0; i < 3; i++) {
        digitalWrite(ROW_PINS[i], (addr >> i) & 1);
    }
}

void setCols(int colAddress, uint8_t colData) {
    int address = colAddress;
    if (address > 11) {
        address += 4;
    }
    
    // Setze Adress-Pins
    for (int i = 0; i < 5; i++) {
        digitalWrite(COL_ADDRESS_PINS[i], (address >> i) & 1);
    }
    
    // Setze Daten-Pins
    for (int i = 0; i < 8; i++) {
        digitalWrite(COL_DATA_PINS[7-i], (colData >> i) & 1);
    }
    
    // Enable-Puls
    digitalWrite(ENABLE_PIN, LOW);      // Datenbyte aktivieren
    digitalWrite(ENABLE_PIN, HIGH);     // Datenbyte deaktivieren
}

void refreshRow(int row) {
    setRow(7);  // Alle Zeilen aus während Daten geladen werden
    
    for (int block = 0; block < 16; block++) {
        uint8_t data;
        if (block < 12) {
            data = framebuffer[row * 15 + block];
        } else if (block < 15) {
            data = ((framebuffer[row * 15 + (block - 1)] & 0x01) << 7) | 
                   (framebuffer[row * 15 + block] >> 1);
        } else {
            data = (framebuffer[row * 15 + block - 1] & 0x01) ? 0x80 : 0x00;
        }
        setCols(block, data);
    }
    
    setRow(row);  // Zeile aktivieren
}

void multiplex() {
    refreshRow(currentRow);
    currentRow = (currentRow + 1) % ROWS;
    //setRow(7);  // Zeile deaktivieren nach Refresh
}

void clearDisplay() {
    memset(framebuffer, 0, FRAMEBUFFER_SIZE);
}

void fillDisplay() {
    memset(framebuffer, 0xFF, FRAMEBUFFER_SIZE);
}

void setPixel(int x, int y, bool value) {
    if (x >= 0 && x < COLS && y >= 0 && y < ROWS) {
        int byteIdx = (y * 15) + (x / 8);
        int bitIdx = 7 - (x % 8);
        
        if (value) {
            framebuffer[byteIdx] |= (1 << bitIdx);
        } else {
            framebuffer[byteIdx] &= ~(1 << bitIdx);
        }
    }
}

// ==================== Serielle Kommunikation ====================

void processImage() {
    // Konvertierung: 7-Bit Format -> 8-Bit Format
    // Eingabe: 120 Bytes mit je 7 Datenbits
    // Ausgabe: 105 Bytes mit je 8 Datenbits
    
    // Extrahiere alle Bits aus den 7-Bit Bytes
    uint8_t allBits[840];  // 120 * 7 = 840 Bits
    int bitIndex = 0;
    
    for (int i = 0; i < EXPECTED_IMAGE_BYTES; i++) {
        for (int bitPos = 6; bitPos >= 0; bitPos--) {
            allBits[bitIndex++] = (imageBuffer[i] >> bitPos) & 1;
        }
    }
    
    // Packe die Bits in 8-Bit Bytes für den Framebuffer
    for (int byteIdx = 0; byteIdx < FRAMEBUFFER_SIZE; byteIdx++) {
        uint8_t byteValue = 0;
        for (int bitOffset = 0; bitOffset < 8; bitOffset++) {
            int srcIdx = byteIdx * 8 + bitOffset;
            if (srcIdx < 840) {
                byteValue |= (allBits[srcIdx] << (7 - bitOffset));
            }
        }
        framebuffer[byteIdx] = byteValue;
    }
}

void receivePixel(byte val) {
    byte localBuffer[2];
    Serial2.readBytes(localBuffer, 2);
    int x = localBuffer[0];
    int y = localBuffer[1];
    setPixel(x, y, val);
}

void receivePicture() {
  Serial2.readBytes(imageBuffer, EXPECTED_IMAGE_BYTES);
  processImage();
}


// ==================== Setup und Loop ====================

void setup() {
    // Debug-Serial initialisieren
    Serial.begin(115200);
    Serial.println("BoLED ESP32 gestartet");
    
    // UART2 für externe Kommunikation
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17, 8N1 = 8 Datenbits, keine Parität, 1 Stopbit
    Serial.println("UART2 initialisiert (TX=GPIO17, RX=GPIO16, 115200 Baud)");
    
    // Pins initialisieren
    initPins();
    
    // Framebuffer leeren
    clearDisplay();
    
    Serial.println("Display gestartet (120x7 Pixel)");
    Serial.printf("Framebuffer-Größe: %d Bytes\n", FRAMEBUFFER_SIZE);
    
    clearDisplay();
    
    Serial.println("==================================================");
    Serial.println("Serial-Steuerung über UART2 gestartet");
    Serial.println("Verbinde mit: TX=GPIO17, RX=GPIO16, 115200 Baud");
    Serial.println();
    Serial.println("Pixel-Befehle (3 Bytes pro Befehl):");
    Serial.printf("  Byte 1: 0x83 (%d) -> Pixel EIN\n", CMD_PIXEL_ON);
    Serial.printf("  Byte 1: 0x82 (%d) -> Pixel AUS\n", CMD_PIXEL_OFF);
    Serial.println("  Byte 2: X-Koordinate (0-119)");
    Serial.println("  Byte 3: Y-Koordinate (0-6)");
    Serial.println();
    Serial.println("Bild-Upload (121 Bytes):");
    Serial.printf("  Byte 1: 0x81 (%d) -> Bild-Upload\n", CMD_IMAGE);
    Serial.println("  Byte 2-121: 120 Bytes mit je 7 Datenbits (MSB=0)");
    Serial.println("              Format: 0b0xxxxxxx (840 Pixel = 120x7)");
    Serial.println();
    Serial.println("Beispiel: 0x83 0x0A 0x03 -> Pixel an Position (10, 3) einschalten");
    Serial.println("==================================================");
    Serial.println();
}

void loop() {
    unsigned long starttime = micros();

    if(Serial2.available() > 0) {
        int input = Serial2.read();
        /* if(input == 0b11110000) {
            serialEcho();
        } */
        if(input == CMD_PIXEL_ON) {
            receivePixel(1);
            // Serial.println("1");
        }
        if(input == CMD_PIXEL_OFF) {
            receivePixel(0);
            // Serial.println("0");
        }
        if(input == CMD_IMAGE) {
            receivePicture();
            // Serial.println("p");
        }
    }
    
    // Display-Multiplexing
    multiplex();
    
    // Timing-Statistik (optional)
    unsigned long timediff = micros() - starttime;
    sum += timediff;
    counter++;
    
    if (counter >= 10) {
        // Debug-Ausgabe der durchschnittlichen Loop-Zeit
        // Serial.printf("%.2f ms\n", (float)summ / 10000.0);
        counter = 0;
        sum = 0;
    }
}
