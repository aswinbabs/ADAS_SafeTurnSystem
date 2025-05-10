#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>

// MCP2515 pins
#define CAN_CS_PIN  10
#define CAN_INT_PIN 4  

// SoftwareSerial object for Nextion display
SoftwareSerial nextion(2, 3); // RX, TX

MCP_CAN CAN(CAN_CS_PIN);  // Set CS pin

// xpected CAN IDs
#define CAN_ID_LEFT_INDICATOR      0x100
#define CAN_ID_RIGHT_INDICATOR     0x101
#define CAN_ID_FOGLIGHT            0x102
#define CAN_ID_HEADLIGHT           0x103
#define CAN_ID_LEFT_IND_WARNING    0x104
#define CAN_ID_RIGHT_IND_WARNING   0x105
#define CAN_ID_STEERING_ANGLE      0x106

// Array of expected IDs for quick lookup
const unsigned long expected_ids[] = {
  CAN_ID_LEFT_INDICATOR,
  CAN_ID_RIGHT_INDICATOR,
  CAN_ID_FOGLIGHT,
  CAN_ID_HEADLIGHT,
  CAN_ID_LEFT_IND_WARNING,
  CAN_ID_RIGHT_IND_WARNING,
  CAN_ID_STEERING_ANGLE
};

void setup() {
  Serial.begin(9600);  // For debugging
  nextion.begin(115200); // Initialize Nextion communication
  
  Serial.println("Initializing CAN Bus...");

  // Init MCP2515
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN BUS initialization failed, retrying...");
    delay(500);
  }
  Serial.println("CAN BUS initialized successfully!");
  
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);
  
  // Wait for Nextion to initialize
  delay(1000);
  
  // Reset all Nextion variables to default state
  sendToNextion("va0.val", 0);  // Headlight OFF  (1-ON, 0-OFF)
  sendToNextion("va1.val", 0);  // Foglight OFF   (1-ON, 0-OFF)
  sendToNextion("va2.val", 0);  // Indicator OFF  (1-LEFT, 2-RIGHT, 0-OFF)
  sendToNextion("va3.val", 0);  // Indicator warnings OFF  (1-LEFT WARNING, 2-RIGHT WARNING, 0-OFF)
  sendToNextion("va5.val", 100); // Steering angle to center position
}

void loop() {
  if (!digitalRead(CAN_INT_PIN)) {
    unsigned long id;
    byte len;
    byte buf[8];

    CAN.readMsgBuf(&id, &len, buf);
    
    // Debug output
    Serial.print("ID: 0x");
    Serial.print(id, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Process specific CAN IDs
    switch (id) {
      case CAN_ID_LEFT_INDICATOR:
        // Map to Nextion variable va2
        // If buf[0] is 1, left indicator is ON, otherwise OFF
        sendToNextion("va2.val", buf[0] > 0 ? 1 : 0);
        break;
        
      case CAN_ID_RIGHT_INDICATOR:
        // Map to Nextion variable va2
        // If buf[0] is 1, right indicator is ON, otherwise OFF
        sendToNextion("va2.val", buf[0] > 0 ? 2 : 0);
        break;
        
      case CAN_ID_HEADLIGHT:
        // Map to Nextion variable va0
        // If buf[0] is 1, headlight is ON, otherwise OFF
        sendToNextion("va0.val", buf[0] > 0 ? 1 : 0);
        break;
        
      case CAN_ID_FOGLIGHT:
        // Map to Nextion variable va1
        // If buf[0] is 1, foglight is ON, otherwise OFF
        sendToNextion("va1.val", buf[0] > 0 ? 1 : 0);
        break;
        
      case CAN_ID_LEFT_IND_WARNING:
        // Map to Nextion variable va3
        // If buf[0] is 1, set warning state to 1 (left warning), otherwise OFF
        sendToNextion("va3.val", buf[0] > 0 ? 1 : 0);
        break;
        
      case CAN_ID_RIGHT_IND_WARNING:
        // Map to Nextion variable va3
        // If buf[0] is 1, set warning state to 2 (right warning), otherwise OFF
        sendToNextion("va3.val", buf[0] > 0 ? 2 : 0);
        break;
        
      case CAN_ID_STEERING_ANGLE:
        // Convert from 0-255 range to -180 to 180 degrees
        int steeringAngle = map(buf[0], 0, 255, -180, 180);
        
        // Convert steering angle to gauge position
        int gaugePosition;
        
        if (steeringAngle <= -180) gaugePosition = 320;
        else if (steeringAngle <= -160) gaugePosition = map(steeringAngle, -180, -160, 320, 335);
        else if (steeringAngle <= -140) gaugePosition = map(steeringAngle, -160, -140, 335, 350);
        else if (steeringAngle <= -120) gaugePosition = map(steeringAngle, -140, -120, 350, 5);
        else if (steeringAngle <= -100) gaugePosition = map(steeringAngle, -120, -100, 5, 20);
        else if (steeringAngle <= -80) gaugePosition = map(steeringAngle, -100, -80, 20, 35);
        else if (steeringAngle <= -60) gaugePosition = map(steeringAngle, -80, -60, 35, 60);
        else if (steeringAngle <= -40) gaugePosition = map(steeringAngle, -60, -40, 60, 75);
        else if (steeringAngle <= -20) gaugePosition = map(steeringAngle, -40, -20, 75, 85);
        else if (steeringAngle <= 0) gaugePosition = map(steeringAngle, -20, 0, 85, 100);
        else if (steeringAngle <= 20) gaugePosition = map(steeringAngle, 0, 20, 100, 110);
        else if (steeringAngle <= 40) gaugePosition = map(steeringAngle, 20, 40, 110, 125);
        else if (steeringAngle <= 60) gaugePosition = map(steeringAngle, 40, 60, 125, 135);
        else if (steeringAngle <= 80) gaugePosition = map(steeringAngle, 60, 80, 135, 150);
        else if (steeringAngle <= 100) gaugePosition = map(steeringAngle, 80, 100, 150, 160);
        else if (steeringAngle <= 120) gaugePosition = map(steeringAngle, 100, 120, 160, 170);
        else if (steeringAngle <= 140) gaugePosition = map(steeringAngle, 120, 140, 170, 185);
        else if (steeringAngle <= 160) gaugePosition = map(steeringAngle, 140, 160, 185, 200);
        else if (steeringAngle <= 180) gaugePosition = map(steeringAngle, 160, 180, 200, 215);
        else gaugePosition = 215;
        
        // Send the gauge position to Nextion
        sendToNextion("va5.val", gaugePosition);
        break;
        
      default:
        // Unknown ID - ignore
        break;
    }
  }
}

// Helper function to send values to Nextion display
void sendToNextion(const char* component, int value) {
  // Debug output
  Serial.print("Sending to Nextion: ");
  Serial.print(component);
  Serial.print("=");
  Serial.println(value);
  
  // Actual Nextion communication
  nextion.print(component);
  nextion.print("=");
  nextion.print(value);
  nextion.write(0xFF);
  nextion.write(0xFF);
  nextion.write(0xFF);
}