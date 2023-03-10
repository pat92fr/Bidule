//#include <M5StickC.h>
//#include <M5StickCPlus.h>
#include <M5Stack.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/// THIS ESPNOW 78:21:84:9D:85:6C,
/// THIS ESPNOW 24:A1:60:45:BA:18 (M5Stick Plus)

struct message_format {
  int x0;
  int y0;
  int btn0;
  int x1;
  int y1;
  int btn1;
};

message_format msg;


// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      600         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#define LED_BUILTIN 2
#define RXD2 16
#define TXD2 17

//#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(RXD2,TXD2);        // RX, TX
#define GPIO_PIN26 26
#define GPIO_PIN36 36
HardwareSerial  HoverSerial(2);


// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;


// callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&msg, incomingData, sizeof(msg));
  // Serial.print(" x0:");
  // Serial.println(msg.x0);
  // Serial.print(" y0:");
   Serial.println(msg.y0);
  // Serial.print(" btn0:");
  // Serial.println(msg.btn0);
  // Serial.print(" x1:");
  // Serial.println(msg.x1);
  // Serial.print(" y1:");
   Serial.println(msg.y1);
  // Serial.print(" btn1:");
  // Serial.println(msg.btn1);

  // Serial.println();
}

float accX = 0.0F;  // Define variables for storing inertial sensor data
float accY = 0.0F;  //??????????????????????????????????????????????????????
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

void setup() {
  M5.begin(); // Initialize host device.
  M5.Power.begin();
  M5.IMU.Init();

  M5.Lcd.fillScreen(BLACK);  // Set the screen background color to black.
  M5.Lcd.setRotation(1); // Rotation screen. v
  M5.Lcd.setTextSize(2);  // Set the font size.
  M5.Lcd.setTextColor(GREEN, BLACK);  // Sets the foreground color and background color of the
                        // displayed text.  
// greetings
  M5.Lcd.println("Mini Pupper v2 PRO R/C"); 
  

  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    M5.Lcd.print("Error initializing ESP-NOW"); 
    return;
  }
  Serial.print(F("Reciever initialized : "));
  M5.Lcd.println("RX initialized"); 
  Serial.print("Mac: ");
  Serial.println(WiFi.macAddress());    
  M5.Lcd.print("Mac: "); 
  M5.Lcd.println(WiFi.macAddress());  

  M5.Lcd.fillScreen(BLACK);

  // Define receive function
  esp_now_register_recv_cb(OnDataRecv);  

  
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1,GPIO_PIN36,GPIO_PIN26);

}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
bool Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return false;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return false;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
    return true;
}

// ########################## LOOP ##########################

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
float roll_offset = 0;
float alpha = 0.05F;
float err = 0;

void loop(void)
{ 
  unsigned long timeNow = millis();

  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);  // Stores the triaxial accelerometer.
  M5.IMU.getAhrsData(&pitch, &roll,&yaw);  // Stores the inertial sensor attitude.

/*
  M5.Lcd.setCursor(0, 20);  // Move the cursor position to (x,y).  ?????????????????????(x,y)???
  M5.Lcd.printf("gyroX,  gyroY, gyroZ");  // Screen printingformatted string.
  M5.Lcd.setCursor(0, 42);
  M5.Lcd.printf("%6.1f %6.1f%6.1f d/s", gyroX, gyroY, gyroZ);

  // Accelerometer output is related
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("accX,   accY,  accZ");
  M5.Lcd.setCursor(0, 92);
  M5.Lcd.printf("%5.3f  %5.3f  %5.3f G", accX, accY, accZ);

  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf("pitch,  roll,  yaw");
  M5.Lcd.setCursor(0, 142);
  M5.Lcd.printf("%5.2f  %5.2f  %5.2f deg", pitch, roll, yaw);

  M5.Lcd.setCursor(0, 170);
  M5.Lcd.printf("roll_offset %5.2f", roll_offset);
*/
  int m1 = 0;
  int m2 = 0;

  if(millis() < 10000)
  {
    if(roll_offset == 0)
      roll_offset = roll;
    roll_offset = alpha * roll + (1 - alpha) * roll_offset; // Mean
    m1 = 0;
    m2 = 0;
  }
  else
  {
    float oc = (roll - roll_offset) * 30.0f + ((roll - roll_offset) - err) * 0.0000f;
    err = roll - roll_offset;
    m1 = oc;
    m2 = oc;
  }

  // Check for new received data
  if(Receive())
    Send(m1,m2);
  //Send((msg.y0-100)*4, (msg.y1-100)*4);
}

// ########################## END ##########################