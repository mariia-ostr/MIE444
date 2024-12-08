#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// WiFi credentials
const char* ssid = "Ruiling";
const char* password = "10351035";
WebServer server(80);

// ToF sensor
VL53L0X sensor;
bool sensorConnected = false;

// Vacuum system pins
const int SOLENOID_PIN = 26;
const int RELAY_PIN = 27;
const int SERVO_PIN = 33;
const int PICKUP_DELAY = 500;    // Time to ensure good suction (ms)
const int RELEASE_DELAY = 200;   // Time to ensure item is released (ms)

// Stepper motor pins
#define DIR_PIN_LEFT 16
#define STEP_PIN_LEFT 17
#define DIR_PIN_RIGHT 18
#define STEP_PIN_RIGHT 19
#define ENABLE_PIN_LEFT 23 // left
#define ENABLE_PIN_RIGHT 32 // right
#define MOTOR_INTERFACE_TYPE 1

// I2C Pins for ToF sensor
#define SDA_PIN 21
#define SCL_PIN 22

// Ultrasonic sensor pins
const int TRIG_PIN_1 = 13;  // bottom 
const int ECHO_PIN_1 = 14; // bottom
const int TRIG_PIN_2 = 15;  // top sensor
const int ECHO_PIN_2 = 4;   // top sensor

// Motor configuration
#define MAX_SPEED 1000
#define RUNNING_SPEED 120
#define ACCELERATION 500

float duration1, distance1;  // For bottom ultrasonic sensor
float duration2, distance2;  // For top ultrasonic sensor

// Robot physical parameters
#define STEPS_PER_CM 7.575
#define WHEEL_SEPARATION 13
#define TURN_FACTOR 5.2

// Servo configuration
const int minUs = 500;
const int maxUs = 2400;
int currentPos = 0;    // Default position 10 degrees

// State variables
bool pumpRunning = false;
Servo myservo;
int distance = 0;

// Create stepper instances
AccelStepper stepperLeft(MOTOR_INTERFACE_TYPE, STEP_PIN_LEFT, DIR_PIN_LEFT);
AccelStepper stepperRight(MOTOR_INTERFACE_TYPE, STEP_PIN_RIGHT, DIR_PIN_RIGHT);

const char* MAIN_page = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial; 
            text-align: center; 
            margin: 20px;
            background-color: #f0f2f5;
        }
        .control-panel { 
            max-width: 800px; 
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .section {
            border: 1px solid #ddd;
            padding: 15px;
            margin: 10px;
            border-radius: 8px;
        }
        .button {
            width: 80px;
            height: 80px;
            margin: 5px;
            font-size: 24px;
            border-radius: 10px;
            border: none;
            background-color: #4CAF50;
            color: white;
            cursor: pointer;
            transition: all 0.3s;
        }
        .button:active { 
            background-color: #45a049; 
            transform: scale(0.95);
        }
        .button:disabled {
            background-color: #cccccc;
            cursor: not-allowed;
        }
        .pump-button {
            width: 120px;
            height: 40px;
            margin: 5px;
            font-size: 16px;
            background-color: #2196F3;
        }
        .stop-button {
            background-color: #f44336;
        }
        input[type="number"] {
            width: 100px;
            padding: 8px;
            margin: 10px;
            border: 1px solid #ddd;
            border-radius: 5px;
        }
        .sensor-readings {
            font-size: 24px;
            margin: 10px;
            padding: 15px;
            background-color: #f8f9fa;
            border-radius: 8px;
        }
        .sensor-value {
            color: #2196F3;
            margin: 8px 0;
        }
        #status {
            margin-top: 15px;
            
            padding: 10px;
            border-radius: 5px;
            font-weight: bold;
        }
        .success {
            color: #388e3c;
            background-color: #e8f5e9;
        }
        .warning-banner {
            background-color: #ff4444;
            color: white;
            padding: 10px;
            margin-bottom: 20px;
            border-radius: 5px;
            display: none;
            font-weight: bold;
            animation: blink 1s infinite;
        }
        @keyframes blink {
            50% { opacity: 0.8; }
        }
    </style>
</head>
<body>
    <h1>Robot Control Panel</h1>
    <div class="control-panel">
        <div class="sensor-readings">
    <div id="tofdistance" class="sensor-value">ToF Distance: -- mm</div>
    <div id="topDistance" class="sensor-value">Top Distance: -- cm</div>
    <div id="bottomDistance" class="sensor-value">Bottom Distance: -- cm</div>
    <div id="warning" class="warning-banner">
        WARNING: Object too close! Distance less than 65mm
    </div>
</div>
        
       

      


        <div id="status"></div>
    </div>

    <script>
        let commandInProgress = false;

        function sendCommand(cmd) {
            
        // Update sensor readings every 100ms
        setInterval(() => {
    // Update ToF sensor
    fetch('/distance')
        .then(response => response.text())
        .then(data => {
            document.getElementById('tofdistance').innerHTML = 'ToF Distance: ' + data + ' mm';
        });
    console.log('distance should be updated now')
    // Update bottom ultrasonic sensor
    fetch('/ultrasonic1')
        .then(response => response.text())
        .then(data => {
            document.getElementById('bottomDistance').innerHTML = 'Bottom Distance: ' + data + ' cm';
        });
    
    // Update top ultrasonic sensor
    fetch('/ultrasonic2')
        .then(response => response.text())
        .then(data => {
            document.getElementById('topDistance').innerHTML = 'Top Distance: ' + data + ' cm';
        });
     fetch('/warning')
            .then(response => response.text())
            .then(data => {
                const warningElement = document.getElementById('warning');
                if (data === "1") {
                    warningElement.style.display = 'block';
                } else {
                    warningElement.style.display = 'none';
                }
            });
}, 100);

        // Update servo position every second
        setInterval(updateServoPosition, 1000);
    </script>
</body>
</html>
)html";

// Power management functions
void enableMotors() {
    digitalWrite(ENABLE_PIN_RIGHT, LOW);
    digitalWrite(ENABLE_PIN_LEFT, LOW);
}

void disableMotors() {
    digitalWrite(ENABLE_PIN_RIGHT, HIGH);
    digitalWrite(ENABLE_PIN_LEFT, HIGH);
}

void holdAndDisable() {
    delay(10);
    disableMotors();
}

void scanI2C() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Scanning I2C...");
    for(byte i = 8; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Device found at: 0x%02X\n", i);
        }
    }
}

bool setupTOF() {
    Wire.begin(SDA_PIN, SCL_PIN);

    delay(50);

    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0) {
        Serial.println("ToF sensor not found!");
        return false;
    }

    if (!sensor.init()) {
        Serial.println("Failed to initialize ToF sensor!");
        return false;
    }

    sensor.setTimeout(500);
    sensor.startContinuous();
    Serial.println("ToF sensor setupTOF has finished");
    return true;
}

void setupUltrasonicSensors() {
    pinMode(TRIG_PIN_1, OUTPUT);
    pinMode(ECHO_PIN_1, INPUT);
    pinMode(TRIG_PIN_2, OUTPUT);
    pinMode(ECHO_PIN_2, INPUT);
}


void readUltrasonicSensors() {
    // Read first sensor
    digitalWrite(TRIG_PIN_1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_1, LOW);
    duration1 = pulseIn(ECHO_PIN_1, HIGH);
    //distance1 = (duration1 * 0.034 / 2);  // Convert to cm
    distance1 = (-0.0036 * (duration1 * 0.034 / 2) * (duration1 * 0.034 / 2)) + (1.1220 * (duration1 * 0.034 / 2)) - 0.3135; 

    // Read second sensor
    digitalWrite(TRIG_PIN_2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_2, LOW);
    duration2 = pulseIn(ECHO_PIN_2, HIGH);
    //distance2 = (duration2 * 0.034 / 2);  // Convert to cm
    distance2 = (-0.0002 * (duration2 * 0.034 / 2) * (duration2 * 0.034 / 2)) + (1.0702 * (duration2 * 0.034 / 2)) - 1.3030;

}


void resetMotors() {
    stepperLeft.setCurrentPosition(0);
    stepperRight.setCurrentPosition(0);
    stepperLeft.stop();
    stepperRight.stop();
}

// Movement functions
void goForward(float distance_cm) {
    enableMotors();
    long steps = distance_cm * STEPS_PER_CM;
    stepperLeft.moveTo(steps); //stepperLeft.moveTo(-steps); 
    stepperRight.moveTo(steps); //tepperRight.moveTo(steps);
    
    while(stepperLeft.distanceToGo() != 0 || stepperRight.distanceToGo() != 0) {
        stepperLeft.run();
        stepperRight.run();
    }
    resetMotors();
    holdAndDisable();
}

void goBack(float distance_cm) {
    enableMotors();
    long steps = distance_cm * STEPS_PER_CM;
    stepperLeft.moveTo(-steps); //    stepperLeft.moveTo(steps); stepperRight.moveTo(-steps);
    stepperRight.moveTo(-steps);
    
    while(stepperLeft.distanceToGo() != 0 || stepperRight.distanceToGo() != 0) {
        stepperLeft.run();
        stepperRight.run();
    }
    resetMotors();
    holdAndDisable();
}




void turn(float angle, bool turnRight) {
    enableMotors();

    float arc_length = (angle * PI * WHEEL_SEPARATION * 0.9) / 360.0;
    long steps = arc_length * STEPS_PER_CM * TURN_FACTOR/4;
    
    if(turnRight) {
        stepperLeft.moveTo(-steps); //no negative sign 
        stepperRight.moveTo(steps);
    } else {
        stepperLeft.moveTo(steps); //with negative sign 
        stepperRight.moveTo(-steps);
    }
    
    while(stepperLeft.distanceToGo() != 0 || stepperRight.distanceToGo() != 0) {
        stepperLeft.run();
        stepperRight.run();
    }
    resetMotors();
    holdAndDisable();
}

// Vacuum system functions
void startPump() {
    digitalWrite(RELAY_PIN, HIGH);
    pumpRunning = true;
    Serial.println("Pump started");
}

void stopPump() {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);
    pumpRunning = false;
    Serial.println("Pump stopped");
}

void pickup() {
    digitalWrite(SOLENOID_PIN, HIGH);
    Serial.println("Item picking up");
}

void release() {
    digitalWrite(SOLENOID_PIN, LOW);
    Serial.println("Item released");
}

// Servo control functions
void Roller_down(int degrees) {
    int targetPos = currentPos + degrees;
    if (targetPos > 180) targetPos = 180;
    
    for (int pos = currentPos; pos <= targetPos; pos++) {
        myservo.write(pos);
        delay(15);
    }
    currentPos = targetPos;
}

void Roller_up(int degrees) {
    int targetPos = currentPos - degrees;
    if (targetPos < 0) targetPos = 0;
    
    for (int pos = currentPos; pos >= targetPos; pos--) {
        myservo.write(pos);
        delay(15);
    }
    currentPos = targetPos;
}

// Web server handlers
void handleRoot() {
    server.send(200, "text/html", MAIN_page);
}

void handleDistance() {
    //String distanceStr = String(distance);
    server.send(200, "text/plain", String(distance));
}

void handleUltrasonic1() {
    server.send(200, "text/plain", String(distance1,1));
}

void handleUltrasonic2() {
    server.send(200, "text/plain", String(distance2,1));
}

void handleServoPosition() {
    server.send(200, "text/plain", String(currentPos));
}
void handleWarning() {
    bool isWarning = (distance < 65 && distance != 9999);  // Check if distance is less than 65mm and not a timeout value
    server.send(200, "text/plain", String(isWarning));
}



void setup() {
    Serial.begin(115200);
    
    // Configure vacuum system pins
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
    
    // Configure
    // Configure servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);
    myservo.attach(SERVO_PIN, minUs, maxUs);
    myservo.write(10);
    
    // Configure stepper motors
    stepperLeft.setMaxSpeed(MAX_SPEED);
    stepperRight.setMaxSpeed(MAX_SPEED);
    stepperLeft.setAcceleration(ACCELERATION);
    stepperRight.setAcceleration(ACCELERATION);
    
    // Configure motor enable pins
    pinMode(ENABLE_PIN_LEFT, OUTPUT);
    pinMode(ENABLE_PIN_RIGHT, OUTPUT);
    disableMotors();  // Start with motors disabled
    setupUltrasonicSensors();
    


    // Initialize ToF sensor
    scanI2C();
    sensorConnected = setupTOF();
    if (!sensorConnected) {
        Serial.println("Warning: Distance readings will not be available!");
    }
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/command", handleCommand);
    server.on("/distance", handleDistance);
    server.on("/servo_position", handleServoPosition);
    server.on("/ultrasonic1", handleUltrasonic1);
    server.on("/ultrasonic2", handleUltrasonic2);
    server.on("/warning", handleWarning); 
    server.begin();
    
    Serial.println("HTTP server started");
    Serial.println("\nAvailable Serial Commands:");
    Serial.println("Movement commands:");
    Serial.println("  w XX - Move forward XX cm");
    Serial.println("  s XX - Move backward XX cm");
    Serial.println("  a XX - Turn left XX degrees");
    Serial.println("  d XX - Turn right XX degrees");
    Serial.println("  x - Stop movement");
    Serial.println("\nVacuum commands:");
    Serial.println("  PUMP_START - Start vacuum pump");
    Serial.println("  PUMP_STOP - Stop vacuum pump");
    Serial.println("  PICKUP - Activate solenoid for pickup");
    Serial.println("  RELEASE - Deactivate solenoid to release");
    Serial.println("\nServo commands:");
    Serial.println("  UP XX - Move servo up XX degrees");
    Serial.println("  DOWN XX - Move servo down XX degrees");
    Serial.println("  POS - Get current servo position");
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        //command.toUpperCase();
        
        // Parse command and value
        int spaceIndex = command.indexOf(' ');
        String cmd = (spaceIndex == -1) ? command : command.substring(0, spaceIndex);
        float value = (spaceIndex == -1) ? 0 : command.substring(spaceIndex + 1).toFloat();
        
        // Movement commands
        if (cmd == "w") {
            goForward(value);
            Serial.println("Moving forward:"+String(value));
        }
        else if (cmd == "s") {
            goBack(value);
            Serial.println("Moving backward:"+String(value));
        }
        else if (cmd == "a") {
            turn(value, false);
            Serial.println("Turning left:"+String(value));
        }
        else if (cmd == "d") {
            turn(value, true);
            Serial.println("Turning right:"+String(value));
        }
        else if (cmd == "x") {
            resetMotors();
            holdAndDisable();
            Serial.println("Stopped");
        }
        // Vacuum commands
        else if (cmd == "p") {
            startPump();
        }
        else if (cmd == "o") {
            stopPump();
        }
        else if (cmd == "l") {
            if (pumpRunning) {
                pickup();
            } else {
                Serial.println("Error: Start pump first!");
            }
        }
        else if (cmd == "k") {
            if (pumpRunning) {
                release();
            } else {
                Serial.println("Error: Start pump first!");
            }
        }
        // Servo commands
        else if (cmd == "m") {
            Roller_up(value);
            Serial.print("Current position: "+String(value));
            Serial.println(currentPos);
        }
        else if (cmd == "n") {
            Roller_down(value);
            Serial.print("Current position: "+String(value));
            Serial.println(currentPos);
        }
        else if (cmd == "b") {
            Serial.print("Current servo position: "+String(value));
            Serial.println(currentPos);
        }
        else if (cmd=="c"){
          Serial.println("complete blovk pick up");
        }
        else {
            Serial.println("Invalid command. Type 'HELP' for available commands.");
        }
    }
}

void loop() {
    server.handleClient();
    handleSerialCommands();
    
    // Update distance reading
    static unsigned long lastDistanceUpdate = 0;
    if (millis() - lastDistanceUpdate > 50) {  // Update every 50ms
        lastDistanceUpdate = millis();
        if (sensorConnected) {
            distance = sensor.readRangeContinuousMillimeters();
            if (sensor.timeoutOccurred()) {
                Serial.println("ToF TIMEOUT");
                distance = 9999;
            } //else{
              //Serial.print("ToF Distance:");
              //Serial.println(distance);
            //}
        }
        else
        {
            Serial.println("ToF sensor seems to be not connected?");
        }
        readUltrasonicSensors();
    }
}
