#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Wire.h>

// HARDWARE PINS 
#define LED_AILERON_LEFT    13  // Left aileron indicator (HARD RT)
#define LED_AILERON_RIGHT   12  // Right aileron indicator (HARD RT)
#define LED_SYSTEM_STATUS   14  // System heartbeat (monitoring)
#define LED_TELEMETRY       27  // Telemetry update indicator (SOFT RT)

#define BTN_EMERGENCY_STOP  15  // Emergency cutoff button (HARD RT ISR)
#define POTENTIOMETER_PIN   34  // Flight stick input (analog)
#define ALTITUDE_SENSOR_SCL 22  // I2C altitude sensor
#define ALTITUDE_SENSOR_SDA 21

// TIMING CONSTANTS 
#define FLIGHT_CONTROL_PERIOD_MS    20   // 50Hz - HARD deadline
#define SENSOR_FUSION_PERIOD_MS     50   // 20Hz - HARD deadline
#define TELEMETRY_PERIOD_MS         100  // 10Hz - SOFT deadline
#define DATA_LOGGING_PERIOD_MS      500  // 2Hz - SOFT deadline, VARIABLE time

// GLOBAL DATA STRUCTURES
struct FlightData {
    int16_t stickPosition;      // -100 to +100
    int16_t aileronAngle;       // -45 to +45 degrees
    uint16_t altitude;          // meters
    uint32_t timestamp;         // milliseconds
    bool emergencyActive;
};

//SYNCHRONIZATION PRIMITIVES 
QueueHandle_t flightDataQueue;          // Inter-task communication (Flight Control -> Telemetry)
SemaphoreHandle_t altitudeMutex;        // Protect shared altitude reading
SemaphoreHandle_t emergencySemaphore;   // Binary semaphore for emergency signaling
volatile bool emergencyStopFlag = false;
portMUX_TYPE criticalMux = portMUX_INITIALIZER_UNLOCKED; // Critical section for ISR

// Shared variables (protected by mutex/critical sections)
volatile uint16_t currentAltitude = 1000; // meters
volatile int16_t currentStickPosition = 0;

// ISR: EMERGENCY CUTOFF 
// HARD REAL-TIME: Must respond within 1ms
// Simulates ejection seat or critical system shutdown
void IRAM_ATTR emergencyStopISR() {
    portENTER_CRITICAL_ISR(&criticalMux);
    emergencyStopFlag = true;
    portEXIT_CRITICAL_ISR(&criticalMux);
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(emergencySemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//  TASK 1: FLIGHT CONTROL SURFACE 
// HARD REAL-TIME: 20ms period (50Hz)
// Controls aileron positions based on stick input
// Deadline miss could cause loss of control authority in simulator
void taskFlightControl(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(FLIGHT_CONTROL_PERIOD_MS);
    
    FlightData flightData;
    
    while (1) {
        uint32_t startTime = millis();
        
        // Check emergency flag with critical section
        bool emergencyActive;
        portENTER_CRITICAL(&criticalMux);
        emergencyActive = emergencyStopFlag;
        portEXIT_CRITICAL(&criticalMux);
        
        if (emergencyActive) {
            // Emergency mode: center all controls
            digitalWrite(LED_AILERON_LEFT, LOW);
            digitalWrite(LED_AILERON_RIGHT, LOW);
            flightData.aileronAngle = 0;
            flightData.stickPosition = 0;
        } else {
            // Normal operation: read stick and compute control surface positions
            int rawStick = analogRead(POTENTIOMETER_PIN);
            currentStickPosition = map(rawStick, 0, 4095, -100, 100);
            
            // Compute aileron deflection (-45 to +45 degrees)
            int16_t aileronAngle = map(currentStickPosition, -100, 100, -45, 45);
            
            // Update aileron indicators
            if (aileronAngle > 10) {
                digitalWrite(LED_AILERON_RIGHT, HIGH);
                digitalWrite(LED_AILERON_LEFT, LOW);
            } else if (aileronAngle < -10) {
                digitalWrite(LED_AILERON_LEFT, HIGH);
                digitalWrite(LED_AILERON_RIGHT, LOW);
            } else {
                digitalWrite(LED_AILERON_LEFT, LOW);
                digitalWrite(LED_AILERON_RIGHT, LOW);
            }
            
            flightData.stickPosition = currentStickPosition;
            flightData.aileronAngle = aileronAngle;
        }
        
        flightData.timestamp = millis();
        flightData.emergencyActive = emergencyActive;
        
        // Send to telemetry queue (non-blocking)
        xQueueSend(flightDataQueue, &flightData, 0);
        
        uint32_t executionTime = millis() - startTime;
        Serial.printf("[FLIGHT_CTRL] Stick: %d, Aileron: %d deg, ExecTime: %lu ms\n", 
                      flightData.stickPosition, flightData.aileronAngle, executionTime);
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

//  TASK 2: SENSOR FUSION 
// HARD REAL-TIME: 50ms period (20Hz)
// Processes altitude sensor data for collision avoidance
// Deadline miss could cause missed obstacle detection
void taskSensorFusion(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(SENSOR_FUSION_PERIOD_MS);
    
    while (1) {
        uint32_t startTime = millis();
        
        // Simulate altitude sensor reading (in real system: I2C barometer)
        uint16_t newAltitude = 1000 + (rand() % 200) - 100; // 900-1100m range
        
        // Update shared altitude with mutex protection
        if (xSemaphoreTake(altitudeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentAltitude = newAltitude;
            xSemaphoreGive(altitudeMutex);
        }
        
        uint32_t executionTime = millis() - startTime;
        Serial.printf("[SENSOR_FUSION] Altitude: %u m, ExecTime: %lu ms\n", 
                      newAltitude, executionTime);
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// TASK 3: TELEMETRY DISPLAY 
// SOFT REAL-TIME: 100ms period (10Hz)
// Updates pilot display - deadline miss causes display lag but not danger
void taskTelemetry(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    
    FlightData receivedData;
    
    while (1) {
        uint32_t startTime = millis();
        
        // Receive flight data from queue
        if (xQueueReceive(flightDataQueue, &receivedData, 0) == pdTRUE) {
            // Get altitude with mutex
            uint16_t altitude;
            if (xSemaphoreTake(altitudeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                altitude = currentAltitude;
                xSemaphoreGive(altitudeMutex);
            } else {
                altitude = 0;
            }
            
            // Update telemetry indicator LED
            digitalWrite(LED_TELEMETRY, !digitalRead(LED_TELEMETRY));
            
            // Send telemetry via UART (external communication)
            Serial.printf("=== TELEMETRY DISPLAY ===\n");
            Serial.printf("Stick Position: %d%%\n", receivedData.stickPosition);
            Serial.printf("Aileron Angle: %d deg\n", receivedData.aileronAngle);
            Serial.printf("Altitude: %u m\n", altitude);
            Serial.printf("Emergency: %s\n", receivedData.emergencyActive ? "ACTIVE" : "Normal");
            Serial.printf("Timestamp: %lu ms\n", receivedData.timestamp);
            Serial.println("========================\n");
        }
        
        uint32_t executionTime = millis() - startTime;
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// TASK 4: DATA LOGGING 
// SOFT REAL-TIME: 500ms period (2Hz)
// VARIABLE EXECUTION TIME - simulates writing varying amounts of data
// Records flight data for post-mission analysis
void taskDataLogging(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(DATA_LOGGING_PERIOD_MS);
    
    uint32_t logCounter = 0;
    
    while (1) {
        uint32_t startTime = millis();
        
        // Variable execution time simulation (10-50ms range)
        // Simulates varying data volume in log writes
        uint32_t variableDelay = 10 + (rand() % 40);
        
        // Get current system state
        uint16_t altitude;
        if (xSemaphoreTake(altitudeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            altitude = currentAltitude;
            xSemaphoreGive(altitudeMutex);
        } else {
            altitude = 0;
        }
        
        // Simulate variable logging work
        delay(variableDelay);
        
        logCounter++;
        uint32_t executionTime = millis() - startTime;
        
        Serial.printf("[DATA_LOG #%lu] Alt: %u m, Stick: %d, ExecTime: %lu ms (VARIABLE)\n", 
                      logCounter, altitude, currentStickPosition, executionTime);
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// TASK 5: SYSTEM HEARTBEAT 
// Monitoring task - blinks LED to prove system determinism
// Used with logic analyzer to verify timing
void taskHeartbeat(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); // 1Hz
    
    while (1) {
        digitalWrite(LED_SYSTEM_STATUS, !digitalRead(LED_SYSTEM_STATUS));
        Serial.println(">>> HEARTBEAT: All systems operational <<<");
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

//TASK 6: EMERGENCY HANDLER 
// Waits on semaphore from ISR to handle emergency shutdown
void taskEmergencyHandler(void *parameter) {
    while (1) {
        // Block waiting for emergency semaphore from ISR
        if (xSemaphoreTake(emergencySemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("\n!!! EMERGENCY STOP ACTIVATED !!!");
            Serial.println("!!! ALL FLIGHT CONTROLS DISABLED !!!");
            Serial.println("!!! SIMULATING EJECTION SEQUENCE !!!\n");
            
            // Emergency handling logic here
            // In real system: activate safety protocols, log emergency event, etc.
        }
    }
}

// SETUP 
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("F-35 FLIGHT SIMULATOR - REAL-TIME SYSTEM");
    Serial.println("Lockheed Martin Orlando");
    Serial.println("========================================\n");
    
    // Initialize hardware
    pinMode(LED_AILERON_LEFT, OUTPUT);
    pinMode(LED_AILERON_RIGHT, OUTPUT);
    pinMode(LED_SYSTEM_STATUS, OUTPUT);
    pinMode(LED_TELEMETRY, OUTPUT);
    pinMode(BTN_EMERGENCY_STOP, INPUT_PULLUP);
    pinMode(POTENTIOMETER_PIN, INPUT);
    
    // Initialize I2C for altitude sensor
    Wire.begin(ALTITUDE_SENSOR_SDA, ALTITUDE_SENSOR_SCL);
    
    // Create synchronization primitives
    flightDataQueue = xQueueCreate(5, sizeof(FlightData));
    altitudeMutex = xSemaphoreCreateMutex();
    emergencySemaphore = xSemaphoreCreateBinary();
    
    // Attach emergency ISR
    attachInterrupt(digitalPinToInterrupt(BTN_EMERGENCY_STOP), emergencyStopISR, FALLING);
    
    Serial.println("Creating FreeRTOS tasks...\n");
    
    // Create tasks with priorities (higher number = higher priority)
    xTaskCreatePinnedToCore(taskFlightControl, "FlightControl", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(taskSensorFusion, "SensorFusion", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskTelemetry, "Telemetry", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskDataLogging, "DataLogging", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskHeartbeat, "Heartbeat", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskEmergencyHandler, "EmergencyHandler", 2048, NULL, 5, NULL, 1);
    
    Serial.println("All tasks created successfully!");
    Serial.println("System operational - monitoring timing...\n");
}

void loop() {
    // Empty - all work done in FreeRTOS tasks
    vTaskDelay(portMAX_DELAY);
}
