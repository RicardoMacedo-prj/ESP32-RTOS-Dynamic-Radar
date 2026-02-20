#include <Arduino.h>
#include <math.h> 
#include <BluetoothSerial.h>


// --- PINS Configuration ---
#define PIN_TRIG 32       
#define PIN_ECHO 25       
#define PIN_THERMISTOR 34
#define PIN_GREEN_LED 22
#define PIN_RED_LED 23

// --- NTC Thermistor Parameters ---
#define R_FIXED 10000.0 
#define BETA 3950.0     
#define T0 298.15    
#define R0 10000.0

// --- Flags Config ----
#define Functional (1 << 0)

// --- Filter Config ---
#define DISTANCE_BUFFER_SIZE 5

// --- RTOS Handles ----
SemaphoreHandle_t xSemaphore;
QueueHandle_t xQueueUSB;
QueueHandle_t xQueueBT;
SemaphoreHandle_t xMutex;
BluetoothSerial SerialBT;
EventGroupHandle_t xEventGroup;


// --- Shared Volatile Variables ---
volatile unsigned long StartEcho = 0;
volatile unsigned long EndEcho = 0;
volatile float SoundSpeedFactor = 0.0343;
volatile float temperatureC = 0;

// --- TASK: Radar Core (High Priority) ---
void taskRadar(void *pvParameters) { 
  float distance;
  unsigned int BufferIndex = 0;
  float distanceAvgBuffer[DISTANCE_BUFFER_SIZE] = {0};

  while(1){
     float distanceSum = 0;

     // Wait for system to be active (Suspend task if OFF)
    xEventGroupWaitBits(xEventGroup, Functional, pdFALSE, pdTRUE, portMAX_DELAY);

    // 10us trigger pulse (HC-SR04 spec)
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // Block task until ISR captures echo (30ms timeout)
      if (xSemaphoreTake(xSemaphore, 30/portTICK_PERIOD_MS) == pdTRUE){
        unsigned long duration = EndEcho - StartEcho;
        
        // Calculate distance using dynamic sound speed
        distance = (float)((duration * SoundSpeedFactor)/2);

        // Update circular buffer
        distanceAvgBuffer[BufferIndex] = distance;
        BufferIndex = (BufferIndex + 1) % DISTANCE_BUFFER_SIZE;

        for(int i = 0; i<DISTANCE_BUFFER_SIZE; i++){
          distanceSum += distanceAvgBuffer[i];
        }

        float distanceAVG = distanceSum / DISTANCE_BUFFER_SIZE;

        // Dispatch filtered data to queues
        xQueueSend(xQueueUSB,(void *)&distanceAVG,0);
        xQueueSend(xQueueBT,(void *)&distanceAVG,0);
      }

      vTaskDelay(60 / portTICK_PERIOD_MS); 
  }
}


// --- HARDWARE ISR: Echo Pin ---
void IRAM_ATTR RadarISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if ((GPIO.in >> PIN_ECHO) & 0x1){
    StartEcho = micros(); // Rising edge
  
  }else{
    EndEcho = micros(); // Falling edge

    // Unblock taskRadar
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } 

}


// --- TASK: User Interface & Feedback ---
void taskFeedback (void *pvParameter){ //Receive value stored in the queue
  float ReceivedDistance;
  static bool ActiveLEDState = LOW;
  static unsigned long LEDBlinkTimer = 0;

  while(1){
    EventBits_t EVBits = xEventGroupGetBits(xEventGroup);

    // Fetch new filtered data
    if(xQueueReceive( xQueueUSB,&ReceivedDistance, 30 / portTICK_PERIOD_MS) == pdTRUE){
      
      // Thread-safe print using Mutex
      xSemaphoreTake(xMutex, portMAX_DELAY);
      
      Serial.print("Distance: ");
      Serial.print(ReceivedDistance);
      Serial.println(" cm");
      Serial.println();
      Serial.print("Temperature: ");
      Serial.print(temperatureC);
      Serial.println("ºC");
      
      xSemaphoreGive(xMutex);

      // Alarm logic
      if(EVBits & Functional){
        if ((ReceivedDistance)< 20){
          digitalWrite(PIN_RED_LED, HIGH);
        }else{
          digitalWrite(PIN_RED_LED, LOW);
        }
      }
    }

    // System functioning heartbeat 
    if(EVBits & Functional){
      if(millis() - LEDBlinkTimer >= 1000){
        LEDBlinkTimer = millis();
        ActiveLEDState = !ActiveLEDState;
        digitalWrite(PIN_GREEN_LED, ActiveLEDState);
      }
    }else{
      // Standby mode
      digitalWrite(PIN_GREEN_LED, LOW);
      digitalWrite(PIN_RED_LED, LOW);
      ActiveLEDState = LOW;
    }

  }
}

// --- TASK: Thermal Compensation (Low Priority) ---
void TaskThermistor(void *pvParameter){
  while(1){
    // Read the analog value from the sensor pin (0 - 4095)
    int adcValue = analogRead(PIN_THERMISTOR);

    // Simple check to avoid errors with 0 or max value
    if (adcValue > 0 && adcValue < 4095) {
      
      // Calculate the current resistance of the thermistor
      float R_NTC = R_FIXED * ((4095.0 / (float)adcValue) - 1.0);

      // Convert resistance to temperature in Kelvin using the Beta formula
      float temperatureK = 1.0 / ((1.0 / T0) + (log(R_NTC / R0) / BETA));

      // Convert Kelvin to Celsius
      temperatureC = temperatureK - 273.15;

      // Calculate the speed of sound adjusted for the current temperature
      // Formula: v = 331.3 + 0.606 * T
      float newSpeed = (331.3 + (0.606 * temperatureC)) / 10000.0;

      SoundSpeedFactor = newSpeed;
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// --- TASK: Bluetooth Control Interface ---
void TaskBluetoothControl (void *pvParameter){
  float ReceivedDistanceBT;
  String InputBT;

  while(1){
    // Stream data to client if connected
    if (xQueueReceive(xQueueBT,&ReceivedDistanceBT, 30 / portTICK_PERIOD_MS) == pdTRUE){
      if(SerialBT.hasClient()){
        SerialBT.print("BT_Dist: ");
        SerialBT.print(ReceivedDistanceBT);
        SerialBT.println(" cm");
      }
    }

    // Read text commands to switch system ON or OFF
    if(SerialBT.available()){
      InputBT = SerialBT.readStringUntil('\n');
      InputBT.trim();
      InputBT.toLowerCase();

      EventBits_t EVBits = xEventGroupGetBits(xEventGroup); 

      if(InputBT == "off"){
        if(EVBits & Functional){
          SerialBT.println("Radar Deactivated");
          xEventGroupClearBits(xEventGroup, Functional);
        }else{
          SerialBT.println("Radar is Already Deactivated");
        }
      }
      else if(InputBT == "on"){
        if(!(EVBits & Functional)){
          SerialBT.println("Radar Activated");
          xEventGroupSetBits(xEventGroup, Functional);
        }else{
          SerialBT.println("Radar is Already Activated");
        }
      }
      else{
        SerialBT.println("Unknown Command");
      }
    } 
  }
}


void setup() {
  Serial.begin(115200);
  
  // Hardware initialization
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);

  Serial.println("--- Security System Initiated ---");
  SerialBT.begin("ESP32_BT");

  // RTOS Primitives initialization
  xEventGroup = xEventGroupCreate();
  xEventGroupSetBits(xEventGroup, Functional);

  xSemaphore = xSemaphoreCreateBinary();

  xQueueUSB = xQueueCreate(5,sizeof(float));
  xQueueBT = xQueueCreate(5,sizeof(float));

  xMutex = xSemaphoreCreateMutex();
  
  // Tasks instantiation
  xTaskCreate(taskRadar,"Radar Function",2048,NULL,2,NULL);
  xTaskCreate(taskFeedback, "Distance Receiver",2048,NULL,1,NULL);
  xTaskCreate(TaskBluetoothControl, "Bluetooth Control", 8192,NULL,1,NULL);
  xTaskCreate(TaskThermistor, "Thermistor Function", 2048, NULL, 1, NULL);

  // Attach hardware interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO),RadarISR,CHANGE);

}

void loop() {
  vTaskDelete(NULL);
}

