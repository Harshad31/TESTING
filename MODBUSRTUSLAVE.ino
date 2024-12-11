#include <ModbusRTUSlave.h>
#include <Adafruit_MAX31865.h>

#define MAX31865_MISO 19
#define MAX31865_MOSI 23
#define MAX31865_CLK 18
#define MAX31865_CS 5

// Create MAX31865 object
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

// Define Modbus parameters
#define SLAVE_ID 1
#define DE_PIN 4  // Driver Enable pin (RE and DE connected to the same pin)
#define RE_PIN 4  // Receiver Enable pin (RE and DE connected to the same pin)
#define RX_PIN 16 // RX Pin for MAX485
#define TX_PIN 17 // TX Pin for MAX485

// Define holding registers for Modbus (1st register stores the temperature value)
uint16_t holdingRegisters[10];  // Example: 10 holding registers

// Constants for the Callendar-Van Dusen equation
#define R0 100.0  // Nominal resistance of the RTD at 0°C
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

// Create Modbus slave object
ModbusRTUSlave modbusSlave(Serial2, DE_PIN, RE_PIN);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // RS485 communication with Modbus
  modbusSlave.begin(SLAVE_ID, 9600, SERIAL_8N1);   // Set up Modbus slave communication
  
  thermo.begin(MAX31865_3WIRE);  // Initialize the MAX31865 sensor (3-wire RTD configuration)
  
  // Configure holding registers (start with the first 10 registers)
  modbusSlave.configureHoldingRegisters(holdingRegisters, 10);
}

void loop() {
  // Poll Modbus to handle incoming requests from the master (Delta HMI)
  modbusSlave.poll();

  // Read RTD data from the sensor
  uint16_t rtd = thermo.readRTD();
  float Rt = (rtd / 32768.0) * 430.0;  // RTD resistance (in ohms)
  
  // Calculate temperature from the RTD resistance
  float temperature = calculateTemperature(Rt);

  // Store the calculated temperature in the first holding register (40001)
  holdingRegisters[0] = (uint16_t)(temperature * 100);  // Storing temperature in tenths of degrees (for better precision)

  // Output temperature to serial monitor for debugging
  Serial.print("Calculated Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  // Check for faults in the MAX31865 sensor
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); 
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();  // Clear the fault flag
  }

  delay(1000);  // Delay for 1 second (for better Modbus communication handling)
}

float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    // For temperatures above 0°C
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    // For temperatures below 0°C
    float tolerance = 0.001;  // Convergence tolerance
    int maxIterations = 100;
    int iteration = 0;
    float diff;
    t = Rt;  // Initial guess for temperature

    do {
      float fValue = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - Rt;
      float fDerivative = R0 * (A + 2 * B * t + 3 * C * (t - 100) * t * t + C * t * t * t);
      float nextT = t - fValue / fDerivative;
      diff = abs(nextT - t);
      t = nextT;
      iteration++;
    } while (diff > tolerance && iteration < maxIterations);

    if (iteration == maxIterations) {
      Serial.println("Max iterations reached. Convergence not achieved.");
    }
  }

  return t;
}
