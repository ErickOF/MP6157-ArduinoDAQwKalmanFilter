#include <LM35.h>


// Pin for the analog version of the temperature sensor
#define PIN_ANA_TEMPERATURE A0
// Default time between samples in miliseconds
#define DEFAULT_SAMPLE_RATE 60000.0


// Temperature sensor
LM35 lm35(PIN_ANA_TEMPERATURE);

// Variable to store the temperature
double temperature;
// Variable to store the current
double sampleRate;
// Variables to measure time
unsigned long startTime;
unsigned long elapseTime;


void setup()
{
  // Setup temperature data pins
  pinMode(PIN_ANA_TEMPERATURE, INPUT);
  // Setting values
  sampleRate = DEFAULT_SAMPLE_RATE;
  // Init Serial communication
  Serial.begin(9600);
}

void loop()
{
  // Measure start time
  startTime = micros();
  // Read the temperature
  temperature = lm35.getTemp(CELCIUS);
  // Measure total time
  elapseTime = micros() - startTime;

  Serial.print("Temperature sample time: ");
  Serial.println(elapseTime);

  // Measure start time
  startTime = micros();
  // Send the data
  Serial.println(temperature);
  // Measure total time
  elapseTime = micros() - startTime;

  Serial.print("Send data time: ");
  Serial.println(elapseTime);

  delay(100);
}
