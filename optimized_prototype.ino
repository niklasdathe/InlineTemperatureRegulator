#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>

// Debug flag
#define DEBUG 0 // Set to 1 to enable Serial prints

#if DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Pin definitions
#define RELAY_PIN 5             // Digital Pin 5 (PD5)
#define CURRENT_SENSOR_PIN A7   // Analog Pin A7 (ADC7)
#define ENCODER_SWITCH_PIN 2    // Digital Pin 2 (PD2, INT0)
#define ENCODER_DT_PIN 9        // Digital Pin 9 (PB1, PCINT1)
#define ENCODER_CLK_PIN 10      // Digital Pin 10 (PB2, PCINT2)

// OLED display size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Create global display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// Create AHT10 sensor object
Adafruit_AHTX0 aht;

// Variables in fixed-point arithmetic (multiplied by 10)
int targetTemperature = 250; // Default target temperature (25.0°C * 10)
const int hysteresis = 5;    // Hysteresis value (0.5°C * 10)
const float voltage_Vrms = 230.0; // Voltage in Volts (AC mains voltage)

// Watchdog Timer configuration
volatile uint8_t wdtCounter = 0;
volatile uint8_t displayOnTimeCounter = 0;
volatile bool userInputFlag = false;
bool displayOnFlag = false;
#define DISPLAY_ON_DURATION_WDT_COUNTS 4 // Number of WDT intervals to keep the display on

// Sensor readings
float current_mA = 0.0;
int temperature = 0; // In tenths of degrees
int humidity = 0;    // In tenths of percent
float mWhPerDay = 0.0; // Accumulated energy in milliWatt-hours (mWh)

bool relayOn = false;

// Encoder variables
volatile bool encoderRotatedFlag = false;
volatile int8_t encoderDirection = 0; // +1 or -1
volatile uint8_t lastClkState = 0;

// Calibration constants for current measurement
// These values need to be determined experimentally
const float calibration_factor = 0.1; // Example value (replace with actual)
const float calibration_offset = 0.0; // Example value (replace with actual)

// Elapsed time in seconds (volatile since updated in ISR)
volatile unsigned long elapsedTime = 0; // in seconds

void setup() {
  #if DEBUG
    Serial.begin(9600);
    DEBUG_PRINTLN(F("Serial begun."));
  #endif

  initializePeripherals();
  initializeDisplay();

  // Set up user input interrupt (button on pin 2)
  DDRD &= ~(1 << DDD2); // Set PD2 as input
  PORTD |= (1 << PORTD2); // Enable pull-up resistor on PD2

  // Configure INT0 to trigger on FALLING edge
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  // Enable INT0 External Interrupt
  EIMSK |= (1 << INT0);
  sei(); // Enable global interrupts

  // Initialize encoder pins
  DDRB &= ~(1 << DDB2); // Set PB2 (digital pin 10) as input
  PORTB |= (1 << PORTB2); // Enable pull-up on PB2

  DDRB &= ~(1 << DDB1); // Set PB1 (digital pin 9) as input
  PORTB |= (1 << PORTB1); // Enable pull-up on PB1

  // Read initial state of encoder CLK pin
  lastClkState = (PINB & (1 << PINB2)) ? HIGH : LOW;

  // Enable pin change interrupt on encoder CLK pin (PCINT2)
  PCICR |= (1 << PCIE0); // Enable pin change interrupt 0 (PCINT0 to PCINT7)
  PCMSK0 |= (1 << PCINT2); // Enable PCINT2 (PB2, digital pin 10)

  // Disable unused peripherals to save power
  power_spi_disable();
  power_timer1_disable();
  power_timer2_disable();

  // Set up Watchdog Timer for 8-second interval
  wdt_reset();
  MCUSR = 0;
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // 8 seconds

  readSensorsWriteRelay();
  updateDisplay();

  DEBUG_PRINTLN(F("Setup complete."));
}

void loop() {
  // Enter sleep mode
  DEBUG_PRINTLN("Going to sleep.");
  #if DEBUG
    Serial.flush();
  #endif

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei(); // Ensure interrupts are enabled
  sleep_mode(); // MCU sleeps here

  // MCU wakes up here after WDT interrupt or user input
  sleep_disable(); // Disable sleep mode
  DEBUG_PRINTLN("Just woke up!");

  // Handle periodic sensor updates
  if (wdtCounter >= 1) {
    wdtCounter = 0;
    readSensorsWriteRelay();
  }

  // Handle encoder rotation
  if (encoderRotatedFlag) {
    encoderRotatedFlag = false;

    targetTemperature += encoderDirection * 5; // Adjust as needed
    targetTemperature = constrain(targetTemperature, 100, 300); // Limit between 10.0°C and 30.0°C

    // Wake up display
    userInputFlag = true;
    displayOnTimeCounter = 0;

    DEBUG_PRINT("Encoder rotated: ");
    DEBUG_PRINTLN(encoderDirection);
  }

  // Handle user input and display updates
  if (userInputFlag || displayOnFlag) {
    if (!displayOnFlag) {
      // Turn on the display
      displayOnFlag = true;
      displayOnTimeCounter = 0;
      display.ssd1306_command(SSD1306_DISPLAYON);
      DEBUG_PRINTLN(F("Display turned on."));
    }

    updateDisplay();

    // Check if the display should be turned off
    if (displayOnTimeCounter >= DISPLAY_ON_DURATION_WDT_COUNTS) {
      // Turn off the display
      displayOnFlag = false;
      userInputFlag = false;
      display.clearDisplay();
      display.display();
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      DEBUG_PRINTLN(F("Display turned off."));
    }
  }

  // Reset energy accumulation daily
  if (elapsedTime >= 86400) { // 24 hours in seconds
    mWhPerDay = 0.0;
    elapsedTime = 0;
    DEBUG_PRINTLN("Energy accumulation reset for new day.");
  }
}

// Interrupt Service Routine for INT0 (Digital Pin 2)
ISR(INT0_vect) {
  userInputFlag = true;
  displayOnTimeCounter = 0;
}

// Interrupt Service Routine for Pin Change Interrupt 0 (Digital Pin 10)
ISR(PCINT0_vect) {
  uint8_t clkState = (PINB & (1 << PINB2)) ? HIGH : LOW;

  if (clkState != lastClkState) {
    if (clkState == HIGH) {
      uint8_t dtState = (PINB & (1 << PINB1)) ? HIGH : LOW;

      if (dtState != clkState) {
        encoderDirection = +1;
      } else {
        encoderDirection = -1;
      }

      encoderRotatedFlag = true;
    }
    lastClkState = clkState;
  }
}

// Watchdog Timer Interrupt Service Routine
ISR(WDT_vect) {
  wdtCounter++;
  if (displayOnFlag) {
    displayOnTimeCounter++;
  }
  // Increment elapsed time by 8 seconds
  elapsedTime += 8;
}

void initializePeripherals() {
  // Initialize relay pin
  DDRD |= (1 << DDD5);        // Set PD5 (Digital Pin 5) as output
  PORTD &= ~(1 << PORTD5);    // Start with relay off

  // Initialize AHT10 sensor
  if (!aht.begin()) {
    DEBUG_PRINTLN(F("Could not find AHT10 sensor!"));
    while (1)
      delay(10); // Halt execution if sensor initialization fails
  }
  DEBUG_PRINTLN(F("Peripherals initialized."));
}

void readSensorsWriteRelay() {
  #define ADC_CHANNEL 7 // A7 corresponds to ADC7

  // Configure ADC to read from ADC7 with AVcc reference
  ADMUX = (1 << REFS0) | (ADC_CHANNEL & 0x07);
  ADCSRA = (1 << ADEN)  // Enable ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler of 128
  ADCSRA |= (1 << ADSC); // Start ADC conversion
  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
  int analogValue = ADCL;
  analogValue |= (ADCH << 8);
  ADCSRA &= ~(1 << ADEN); // Disable ADC to save power

  // Calculate current using calibration constants
  current_mA = (analogValue * calibration_factor) + calibration_offset;

  // Read from the AHT10 sensor
  sensors_event_t hum, temp;
  if (!aht.getEvent(&hum, &temp)) {
    DEBUG_PRINTLN(F("Error reading sensor data."));
    return;
  }
  temperature = (int)(temp.temperature * 10.0); // Convert to tenths
  humidity = (int)(hum.relative_humidity * 10.0); // Convert to tenths

  // Control the relay based on temperature
  if (temperature < (targetTemperature - hysteresis)) {
    PORTD |= (1 << PORTD5); // Turn relay ON
    if (!relayOn) {
      relayOn = true;
      DEBUG_PRINTLN(F("Relay turned ON."));
    }
  } else if (temperature > (targetTemperature + hysteresis)) {
    PORTD &= ~(1 << PORTD5); // Turn relay OFF
    if (relayOn) {
      relayOn = false;
      DEBUG_PRINTLN(F("Relay turned OFF."));
    }
  }

  // Calculate power in Watts
  float power_W = (voltage_Vrms * (current_mA / 1000.0)); // Convert mA to A

  // Time interval per WDT interrupt, which is 8 seconds
  float deltaTime_hours = 8.0 / 3600.0; // Convert 8 seconds to hours

  // Calculate energy in Wh
  float energy_Wh = power_W * deltaTime_hours;

  // Accumulate energy in mWh
  mWhPerDay += energy_Wh * 1000.0; // Convert Wh to mWh

  // Debugging output
  DEBUG_PRINT(F("Current: "));
  DEBUG_PRINT(current_mA);
  DEBUG_PRINTLN(F(" mA"));

  DEBUG_PRINT(F("Temperature: "));
  DEBUG_PRINT(temperature / 10.0);
  DEBUG_PRINTLN(F(" C"));

  DEBUG_PRINT(F("Humidity: "));
  DEBUG_PRINT(humidity / 10.0);
  DEBUG_PRINTLN(F(" %"));

  DEBUG_PRINT(F("Power: "));
  DEBUG_PRINT(power_W);
  DEBUG_PRINTLN(F(" W"));

  DEBUG_PRINT(F("Energy increment: "));
  DEBUG_PRINT(energy_Wh);
  DEBUG_PRINTLN(F(" Wh"));

  DEBUG_PRINT(F("Total Energy per Day: "));
  DEBUG_PRINT(mWhPerDay / 1000.0);
  DEBUG_PRINTLN(F(" Wh"));
}

void initializeDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DEBUG_PRINTLN(F("SSD1306 allocation failed"));
    for (;;);
  }
  DEBUG_PRINTLN(F("Display initialized."));
  display.ssd1306_command(SSD1306_DISPLAYOFF); // Turn off display initially
}

void displaySensorData() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Display target temperature
  display.setCursor(0, 0);
  display.print(F("Set Temp: "));
  display.print(targetTemperature / 10);
  display.print(F("."));
  display.print(abs(targetTemperature % 10));
  display.print(F(" C"));

  // Display relay status
  if (relayOn) {
    display.fillCircle(120, 4, 4, SSD1306_WHITE);
  } else {
    display.drawCircle(120, 4, 4, SSD1306_WHITE);
  }

  // Display current temperature
  display.setCursor(0, 16);
  display.print(F("Temp: "));
  display.print(temperature / 10);
  display.print(F("."));
  display.print(abs(temperature % 10));
  display.print(F(" C"));

  // Display humidity
  display.setCursor(0, 28);
  display.print(F("Humidity: "));
  display.print(humidity / 10);
  display.print(F("."));
  display.print(abs(humidity % 10));
  display.println(F(" %"));

  // Display accumulated energy
  display.setCursor(0, 40);
  display.print(F("Energy/24h: "));
  display.print(mWhPerDay / 1000.0); // Convert mWh to Wh for display
  display.print(F(" Wh"));

  display.display();
}

void updateDisplay() {
  displaySensorData();
}
