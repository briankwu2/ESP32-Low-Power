// Author: Brian Wu
// Written 10/13/22
// FOR ESP32
/*
Lab 3 features an additional implementation to lab 2.
We are to implement 2 more states, light-sleep mode, and deep-sleep mode.
The ESP32 now cycles between 5 states:
1. Off Mode (High Power)
2. Steady State (LED ON)
3. Flashing State
4. Light-Sleep Mode
5. Deep-Sleep Mode
*/

#define pinToPoll 0
#define PRESS_DELAY 200 // defined in microseconds
#define ledR 16
#define ledG 17
#define ledB 18

// Defines an enum STATE_T that can contain 5 states.
typedef enum
{
    STATE_OFF = 0,
    STATE_ON = 1,
    STATE_FLASHING = 2,
    STATE_LIGHT_SLEEP = 3,
    STATE_DEEP_SLEEP = 4
} STATE_T;

// Function Prototypes
static void turnLEDtoHue(uint8_t red, uint8_t green, uint8_t blue);
void IRAM_ATTR handle_state();
void IRAM_ATTR debounceHandle_state();
void print_wakeup_reason();

// Global Variables
// RTC_DATA_ATTR qualifier so that the data will be saved within the RTC memory, which means that it will be able
// to accessed in deep sleep mode
RTC_DATA_ATTR volatile static STATE_T state = STATE_OFF;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR static int numHandles = 0;
volatile unsigned long lastDebounceTime = 0;


void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(10);
    if (bootCount == 0)
    {
      Serial.println("--------------------");
      Serial.println("First bootup...");
    }

    Serial.println("\nBooting up ESP32...");
    print_wakeup_reason(); // Prints the reason for wakeup.
    Serial.print("The current state is ");
    Serial.println(state);    
    Serial.print("This device has been booted ");
    Serial.print(bootCount++);
    Serial.println(" amount of times");

    
    if (state == STATE_LIGHT_SLEEP || state == STATE_DEEP_SLEEP)
    {
        handle_state(); // If waking up from light sleep or deep sleep, change the state (because interrupt cannot be reliably accessed);
    }

    ledcAttachPin(ledR, 1); // assign CMY led pins to channels
    ledcAttachPin(ledG, 2);
    ledcAttachPin(ledB, 3);
    // initialize digital pin LED_BUILTIN as an output.

    // Initialize channels
    // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
    // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);

    // Sets up the ability to range in values from 0 - 255 in resolution.
    // Sets up the frequency as well.
    ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
    ledcSetup(2, 12000, 8);
    ledcSetup(3, 12000, 8);

    pinMode(pinToPoll, INPUT_PULLUP); // Defines a switch
    turnLEDtoHue(0, 0, 0);            // Turns the led on, but "off"
    attachInterrupt(digitalPinToInterrupt(pinToPoll), debounceHandle_state, RISING);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW); // If the button is pressed, it will go into a deep sleep.
}

void loop()
{
    // put your main code here, to run repeatedly:

    // Debugging code to make sure that the interrupt is not called several times because of switch bouncing
    // Serial.print("The State is: ");
    // Serial.println(state);
    // Serial.print("The digital Read is: ");
    // Serial.println(digitalRead(pinToPoll));
    // Serial.print("The number of times handle_state has been called is: ");
    // Serial.println(numHandles);
    // delay(500);
    switch (state)
    {
    case STATE_OFF:
        turnLEDtoHue(0, 0, 0); // Turns the LED OFF
        break;
    case STATE_ON:
        turnLEDtoHue(0, 0, 255); // Turns the LED Blue
        break;
    case STATE_FLASHING:
        turnLEDtoHue(0, 0, 0);
        while (state == STATE_FLASHING) // While the switch is not pressed (button not pressed is HIGH(1), button pressed is LOW(0))
        {
            turnLEDtoHue(0, 255, 0); // Turn green for 50ms
            delay(50);
            turnLEDtoHue(0, 0, 0); // Turn off
            delay(950);            // Wait 950ms to complete the cycle time.
        }
        break;
    case STATE_LIGHT_SLEEP:
      esp_light_sleep_start();
      break;
    case STATE_DEEP_SLEEP:
      esp_deep_sleep_start();
      break;
    default:
      ; // no default value.
    }
}

// Turns the value into RGB. Turns on the LED
static void turnLEDtoHue(uint8_t red, uint8_t green, uint8_t blue)
{
    ledcWrite(1, 255 - red);
    ledcWrite(2, 255 - green);
    ledcWrite(3, 255 - blue);
}

// Primary Interrupt function to handle switch bouncing (switch noise)
// IRAM_ATTR stores the function in flash ram so it is fast as possible
void IRAM_ATTR debounceHandle_state()
{
    // Makes it so that the actual interrupt function will not be called until a certain delay
    // has passed between the last bounce
    if ((millis() - lastDebounceTime) >= PRESS_DELAY)
    {
        handle_state();
        lastDebounceTime = millis();
    }
}

// Actual interrupt function that switches the states.
void IRAM_ATTR handle_state()
{
    switch (state)
    {
    case STATE_OFF:
        state = STATE_ON;
        break;
    case STATE_ON:
        state = STATE_FLASHING;
        break;
    case STATE_FLASHING:
        state = STATE_LIGHT_SLEEP;
        break;
    case STATE_LIGHT_SLEEP:
        state = STATE_DEEP_SLEEP;
        break;
    case STATE_DEEP_SLEEP:
        state = STATE_OFF;
        break;
    default:; // no default value.
    }
    numHandles++; // Counts the number of times this function is called for debugging purposes
    Serial.print("The number of times the handle_state function has been called is: ");
    Serial.println(numHandles);
    Serial.print("The new state is at ");
    Serial.println(state);
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

/*
*/
