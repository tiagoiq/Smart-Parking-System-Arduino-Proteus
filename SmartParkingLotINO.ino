#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// Define the pins for the buttons and servos
#define buttonPin 2
#define servoPin 3
#define buttonPin2 4
#define servoPin2 5

// Initialize two Servo objects to control two servos
Servo servo;
Servo servo2;

// DECLARATION OF THE ADDRESSES SET IN PCF8574
// Initialize two I2C LCD screens at different I2C addresses
LiquidCrystal_I2C lcd1(0x27, 20, 4);  // LCD1 at address 0x27 with 20 columns and 4 rows
LiquidCrystal_I2C lcd2(0x20, 20, 4);  // LCD2 at address 0x20 with 20 columns and 4 rows

// Initialize an LCD without I2C interface using specific pins
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

// Configuration of pins for parking space sensors
const int sensores[] = {25, 26, 27, 28, 29};  // Pins for the sensors
const int numSensores = 5;                    // Total number of sensors
const int ledRojo = 22;                        // Pin for the red LED
const int ledAmarillo = 23;                    // Pin for the yellow LED
const int ledVerde = 24;                       // Pin for the green LED
const int ledRojoBarreraEntrada = 30;          // Pin for the red LED at the entry barrier
const int ledVerdeBarreraEntrada = 31;         // Pin for the green LED at the entry barrier
const int ledRojoBarreraSalida = 32;           // Pin for the red LED at the exit barrier
const int ledVerdeBarreraSalida = 33;          // Pin for the green LED at the exit barrier

int luzRele = 6;                               // Pin for the light relay
int a, b, c;                                   // Auxiliary variables

void setup() {
  Serial.begin(9600);                          // Start serial communication
  servo.attach(servoPin);                      // Attach the main servo pin
  servo2.attach(servoPin2);                    // Attach the second servo pin

  pinMode(luzRele, OUTPUT);                    // Set the relay pin as output
  pinMode(buttonPin, INPUT_PULLUP);            // Set button 1 as input with pull-up resistor
  pinMode(buttonPin2, INPUT_PULLUP);           // Set button 2 as input with pull-up resistor

  pinMode(ledRojo, OUTPUT);                    // Set red LED as output
  pinMode(ledAmarillo, OUTPUT);                // Set yellow LED as output
  pinMode(ledVerde, OUTPUT);                   // Set green LED as output
  pinMode(ledRojoBarreraEntrada, OUTPUT);      // Set entry barrier red LED as output
  pinMode(ledVerdeBarreraEntrada, OUTPUT);     // Set entry barrier green LED as output
  pinMode(ledRojoBarreraSalida, OUTPUT);       // Set exit barrier red LED as output
  pinMode(ledVerdeBarreraSalida, OUTPUT);      // Set exit barrier green LED as output

  lcd.begin(20, 4);                            // Initialize the standard LCD with 20 columns and 4 rows

  // Set the sensor pins as inputs
  for (int i = 0; i < numSensores; i++) {
    pinMode(sensores[i], INPUT);               // Set each sensor as input
  }

  lcd1.init();                                 // Initialize the first I2C LCD
  lcd2.init();                                 // Initialize the second I2C LCD

  lcd1.backlight();                            // Turn on the backlight of the first LCD
  delay(300);                                  
  lcd2.backlight();                            // Turn on the backlight of the second LCD
  delay(300);

  // Welcome messages on both LCDs
  lcd1.setCursor(0, 0);
  lcd1.print("WELCOME");
  lcd1.setCursor(0, 2);
  lcd1.print("Press the Button");
  lcd1.setCursor(0, 3);
  lcd1.print("to open the barrier");
  delay(1000);

  lcd2.setCursor(0, 0);
  lcd2.print("Goodbye!");
  lcd2.setCursor(0, 2);
  lcd2.print("Press the Button");
  lcd2.setCursor(0, 3);
  lcd2.print("to open the barrier");
  delay(1000);
}

void loop() {
  int posi = 90;                               // Initial position of the first servo
  int buttonState = digitalRead(buttonPin);    // Read the state of button 1
  int posi2 = 90;                              // Initial position of the second servo
  int buttonState2 = digitalRead(buttonPin2);  // Read the state of button 2

  // Control of the first servo when button 1 is pressed
  
  int espaciosOcupados = 0;
  // Count how many sensors are activated (occupied spaces)
  for (int i = 0; i < numSensores; i++) {
    if (digitalRead(sensores[i]) == HIGH) { // If the sensor is activated
      espaciosOcupados++;
    }
  }

  int espaciosDisponibles = numSensores - espaciosOcupados;

  // Control of the LEDs based on the number of available spaces
  if (espaciosDisponibles == 0) {
    digitalWrite(ledRojo, HIGH);     // Red LED if no spaces are available
    digitalWrite(ledAmarillo, LOW);
    digitalWrite(ledVerde, LOW);
  } else if (espaciosDisponibles <= 2) {
    digitalWrite(ledAmarillo, HIGH); // Yellow LED if few spaces are available
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledVerde, LOW);
  } else {
    digitalWrite(ledVerde, HIGH);    // Green LED if many spaces are available
    digitalWrite(ledRojo, LOW);
    digitalWrite(ledAmarillo, LOW);
  }

  a = analogRead(A0);                // Read the light sensor value
  b = map(a, 0, 1023, 0, 255);       // Map the light value
  Serial.println(b);
  if (b < 220) {                     // If the light is low, turn on the relay
    digitalWrite(luzRele, HIGH);
  }
  if (b > 220) {                     // If the light is high, turn off the relay
    digitalWrite(luzRele, LOW);
  }

  // Display the number of available spaces on the standard LCD
  lcd.setCursor(0, 0);
  lcd.print("Available Spaces");
  lcd.setCursor(0, 1);
  lcd.print("Available: ");
  lcd.print(espaciosDisponibles);     // Display the number of spaces
  delay(2000);
  lcd.clear();                        // Clear the screen to update

//If the entry button for parking is pressed, move the first servo from 0° to 90° and back, as long as there is at least 1 space available
//A red LED will turn on when the barrier stays down and a green LED will turn on when the barrier is opening and stays open.
digitalWrite(ledRojoBarreraEntrada, HIGH);
if (buttonState == HIGH) {
    servo.write(posi);
  digitalWrite(ledRojoBarreraEntrada, LOW);
  
    for (posi = 90; posi >= 0; posi -= 1 & espaciosDisponibles >= 1 ) {
      servo.write(posi);
      delay(27);
      Serial.println(posi);
      digitalWrite(ledVerdeBarreraEntrada, HIGH);
    }
    for (posi = 0; posi <= 90; posi += 1) {
      servo.write(posi);
      delay(27);
      Serial.println(posi);
      digitalWrite(ledVerdeBarreraEntrada, HIGH);
    }
    digitalWrite(ledVerdeBarreraEntrada, LOW);
    }
  

  //If the exit button is pressed, move the second servo from 0° to 90° and back.
  //A red LED will turn on when the barrier stays down and a green LED will turn on when the barrier is opening and stays open.
  digitalWrite(ledRojoBarreraSalida, HIGH);
  if (buttonState2 == HIGH) {
    servo2.write(posi2);
    digitalWrite(ledRojoBarreraSalida, LOW);

    // Move the second servo from 0° to 90° and back
    for (posi2 = 90; posi2 >= 0; posi2 -= 1) {
      servo2.write(posi2);
      delay(27);
      Serial.println(posi2);
      digitalWrite(ledVerdeBarreraSalida, HIGH);
    }
    for (posi2 = 0; posi2 <= 90; posi2 += 1) {
      servo2.write(posi2);
      delay(27);
      Serial.println(posi2);
    }
    digitalWrite(ledVerdeBarreraSalida, LOW);
  }

}
