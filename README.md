# Arduino-joysyick-controlled-quadcopter
Transmitter code:
// For outputing formatted calibrarion information.
// See: https://www.arduino.cc/reference/en/libraries/libprintf/
#include <LibPrintf.h>

// For radio transmission:
// See: https://www.arduino.cc/reference/en/libraries/rf24/
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#define LEFT_JOY_X        A0 // analog pin connected to X output
#define LEFT_JOY_Y        A1 // analog pin connected to Y output
#define LEFT_JOY_SWITCH   6  // digital pin connected to switch output

#define RIGHT_JOY_X       A2 // analog pin connected to X output
#define RIGHT_JOY_Y       A3 // analog pin connected to Y output
#define RIGHT_JOY_SWITCH  5  // digital pin connected to switch output

#define CE_PIN   7
#define CSN_PIN  8

// How often to send data (in milliseconds)
#define UPDATE_INTERVAL 10
// Howe often to print out debug information (in milliseconds)
#define DEBUG_PRINT_INTERVAL 1000

unsigned long last_debug_print_time = 0;

//
// Elegoo Uno R3 SPI Pins
// 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK).
//

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

struct signal
{
  int throttle;
  int yaw;
  int pitch;
  int roll;
  byte aux1;
  byte aux2;
};

typedef struct signal Signal;
Signal signal;
Signal raw_data;

uint8_t addresses[][6] = {"1Node", "2Node"};

void setup()
{
  Serial.begin(9600);
  setupJoyStick();
  setUpRadio();
}

void setupJoyStick() {
  pinMode(LEFT_JOY_SWITCH, INPUT_PULLUP);
  pinMode(RIGHT_JOY_SWITCH, INPUT_PULLUP);
}

void setUpRadio() {

  // Check that the radio is correctly connected and up and running
  if (!radio.begin()) {
    Serial.println(F("Radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  } else {
    Serial.print(F("Radio hardware is responding"));
    if (radio.isChipConnected()) {
      Serial.println(F(" (chip is connected)"));
    } else {
      Serial.println(F(" (chip is not connected!)"));
    }
  }

  radio.setChannel(115);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(signal));
  radio.stopListening();
  radio.openWritingPipe(addresses[0]);

}

// Map raw joystick reading into a standard range by specifying the
// joystick's particular maximum, minimum and resting (middle) reading.
int mapJoystickValue(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 512);
  else
    val = map(val, middle, upper, 512, 1023);
  return ( reverse ? 1023 - val : val );
}

void readJoystickInfo()
{

  //  Raw data values are the values read directly from
  //  the joysticks before adjusting them.
  raw_data.throttle = analogRead(LEFT_JOY_X);
  raw_data.yaw = analogRead(LEFT_JOY_Y);
  raw_data.pitch = analogRead(RIGHT_JOY_X);
  raw_data.roll = analogRead(RIGHT_JOY_Y);
  raw_data.aux1 = HIGH - digitalRead(LEFT_JOY_SWITCH);
  raw_data.aux2 = HIGH - digitalRead(RIGHT_JOY_SWITCH);


  // Each joystick value _should_ vary between a minimun of 0 and a
  // maximum of 1023 whilst resting in the middle at 512 however in
  // practice these values vary from one joystick to the next.
  //
  // Here we make an adjustment for each joystick's variances based
  // on the values observed in the output from printJoyStickInfo
  // and map their values to a standard 0-1023 with a 512 midpoint.
  signal.throttle = mapJoystickValue(raw_data.throttle, 23, 508, 1005, true);
  signal.yaw = mapJoystickValue(raw_data.yaw,     35, 506, 1016, true);
  signal.pitch = mapJoystickValue(raw_data.pitch, 22, 515, 1019, true);
  signal.roll = mapJoystickValue(raw_data.roll,   19, 522, 1016, true);

  // AUX1 and AUX2 are just on or off.
  signal.aux1 = raw_data.aux1;
  signal.aux2 = raw_data.aux2;

}

void printJoyStickInfo()
{
  unsigned long current_time = millis();
  // If we've already output information within the last debug
  // print interval time then just return.
  if (last_debug_print_time + DEBUG_PRINT_INTERVAL > current_time) {
    return;
  }
  // Otherwise print the current set of raw and adjusted readings.
  printf("Reading   Raw  Signal\n");
  printf("-------   ---  ------\n");
  printf("Throttle  %3u    %3u\n", raw_data.throttle, signal.throttle);
  printf("Yaw       %3u    %3u\n", raw_data.yaw,      signal.yaw);
  printf("Pitch     %3u    %3u\n", raw_data.pitch,    signal.pitch);
  printf("Roll      %3u    %3u\n", raw_data.roll,     signal.roll);
  printf("Button1   %3u    %3u\n", raw_data.aux1,     signal.aux1);
  printf("Button2   %3u    %3u\n", raw_data.aux2,     signal.aux2);
  Serial.println();
  last_debug_print_time = current_time;
}

void transmitJoyStickInfo()
{
  radio.write(&signal, sizeof(signal));
}

void loop()
{
  // Time the current loop starts.
  unsigned long time = millis();

  readJoystickInfo();
  transmitJoyStickInfo();
  printJoyStickInfo();

  // Time elapsed whilst we've been reading, transmitting
  // and printing the joystick information.
  time = millis() - time;

  // sleep till it is time for the next update
  if (time < UPDATE_INTERVAL)
    delay(UPDATE_INTERVAL  - time);

}
