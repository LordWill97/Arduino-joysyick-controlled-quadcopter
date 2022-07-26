// For radio signal receiving.
// See: https://www.arduino.cc/reference/en/libraries/rf24/
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

// For sending PPM signal to flight controller.
// See: https://www.arduino.cc/reference/en/libraries/ppmencoder/
#include <PPMEncoder.h>

#define CE_PIN        7
#define CSN_PIN       8
//#define THROTTLE_PIN  3
//#define YAW_PIN       5
#define PPM_PIN       3
#define PPM_MAX       2000
#define PPM_MIN       1000

#define DEBUG_PRINT_INTERVAL 1000 // milliseconds

unsigned long last_debug_print_time = 0;

// Create an object for the nRF24L01 transceiver
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
Signal previous_signal;

uint8_t addresses[][6] = {"1Node", "2Node"};

void setup()
{
  Serial.begin(9600);
  setUpPpmEncoder();
  setUpRadio();
}

// Sets all 8 PPM channels to zero. As we only use the first
// six channels (0-5) to send our signals the highest two
// channels will stay at zero.
void setUpPpmEncoder() {
  ppmEncoder.begin(PPM_PIN);
  for (int i = 0; i < 8; i++) {
    ppmEncoder.setChannel(i, PPM_MIN);
  }
}

void setUpRadio() {

  // Check that the radio is correctly connected and up and running
  if (!radio.begin()) {
    Serial.println("Radio hardware is not responding!!");
    while (1) {} // hold in infinite loop
  } else {
    Serial.print("Radio hardware is responding");
    if (radio.isChipConnected()) {
      Serial.println(" (chip is connected)");
    } else {
      Serial.println(" (chip is not connected!)");
    }
  }

  radio.setChannel(115);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS) ;
  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(signal));
  radio.openReadingPipe(1, addresses[0]);
  radio.startListening();

}

void receiveJoyStickSignal() {
  if (radio.available())
  {
    while (radio.available())
    {
      radio.read(&signal, sizeof(signal));
    }
  }
}

void printJoyStickInfo()
{
  unsigned long current_time = millis();
  if (last_debug_print_time + DEBUG_PRINT_INTERVAL > current_time) {
    return;
  }
  Serial.println("Reading   Signal");
  Serial.println("-------   ------");
  Serial.print("Throttle: "); Serial.println(signal.throttle);
  Serial.print("Yaw:      "); Serial.println(signal.yaw);
  Serial.print("Pitch:    "); Serial.println(signal.pitch);
  Serial.print("Roll:     "); Serial.println(signal.roll);
  Serial.print("Button1:  "); Serial.println(signal.aux1);
  Serial.print("Button2:  "); Serial.println(signal.aux2);
  Serial.println();
  last_debug_print_time = current_time;
}

void updateFlighController() {
  if (differentSignal()) {
    ppmEncoder.setChannel(0, map(signal.throttle, 0, 1023, PPM_MIN, PPM_MAX));
    ppmEncoder.setChannel(1, map(signal.yaw,      0, 1023, PPM_MIN, PPM_MAX));
    ppmEncoder.setChannel(2, map(signal.pitch,    0, 1023, PPM_MIN, PPM_MAX));
    ppmEncoder.setChannel(3, map(signal.roll,     0, 1023, PPM_MIN, PPM_MAX));
    ppmEncoder.setChannel(4, signal.aux1 == 1 ? PPM_MAX : PPM_MIN);
    ppmEncoder.setChannel(5, signal.aux2 == 1 ? PPM_MAX : PPM_MIN);
    rememberSignal();
  }
}

void rememberSignal() {
  previous_signal = signal;
}

bool differentSignal() {
  return previous_signal.throttle != signal.throttle ||
         previous_signal.yaw != signal.yaw ||
         previous_signal.pitch != signal.pitch ||
         previous_signal.roll != signal.roll ||
         previous_signal.aux1 != signal.aux1 ||
         previous_signal.aux2 != signal.aux2;
}

void loop()
{
  receiveJoyStickSignal();
  printJoyStickInfo();
  updateFlighController();
}
