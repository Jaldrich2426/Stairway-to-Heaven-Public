/*
 * IRremoteExtensionTest.cpp
 * Simple test using the IRremoteExtensionClass.
 */
#include <Arduino.h>

//#define RAW_BUFFER_LENGTH  750  // 750 is the value for air condition remotes.

#include "PinDefinitionsAndMore.h"

#include <IRremote.hpp>

#include "IRremoteExtensionClass.h"

/*
 * Create the class, which itself uses the IRrecv class from IRremote
 */
IRExtensionClass IRExtension(&IrReceiver);

void setup() {
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) || defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
// Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);


    Serial.print(F("Ready to receive IR signals at pin "));
#if defined(ARDUINO_ARCH_STM32) || defined(ESP8266)
    Serial.println(IR_RECEIVE_PIN_STRING);
#else
    Serial.println(IR_RECEIVE_PIN);
#endif

}

void loop() {
    if (IrReceiver.decode()) {
        IrReceiver.printIRResultShort(&Serial);
        IRExtension.resume(); // Use the extended function
    }
    delay(100);
}
