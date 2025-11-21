/*
 * Simple class for a 3 colour LED driver
 * They have internal Red Green and Blue LEDS and 4 connection pins
 * They can be common cathode, or common anode. The former has common GND pin
 * The latter has a common VCC pin
 *
 * This class offers the following methods;
 *    TricolourLED() Constructor - must specifiy 3 GPIO pins and whether common anode or cathode
 *    setState() -ON, OFF or BLINKING - blinking gives a short flash every 5 seconds (saves battery)
 *    setColour() - RED, AMBER, GREEN - traffic light colours
 */

#include <Arduino.h>
class TricolourLED; //Forward declaration

//Used by the blinker method
#define BLINK_PERIOD 3000 //- 3seconds
#define DUTY_CYCLE 20 // - 20% ON, 80% OFF


class TricolourLED {
   public:
      enum LED_COMMON { COMMON_ANODE, COMMON_CATHODE};
      enum LED_STATE { OFF, ON, BLINKING};
      enum LED_COLOUR { RED, GREEN, AMBER};
      //constructor
      TricolourLED(uint8_t p1, uint8_t p2, uint8_t p3, LED_COMMON p4) :
          red_pin(p1),
          green_pin(p2),
          blue_pin(p3),
          LED_type(p4)
      {
          if (LED_type == COMMON_CATHODE) _OnValue = 0xFF; else _OnValue = 0;
      }

      void begin() {
        //configure GPIO pins
        pinMode(red_pin, OUTPUT);
        pinMode(green_pin, OUTPUT);
        pinMode(blue_pin, OUTPUT);
        //Set up the RTOS blinker Task and suspend it for now
        xTaskCreate(blink, "Blink", 4000, (void *)this, 1, &blinkerTask);
        vTaskSuspend(blinkerTask);
        isBright = false;
      }

      void setState(LED_STATE new_state){
        if (new_state != state ) {
          state = new_state;
          refresh();
        }
      }

      void setColour(LED_COLOUR new_colour){
        if (new_colour != colour) {
          colour = new_colour;
          refresh();
        }
      }
      void setRed() {
        digitalWrite(red_pin, _OnValue);
        digitalWrite(green_pin, !_OnValue);
        digitalWrite(blue_pin, !_OnValue); 
      }
      void setGreen() {
        digitalWrite(red_pin, !_OnValue);
        digitalWrite(green_pin, _OnValue);
        digitalWrite(blue_pin, !_OnValue);
      }
      void setAmber() {
        digitalWrite(red_pin, _OnValue);
        digitalWrite(green_pin, _OnValue);
        digitalWrite(blue_pin, !_OnValue);
      }
      void setOff() {
        digitalWrite(red_pin, !_OnValue);
        digitalWrite(green_pin, !_OnValue);
        digitalWrite(blue_pin, !_OnValue);
      }
      void setOn() {
        switch (colour) {
          case RED:
            setRed();
            break;
          case GREEN:
            setGreen();
            break;
          case AMBER:
            setAmber();
            break;
          }
      }
  private:
      uint8_t red_pin, green_pin, blue_pin;
      LED_COMMON LED_type;
      LED_COLOUR colour;
      LED_STATE state;
      uint8_t _OnValue;
      bool isBright;
      TaskHandle_t blinkerTask;
      void refresh() {
//        Serial.println("refresh called");
        switch (state) {
          case ON:
            vTaskSuspend(blinkerTask);
            setOn();
             break;
          case OFF:
            vTaskSuspend(blinkerTask);
            setOff();
            break;
          case BLINKING:
            vTaskResume(blinkerTask);
            break;
        }
      }
      //This is the RTOS blinker Task. It is effectively a timer interrupt handler
      static void blink(void* pvParam) {
        TickType_t xLastWakeTime;
        TricolourLED *myLED = (TricolourLED *)pvParam;
        const TickType_t onDelay = (portTICK_PERIOD_MS * BLINK_PERIOD * DUTY_CYCLE) / 100;
        const TickType_t offDelay = (portTICK_PERIOD_MS * BLINK_PERIOD * (100 - DUTY_CYCLE)) / 100;; 
        xLastWakeTime = xTaskGetTickCount ();

        while(1) {//Task always puts iself to sleep
        //If LED is ON, turn it OFF and wait for OFFdelay period
          if (myLED->isBright) {
            myLED->isBright = false;
            myLED->setOff();
            xTaskDelayUntil(&xLastWakeTime,offDelay);
          } else {
            myLED->isBright = true;
            myLED->setOn();
            xTaskDelayUntil(&xLastWakeTime,onDelay);
          }  
       }
    }
}; 


