const int inputPins[7]={0,1,2,3,4,5,6};
const int outputPins[7]={7,9,10};
const int analogPins[6]={14,15,16,17,18,19};
const int buttonPin = 8;
const int ledPin = 11;
const int chargingPin = 12;
const int chargingFaultPin = 13;

unsigned long buttonMillis = 0;
unsigned long buttonMillisDebounce = 0;
unsigned long ledMillis = 0;
unsigned long analogSensMillis = 0;

const unsigned int analogSenPeriod = 100;

bool buttonState = false;
bool buttonStateLast = false;
bool ledState = false;
const long intervalButton = 3000;
unsigned short programState = 255;
unsigned int analogSen[6] = {0};

void setup() {
    int i;
    for (i = 0; i < sizeof(inputPins)/sizeof(int); i++){
        pinMode(inputPins[i], INPUT);
    }
    for (i = 0; i < sizeof(outputPins)/sizeof(int); i++){
        pinMode(outputPins[i], OUTPUT);
        digitalWrite(outputPins[i], LOW);
    }
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    pinMode(chargingPin, OUTPUT);
    pinMode(chargingFaultPin, INPUT_PULLUP);
    digitalWrite(ledPin, LOW);
    digitalWrite(chargingPin, LOW);
    Serial.begin(9600);
} 

void loop(){
  /*
   * Communication Format:
   * Computer to Arduino: $cmd&: command: 1 byte command type + customizable payload
   * Arduino to Computer: @ret%: return: 1 byte indicator + customizable payload
   */
  char cmd;
  while (Serial.available() > 0){
    Serial.readBytes(&cmd, 1);
    if (cmd == '$'){  // a valid frame
      Serial.readBytes(&cmd, 1);  // read in the command byte.
          char tail;
      switch (cmd){
        case 'Q':       // command for query analog values
          Serial.readBytes(&cmd, 1);
          Serial.readBytes(&tail, 1);
          if (tail == '&'){ // valid command
            int which = cmd-48; // ASCII to int
            char buf[8];
            sprintf(buf, "@A%04d%%", (int)analogSen[which]);
            Serial.print(buf);
          }
          break;
        case 'S':               // command for setting the button state machine from the computer
          Serial.readBytes(&cmd, 1);
          Serial.readBytes(&tail, 1);
          if (tail == '&'){   // valid command
            int state = cmd-48; // ASCII to int
            //Serial.println(cmd);
            if (state == 0 || state == 3 || state == 5){
              programState = state;
              char buf[5];
              sprintf(buf, "@S%1d%%", state);
              Serial.print(buf);  // echo the state setting
            }
          }
          break;
        case 'C':                // command for setting the charging state from th computer
          Serial.readBytes(&cmd, 1);
          Serial.readBytes(&tail, 1);
          if (tail == '&'){   // valid command
            int state = cmd-48; // ASCII to int
            if (state == 1 || state == 0){
              digitalWrite(chargingPin, HIGH*state);
              char buf[5];
              sprintf(buf, "@C%1d%%", state);
              Serial.print(buf);  // echo the state setting
            }
            else if (cmd == 'Q'){
              state = !digitalRead(chargingFaultPin);
              char buf[5];
              sprintf(buf, "@F%1d%%", state>0? 1 : 0);
              Serial.print(buf);  // echo the state setting
            }
          }
          break;
        default:
          break;        // stops processing the current frame.
      }
    }
  }
  unsigned long curMillis = millis();
    pushbutton(curMillis);
    //relay();
    //currentsensor(curMillis);
    led(curMillis);
    //digitalWrite(buttonPin, LOW);
    //Serial.print(relayState);
    //Serial.println(digitalRead(buttonPin));
    //Serial.println(programState);
    //delay(100);
}

//For Push Button
void pushbutton(unsigned long curMillis)
{
    int buttonRead = !digitalRead(buttonPin);
    //Serial.println(buttonRead);
    if (buttonRead != buttonStateLast){
        buttonMillisDebounce = curMillis;
    }
    if (curMillis - buttonMillisDebounce > 10){
        if (buttonState != buttonRead)
        buttonState = buttonRead;
    }
    buttonStateLast = buttonRead;
    //Serial.println(buttonState);
    // MAIN STATE MACHINE CONTROLLED BY THE BUTTON
    /* USAGE:
     * Robot Ready (LED Slow flashing) --> Long Press Button (LED Lit UP) --> Running State (LED Solid On)
     * Robot Ready (LED Slow flashing) --> Short Press Button --> Robot Ready (LED Slow flashing)
     * Running State --> Press Button --> Paused State (LED Fast flashing)
     * Running State --> Software-Generated S3 Cmd --> Paused State (when goal is reached)
     * Paused State --> Short Press Button --> Resume Running State (LED Solid On)
     * Paused State --> Long Press Button (LED Goes OUT) --> Quit: back to Robot Ready
     */
    switch (programState){
        case 0:         // Pre-launch state: robot is ready
            if (buttonState){
                programState = 1;
                buttonMillis = curMillis;
            }
            break;
        case 1:         // intermediate state for determining long press that transits state 0 to state 2
            if (buttonState){
                if (curMillis - buttonMillis > intervalButton){
                    programState = 2;
                }
          }
            else{
               programState = 0;
            }
            break;
        case 2:         // Ready-to-launch state: robot will register a new nav goal when button is released
            if (!buttonState){
                programState = 3;
                Serial.println("@S3%");
            }
            break;
        case 3:         // Active state: robot has a new nav target to achieve
            if (buttonState){
                programState = 4;
                Serial.println("@S5%");
            }
            break;
        case 4:         // Pause state: robot is paused manually
            if (!buttonState){
                programState = 5;
            }
            break;
        case 5:         // Pause state: Goal reached / paused.
            if (buttonState){
                programState = 6;
                buttonMillis = curMillis;
            }
            break;
        case 6:         // Intermediate state for resume/shutdown branch
            if (buttonState){
                if (curMillis - buttonMillis > intervalButton){
                    programState = 7;
                }
            }
            else{
               programState = 3;
               Serial.println("@S3%");
            }
            break;
        case 7:         // Ready-to-shutdown state: robot will deactivate (state 0) when button released.
            if (!buttonState){
                programState = 0;
                Serial.println("@S0%");
            }
            break;
        default:        // robot is not ready.
            break;
    }
    //Serial.println(programState);
}

//For Analog Sensor
void analogSensor(unsigned long analogMillis){
    //Read current sensor value
    if (analogMillis - analogSensMillis > analogSenPeriod){
        int i;
        for (i = 0; i < sizeof(analogPins)/sizeof(int); i ++){
            analogSen[i] = analogRead(analogPins[i]);
        }
        analogSensMillis = analogMillis;
    }
}

void led(unsigned long curMillis){
    switch (programState){
        case 0:
            if (curMillis - ledMillis > 1000){
                ledState = !ledState;
                ledMillis = curMillis;
                digitalWrite(ledPin, ledState);  
            }
            break;
        case 5:
            if (curMillis - ledMillis > 500){
                ledState = !ledState;
                ledMillis = curMillis;
                digitalWrite(ledPin, ledState);  
            }
            break;
        case 2:
        case 3:
        case 6:
            digitalWrite(ledPin,HIGH);
            break;
        default:
            digitalWrite(ledPin,LOW);
            break;
    }
}
