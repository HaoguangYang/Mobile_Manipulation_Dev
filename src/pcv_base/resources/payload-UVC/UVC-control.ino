const int buttonPin = 11;
const int ledPin = 8;
const int rel = 3;
const int currentSensorPin = 14;

int buttonState;
long buttonMillis = 0;
long ledMillis = 0;
bool ledState = false;
const long intervalButton = 3000;
unsigned short programState = 255;
bool relayState = false;
const unsigned short curReportInterval = 10;
unsigned short curSenState = 0;
float curSenAvg = 512.;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(rel, OUTPUT);
  digitalWrite(rel,HIGH);
  Serial.begin(9600);
}

void loop(){
	char cmd;

	if (Serial.available() > 0){
		Serial.readBytes(&cmd, 1);
		if (cmd == '$'){	// a valid frame
			Serial.readBytes(&cmd, 1);	// read in the command byte.
      char tail;
			switch (cmd){
				case 'L':				// command for the light
					Serial.readBytes(&cmd, 1);
					Serial.readBytes(&tail, 1);
					if (tail == '&' && cmd=='1'){	// valid command
						relayState = true;
					}
					else {
						relayState = false;			// just to be safe
					}
        case 'S':
          Serial.readBytes(&cmd, 1);
          Serial.readBytes(&tail, 1);
          if (tail == '&' && cmd=='0'){  // valid command
            relayState = false;
            programState = 0;
          }
				default:
					break;				// stops processing the current frame.
			}
		}
	}

  	pushbutton();
  	relay();
  	currentsensor();
    //digitalWrite(buttonPin, LOW);
    //Serial.print(relayState);
    //Serial.println(digitalRead(buttonPin));
    //Serial.println(programState);
  	delay(100);
}

//For Push Button
void pushbutton()
{
  unsigned long currentMillis = millis();
  buttonState = digitalRead(buttonPin);
 
  if (buttonState == HIGH && (programState == 0 || programState == 3)) {
    buttonMillis = currentMillis;
    //write the program to start the Trajecotry of the omni-veyor
    programState = 1;
    Serial.print("@S0%");
    digitalWrite(ledPin, LOW);
  }
  else if (programState == 1){		// HIGH 1 OR LOW 1
  	if (buttonState == LOW){		// Button is Released
  		programState = 0; //reset
  		Serial.print("@S0%");
      digitalWrite(ledPin, LOW);
  	}
  	else{							// HIGH 1
  		if(currentMillis - buttonMillis > intervalButton){	// LONG PRESS
  			programState = 2;		// S1 is sent.
  			Serial.print("@S1%");
        digitalWrite(ledPin, HIGH);
  		}
  	}
  }
  else if (programState == 2){							// 2
  	if (buttonState == LOW){		// Button is Released
  		programState = 3;
  	}
  }
 }
 
//For Relay
void relay(){
  //Set the relay coil as high to activate it
  if (relayState){
  	digitalWrite(rel, LOW);
  }
  else{
  	digitalWrite(rel, HIGH);
  }
}

//For Current Sensor
void currentsensor(){
  //Read current sensor value
  int curSen = analogRead(currentSensorPin);
  curSenAvg += (float)curSen/(float)curReportInterval;
  if (!(curSenState%curReportInterval)){
	  char buf[8];
	  sprintf(buf, "@A%04d%%", (int)curSenAvg);
    curSenAvg = 0.;
	  Serial.print(buf);
	}
	curSenState += 1;
}

void led(){
  if (programState == 0){
    unsigned long currentMillis = millis();
    if (currentMillis - ledMillis > 1000){
      ledState = !ledState;
      digitalWrite(ledPin, ledState);  
    }
  }
}
