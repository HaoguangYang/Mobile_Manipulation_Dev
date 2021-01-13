const int buttonPin = 11;
const int ledPin = 8;
const int rel = 4;
const int currentSensorPin = 14;

unsigned long buttonMillis = 0;
unsigned long buttonMillisDebounce = 0;
unsigned long ledMillis = 0;
unsigned long curSensMillis = 0;

const unsigned int curSenPeriod = 100;

bool buttonState = false;
bool buttonStateLast = false;
bool ledState = false;
bool relayState = false;
const long intervalButton = 3000;
unsigned short programState = 255;
const unsigned short curReportInterval = 10;
unsigned short curSenState = 0;
float curSenAvg = 460.8;  //0.9*512

void setup() {
  pinMode(buttonPin, INPUT);
  //digitalWrite(buttonPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(rel, OUTPUT);
  digitalWrite(rel,HIGH);
  Serial.begin(9600);
}

void loop(){
	char cmd;
	while (Serial.available() > 0){
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
					break;
		        case 'S':
					Serial.readBytes(&cmd, 1);
					Serial.readBytes(&tail, 1);
					if (tail == '&'){
						if (cmd=='0'){  			// valid command
							relayState = false;
							programState = 0;
              Serial.print("@S0%");
						}
					}
					break;
				default:
					break;				// stops processing the current frame.
			}
		}
	}
	unsigned long curMillis = millis();
  	pushbutton(curMillis);
  	relay();
  	currentsensor(curMillis);
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
  int buttonRead = digitalRead(buttonPin);
  if (buttonRead != buttonStateLast){
    buttonMillisDebounce = curMillis;
  }
  if (curMillis - buttonMillisDebounce > 10){
    if (buttonState != buttonRead)
      buttonState = buttonRead;
  }
  buttonStateLast = buttonRead;
  //Serial.println(buttonState);
  switch (programState){
  	case 0:
  	case 3:
  		if (buttonState){
  			programState = 1;
			buttonMillis = curMillis;
			Serial.print("@S0%");
			relayState = false;
  		}
  		break;
  	case 1:
  		if (buttonState){
  			if (curMillis - buttonMillis > intervalButton){
  				programState = 2;
  			}
  		}
  		else{
  			programState = 0;
  		}
  		break;
  	case 2:
  		if (!buttonState){
  			programState = 3;
  			Serial.print("@S1%");
  		}
  		break;
  	default:
  		break;
  }
}
 
//For Relay
void relay(){
  //Set the relay coil as low/high to activate it
  if (relayState){
  	digitalWrite(rel, HIGH);
  }
  else{
  	digitalWrite(rel, LOW);
  }
}

//For Current Sensor
void currentsensor(unsigned long curMillis){
  //Read current sensor value
  if (curMillis - curSensMillis > curSenPeriod){
  int curSen = analogRead(currentSensorPin);
  curSenAvg += (float)curSen/(float)curReportInterval;
  if (!(curSenState%curReportInterval)){
	  char buf[8];
	  sprintf(buf, "@A%04d%%", (int)curSenAvg);
    curSenAvg = 0.;
	  Serial.print(buf);
	}
  curSenState += 1;
  curSensMillis = curMillis;
  }
}

void led(unsigned long curMillis){
  if (programState == 0){
    if (curMillis - ledMillis > 1000){
      ledState = !ledState;
      ledMillis = curMillis;
      digitalWrite(ledPin, ledState);  
    }
  }
  else if (programState == 1 || programState == 255){
    digitalWrite(ledPin, LOW);
  }
  else{
    digitalWrite(ledPin,HIGH);
  }
}
