// This #include statement was automatically added by the Particle IDE.
#include "FiniteStateMachine/FiniteStateMachine.h"

// This #include statement was automatically added by the Particle IDE.
#include "elapsedMillis/elapsedMillis.h"

// This #include statement was automatically added by the Particle IDE.
#include "NCD2Relay/NCD2Relay.h"

// This #include statement was automatically added by the Particle IDE.
#include "HttpClient/HttpClient.h"

// This #include statement was automatically added by the Particle IDE.
#include "CE_BME280/CE_BME280.h"

// This #include statement was automatically added by the Particle IDE.
#include "Fob_Alarm/Fob_Alarm.h"

// This #include statement was automatically added by the Particle IDE.
#include "Fob_Alarm/KeyFob.h"

#include "application.h"

// A1332 I2C address is 0x0C(12)
#define Addr 0x0C

#define INFLUXDB_HOST "server"
#define INFLUXDB_PORT 8086
#define INFLUXDB_DB "12345"

HttpClient http;

// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
    { "Host", INFLUXDB_HOST },
    { "Connection", "close" },
    { "Accept" , "application/json" },
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};
    
bool sendInflux(String payload) {   
    http_request_t     request;
    http_response_t    response;
    
    request.hostname = INFLUXDB_HOST;
    request.port     = INFLUXDB_PORT;
    request.path     = "/write?db=" + String(INFLUXDB_DB);
    request.body     = payload;
   
    http.post(request, response, headers);
    
    if (response.status == 204) {
        return true;
        Serial.println("Success");

    } else {
        return false;
        Serial.println("Error");
    }
}
 
 
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

NCD2Relay relayController;

SYSTEM_MODE(AUTOMATIC);

unsigned long interval = 10 * 60000;
unsigned long previousMillis = 0;
CE_BME280 bme; // I2C


void processAngle(int vals[]);


int triggerRelay(String command);

bool tripped[6];

int debugTrips[6];

int minTrips = 5;

int count = 0;
float oldAngle = 0.0;
float newAngle = 0.0;

//Class Objects
KeyFob fob;

bool checkFob = false;
int alarmReceiveConfirm(String data);

bool state;

void garageStatus()
{
  state = true;
}

void subscribeHandler(const char* event, const char* data) {

	// data set to the device ID that wants to sleep.
    String id = Particle.deviceID();
    Serial.println("Do Button command received");
    // see if this was a notification from this core that we can sleep
    if (!strcmp(data, id.c_str())) {
        Serial.println("Toggle relays");
        //triggerRelay("turnonallrelays");
        //relayController.toggleRelay(1);
        //relayController.toggleRelay(2);
        relayController.turnOnRelay(1);
		delay(500);
	    relayController.turnOffRelay(1);
    }
    else
    {
    	Serial.println("deviceID didn't match");
    }
}

/* This function is called once at start up ----------------------------------*/
void setup()
{
    Particle.subscribe("Garage_Door", subscribeHandler, "27001c000347343337373737");  // listen for events

	Particle.function("controlRelay", triggerRelay);
	//
	//Setup communication to KeyFob interface module
    attachInterrupt(A3, evalFob, CHANGE);
    
    state = false;
    pinMode(D3, INPUT_PULLUP);
    attachInterrupt(D3, garageStatus, CHANGE);
    
    // Initialise I2C communication as Master
    Wire.begin();
    
   	Serial.begin(115200);
	relayController.setAddress(0,0,0);
	
	if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
	
}

/* This function loops forever --------------------------------------------*/
void loop()
{   
    newAngle = readAngle();
    //Serial.println(readAngle());
    if(oldAngle > 359.5 && newAngle < 0.5){
        count++;
    } else if(oldAngle < 0.5 && newAngle > 359.5) {
        count--;
    }
    
    
    oldAngle = newAngle;
    Serial.println(count);

    unsigned long currentMillis = millis();
  
    if(currentMillis - previousMillis >= interval || state == true) {
        
        String string = readBME280();
        
        if(digitalRead(D3) == HIGH)
        {
            string += "1";
        } else {
            string += "0";
        }
        
        sendInflux(string);
 
        Particle.publish("Garage_Door", string, 60, PRIVATE);
        
        state = false;
        
        delay(250);
        
        previousMillis = currentMillis;
    }

        
        
    
    //If an interrupt has fired on A3 which is connected to the fob module then a button has been pressed or released so we need to determine what button was pressed or released.
    if(checkFob){
		checkFob = false;
		delay(10);
		fob.evalFob();
		String action;
		switch(fob.recentAction){
		case(fob.button1press):
			Serial.println("Button 1 pressed");
		    action = "Button1Press";
		break;
		case(fob.button2press):
			Serial.println("Button 2 pressed");
		    action = "Button2Press";
		break;
		case(fob.button3press):
			Serial.println("Button 3 pressed");
		    action = "Button3Press";
		break;
		case(fob.button4press):
			Serial.println("Button 4 pressed");
		    action = "Button4Press";
		break;
		case(fob.button5press):
			Serial.println("Button 5 pressed");
		    action = "Button5Press";
		break;
		case(fob.button6press):
			Serial.println("Button 6 pressed");
		    action = "Button6Press";
		break;
		case(fob.button7press):
			Serial.println("Button 7 pressed");
		    action = "Button7Press";
		break;
		case(fob.button8press):
			Serial.println("Button 8 pressed");
		    action = "Button8Press";
		break;
		}
		//Publish what happened to the Particle cloud and make sure the event publish to the cloud worked.  We are monitoring a 1 button key fob remote.
		//This button mounts as button 5 in software so we monitor Button5Press.
		if(action.equalsIgnoreCase("Button2Press")){
		    if(Particle.publish("eventAlarm","Alarm",60,PRIVATE)){
                relayController.turnOnRelay(1);
		        delay(500);
	            relayController.turnOffRelay(1);
		    }
		}
		if(action.equalsIgnoreCase("Button4Press")){
		    if(Particle.publish("eventAlarm","Alarm",60,PRIVATE)){
                relayController.toggleRelay(2);
		        //Let the user know the alarm has been sent
		        delay(300);
		    }
		}
		

	}
	

}

int triggerRelay(String command){
	if(command.equalsIgnoreCase("turnonallrelays")){
		relayController.turnOnAllRelays();
		return 1;
	}
	if(command.equalsIgnoreCase("turnoffallrelays")){
		relayController.turnOffAllRelays();
		return 1;
	}
	if(command.startsWith("setBankStatus:")){
		int status = command.substring(14).toInt();
		if(status < 0 || status > 255){
			return 0;
		}
		Serial.print("Setting bank status to: ");
		Serial.println(status);
		relayController.setBankStatus(status);
		Serial.println("done");
		return 1;
	}
	//Relay Specific Command
	int relayNumber = command.substring(0,1).toInt();
	Serial.print("relayNumber: ");
	Serial.println(relayNumber);
	String relayCommand = command.substring(1);
	Serial.print("relayCommand:");
	Serial.print(relayCommand);
	Serial.println(".");
	if(relayCommand.equalsIgnoreCase("on")){
		Serial.println("Turning on relay");
		relayController.turnOnRelay(relayNumber);
		Serial.println("returning");
		return 1;
	}
	if(relayCommand.equalsIgnoreCase("off")){
		relayController.turnOffRelay(relayNumber);
		return 1;
	}
	if(relayCommand.equalsIgnoreCase("toggle")){
		relayController.toggleRelay(relayNumber);
		return 1;
	}
	if(relayCommand.equalsIgnoreCase("momentary")){
		relayController.turnOnRelay(relayNumber);
		delay(300);
		relayController.turnOffRelay(relayNumber);
		return 1;
	}
	return 0;
	
	
	
}

void evalFob(){
	Serial.println("Fob button pressed");
	checkFob = true;
}

String readBME280() {

    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pres = bme.readPressure() / 100;
    
    //bool sent = Particle.publish("Garage_Door", String::format("{\"Temperatur\":%.2f,\"Feuchtigkeit\":%.2f,\"Luftdruck\":%.2f}", temp, hum, pres), 60, PRIVATE); 
    String data = "sensors,Location=Sonnberg Temp=" + String(temp, 2) + ",Hum=" + String(hum, 2) + ",Pres=" + String(pres,2) + ",Status=";
    

    return data;
}



float readAngle() {
    unsigned int data[2];

    // Start I2C Transmission 
    Wire.beginTransmission(Addr);
    // Stop I2C Transmission 
    Wire.endTransmission();

    // Request 2 byte of data
    Wire.requestFrom(Addr, 2);
  
    // Read 2 bytes of data
    // raw_adc msb, raw_adc lsb 
    if(Wire.available() == 2)
    {
      data[0] = Wire.read();
       data[1] = Wire.read();
     }

    // Checking valid data
    while((data[0] == 0) && (data[1] == 0))
     {
        // Request 2 byte of data
        Wire.requestFrom(Addr, 2);
  
        // Read 2 bytes of data
        // raw_adc msb, raw_adc lsb 
        if(Wire.available() == 2)
        {
            data[0] = Wire.read();
            data[1] = Wire.read();
        }
    }
    
    // Convert the data to 12-bits
    int raw_adc = ((data[0] & 0x0F) * 256) + (data[1] & 0xFF);
    float angle = (raw_adc * 360.0) / 4096.0;
    
    return angle;
}

