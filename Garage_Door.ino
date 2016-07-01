// This #include statement was automatically added by the Particle IDE.
#include "FiniteStateMachine/FiniteStateMachine.h"

// This #include statement was automatically added by the Particle IDE.
#include "elapsedMillis/elapsedMillis.h"

// This #include statement was automatically added by the Particle IDE.
#include "NCD2Relay/NCD2Relay.h"

// This #include statement was automatically added by the Particle IDE.
#include "CE_BME280/CE_BME280.h"

// This #include statement was automatically added by the Particle IDE.
#include "Fob_Alarm/Fob_Alarm.h"

// This #include statement was automatically added by the Particle IDE.
#include "Fob_Alarm/KeyFob.h"

#include "application.h"

// A1332 I2C address is 0x0C(12)
#define Addr 0x0C


#define APP_NAME "Garge_Door"
String VERSION = "Version 0.0.3";

const int TIME_ZONE = +1;

/*******************************************************************************
 initialize FSM states with proper enter, update and exit functions
*******************************************************************************/
State initState = State( initEnterFunction, initUpdateFunction, initExitFunction );
State idleState = State( idleEnterFunction, idleUpdateFunction, idleExitFunction );

//State queryState = State( queryEnterFunction, queryUpdateFunction, queryExitFunction );
State garageOpenState = State( garageOpenEnterFunction, garageOpenUpdateFunction, garageOpenExitFunction );
State garageOpeningState = State( garageOpeningEnterFunction, garageOpeningUpdateFunction, garageOpeningExitFunction );
State garageCloseState = State( garageCloseEnterFunction, garageCloseUpdateFunction, garageCloseExitFunction );
State garageClosingState = State( garageClosingEnterFunction, garageClosingUpdateFunction, garageClosingExitFunction );
State garageErrorState = State( garageErrorEnterFunction, garageErrorUpdateFunction, garageErrorExitFunction );

//initialize state machine, start in state: Idle
FSM garageDoorStateMachine = FSM(initState);

//milliseconds for the init cycle, so temperature samples get stabilized
//this should be in the order of the 5 minutes: 5*60*1000==300000
//for now, I will use 1 minute
#define INIT_TIMEOUT 10000
elapsedMillis initTimer;

//minimum number of milliseconds to leave the system in idle state
// to protect the fan and the heating/cooling elements
#define MINIMUM_IDLE_TIMEOUT 2000
elapsedMillis minimumIdleTimer;

#define MINIMUM_OPENING_TIMEOUT 500
elapsedMillis minimumOpeningTimer;

#define MINIMUM_CLOSING_TIMEOUT 500
elapsedMillis minimumClosingTimer;

#define MINIMUM_OPEN_TIMEOUT 1000
elapsedMillis minimumOpenTimer;

#define MINIMUM_CLOSE_TIMEOUT 1000
elapsedMillis minimumCloseTimer;

/*******************************************************************************
 IO mapping
*******************************************************************************/
uint8_t reedSwitch = D3;



/*******************************************************************************
 BME280 sensor
*******************************************************************************/
#define BME280_SAMPLE_INTERVAL   5000    // Sample BME280 every 5 seconds
                                         //  this is then averaged in temperatureAverage


CE_BME280 bme; // I2C

elapsedMillis bme280SampleInterval;
// how many samples to take and average, more takes longer but measurement is smoother
const int NUMBER_OF_SAMPLES = 10;
#define DUMMY -100
#define DUMMY_ARRAY { DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY, DUMMY };
float temperatureSamples[NUMBER_OF_SAMPLES] = DUMMY_ARRAY;
float averageTemperature;
float pressureSamples[NUMBER_OF_SAMPLES] = DUMMY_ARRAY;
float averagePressure;
float humiditySamples[NUMBER_OF_SAMPLES] = DUMMY_ARRAY;
float averageHumidity;

//temperature related variables - internal
float currentTemp = 0.0;
float currentPressure = 0.0;
float currentHumidity = 0.0;

//BME280 related variables - to be exposed in the cloud
String currentTempString = String(currentTemp); //String to store the target temp so it can be exposed and set
String currentPressureString = String(currentPressure); //String to store the sensor's temp so it can be exposed
String currentHumidityString = String(currentHumidity); //String to store the sensor's humidity so it can be exposed

#define INFLUXDB_HOST ""
#define INFLUXDB_PORT 
#define INFLUXDB_DB ""

STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

NCD2Relay relayController;
int relayOne = 1;

/*******************************************************************************
 BME280 related declarations
*******************************************************************************/
//BME280 difference with real temperature, pressure and humidity (if none set to zero)
//use this variables to fix BME280 measurements with your reference device
float temperatureCalibration = 0;
float pressureCalibration = 0;
float humidityCalibration = 0;

//TESTING_HACK
// this allows me to system test the project
bool testing = false;

void processAngle(int vals[]);

int triggerRelay(String command);

int count = 0;
float oldAngle = 0.0;
float newAngle = 0.0;

//Class Objects
KeyFob fob;

bool checkFob = false;
bool button2pressed = false;
bool doButton = false;
bool status = false;;

void garageStatus()
{
  status = true;
}

void subscribeHandler(const char* event, const char* data) {

  // data set to the device ID that wants to sleep.
    String id = Particle.deviceID();
    // see if this was a notification from this Photon that we can sleep
    if (!strcmp(data, id.c_str())) {
        
        doButton = true;
        //Particle.publish(APP_NAME, "Do Button", 60, PRIVATE);
        //Serial.println("Toggle relays");
        //relayController.turnOnRelay(1);
    //delay(500);
      //relayController.turnOffRelay(1);
    }
    else
    {
      Particle.publish(APP_NAME, "deviceID didn't match", 60, PRIVATE);
    }
}

/* This function is called once at start up ----------------------------------*/
void setup()
{   
    Particle.subscribe("Garage_Door", subscribeHandler, "27001c000347343337373737");  // listen for events "27001c000347343337373737" 27001c000347343337373737

    //publish startup message with firmware version
    Particle.publish(APP_NAME, VERSION, 60, PRIVATE);
    
    
    Particle.variable("Temperature", currentTempString);
    Particle.variable("Humidity", currentHumidityString);
    Particle.variable("Pressure", currentPressureString);

  //Setup communication to KeyFob interface module
    attachInterrupt(A3, evalFob, CHANGE);
    
    pinMode(reedSwitch, INPUT_PULLUP);
    attachInterrupt(reedSwitch, garageStatus, CHANGE);
    
    // Initialise I2C communication as Master
    Wire.begin();
    
    Serial.begin(115200);
  relayController.setAddress(0,0,0);
  
  if (!bme.begin()) {
        Particle.publish("Could not find a valid BME280 sensor, check wiring!");
        delay(2000);
        while (1);
    }
    
    Time.zone(TIME_ZONE);
    
    //reset samples array to default so we fill it up with new samples
    uint8_t i;
    for (i=0; i<NUMBER_OF_SAMPLES; i++) {
        temperatureSamples[i] = DUMMY;
        pressureSamples[i] = DUMMY;
        humiditySamples[i] = DUMMY;
    }
    delay(1000);
}

/* This function loops forever --------------------------------------------*/
void loop()
{   
    readBME280();
    readReedSwitch();
    readFob();

  garageDoorStateMachine.update();
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 FINITE STATE MACHINE FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

void initEnterFunction(){
  //start the timer of this cycle
  initTimer = 0;
}
void initUpdateFunction(){
  //time is up?
  if (initTimer < INIT_TIMEOUT) {
    //garageDoorStateMachine.transitionTo(idleState);
    return;
  }

  if(status == true) {
      garageDoorStateMachine.transitionTo(garageCloseState);
  } else {
      garageDoorStateMachine.transitionTo(garageOpenState);
  }
}
void initExitFunction(){
  Particle.publish(APP_NAME, "Initialization done", 60, PRIVATE);
}

void idleEnterFunction(){

  //start the minimum timer of this cycle
  minimumIdleTimer = 0;
}
void idleUpdateFunction(){

  //is minimum time up? not yet, so get out of here
  if (minimumIdleTimer < MINIMUM_IDLE_TIMEOUT) {
    return;
  }
  
    //status == false --> closed
  if ( (button2pressed == true || doButton == true) && status == true ){
    button2pressed = false; 
    doButton = false;
    garageDoorStateMachine.transitionTo(garageOpeningState);
  }
  
  if ( (button2pressed == true || doButton == true) && status == false ){
    button2pressed = false; 
    doButton = false;
    garageDoorStateMachine.transitionTo(garageClosingState);
  }
  
  
  
  
  
    /*
  if ( button2pressed == true && status == false ){
    button2pressed = false;  
    garageDoorStateMachine.transitionTo(garageOpeningState);
  }
  
  if ( button2pressed == true && status == true ){
    button2pressed = false;  
    garageDoorStateMachine.transitionTo(garageClosingState);
  }
  
  if ( button2pressed == false && status == false ){
      garageDoorStateMachine.transitionTo(garageCloseState);
  }
  
  if ( button2pressed == false && status == true ){
    //if the temperature is lower than the target, transition to heatingState
      garageDoorStateMachine.transitionTo(garageOpenState);
  }
  */
}

void idleExitFunction(){
}

void garageOpeningEnterFunction() {
    minimumOpeningTimer = 0;
    
    relayController.turnOnRelay(1);

}

void garageOpeningUpdateFunction(){
  if (minimumOpeningTimer < MINIMUM_OPENING_TIMEOUT) {
    return;
  }
  if (relayController.readRelayStatus(relayOne) == 1){
    relayController.turnOffRelay(relayOne);
  }
  delay(1000);
  Particle.publish(APP_NAME, "opening", 60, PRIVATE);

  garageDoorStateMachine.transitionTo(garageOpenState);
}

void garageOpeningExitFunction() {
}


void garageClosingEnterFunction() {
    minimumClosingTimer = 0;
    relayController.turnOnRelay(1);

}

void garageClosingUpdateFunction(){
    if (minimumClosingTimer < MINIMUM_CLOSING_TIMEOUT) {
        return;
    }
    if (relayController.readRelayStatus(relayOne) == 1){
        relayController.turnOffRelay(relayOne);
    }
    if (status == true){
      garageDoorStateMachine.transitionTo(garageCloseState);
  }
}

void garageClosingExitFunction() {
}


void garageOpenEnterFunction() {
    minimumOpenTimer = 0;
}

void garageOpenUpdateFunction(){
    if (minimumOpenTimer < MINIMUM_OPEN_TIMEOUT) {
        return;
    }
    
    Particle.publish(APP_NAME, "OPEN", 60, PRIVATE);
    
    garageDoorStateMachine.transitionTo(idleState);
}

void garageOpenExitFunction() {
}


void garageCloseEnterFunction() {
    minimumCloseTimer = 0;
}

void garageCloseUpdateFunction(){
    if (minimumCloseTimer < MINIMUM_CLOSE_TIMEOUT) {
        return;
    }
    Particle.publish(APP_NAME, "CLOSED", 60, PRIVATE);
    garageDoorStateMachine.transitionTo(idleState);
}

void garageCloseExitFunction() {
}

void garageErrorEnterFunction() {
} 

void garageErrorUpdateFunction() {
} 

void garageErrorExitFunction() {
}

void readFob() {
    
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
    
    if(action.equalsIgnoreCase("Button2Press")){
        button2pressed = true;
        Particle.publish(APP_NAME, "Button pressed", 60, PRIVATE);
    }

    if(action.equalsIgnoreCase("Button4Press")){
                relayController.toggleRelay(2);
                
                relayController.turnOnRelay(1);
            delay(500);
              relayController.turnOffRelay(1);
    }
  }
}


/*******************************************************************************
 * Function Name  : readTemperature
 * Description    : reads the temperature of the BME280 sensor at every BME280_SAMPLE_INTERVAL
                    if testing the app, it returns right away
 * Return         : 0
 *******************************************************************************/
int readBME280() {
    //TESTING_HACK
    //are we testing the app? then no need to acquire from the sensor
    if (testing) {
        return 0;
    }

    //time is up? no, then come back later
    if (bme280SampleInterval < BME280_SAMPLE_INTERVAL) {
        return 0;
    }

    //time is up, reset timer
    bme280SampleInterval = 0;
    //valid samples acquired, adjust BME280 difference if any
    float tmpTemperature = bme.readTemperature();
    tmpTemperature = tmpTemperature + temperatureCalibration;
    float tmpPressure = bme.readPressure() / 100.0F;
    tmpPressure = tmpPressure + pressureCalibration;
    float tmpHumidity = bme.readHumidity();
    tmpHumidity = tmpHumidity + humidityCalibration;
    
    //------------------------------------------------------------------
    //let's make an average of the measured temperature
    // by taking N samples
    uint8_t i;
    for (i=0; i< NUMBER_OF_SAMPLES; i++) {
        //store the sample in the next available 'slot' in the array of samples
        if ( temperatureSamples[i] == DUMMY && pressureSamples[i] == DUMMY && humiditySamples[i] == DUMMY ) {
            temperatureSamples[i] = tmpTemperature;
            pressureSamples[i] = tmpPressure;
            humiditySamples[i] = tmpHumidity;
        break;
        }
    }

    //is the samples array full? if not, exit and get a new sample
    if ( temperatureSamples[NUMBER_OF_SAMPLES-1] == DUMMY && pressureSamples[NUMBER_OF_SAMPLES-1] == DUMMY && humiditySamples[NUMBER_OF_SAMPLES-1] == DUMMY ) {
        return 0;
    }

    // average all the samples out
    averageTemperature = 0;
    averagePressure = 0;
    averageHumidity = 0;

    for (i=0; i<NUMBER_OF_SAMPLES; i++) {
        averageTemperature += temperatureSamples[i];
        averagePressure += pressureSamples[i];
        averageHumidity += humiditySamples[i];


    }
    averageTemperature /= NUMBER_OF_SAMPLES;
    averagePressure /= NUMBER_OF_SAMPLES;
    averageHumidity /= NUMBER_OF_SAMPLES;



    //reset samples array to default so we fill it up again with new samples
    for (i=0; i<NUMBER_OF_SAMPLES; i++) {
        temperatureSamples[i] = DUMMY;
        pressureSamples[i] = DUMMY;
        humiditySamples[i] = DUMMY;

    }
    //------------------------------------------------------------------
    //sample acquired and averaged - go ahead and store temperature and humidity in internal variables
    publish( averageTemperature, averagePressure, averageHumidity );
    return 0;
}

/*******************************************************************************
 * Function Name  : publish
 * Description    : the temperature/humidity passed as parameters get stored in internal variables
                    and then published
 * Return         : 0
 *******************************************************************************/
int publish( float temperature, float pressure, float humidity ) {

  char currentTempChar[32];
  currentTemp = temperature;
  int currentTempDecimals = (currentTemp - (int)currentTemp) * 100;
  sprintf(currentTempChar,"%0d.%d", (int)currentTemp, currentTempDecimals);
  
  char currentPressureChar[32];
  currentPressure = pressure;
  int currentPressureDecimals = (currentPressure - (int)currentPressure) * 100;
  sprintf(currentPressureChar,"%0d.%d", (int)currentPressure, currentPressureDecimals);

  char currentHumidityChar[32];
  currentHumidity = humidity;
  int currentHumidityDecimals = (currentHumidity - (int)currentHumidity) * 100;
  sprintf(currentHumidityChar,"%0d.%d", (int)currentHumidity, currentHumidityDecimals);

  //publish readings into exposed variables
  currentTempString = String(currentTempChar);
  currentPressureString = String(currentPressureChar);
  currentHumidityString = String(currentHumidityChar);

  //publish readings
  Particle.publish(APP_NAME, "Garage: " + currentTempString + "°C " + currentHumidityString + "% " + currentPressureString + "Pa", 60, PRIVATE);

  return 0;
}


/*******************************************************************************
 * Function Name  : publish
 * Description    : the temperature/humidity passed as parameters get stored in internal variables
                    and then published
 * Return         : 0
 *******************************************************************************/
int readReedSwitch() {
    if (digitalRead(reedSwitch) == 1) {
        status = 0;
    } else {
        status = 1;
    }
    return 0;
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


