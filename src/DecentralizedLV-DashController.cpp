/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/mligh/Downloads/DecentralizedLV-DashController/src/DecentralizedLV-DashController.ino"
/*
 * Project DecentralizedLV
 * Description:
 * Author:
 * Date:
 */
#include "neopixel.h"
#include "DecentralizedLV-Boards/DecentralizedLV-Boards.h"

//////////////////////////////////////////////////
/////////////    SYSTEM BEHAVIOR    //////////////
//////////////////////////////////////////////////

void setup();
void loop();
void updateLPOutputs();
#line 14 "c:/Users/mligh/Downloads/DecentralizedLV-DashController/src/DecentralizedLV-DashController.ino"
#define DO_LOW_POWER_MODE       false           //Set this true if you want to enable low power mode based on signals from the power controller.
#define DO_RMS_FAN_CONTROL      false           //Set this true if you will receive the motor temperature from the RMS and have the fan controlled by a corner board. Otherwise turns on fan with Ignition
#define DO_BMS_FAN_CONTROL      false           //Set this true if you will receive the HV battery temperature data from the BMS and have the fan speed controlled by a corner board. Otherwise turns on fan with Ignition
#define DO_ANALOG_DRIVE_SW      false           //Set this true if you have wired the drive switch with multiple modes (sport, eco, etc) using a resistor network. Otherwise, set false if using a simple binary switch (just does normal drive)
#define DO_ANALOG_LIGHT_SW      false           //Set this true if you have wired the headlight switch with multiple modes (headlight, high beam) using a resistor network. Otherwise, set false if using a simple binary switch (just does headlights)
#define DO_SOUNDBOARD_HORN      false           //Set this true if you have installed the soundboard module YX6300 and are using that as a horn. Otherwise, uses a simple binary switch to control a corner board to activate the horn
#define DO_NEOPIXELS            false           //Set this true if you are attaching a neopixel strip to this board (need 12V to 5V regulator) from the two headers and want it to do animations.
#define DO_MOMENTARY_FAN        false           //Set this true if you are using a momentary push button for the driver fan, set false if using an on/off switch

#define DRV_FAN_INIT_MODE       false           //When turning on to accessory, set this as default state of driver fan

#define AUTO_HDL_OFF            1000            //When using auto headlights and the lights are on, values below this from the photoresistor will turn off the headlights. This should be higher than AUTO_HDL_ON for a hysteresis effect
#define AUTO_HDL_ON             600             //When using auto headlights and the lights are on, values below this from the photoresistor will turn off the headlights. This should be higher than AUTO_HDL_ON for a hysteresis effect

#define SPEEDTHR 10
#define MTR_TEMP_THR    60                      //Temperature (in C) of the motor which will enable the radiator/motor fan
#define MTR_LPM_OFFSET  10                      //Offset temperature (in C) of the motor which will enable the radiator/motor fan in low power mode. Value of 10 with MTR_TEMP_THR = 60 would mean fan turns on at 70C in low power mode
#define BATT_TEMP_THR   35

//////////////////////////////////////////////////
/////////////    CAN MESSAGE IDS    //////////////
//////////////////////////////////////////////////

#define CAN_MAIN_COMP   0x110       //CAN ID broadcasted by the main telemetry computer
// byte 0:
// byte 1:
// byte 2: RMS RPM (lower byte)
// byte 3: RMS RPM (upper byte)
// byte 4:
// byte 5:
// byte 6:
// byte 7:

#define CAN_RMS_COMP    0x116       //CAN ID broadcasted by the main telemetry computer with motor controller and BMS data
// byte 0: RMS Temperature (degrees C)
// byte 1: Motor Temperature (degrees C)
// byte 2: Battery Temperature (degrees C)
// byte 3: Battery Percentage (degrees C)
// byte 4:
// byte 5:
// byte 6:
// byte 7:

//////////////////////////////////////////////////
/////////////    INPUT SWITCHES    ///////////////
//////////////////////////////////////////////////
#define HEADLIGHT       A1      //Switch used to set headlight mode - Analog pin for auto-on or manual-on based on voltage
#define DRV_SW          A2      //Switch used to set drive mode - Analog pin for normal, eco, and sport modes
#define FAN_SW          D0      //Switch to control if driver fan is on or off
#define HIGHBEAM        D3      //Switch to control highbeams on or off
#define LTURN           D4      //Read in from the blinker timer from the left turn signal stick
#define RTURN           D5      //Read in from the blinker timer from the right turn signal stick
#define REV_SW          D6      //Switch to change to reverse mode
#define PRK_SW          D7      //Switch to change to park mode

//analogRead thresholds for drive mode and headlight switches (set these based on resistor network values)
#define DRV_ECO_THR     1000
#define DRV_SPT_THR     2000
#define DRV_NRM_THR     3000

#define HDL_AUTO_THR    1000    //Values between this and HDL_HBM_THR will register as high beam
#define HDL_HBM_THR     2000    //Values between this and HDL_MANU_THR will register as high beam
#define HDL_MANU_THR    3000    //Values above this will register the headlight to be on

//////////////////////////////////////////////////
///////////////    LP OUTPUTS   //////////////////
//////////////////////////////////////////////////
#define INST_PWR    A0      //12V output for instrument cluster
#define DRV_FAN     A3      //12V output for driver fan based on switch
#define STEREO      A7      //12V output for stereo
#define FUEL_PWM    A5      //12V PWM pulse for poor man's DAC. See circuit in README.md

//////////////////////////////////////////////////
////////////    MISCELLANEOUS IO   ///////////////
//////////////////////////////////////////////////
#define LIGHT_SENSE A6      //ADC conversion for ambient light sensor (photoresistor)
#define NEOPIX_DATA A4      //Data output pin for neopixel strip

SYSTEM_MODE(SEMI_AUTOMATIC);

//Timer aTimer(10000, startupAnimTimer);
//Timer ssTimer(5000,softStartTimer);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////                                              GLOBAL VARIABLES                                             ///////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
////////////   SWITCH VARIABLES READ BY THIS BOARD     ///////////////
//////////////////////////////////////////////////////////////////////

//Buttons
bool rPress, lPress;            //Flag set by the left or right turn signals being pressed
bool highPress;                 //Flag set by the high beam being pressed
bool fanPress;                  //Flag set by the fan switch being pressed
bool revPress;                  //Flag set by the reverse control switch being pressed. Should put system into reverse mode
bool parkPress;                 //Flag set by the park control switch being pressed. Should put system into park mode

//ADC switch reading variables
uint16_t driveADC = 0;          //Raw ADC reading from the drive switches. Depending on the reading sets normal vs sport vs eco
uint16_t headlightADC = 0;      //Raw ADC reading from the headlight switches. Depending on the reading sets off, auto headlights from brightness, or manual on
uint16_t photoresistorADC = 0;     //Value read by photoresistor, used to update automatic headlight state


///////////////////////////////////////////////////////////////////
////////////    CORNER RECEIVE GLOBAL VARIABLES     ///////////////
///////////////////////////////////////////////////////////////////
//Values received from corner boards
bool switchFault = false;       //Flag set true when a kill switch has been opened - signals a BPS fault on instrument cluster
bool bmsFault = false;          //Flag set true when BMS triggers a fault - signals a BPS fault on instrument cluster

///////////////////////////////////////////////////////////////////
///////////   INSTRUMENT CLUSTER GLOBAL VARIABLES   ///////////////
///////////////////////////////////////////////////////////////////
//Values received from the instrument cluster used by this board and other boards
uint16_t speed;                 //Vehicle speed over CAN - used to determine state of fans and display on instrument cluster
uint16_t rmsTemp;               //Temperature of RMS over CAN - used to determine states of pumps/fans
uint16_t rmsRPM;                //RPM of the motor to display on the instrument cluster
uint16_t motorTemp;             //Temperature of the motor in celcius, used to turn on and off cooling fans
uint16_t battTemp;              //Average temperature of the battery cells in the HV pack, used to change fan speed - NOTE: Legacy, HV Controller with replace this functionality


///////////////////////////////////////////////////////////////////
////////////       COMPUTER GLOBAL VARIABLES        ///////////////
///////////////////////////////////////////////////////////////////
//Values received from the car computer containing BMS/RMS data
bool receivedBattTemp = false;  //Flag set true once a message has been received from the BMS about the battery temperature, used by softStart to set fans manually on if no message received
bool receivedMotorTemp = false; //Flag set true once a message has been received from the RMS about the motor temperature, used by softStart to set fans manually on if no message received
uint8_t rmsFault = 1;           //Set to a non-zero value when the motor controller has a fault - illuminates check engine light
uint8_t battPct;                //Battery state of charge reported by BMS, passed through by main computer in Decentralized LV 1.0

//Misc control-flow variables
uint32_t loop_time = 0;

//////////////////////////////////////////////////
///////////    CAN MESSAGE FORMAT   //////////////
//////////////////////////////////////////////////

CAN_Controller canController;

DashController_CAN dashController(DASH_CONTROL_ADDR);

LPDRV_RearLeft_CAN rearLeftDriver(REAR_LEFT_DRIVER);

PowerController_CAN powerController(POWER_CONTROL_ADDR);

CamryCluster_CAN instrumentCluster;

//////////////////////////////////////////////////
///////////    FUNCTION PROTOTYPES   /////////////
//////////////////////////////////////////////////

void configurePins();
void readPins();
void updateGear();
void dashSpoof();
void CANReceive();
void updateLights();
void updateFanControl();
void updateFaultState();

// setup() runs once, when the device is first turned on.
void setup() {
    Serial.begin(115200);

    canController.begin(500000);
    canController.addFilter(powerController.boardAddress);   //Allow incoming messages from Power Controller
    canController.addFilter(rearLeftDriver.boardAddress);    //Allow incoming message from Rear left driver board
    canController.addFilter(CAN_MAIN_COMP);    //Allow incoming messages from Main Telemetry Computer
    canController.addFilter(CAN_RMS_COMP);     //Allow incoming messages from Motor Controller Passthrough

    configurePins();                        //Set up the GPIO pins for reading the state of the switches
    readPins();                             //Get the initial reading of the switches

    dashController.initialize();            //Initializes all of the variables the Dash Controller sends on CAN to their default values
    powerController.initialize();
    rearLeftDriver.initialize();
    instrumentCluster.initialize();

    //powerController.Acc = true;
    //powerController.Ign = true;

    //powerController.FullStart = true;

    battPct = 10;                           //Start with initially low battery percentage displayed to err on side of caution in case we don't hear from BMS

    battTemp = 10;                          //Turn off fans during soft-start period
    speed = 0;                              //Vehicle speed that comes from the dashboard
    rmsTemp = 0;                            //Set motor controller temperature to cold until receiving the first temperature packet or the softStart expires which then turns on the fan
    
    //aTimer.start();                         //Start the timer for the startup animation
    //ssTimer.start();                        //Start the timer for the soft-start behavior
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
    readPins();     //Read the state of each of the pins on the Microcontroller
    updateFaultState();
    updateGear();
    updateLights();
    updateFanControl();
    updateLPOutputs();
    updateLPOutputs();

    dashController.sendCANData(canController);

    CANReceive();   //Receive any incoming messages and parse what they mean

    dashController.bmsFaultDetected = rearLeftDriver.bmsFaultInput;

    dashSpoof();

    //Serial.printlnf("L: %d, T: %d",loop_time,millis());
    while(millis()-loop_time < 10) delayMicroseconds(1);
    loop_time = millis();
}

//Code that sends out spoof CAN Messages for the airbag and ABS so the lights turn off
void dashSpoof(){
    //Handle which drive mode is displayed
    instrumentCluster.driveMode = dashController.driveMode;
    instrumentCluster.ecoMode = powerController.LowPowerMode;
    
    //Handle which lights are on. Backlight dims if headlights are on
    instrumentCluster.headlight = dashController.headlight;
    instrumentCluster.highbeam = dashController.highbeam;
    if(dashController.headlight) instrumentCluster.LCD_Brightness = LCD_BRIGHTNESS_LOW;
    else instrumentCluster.LCD_Brightness = LCD_BRIGHTNESS_HIGH;

    //BMS faults. Shows "Hybrid System Stopped" if we have a BMS fault
    if(dashController.bmsFaultDetected) instrumentCluster.LCD_PowerPrompt = LCD_HYBRID_SYSTEM_STOPPED;
    else if(powerController.Ign && !powerController.FullStart) instrumentCluster.LCD_PowerPrompt = LCD_IGNITION_PROMPT;
    else instrumentCluster.LCD_PowerPrompt = LCD_POWER_GOOD;

    //12V battery status, if low.
    instrumentCluster.chargingSystemMalfunction = powerController.LowACCBattery;

    //RMS fault information
    if(dashController.rmsFaultDetected && !dashController.bmsFaultDetected) instrumentCluster.LCD_EngineStoppedCode = LCD_ENGINE_STOPPED;
    else instrumentCluster.LCD_EngineStoppedCode = LCD_ENGINE_NORMAL;
    instrumentCluster.checkEngineOn = dashController.rmsFaultDetected;

    instrumentCluster.readyToDrive = powerController.FullStart;         //If the car can drive, then enable this (shows the park icon on cluster)

    //RPM and speed gauges
    instrumentCluster.rpmGauge = 1000;  //Do something with this...
    instrumentCluster.speedGauge = 50;

    //RMS temperature
    instrumentCluster.motorTempDegC = 60;   //Do something with this...

    //Now that we've update all the fields, just call sendCANData, and the object will automatically handle all the variables!
    instrumentCluster.sendCANData(canController);
}

void CANReceive(){
    LV_CANMessage receivedMessage;
    while(canController.receive(receivedMessage)){ //Check if we received a message over the CAN bus
        powerController.receiveCANData(receivedMessage);
        rearLeftDriver.receiveCANData(receivedMessage);
    }
//    while(can.receive(inputMessage)){                                       //Receive any CAN Bus messages in the buffer
//        if(inputMessage.id == CAN_MAIN_COMP){                          //Message from Joe's computer on the RPM of the motor, gets sent to dash
//            if(!USING_CAMRY_DASH) rmsRPM = (inputMessage.data[2] + (inputMessage.data[3] << 8)) << 2;   //Calculate RMS rpm divided by four if using VW dashboard - Note: Legacy, remove once using Camry dash
//            else rmsRPM = inputMessage.data[2] + (inputMessage.data[3] << 8);                           //Calculate RMS rpm to be sent to Camry dashboard
//        }
//        else if(inputMessage.id == CAN_DRV_RL){                             //Message from back-left corner board if the fault flasher is on, turns on/off the dash indicator
//            if(inputMessage.data[2] == 1 || inputMessage.data[3] == 1){     //Byte 2 contains if a kill switch was pressd, byte 3 contains if the BMS signaled a fault
//                switchFault = inputMessage.data[2];                     
//                bmsFault = inputMessage.data[3];
//                errcode = 0x82;                                             //Turn on the dash warning label for the VW dashboard - Note: Legacy
//            }
//            else{
//                bmsFault = false;                                           //If neither of the bytes are a 1, then there is no fault, clear flags
//                switchFault = false;                                        //If neither of the bytes are a 1, then there is no fault, clear flags
//                errcode = 0x88;                                             //Turn off the dash warning label for the VW dashboard - Note: Legacy
//            }
//        }
//        else if(inputMessage.id == CAN_RMS_COMP){                           //Message from Joe Computer of forwarded battery and motor controller temperatures
//            rmsTemp = inputMessage.data[0];
//            motorTemp = inputMessage.data[1];
//            battTemp = inputMessage.data[5];
//            battPct = inputMessage.data[6];
//            receivedBattTemp = true;
//            receivedMotorTemp = true;
//        }
//        else if(inputMessage.id == 0x69A){                                  //Forwarded message from Joe Computer if there is a fault posted by motor controller
//            rmsFault = inputMessage.data[2];
//        }
//    }
}

//Read all of the pins on the device
void readPins(){
    if(DO_ANALOG_LIGHT_SW) headlightADC = analogRead(HEADLIGHT);       //Headlight is an analog voltage based on a series resistor depending on the switch pressed. Auto headlights has 40k Series resistor (6V), Manual has no resitor (12V)
    else headlightADC = 4096 * digitalRead(HEADLIGHT);

    if(DO_ANALOG_DRIVE_SW) driveADC = analogRead(DRV_SW);             //Drive mode is an analog voltage based on a series resistor depending on the switch pressed. Sport has 20k Series resistor (9V), Eco has 40k Series resistor (6V)
    else driveADC = 4096 * digitalRead(DRV_SW);

    highPress = digitalRead(HIGHBEAM);
    lPress = digitalRead(LTURN);
    rPress = digitalRead(RTURN);
    fanPress = digitalRead(FAN_SW);
    parkPress = digitalRead(PRK_SW);
    revPress = digitalRead(REV_SW);

    photoresistorADC = analogRead(LIGHT_SENSE);
}

void updateGear(){
    if(powerController.FullStart){  //Check if the power controller has indicated that the user has fully started the car (brake + PTS)
        if(revPress) dashController.driveMode = DRIVE_MODE_REVERSE;
        else if(driveADC > DRV_SPT_THR && driveADC < DRV_NRM_THR) dashController.driveMode =  DRIVE_MODE_SPORT;
        else if(driveADC > DRV_ECO_THR && driveADC < DRV_SPT_THR) dashController.driveMode = DRIVE_MODE_ECO;
        else if(driveADC > DRV_NRM_THR) dashController.driveMode = DRIVE_MODE_FORWARD;
        else dashController.driveMode = DRIVE_MODE_PARK;
        if(dashController.driveMode != DRIVE_MODE_PARK && (bmsFault || switchFault)) dashController.driveMode = DRIVE_MODE_NEUTRAL;
    }
}

void updateLights(){
    //Logic for controlling headlights. Allows for using a multi-position switch with one wire and a series resistor
    if(headlightADC > HDL_MANU_THR){            //If wire is directly connected, then keep headlights on manually
        dashController.headlight = true;
    }
    //else if (headlightADC > HDL_HBM_THR){
    //    dashController.headlight = true;
    //}
    else if(headlightADC > HDL_AUTO_THR){       //For analog multi-position switch, allow for auto-headlights mode
        if(dashController.headlight && photoresistorADC < AUTO_HDL_OFF) dashController.headlight = false;  //If headlights are on and the photoresistor reads low then the sun is hitting the sensor - turn off the headlights
        else if(!dashController.headlight && photoresistorADC > AUTO_HDL_ON) dashController.headlight = true;  //If headlights are off and the photoresistor reads high then the sun is hitting the sensor - turn on the headlights
    }
    else{                                       //Otherwise, switch is off, turn off headlights
        dashController.headlight = false;
    }

    //Logic for controlling high beam. Reads in low power mode toggle from power controller and disables lights if in LPM.
    if(highPress && !powerController.LowPowerMode){
        dashController.highbeam = true;
    }
    else{
        dashController.highbeam = false;
    }

    dashController.leftTurnPWM = 255*lPress;     //Max brightness whenever switch is pressd
    dashController.rightTurnPWM = 255*rPress;
    dashController.reversePress = (dashController.driveMode == DRIVE_MODE_REVERSE); //If we're in the reverse drive mode, then turn on the toggle for the white reverse LEDs
}

void updateFanControl(){
    if(powerController.Ign){   //Ignition mode, should turn on the battery fans since more power is being drawn
        if(DO_BMS_FAN_CONTROL){
            if(((battTemp << 2) + 25) < 50) dashController.batteryFanPWM = 0;
            else dashController.batteryFanPWM =  (battTemp << 2) + 25;
        }
        else{
            dashController.batteryFanPWM = 255;
        }
        dashController.radiatorFan = true;
        dashController.radiatorPump = true;
    }
    else{   //Accessory, turn on battery fans to low speed to cycle some air through the box
        if(battTemp < 35 && DO_BMS_FAN_CONTROL){    //If we have a battery temperature reading and the battery is cold, turn off fans
            dashController.batteryFanPWM = 0;
        }
        else dashController.batteryFanPWM = 70;   //Otherwise, turn on fan to low speed
        dashController.radiatorFan = false;
        dashController.radiatorPump = false;
    }
}

void updateFaultState(){
    if(bmsFault){
        RGB.control(true);
        RGB.color(255,0,0);
    }
    else if(switchFault){
        RGB.control(true);
        RGB.color(255,255,0);
    }
    else{
        RGB.control(false);
    }
}

void updateLPOutputs(){
    if(powerController.Acc && !powerController.LowPowerMode) digitalWrite(INST_PWR, HIGH);
    else digitalWrite(INST_PWR, LOW);

    if(powerController.Acc && !powerController.LowPowerMode) digitalWrite(DRV_FAN, fanPress);
    else digitalWrite(DRV_FAN, LOW);
}

//Configure all of the pins on the microcontroller
void configurePins(){
    pinMode(HEADLIGHT, INPUT);
    pinMode(DRV_SW, INPUT);
    pinMode(HIGHBEAM, INPUT);
    pinMode(LTURN, INPUT);
    pinMode(RTURN, INPUT);
    pinMode(FAN_SW, INPUT);
    pinMode(PRK_SW, INPUT);
    pinMode(REV_SW, INPUT);

    pinMode(LIGHT_SENSE,INPUT);

    pinMode(INST_PWR, OUTPUT);
    pinMode(DRV_FAN,OUTPUT);
}

//void pwmAnimationHelper(){
//    pwmAnimationTick += 5;
//    if(pwmAnimationTick < 128){
//        lTurn = 0;
//        rTurn = 0;
//    }
//    else if(pwmAnimationTick < 255){
//        lTurn = 0;
//        rTurn = 0;
//    }
//    else if(pwmAnimationTick < 382){
//        lTurn = 0;
//        rTurn = 0;
//    }
//    else if(pwmAnimationTick < 510){
//        lTurn = pwmAnimationTick-382;
//        rTurn = pwmAnimationTick-382;
//    }
//    else if(pwmAnimationTick < 637){
//        lTurn = pwmAnimationTick-382;
//        rTurn = pwmAnimationTick-382;
//    }
//}
//
//void startupAnimTimer(){
//    startAnim = 0;
//    startupHdl = 0;
//    dashAnim = false;
//    aTimer.stopFromISR();
//}
//
//void softStartTimer(){
//    if(!receivedBattTemp){      //If we haven't received a CAN message with the battery temp in the 5 seconds since powerup, turn on the fan manually to be safe
//        fanPWM = 75;            //Set fan PWM to be higher so they run
//        battTemp = 50;          //Assume high battery temperature to be on safe side (runs the fans faster)
//    }
//    if(!receivedMotorTemp){     //If we haven't received a CAN message with the motor controller temp in the 5 seconds, turn on the cooling pump and fan
//        rmsTemp = MTR_TEMP_THR; //Set to the temperature to the threshold so the fan and pump both turn on manually to be safe
//    }
//    ssTimer.stopFromISR();      //Only want to run this timer once on startup, otherwise we are screwing with values every 5 seconds
//}