/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/mligh/OneDrive/Particle/DecentralizedLV-DashController/src/DecentralizedLV-DashController.ino"
/*
 * Project DecentralizedLV
 * Description:
 * Author:
 * Date:
 */
#include "neopixel.h"

void setup();
void loop();
void VWSpoof();
void pwmAnimationHelper();
void startupAnimTimer();
void softStartTimer();
#line 9 "c:/Users/mligh/OneDrive/Particle/DecentralizedLV-DashController/src/DecentralizedLV-DashController.ino"
#define SPEEDTHR 10
#define MTR_TEMP_THR    60
#define MTR_LPM_OFFSET  10
#define BATT_TEMP_THR   35

#define PUMP_INIT_MODE 0
#define DRV_FAN_INIT_MODE 1

#define USING_CAMRY_DASH true

//////////////////////////////////////////////////
/////////////    CAN MESSAGE IDS    //////////////
//////////////////////////////////////////////////

#define CAN_SNS         0x99        //Sense address LP and ULPDRV boards receive on
#define CAN_PCTL        0x120       //CAN ID broadcasted by the Power Controller
#define CAN_DRV_FL      0x101       //CAN ID broadcasted by the front-left driver board
#define CAN_DRV_FR      0x102       //CAN ID broadcasted by the front-right driver board
#define CAN_DRV_RL      0x103       //CAN ID broadcasted by the rear-left driver board
#define CAN_DRV_RR      0x104       //CAN ID broadcasted by the rear-right driver board
#define CAN_MAIN_COMP   0x110       //CAN ID broadcasted by the main telemetry computer
#define CAN_RMS_COMP    0x116       //CAN ID broadcasted by the main telemetry computer with motor controller data
#define CAN_MPPT        0x690       //CAN ID broadcasted by MPPT with power data

//#define USING_VW_DASH
//#define USING_CAMRY_DASH

//////////////////////////////////////////////////
/////////////    INPUT SWITCHES    ///////////////
//////////////////////////////////////////////////
#define HEADLIGHT       A1      //Switch used to set headlight mode - Analog pin for auto-on or manual-on basedd on voltage
#define DRV_SW          A2      //Switch used to set drive mode - Analog pin for normal, eco, and sport modes
#define FAN_SW          D0      //Switch to control if driver fan is on or off
#define HIGHBEAM        D3      //Switch to control highbeams on or off
#define LTURN           D4      //Read in from the blinker timer from the left turn signal stick
#define RTURN           D5      //Read in from the blinker timer from the right turn signal stick
#define REV_SW          D6      //Switch to change to reverse mode
#define PRK_SW          D7      //Switch to change to park mode

#define DRV_ECO_THR     1000
#define DRV_SPT_THR     2000
#define DRV_NRM_THR     3000

#define HDL_AUTO_THR    1500
#define HDL_MANU_THR    3000

//////////////////////////////////////////////////
///////////////    LP OUTPUTS   //////////////////
//////////////////////////////////////////////////
#define INST_PWR    A0      //12V output for instrument cluster
#define DRV_FAN     A3      //12V output for driver fan based on switch
//A7
//A5

//////////////////////////////////////////////////
////////////    MISCELLANEOUS IO   ///////////////
//////////////////////////////////////////////////
#define LIGHT_SENSE A6      //ADC conversion for ambient light sensor (photoresistor)
#define NEOPIX_DATA A4      //Data output pin for neopixel strip

SYSTEM_MODE(SEMI_AUTOMATIC);

Timer aTimer(10000, startupAnimTimer);
Timer ssTimer(5000,softStartTimer);

//////////////////////////////////////////////////
////////////    GLOBAL VARIABLES   ///////////////
//////////////////////////////////////////////////

//Switch variables read by this board
bool rPress, lPress;            //Flag set by the left or right turn signals being pressed
bool highPress;                 //Flag set by the high beam being pressed
bool drvFanPress;               //Flag set by the driver fan switch being pressed
bool revPress;                  //Flag set by the reverse control switch being pressed. Should put system into reverse mode
bool parkPress;                 //Flag set by the park control switch being pressed. Should put system into park mode

//ADC switch reading variables
uint16_t driveADC = 0;          //Raw ADC reading from the drive switches. Depending on the reading sets normal vs sport vs eco
uint16_t headlightADC = 0;      //Raw ADC reading from the headlight switches. Depending on the reading sets off, auto headlights from brightness, or manual on

//Operation flags set by this board based on timing for animations
bool startAnim = true;
bool startupHdl = true;
uint32_t pwmAnimationTick;
bool dashAnim = true;

//Operation flags/vars determined from state of switches
uint8_t rTurn, lTurn;           //PWM controllable lights
uint8_t driveMode = 0;          //Drive Mode: b0: Drive, b1: Sport, b2: Eco, b3: Reverse, b4: Neutral (BPS fault), 0 = Park
bool reverseActive = false;     //Flag set true if in reverse
bool headlight = false;         //Flag set true when the headlights should be turned on
bool highbeam = false;          //Flag set true when the highbeams should be turned on
bool last_headlight = true;     //Flag that tracks the last state of the headlight switch, used to send out a CAN packet to the Camry dash when the switch changes
bool last_highbeam = true;      //Flag that tracks the last state of the highbeam switch, used to send out a CAN packet to the Camry dash when the switch changes

//Operation flags/vars updated by CAN messages received from other system boards
bool accessory = false;         //Boolean flag set if the Power Controller is in ACC mode (Start Short Press)
bool ignition = false;          //Boolean flag set if the Power Controller is in Ignition Mode (Start Long Hold)
bool started = false;           //Boolean flag set if the Power Controller is in Start Mode (Brake + Start button)
bool hornPress = false;         //Boolean flag set if the horn is pressed - from Power Controller
bool lowACCBatt = false;        //Boolean flag set if the 12V battery is low
bool radFan = false;            //Boolean flag to set radiator fan based on temperature when in ignition mode
bool radPump = false;           //Boolean flag to set radiator pump based when in ignition mode
uint8_t fanPWM;                 //PWM for battery box fans determined from main computer - NOTE: Legacy, HV Controller with replace this functionality
bool solChg = false;            //Boolean flag set by Power Controller when in Solar Charge Mode
bool acChg = false;             //Boolean flag set by Power Controller when in AC Charge Mode (J1772 Plugged In)
bool LPMode;                    //Boolean flag set by Power Controller when operating in low power mode (timeout, low battery, etc)
uint16_t speed;                 //Vehicle speed over CAN - used to determine state of fans and display on instrument cluster
uint16_t rmsTemp;               //Temperature of RMS over CAN - used to determine states of pumps/fans
uint16_t rmsRPM;                //RPM of the motor to display on the instrument cluster
uint16_t motorTemp;             //Temperature of the motor in celcius, used to turn on and off cooling fans
uint16_t battTemp;              //Average temperature of the battery cells in the HV pack, used to change fan speed - NOTE: Legacy, HV Controller with replace this functionality
bool switchFault = false;       //Flag set true when a kill switch has been opened - signals a BPS fault on instrument cluster
bool bmsFault = false;          //Flag set true when BMS triggers a fault - signals a BPS fault on instrument cluster
uint8_t battPct;                //Battery state of charge reported by BMS, passed through by main computer in Decentralized LV 1.0
uint8_t errcode = 0;            //Error combination sent to VW dash - NOTE: Legacy, replaced with Camry instrument cluster in Decentralized LV 2.0
uint8_t rmsFault = 1;           //Set to a non-zero value when the motor controller has a fault - illuminates check engine light
bool receivedBattTemp = false;  //Flag set true once a message has been received from the BMS about the battery temperature, used by softStart to set fans manually on if no message received
bool receivedMotorTemp = false; //Flag set true once a message has been received from the RMS about the motor temperature, used by softStart to set fans manually on if no message received

//Misc control-flow variables
uint32_t loop_time = 0;
bool lastIgn = false;
uint8_t spoofNum = 0;

//////////////////////////////////////////////////
///////////    CAN MESSAGE FORMAT   //////////////
//////////////////////////////////////////////////

CANMessage pinStatus; //Main transmit message for switch data
// byte 0: Right Turn PWM 0-255
// byte 1: Left Turn PWM 0-255
// byte 2: 
// byte 3: HV Battery Fan PWM 0-255
// byte 4: b0: Headlight, b1: High-Beam, b2: Trunk-Release, b3: Driver-Fan, b4: Power-Steer-Relay, b5: reverse-sense-camera
// byte 5: b1: Low-Power Mode, b2: Startup animation flag, b3: Startup headlight
// byte 6: Drive Mode: b0: Drive, b1: Sport, b2: Eco, b3: Reverse, b4: Neutral (BPS fault)
// byte 7: b0: Radiator Fan, b1: Radiator pump, b2: Brake Boost, b3: Battery Fan, b4: MPPT-On

CANMessage inputMessage;

CANChannel can(CAN_D1_D2);

//////////////////////////////////////////////////
///////////    FUNCTION PROTOTYPES   /////////////
//////////////////////////////////////////////////

void configurePins();
void readPins();
void parseSwitches();
void dashSpoof();
void CANReceive();
void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7);

// setup() runs once, when the device is first turned on.
void setup() {
    Serial.begin(115200);

    can.begin(500000);                      //Start CAN at 500kbps
    can.addFilter(CAN_PCTL, 0x7FF);         //Allow incoming messages from Power Controller
    can.addFilter(CAN_DRV_RL, 0x7FF);       //Allow incoming messages from Rear-left driver for Kill Switch signal
    can.addFilter(CAN_MAIN_COMP, 0x7FF);    //Allow incoming messages from Main Telemetry Computer
    can.addFilter(CAN_RMS_COMP, 0x7FF);     //Allow incoming messages from Motor Controller Passthrough

    LPMode = 0;                             //Low power mode is false by default

    configurePins();                        //Set up the GPIO pins for reading the state of the switches
    readPins();                             //Get the initial reading of the switches

    rTurn = 0;                              //Set the lights to be off initially
    lTurn = 0;                              //Set the lights to be off initially

    battPct = 10;                           //Start with initially low battery percentage displayed to err on side of caution in case we don't hear from BMS

    fanPWM = 0;                             //Turn off fans during soft-start period
    radFan = 0;                             //Turn off the pumps and fans until we get the ignition signal or end the soft-start period
    radPump = 0;                            //Turn off the pumps and fans until we get the ignition signal or end the soft-start period
    battTemp = 10;                          //Turn off fans during soft-start period
    speed = 0;                              //Vehicle speed that comes from the dashboard
    rmsTemp = 0;                            //Set motor controller temperature to cold until receiving the first temperature packet or the softStart expires which then turns on the fan

    pwmAnimationTick = 0;
    
    aTimer.start();                         //Start the timer for the startup animation
    ssTimer.start();                        //Start the timer for the soft-start behavior
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
    readPins();     //Read the state of each of the pins on the Microcontroller
    parseSwitches();    //Update flags and variables based on other factors in the system, like temperature, speed, current
    //Main calculation for the CAN message sent to the corner boards
    byte tx2;
    byte tx4 = headlight + (highbeam << 1) + (solChg << 3) + (revPress << 5);
    byte tx5 = (LPMode << 1) + (startAnim << 2) + (startupHdl << 3);
    byte tx6 = driveMode;
    byte tx7 = radFan + (radPump << 1);
    CANSend(CAN_SNS, rTurn,lTurn,tx2,fanPWM,tx4,tx5,tx6,tx7);   //Send out the main message to the corner boards
    CANReceive();   //Receive any incoming messages and parse what they mean

    dashSpoof();

    //Serial.printlnf("L: %d, T: %d",loop_time,millis());
    while(millis()-loop_time < 10) delayMicroseconds(1);
    loop_time = millis();
}

//Code that sends out spoof CAN Messages for the airbag and ABS so the lights turn off
void dashSpoof(){
    if(!USING_CAMRY_DASH){      //Legacy - remove support for VW dash in Car 2.0
        CANSend(0x280, 0x49, 0x0E, rmsRPM&255, rmsRPM>>8, 0x0E, 0x00, 0x1B, 0x0E);  //Send the code to emulate the RPM dial
        VWSpoof();                                          //Spoof the dash CAN messages so the error lights turn off
    }
    else{
        uint8_t lowACC = 0;                                         //Default to not a low accessory battery
        if(lowACCBatt) lowACC = 0x04;                               //If ACC battery is low, display prompt on LCD
        uint8_t powerSteerState = 0x0;                              //Clear steering wheel icon
        uint8_t sysState = 0x0;                                     //Clear prompt
        if(bmsFault || switchFault) sysState = 0x04;                //Display Hybrid System Stopped prompt
        else if(ignition && !started) sysState = 0x50;              //Display ignition mode prompt
        else if(accessory && !ignition) sysState = 0x30;            //Display accessory mode startup instructions
        uint8_t engineFault = 0x40;                                 //0x40 turns off check engine light from instrument cluster
        if(ignition && rmsFault) engineFault = 0x0;                 //Turn on check engine light if ignition is set and there is an RMS fault
        uint8_t otherGear = 0;                                      //Variable to check if in neutral or reverse, default to no gear
        if(reverseActive) otherGear = 0x10;                         //If reverseActive flag is set, set to reverse mode (0x10 to instrument cluster signals reverse)
        else if(driveMode == 16) otherGear = 0x08;                  //If bit 4 is set in driveMode, we are in neutral (0x08 to instrument cluster signals neutral)
        uint8_t driveSet = 0;                                       //Drive setting variable, default of 0 (no gear)
        if(driveMode >= 1 && driveMode <= 7) driveSet = 0x80;       //If any drive mode bits 0-2 are set, then we are in a drive mode of some kind
        uint8_t driveModifier = 0;                                  //Default normal drive mode (not eco or sport)      
        if(driveMode == 3) driveModifier = 0x10;                    //If bit 1 is set, then we are in sport mode (0x10 to instrument cluster signals sport)
        if(driveMode == 5) driveModifier = 0x30;                    //If bit 2 is set, then we are in eco mode (0x30 to instrument cluster signals eco)
        CANSend(0x3B7,0,0,0,0,0,0,0,0);                             //Spoof Anti-Lock brakes (All 0's clears errors)
        CANSend(0x3B1,0,0,0,0,0,0,0,0);                             //Spoof SRS Airbag system (All 0's clears errors)
        CANSend(0x3BB,engineFault,lowACC,(motorTemp*1.59)+65,0,0,0,0,0);    //Spoof for engine controller. Takes a flag that sets check engine, alternator failure and motor temperature
        CANSend(0x394,0,powerSteerState,0,0,0,0,0,0);               //Spoof for Power Steering, byte 1 controls steering wheel icon on cluster
        CANSend(0x32C,0,0,0,0,0,0,0,0);
        CANSend(0x378,0,0,0,0,0,0,0,0);
        CANSend(0x412,0,0,0,0,0,0,0,0);                             //Lane departure spoof
        CANSend(0x411,0,0,0,0,0,0,0,0);                             //Precollision spoof
        CANSend(0x43a,1,1,1,1,0,0,0,0);                             //Parking sonar spoof
        CANSend(0x633,0x81,0,0,0,0,0,sysState,ignition?0x0D:0);     //Smart Key and Push to Start instructions
        CANSend(0x1EA,0,0,0,0,0,0,rmsRPM/200,rmsRPM%200);           //Motor spoof for RPM dial
        CANSend(0x3BC,0,otherGear,0,0,0,driveSet,0,driveModifier);  //Transmission controller spoof, sets drive gear and sport/eco/normal modes
        CANSend(0x620,0x10,0,0,0,headlight ? 0xF0:0xB0,dashAnim ? 0x0: 0x40,0x08,0);    //Spoof for instrument cluster animations and backlight dimming
        if(started) CANSend(0x1C4,0,0,0,0,0,1,0,0);                 //Spoof for fuel economy meter
        
        if(headlight != last_headlight || highbeam != last_highbeam){       //This CAN id only needs to get sent if the value has changed
            CANSend(0x622,0x12,00,0xE8,(headlight << 5) + (highbeam << 6),00,00,00,00); //Send out spoof for headlights/high beam system when headlight switches have changed
            last_headlight = headlight;
            last_highbeam = highbeam;
        }
    }
}

void VWSpoof(){
    //Spoof num selects the ID which is sent each time around the loop so it's not sent too frequently
    if(spoofNum == 0){  //Spoof for ABS
        //Errcode is a flag to set different error lights on the dashboard
        CANSend(0x1A0, 0x18, errcode, 0x00, 0x00, 0xfe, 0xfe, 0x00, 0xff);
        spoofNum = 1;
    }
    else if(spoofNum == 1){ //Spoof for airbag
        CANSend(0x050, 0x00, 0x80, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00);
        spoofNum = 0;
    }
}

void CANReceive(){
    while(can.receive(inputMessage)){                                       //Receive any CAN Bus messages in the buffer
        if(inputMessage.id == CAN_PCTL){                                    //Check if the received ID was from the Power Controller
            //accessory =
            //ignition = 
            //started =
            //solchg =
            //acChg =
            //LPMode =
            //hornPress =
            //lowACCBatt =
        }
        else if(inputMessage.id == CAN_MAIN_COMP){                          //Message from Joe's computer on the RPM of the motor, gets sent to dash
            if(!USING_CAMRY_DASH) rmsRPM = (inputMessage.data[2] + (inputMessage.data[3] << 8)) << 2;   //Calculate RMS rpm divided by four if using VW dashboard - Note: Legacy, remove once using Camry dash
            else rmsRPM = inputMessage.data[2] + (inputMessage.data[3] << 8);                           //Calculate RMS rpm to be sent to Camry dashboard
        }
        else if(inputMessage.id == CAN_DRV_RL){                             //Message from back-left corner board if the fault flasher is on, turns on/off the dash indicator
            if(inputMessage.data[2] == 1 || inputMessage.data[3] == 1){     //Byte 2 contains if a kill switch was pressd, byte 3 contains if the BMS signaled a fault
                switchFault = inputMessage.data[2];                     
                bmsFault = inputMessage.data[3];
                errcode = 0x82;                                             //Turn on the dash warning label for the VW dashboard - Note: Legacy
            }
            else{
                bmsFault = false;                                           //If neither of the bytes are a 1, then there is no fault, clear flags
                switchFault = false;                                        //If neither of the bytes are a 1, then there is no fault, clear flags
                errcode = 0x88;                                             //Turn off the dash warning label for the VW dashboard - Note: Legacy
            }
        }
        else if(inputMessage.id == CAN_RMS_COMP){                           //Message from Joe Computer of forwarded battery and motor controller temperatures
            rmsTemp = inputMessage.data[0];
            motorTemp = inputMessage.data[1];
            battTemp = inputMessage.data[5];
            battPct = inputMessage.data[6];
            receivedBattTemp = true;
            receivedMotorTemp = true;
        }
        else if(inputMessage.id == 0x69A){                                  //Forwarded message from Joe Computer if there is a fault posted by motor controller
            rmsFault = inputMessage.data[2];
        }
    }
}

void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7){
    pinStatus.id = Can_addr;
    pinStatus.len = 8;
    pinStatus.data[0] = data0;
    pinStatus.data[1] = data1;
    pinStatus.data[2] = data2;
    pinStatus.data[3] = data3;
    pinStatus.data[4] = data4;
    pinStatus.data[5] = data5;
    pinStatus.data[6] = data6;
    pinStatus.data[7] = data7;
    
    can.transmit(pinStatus);
}

//Read all of the pins on the device
void readPins(){
    headlightADC = analogRead(HEADLIGHT);       //Headlight is an analog voltage based on a series resistor depending on the switch pressed. Auto headlights has 40k Series resistor (6V), Manual has no resitor (12V)
    driveADC =  analogRead(DRV_SW);             //Drive mode is an analog voltage based on a series resistor depending on the switch pressed. Sport has 20k Series resistor (9V), Eco has 40k Series resistor (6V)

    highPress = digitalRead(HIGHBEAM);
    lPress = digitalRead(LTURN);
    rPress = digitalRead(RTURN);
    drvFanPress = digitalRead(FAN_SW);
    parkPress = digitalRead(PRK_SW);
    revPress = digitalRead(REV_SW);
}

//Take the state of the switches and do stuff with it
void parseSwitches(){
    if(started){                    //Check if system has been started for modifying the gear
        if(parkPress){              //If the user presses the parking mode switch, switch to park mode
            driveMode = 0;
            reverseActive = false;
        }
        else if(revPress){          //If the user presses the reverse mode switch, switch to reverse mode
            driveMode = 8;
            reverseActive = true;
        }
        else if(driveADC > DRV_SPT_THR && driveADC < DRV_NRM_THR){
            driveMode = 3;
            reverseActive = false;
        }
        else if(driveADC > DRV_ECO_THR && driveADC < DRV_SPT_THR){
            driveMode = 5;
            reverseActive = false;
        }
        else if(driveADC > DRV_NRM_THR){
            driveMode = 1;
            reverseActive = false;
        }
        if(driveMode && (bmsFault || switchFault)){     //If we were in drive mode and there is a fault, switch to "neutral"
            driveMode = 16;
            reverseActive = false;
        }
    }

    if(headlightADC > HDL_MANU_THR){

    }
    else if(headlightADC > HDL_AUTO_THR){

    }
    
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

    lTurn = 255*lPress;     //Max brightness whenever switch is pressd
    rTurn = 255*rPress;

    if(lTurn || rTurn){
        startAnim = false;
    }

    if(ignition){
        if(((battTemp << 2) + 25) < 50) fanPWM = 0;
        else fanPWM =  (battTemp << 2) + 25;
    }
    else{
        if(battTemp < 35){
            fanPWM = 0;
        }
        else fanPWM = 70;
    }

    if(LPMode == 0){
        if(ignition != lastIgn){
            lastIgn = ignition;
            startupHdl = ignition;
            aTimer.start();
        }

        if(startAnim) pwmAnimationHelper();
        
        highbeam = highPress;
        if(ignition){
            if(rmsTemp < MTR_TEMP_THR) radFan = 0;
            else radFan = 1;
            radPump = 1;
        }
        else{
            radFan = 0;
            radPump = 0;
        }
    }
    else if(LPMode == 1){
        highbeam = 0;   //Disable high beam, technically unnecesary

        if(ignition){
            if(rmsTemp < MTR_TEMP_THR + MTR_LPM_OFFSET) radFan = 0;
            else radFan = 1;
            if(rmsTemp < MTR_TEMP_THR) radPump = 0;
            else radPump = 1;
        }
        else{
            radFan = 0;
            radPump = 0;
        }
    }
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

void pwmAnimationHelper(){
    pwmAnimationTick += 5;
    if(pwmAnimationTick < 128){
        lTurn = 0;
        rTurn = 0;
    }
    else if(pwmAnimationTick < 255){
        lTurn = 0;
        rTurn = 0;
    }
    else if(pwmAnimationTick < 382){
        lTurn = 0;
        rTurn = 0;
    }
    else if(pwmAnimationTick < 510){
        lTurn = pwmAnimationTick-382;
        rTurn = pwmAnimationTick-382;
    }
    else if(pwmAnimationTick < 637){
        lTurn = pwmAnimationTick-382;
        rTurn = pwmAnimationTick-382;
    }
}

void startupAnimTimer(){
    startAnim = 0;
    startupHdl = 0;
    dashAnim = false;
    aTimer.stopFromISR();
}

void softStartTimer(){
    if(!receivedBattTemp){      //If we haven't received a CAN message with the battery temp in the 5 seconds since powerup, turn on the fan manually to be safe
        fanPWM = 75;            //Set fan PWM to be higher so they run
        battTemp = 50;          //Assume high battery temperature to be on safe side (runs the fans faster)
    }
    if(!receivedMotorTemp){     //If we haven't received a CAN message with the motor controller temp in the 5 seconds, turn on the cooling pump and fan
        rmsTemp = MTR_TEMP_THR; //Set to the temperature to the threshold so the fan and pump both turn on manually to be safe
    }
    ssTimer.stopFromISR();      //Only want to run this timer once on startup, otherwise we are screwing with values every 5 seconds
}