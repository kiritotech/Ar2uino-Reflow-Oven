//---------------------------------------------------------------------------
// Ar2uino - Arduino Reflow Oven Controller - v1.2khg
// More info & FULL description: http://ar2uino.wordpress.com
//
// v1.2khg = Orignal version 1.2, modified by "K"iritoTech, "H"alogen floodlight, new "G"raphic table temperature line
// Modified by KiritoTech 2019-11-18, to work with 1.8" TFT SPI ST7735 and custom PCB and new table graphic
// Work for halogen floodlight
// http://kiritotechh.simplesite.com
//----------------------------------------------------------------------------
// Circuit: Arduino UNO
// Voltage +5 V
//
// Con Ardu Descr
// USB 0 Rx
// USB 1 Tx
// ePinA 2 Rotary encoder pin A Interrupt! (Can ONLY be pin 2 or pin 3!)
// ePinB 3 Rotary encoder pin B (ALL encoder pins should be
// ePinSwitch 4 Rotary encoder switch pin decoupled by 0.1 uF capacitors!)
// CIRCPIN 5 Fan that circulates the air inside the oven (Possible PWM speed control)
// LEDPIN 6 LED to indicate heater power is ON. Connect via 220 ohm resistor
// OPTOPIN 7 Optocoupler via 220 ohm resistor. Controlls the heat!
// COOLPIN 8 LED to indicate COOLING started - or use it to Open the door for cooling!
//
// ----------------- Display:ST7735 1.8" V1.1 Red board 5V ------
// A0 9
// CS 10
// SDA 11
// 12 FREE!!
// SCK 13
// LED + (This gives around 13 mA through LED - reasonably bright)
// RESET +
// VCC +
// GND GND
//
// -----------------
// LIQUIDLED A0 LED that lights if solder is in liquid phase (>183 degrees C) (14)
// THERMDO A1 Thermocouple D0 (15)
// THERMCS A2 Thermocouple CS (16)
// THERMCLK A3 Thermocouple CLK (17)
// FANPIN A4 FREE!! (18) Not used! (Fan that blows cool air into the oven)
// A5 FREE!! (19)
// A6 FREE!! (20) (Only for input!)
// A7 FREE!! (21) (Only for input!)
//----------------------------------------------------------------------------
// Notes:
// EEPROM Usage:
// Pos Len Type Description
// 0 4 char ID string "Ar2R" (65, 114, 50, 82) Read as long=1379037761
// 4 1 byte Interrupts Per Second - Rekommended: 50
// 5 1 byte IPS-steps per centigrade degree - Rekommended: 1-5
// 6 1 byte Current profile number
// 7 1 byte Total number of stored profiles
// 8 2 int First FREE profile address
// 10 2 int Start of last run - Every 5/sec - Store: Goal temp, Actual temp, Power
// 12 2 int Last run - count
// All numbers are stored in lowest byte first order.
//
//----------------------------------------------------------------------------
// Changes:
// Both code (FLASH) and variables (RAM) were very LOW when combining all code parts!
// All text strings had to be moved from RAM to FLASH by using the F()-macro! Gave back
// quite a lot of bytes from RAM! Also some cool visuals had to go to give space! :-(
// Then came the tricky rewrite to save even more RAM!
// Low RAM can give very "FUNNY" errors! Not at all related to what fails!
//
// v029 - Compiled OK after adding EEPROM functions!
// v046 - Finished Reflow heating controller and display
// v050 - Added rotary encoder
// v051 - Integrated rotary encoder, menue and reflow
// v060 - Added miniMenu (last line selection)
// v065 - Rewritten rotary encoder with interrupt and acceleration
// v067 - Manual added and slight recoding of readEncoder
// v073 - readTemp added + a slightly better simulation
// v074 - Added admin() ... changed name from settings()
// v075 - Added dumpEEPROM(), printProfile()
// v076 - Added ticmarks, inverted diff presentation. Used 85% FLASH!
// v077 - Added showCurrentProfile()
// v078 - Added showSteps! VERY CLOSE TO MAXIMUM SIZE HERE! 32214 bytes
// v079 - Added edit().
// v080 - OPTIMIZED code in many places! 28408 bytes in size. A margin to limit!
// v086 - Added manualBasic() a simpler and smaller version. 26538 bytes
// v087 - Added three 10-step user profiles for editing 26538 bytes
// v088 - Optimized miniMenue() with 2 parms - got it down to 26478 bytes
// v089 - Added "Save" after editing - 26866 bytes
// v090 - Optimized resetEEPROM - 26412 bytes
// v091 - Added theromcouple function and MAX6675 lib - 26742 bytes
// v092 - Changed Themocouple pins and Encoder pins to "Release" settings!
// v093 - Changed DFACTOR to PPD meaning % (Power) Per Degree
// v094 - Removed function to edit profile (editP) - 25344 bytes
// v095 - Removed creation of empty user profiles - 25124 bytes
// v1.0 - Tried to make the code more arduino standard! Cost is more FLASH used!
// v1.1 - Removed my custom macros - Flash: 25368 bytes, RAM: 1268 bytes
// v1.2 - Added possibility to BOOSTER temperature for a while
// v1.2 - Changed Remain time to show "Remaining Time to start Cooling"
// V1.2 - Small adjustment for even printout with dumpEEPROM
// V1.2 - Shows version number at top of Admin screen
// v1.2 - Added Manual oven PPD - Flash: 28862 bytes, RAM: 1380 bytes
//----------------------------------------------------------------------------
// ToDo:
//
// BUG! A change in a profile is not updating the precalculated helper arrays!
// Workaround: Save as default profile and restart!
//
// The F() macro have started to give problems with the TFT print() functions!
// Workaround: Removed F() from these functions!
//----------------------------------------------------------------------------
// Notes:
//
//
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190217 Sfenn ar2uino.wordpress.com Original code v1.2
//----------------------------------------------------------------------------
//#include <Helpers.h>
#include <EEPROM.h>
#include <SPI.h>
#include <TFT_ST7735.h> // Lib for TFT Colour Diisplay
#include <max6675.h> // Lib for Thermocouple
// Pin definitions
#define ePinA 2 // Rotary encoder pin A Interrupt! (Can ONLY be pin 2 or pin 3!)
#define ePinB 3 // Rotary encoder pin B (ALL encoder pins should be
#define ePinSwitch 4 // Rotary encoder switch pin decoupled to GND by 0.1 uF capacitors!)
#define CIRCPIN 5 // Fan that circulates the air inside the oven (Possible PWM speed control)
#define LEDPIN 6 // LED to indicate power is ON. Connect via 220 ohm resistor
#define OPTOPIN 7 // Optocoupler via 220 ohm resistor. Controlls power to the heaters!
 // Change in the code to use FAST direct port manipulation (DPM)
 // If other pins than 6 & 7 are used - rewrite the DPM-code!
#define COOLPIN 8 // LED to indicate COOLING started - or use it to Open the door for cooling!
#define __DC 9 // TFT Color display DC pin
#define __CS1 10 // TFT Color display CS1 pin
#define LIQUIDLED A0 // LED that lights if solder is in liquid phase (>183 degrees C)
#define THERMDO A3 // Thermocouple MAX6675 D0
#define THERMCS A2 // Thermocouple MAX6675 CS
#define THERMCLK A1 // Thermocouple MAX6675 CLK
// Setup definitions
#define IPS 5 // Interrupts Per Second (Max=100)
// Also the smallest adjustable power parts (Ex: 100 --> 1% steps)
// IPS - This defines the granularity that the power can be adjusted by!
// Ex: IPS=50 gives that power can be adjusted 50 times per second
// and more importantly that gives 50 "steps" or 2% sized "steps" of power!
#define PROFSTART 33 // Start of Profile linked Storage
#define IDSIGNATURE 1379037761 // ID signature "Ar2R" for "Ar2uino Reflow Oven"
#define CLICKED 10000
#define GOTOMENUE 100
#define CLEARBUFFER 9999
#define AVGC 1 // Number of readings to calculate thermocouple average from (1=No average!)
#define MPPD 8 // Manual oven PPD - used ONLY for manual oven
byte PPD=8; // How many Percent (power) Per (Celcius) Degree
 // The higher PPD the coarser/more agressive is the regulation!
 // Ex: PPD=5 gives 20 steps á 5%. PPD=2 gives 50 steps á 2%.
 // PPD=20 gives 5 steps á 20%
// Global variables
volatile byte pwr = 0;
volatile int eChange, eDiff;
unsigned int timerPreload;
float pPower; // Percent Power On
long unsigned int intervalMillis = 1000, startMillis, nowMillis, nextMillis = 0;
int cycle = 0, profileRunTime, profileCoolTime, rStep, tal;
int doOnceWhenCool = 1, IPSminus1, profileAddr;
byte profileNo, profileSteps, typeP, profileLoaded = 0;
float profileTempNow, tempNow, tRead;
float tempWanted;
byte mode = 0;
// Here you can BOOST power TO the given % for a while to help following reflow profile
// TEST to find out where and how long and how much to boost for best result.
// Start boost a few seconds before you want the first reaction
// Do not boost for very long time as it takes over from temperature regulation
// Setting boostProfile to a nonexisting profile cancels the function
int boostProfile=99, boostStart=245, boostStop=280, boostPercent=150;
// Menue item data
int itemSel = 0; // Menue item selected
int itemCount = 5; // Number of menue items
char* itemText[] = { "Reflow", "Manual", "Edit", "Admin", "Off" };
unsigned int itemTextColour[] = {BLACK, BLACK, BLACK, WHITE, YELLOW};
unsigned int itemBackColour[] = {CYAN, GREEN, YELLOW, BLUE, RED};
// Create Display object
TFT_ST7735 tft = TFT_ST7735(__CS1, __DC);
// Initialize MAX6695 - thermocoulpe amplifier and ADC
MAX6675 thermocouple(THERMCLK, THERMCS, THERMDO);
// Current Profile arrays (Max 10 steps!)
char pName[10];
byte profileTemp[10]; // The temperature for each interval
byte profileTime[10]; // The length in secs for interval
// Profile helper arrays
float profileRamp[10]; // The ramp degrees/sec
int profileSecs[10]; // The continous timeline
int dx = 0;
//////////////////////////////////////////////////////////////////////////////////////
void setup()
{
 int i,s, x = 0, y = 0, cProf;
 // Fill the average temperature buffer (AVGC - Average Count controls over how long)
 for (i = 0; i < AVGC; i++) {
 readTemp(0);
 }

 // Attatch interrupt for encoder - ONLY use Arduino pin 2 or 3 (=ATmega pin 0 or 1)
 attachInterrupt(0, doEncoder, FALLING);
 // Used in the timer interrupt - calculate here to save a little time
 IPSminus1 = IPS - 1;
 // Set pins to output
 pinMode(LEDPIN, OUTPUT); pinMode(COOLPIN, OUTPUT);
 pinMode(OPTOPIN, OUTPUT); pinMode(LIQUIDLED, OUTPUT);
 // Setup encoder pins as inputs - with enabled pull up resistors!
 pinMode(ePinA, INPUT_PULLUP);
 pinMode(ePinB, INPUT_PULLUP);
 pinMode(ePinSwitch, INPUT_PULLUP);
 // Initialize Timer Interrupt
 initializeInterrupts(IPS);
 // Set up serial com
 Serial.begin(115200);
 Serial.println(F("Ar2uino Reflow Oven v1.2khg STARTED!"));
 Serial.println("");
 // FIRST TIME ONLY! Set up Oven Parameters and default profiles in EEPROM
 // Parameter: 0=First time only, 1=Always rewrite
 resetEEPROM(1); // To save 856 bytes: MOVE This whole function to a separate program!
 // As this is executed only ONCE it can be moved if memory is scarce!
// You can also run it ONCE and the COMMENT away the call!
// Saving: 856 bytes FLASH and 28 bytes RAM
 // Read stored current profile number
 cProf = EEPROM.read(6);
 // Load profile
 loadCurrentProfile(cProf);

 // Print some setup info
 Serial.print(F("IPS: ")); Serial.println(IPS);
 Serial.print(F("PPD ")); Serial.println(PPD);
 Serial.println("");
 // Greetings Display 330 bytes
 // BUG! BUG! BUG!
 // The F() macro causes problems so it has been removed below!
 // This causes around 20 more bytes of RAM to be used!
 // It has worked perfectly up until NOW! Why????
 tft.begin();
 tft.setRotation(1);
 tft.fillRect(0, 0, 160, 128, RED, BLUE); // Draw the background w gradient
 tft.setTextScale(3);
 tft.setCursor(40, 25, REL_XY);
 tft.setTextColor(WHITE);
 tft.println(F("Ar2uino"));
 tft.setCursor(40, 50, REL_XY);
 tft.setTextColor(GREEN);
 tft.println(F("Reflow"));
 tft.setCursor(50, 75, REL_XY);
 tft.setTextColor(RED);
 tft.println(F("Oven"));
 for (y = 95; y < 119; y += 2) {
 tft. drawLine(40 + x, y, 108 - x, y, YELLOW);
 x += 4;
 }
 delay(3000);
 // Set starting mode (GOTOMENUE=Menue, 0=Reflow, 1=Manual, 2=Save, 3=Admin, 4=Off)
 mode = GOTOMENUE;
}
//////////////////////////////////////////////////////////////////////////////////////
void loop()
{
 // As a end-line-comment below it is shown the number of bytes each part uses.
 // This has been used to minimize size to fit into processor program memmory (FLASH)
 /////////////////
 // Menue //
 /////////////////
 if (mode == GOTOMENUE) {
 digitalWrite(COOLPIN, 0);
 mode = 0;
 mode = menue(mode); // 472 bytes
 }
 /////////////////
 // Reflow //
 /////////////////
 else if (mode == 0) {
 reflow(); // 3594 bytes
 }
 /////////////////
 // Manual oven //
 /////////////////
 else if (mode == 1) {
 manualBasic(0); // 1116 bytes Rectangular display

 }
 /////////////////
 // Edit //
 /////////////////
 else if (mode == 2) {
// editP(); // 1430 bytes Removed due to oven limitations making it of little use!
 mode = GOTOMENUE; // Just returns to menue
 }
 /////////////////
 // Admin //
 /////////////////
 else if (mode == 3) {
 admin(); // 2430 bytes
 }
 /////////////////
 // OFF! //
 /////////////////
 else if (mode == 4) {

 // NOT Implemented! Using timer switch for this!
 // What should happen in YOUR implementation?
 mode = GOTOMENUE; // In my case --> Just returns to menue
 }
}
//########################################################################################################
//########################################################################################################
//########################################################################################################
void admin(void)
//---------------------------------------------------------------------------
// Administrative tools and settings
//----------------------------------------------------------------------------
// Parm Description
//
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
//
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int sp, sa;
 tft.clearScreen();
 // Write display top & bottom lines
 displayTopBottom("Admin v1.2khg", " ");
 // Show info about current and default profiles
 showProfileInfo();
 // Select admin action - until go back to Main menue is selected!
 while ((sa = adminMenue()) != 4) {
 // Print current profile info
 if (sa == 0) {
 printCurrentProfile();
 }
 // Select profile
 else if (sa == 1) {
 sp = selectProfile();
 Serial.print(F("Selected profile: ")); Serial.println(sp);
 // Load new profile into RAM - will NOT set it to DEFAULT!
 loadCurrentProfile(sp);
 // Show info
 showProfileInfo();
 }
 // Save default
 else if (sa == 2) {
 saveDefault();
 // Show info
 showProfileInfo();
 }
 // Dump EEPROM to Serial monitor!
 else if (sa == 3) {
 dumpEEPROM(1024);
 }
 }
 // Go back to main menue
 mode = GOTOMENUE;
 return;
}
int adminMenue(void)
//---------------------------------------------------------------------------
// Uses minimenue on the last line to select ...
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN The selected profile numeber or CLICKED for Menu
//----------------------------------------------------------------------------
// Notes:
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 int sel = 0, ret;
 // Show current profile info on display
 showMiniMenueItem("> Print Profile", CYAN, LIGHT_GREY);
 while ((ret = readEncoder(sel, 0, 4, false)) != CLICKED) {
 sel = ret;
 // Print current profile info
 if (sel == 0) {
 showMiniMenueItem("> Print Profile", CYAN, LIGHT_GREY);
 }
 // Select Profile
 else if (sel == 1) {
 showMiniMenueItem("> Select Profile", CYAN, LIGHT_GREY);
 }
 // Save current as default
 else if (sel == 2) {
 showMiniMenueItem("> Save Default", CYAN, LIGHT_GREY);
 }
 // Dump EEPROM
 else if (sel == 3) {
 showMiniMenueItem("> Dump EEPROM", CYAN, LIGHT_GREY);
 }
 // Menue (Keep this last in line!)
 else if (sel == 4) {
 showMiniMenueItem("> Exit!", BLUE, YELLOW);
 }
 }
 return (sel);
}
void saveDefault(void)
//---------------------------------------------------------------------------
// Saves the current profile as default profile
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 // Store the current profile number as the default
 EEPROM.write(6, profileNo);
}
int selectProfile(void)
//---------------------------------------------------------------------------
// Uses minimenue on the last line to select profile
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN The selected profile numeber or CLICKED for Menu
//----------------------------------------------------------------------------
// Notes:
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 int sel = 0, ret;
 byte count;
 char pNameShow[8];
 // Number of stored profiles
 count = EEPROM.read(7);
 // Initially show first profile name
 readProfileName(sel, pNameShow);
 showMiniMenueItem(pNameShow, CYAN, LIGHT_GREY);
 while ((ret = readEncoder(sel, 0, count, false)) != CLICKED) {
 sel = ret;
 // Read profile name
 readProfileName(sel, pNameShow);
 // Go to Menue
 if (sel == (count)) {
 showMiniMenueItem("> Back", BLUE, YELLOW);
 }
 // Show profile name
 else {
 showMiniMenueItem(pNameShow, CYAN, LIGHT_GREY);
 }
 }
 if (sel == count) return (CLICKED);
 return (sel);
}
void showMiniMenueItem(char *item, int textColour, int backColour)
//---------------------------------------------------------------------------
// Shows text on the last line in the specified colours
//----------------------------------------------------------------------------
// Parm Description
// item The text to be shown
// textColour Text colour
// backColour Background colour
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 tft.fillRect(0, 119, 160, 9, backColour);
 tft.setCursor(CENTER, 120);
 tft.setTextScale(1);
 tft.setTextColor(textColour, backColour );
 tft.print(item);
}
void showProfileInfo(void)
//---------------------------------------------------------------------------
// Shows current profile on display
//----------------------------------------------------------------------------
// Globals Description
// start Start address (22-800)
// count Count of array elements to store
// pName Name of profile (8 chars)
// temp The end temparature for that interval
// secs The time for that interval
//
// RETURN Next writable EEPROM position
//----------------------------------------------------------------------------
// Notes:
// Byte 20 contains number of profiles stored
// Byte 22 contains next free address to write profile to
// Data stored in EEPROM for each profile as follows:
// Bytes Desc
// 1 Count of array elements
// 8 Name of profile incl blanks to fill 8 chars (Ex: "Lee ")
// 1*count Array with end temperatures for each interval
// 1*count Array with times in secs for each interval
//
// If a profile is deleted ALL remaining profiles will be written to EEPROM!
// (That is to handle garbage collection!)
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 int s, i, x, y;
 byte def;
 char nam[8];
 // Clear area before writing new text
 tft.fillRect(0, 9, 160, 118, BLACK);
 // Profile info
 tft.setTextScale(1);
 tft.setCursor(0, 20);
 tft.setTextColor(WHITE);
 // Show info about currently loaded profile
 tft.print("Current:"); tft.getCursor(x, y); tft.setTextScale(2);
 tft.setTextColor(CYAN); tft.setCursor(40, y - 15); tft.print(pName);
 tft.setTextScale(1); 
 tft.setCursor(40, y - 5); 
 tft.println("");
 y += 10; displayWithTab("Id#:", profileNo,y);
 y += 10; displayWithTab("Steps:", profileSteps,y);
 y += 10; displayWithTab("Type:", typeP,y);
 y += 10; displayWithTab("Secs:", profileRunTime,y);
 y += 10; displayWithTab("St150:", getSecTo150(),y);
 y += 10; displayWithTab("TAL:", getTAL(),y);
 // Read stored default profile number
 def = EEPROM.read(6);
 readProfileName(def, nam);
 // Show name of default profile
 tft.setTextScale(1);
 tft.setCursor(0, 105);
 tft.setTextColor(WHITE);
 tft.print("Default:"); tft.getCursor(x, y); tft.setTextScale(1);
 tft.setTextColor(GREEN); tft.setCursor(40, 105); tft.print(nam);
 tft.setTextScale(1); tft.setCursor(40, y + 2); tft.println("");
}
void displayWithTab(char *txt, int tal, byte y) {
 //int x, y;  
 tft.setCursor(0, y);
 tft.setTextScale(1);
 tft.setTextColor(WHITE);
 tft.print(txt);
 //tft.getCursor(x, y);
 tft.setCursor(40, y);
 tft.setTextColor(YELLOW);
 tft.println(tal);
}
//########################################################################################################
//########################################################################################################
//########################################################################################################
void manualBasic(int start)
//---------------------------------------------------------------------------
// Manual Oven - Basic display to save FLASH memory
//----------------------------------------------------------------------------
// Parm Description
// start Start temperature
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
// Uses three(3) pins as input:
// The ISR uses ePinA and ePinB. This function uses ePinSwitch.
// The nice cool round display had to go to save FLASH memory! :-(
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int pChange, eStep = 1, tShow;
 static unsigned long nowTime, prevTime, prevGapTime, gapTime, updTime = 0, pPos;
 int mi = 0, ma = 250, wrap = false;
 static int result;
 byte keepPPD;
 result = start;
 // Keep Reflow oven PPD
 keepPPD=PPD;
 // Set special "milder" PPD for manual oven
 PPD=MPPD;
 // Clear display
 tft.clearScreen();
 displayTopBottom("Manual", "Exit!");
 // Draw whole manual display w header & footer
 displayManualBasic(0, 0, 0);
 // Transfer values to global variables for encoder interrupt routine
 pChange = eChange = 1;
 // Loop until the encoder button has been clicked
 while (digitalRead(ePinSwitch)) {
 // Position changed! eDiff will indicate +1 or -1
 if ((pChange != eChange)) {
 // eChange is a global var that is changed via the "doEncoder" interrupt routine
 pChange = eChange;
 // Calculate how long between each change
 prevGapTime = gapTime;
 prevTime = nowTime;
 nowTime = millis();
 gapTime = nowTime - prevTime;
 // If a large range - then use bigger steps if the encoder is turned quicker (shorter gaps)
 if ((ma - mi) > 50 && (gapTime + prevGapTime) < 100) eStep = 10; else eStep = 1;
 // Calculate new resulting position
 result += (eDiff * eStep);
 // Jump to next higher/lower even 10-step
 if (eStep > 2) {
 if (eDiff == 1) result = result + (10 - (result % 10));
 else if (eDiff == -1) result = result - (result % 10);
 }
 // Keep selected value within limits and act according to wrap setting
 if (result > ma) result = ma;
 if (result < mi) result = mi;
 // Update just setting on the display
 displayManualBasic(2, tShow, result);
 // Set the desired temperature - the timer interrupt will adjust the power!
 rStep=0;
 adjustTemp(result);
 }
 // Refresh the desired temperature every second - the timer interrupt will adjust the power!
 if (millis() - updTime > 1000) {
 rStep=0;
 adjustTemp(result);
 updTime = millis();
 // if different - Update the display with new read temperature
 if (tempNow != tShow) {
 tShow = tempNow;
 displayManualBasic(1, tShow, result);
 }
 }
 }
 // Switch off oven!
 pwr = 0;
 // Reset PPD back to Reflow oven PPD
 PPD=keepPPD;
 // Animation!
 for (mi = result; mi > -1; mi -= 2) {
 displayManualBasic(2, tShow, mi);
 }
 // Handle contact bounce
 delay(500);
 mode = GOTOMENUE;
 return;
}
void displayManualBasic(int upd, int tCurrent, int setting)
//---------------------------------------------------------------------------
// Displays all the information on the simplified manual oven
//----------------------------------------------------------------------------
// Parm Description
// upd 0=Initial (w header & footer), 1=tCurrent, 2=Setting, 3=Both
// tCurrent Current temperature - as a rectangular band on right
// setting Desired temperature - as a red line across current rectangel
//
// RETURN
//----------------------------------------------------------------------------
// Notes:
// This has been coded to save memory compared to the cooler circular display
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 // Update initial (With header and footer)
 if (upd == 0) {
 }
 // Update initial or update tCurrent
 if (upd == 0 || upd == 1 || upd == 3) {
 bandScale(tCurrent, 250, 80, 20, 47, RED);
 // value, min, max, xpos, ypos, width
 // Print out the headings in the middle of the scale
 tft.setCursor(35, 25, REL_XY);
 tft.setTextScale(1);
 tft.setTextColor(RED);
 tft.print("Current");
 tft.setCursor(103, 14, REL_XY);
 tft.print("Current");
 // Print out the "tCurrent" value i the middle of the scale
 tft.setCursor(35, 42, REL_XY);
 tft.setTextScale(3);
 tft.setTextColor(CYAN);
 tft.fillRect(7, 31, 55, 25, BLACK);
 tft.print((tCurrent < 0) ? 0 : tCurrent, DEC);
 }
 // Update initial or setting
 if (upd == 0 || upd == 2 || upd == 3) {
 // Print out the lower headings in the middle of the scale
 tft.setCursor(35, 67, REL_XY);
 tft.setTextScale(1);
 tft.setTextColor(GREEN);
 tft.print("Set");
 tft.setCursor(73, 14, REL_XY);
 tft.print("Set");
 // Print out the lower "tCurrent" value i the middle of the scale
 tft.setCursor(35, 82, REL_XY);
 tft.setTextScale(2);
 tft.setTextColor(YELLOW);
 tft.fillRect(13, 73, 40, 17, BLACK);
 tft.print(setting, DEC);
 // Draw the ticmarker for the setting
 bandScale(setting, 250, 68, 20, 8, GREEN);
 // value, max, xpos, ypos, width, colour
 }
}
void bandScale(int val, int maxV, byte x, byte y, byte w, unsigned int colour) {
 float scale;
 int sizeH = 96, lower, upper;
 scale = float(sizeH) / float(maxV);
 lower = int(float(val) * scale);
 upper = sizeH - lower;
 tft.fillRect(x, y, w, upper, DARK_GREY);
 tft.fillRect(x, y + upper, w, lower, colour, BLUE);
}
void displayTopBottom(char *topText, char *botText)
//---------------------------------------------------------------------------
// Draws the top and bottom lines on the display in predefined colours
//----------------------------------------------------------------------------
// Parm Description
// topText
// botText
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 // Write display top line
 tft.fillRect(0, 0, 160, 9, BLUE);
 tft.setCursor(CENTER, 1);
 tft.setTextScale(1);
 tft.setTextColor(WHITE, BLUE);
 tft.print(topText);
 // Write display bottom line (Menue item)
 showMiniMenueItem(botText, CYAN, LIGHT_GREY);
}
//########################################################################################################
//########################################################################################################
//########################################################################################################
int menue(int selI)
//---------------------------------------------------------------------------
// Menu selection
//----------------------------------------------------------------------------
// Parm Description
// selI Initially selected item on menue (Numbered from 0-4)
//
// RETURN Selected item (0-4)
//----------------------------------------------------------------------------
// Notes:
// Draws initial screen display
// Calls item selector
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 int i, y = -8, heightItem = 18, gapItem = 3, item = 0, columnItem = 20, textPos = 30;
 tft.clearScreen();
 // Write display top & bottom lines
 displayTopBottom("Select Function", " ");
 // Write menue items
 for (i = 0; i < itemCount; i++) {
 y += heightItem + gapItem;
 tft.fillRect(columnItem, y, 128, heightItem, itemBackColour[i]);
 tft.setTextColor(itemTextColour[i]);
 tft.setTextScale(2);
 tft.setCursor(textPos, y + 2);
 tft.print(itemText[i]);
 }
 // Read selected menue item
 selI = selectItem(selI, heightItem, gapItem, columnItem);
 return (selI);
}
int selectItem(int sel, int heightItem, int gapItem, int columnItem)
//---------------------------------------------------------------------------
// Selects the item on menue display
//----------------------------------------------------------------------------
// Parm Description
// sel Initial selection
// heightItem Height (in pixels) of each item line on menue
// gapItem Gap (in pixels) betwen item line on menue
// columItem Width (in pixels) of first column (where triangle is drawn)
//
// RETURN Selected item number (0-4)
//----------------------------------------------------------------------------
// Notes:
// Draws a triangle in front of currently selected item - also draws a white
// edge along the currently selected item.
// You decide on the currently select the item by clicking the rotary encoder
// button!
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 int y, i, enc;
 // Draw current selection
 y = sel * (heightItem + gapItem) - 8 + heightItem;
 tft.fillTriangle(3, y + 5, 3, y + heightItem - 1, columnItem - 5, y + (heightItem / 2) + 2, YELLOW);
 while ((enc = readEncoder(sel, 0, itemCount - 1, false)) != CLICKED) {
 sel = enc;
 // Erase old selection
 tft.fillTriangle(3, y + 5, 3, y + heightItem - 1, columnItem - 5, y + (heightItem / 2) + 2, BLACK);
 y = sel * (heightItem + gapItem) - 8 + heightItem;
 // Draw current selection
 tft.fillTriangle(3, y + 5, 3, y + heightItem - 1, columnItem - 5, y + (heightItem / 2) + 2, YELLOW);
 }
 return (sel);
}
//########################################################################################################
//########################################################################################################
//########################################################################################################
void reflow(void)
//---------------------------------------------------------------------------
// Reflow process - adjust the power to follow current profile
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN
//----------------------------------------------------------------------------
// Notes:
// Displays PreReflow and Reflow information
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 boolean stopReflow = false;
 byte y = 0;
 int tal = 0, i;
 // Print header info to serial monitor
 Serial.println("");
 Serial.println(F("Reflow started!"));
 Serial.print(F("Profile: ")); Serial.println(pName);
 Serial.print(F("Total time: ")); Serial.print(profileRunTime);
 Serial.println(" secs");
 Serial.println("");
 Serial.print(F("Sec")); Serial.write(9);
 Serial.print(F("Prof")); Serial.write(9);
 Serial.print(F("Temp")); Serial.write(9);
 Serial.print(F("Diff")); Serial.write(9);
 Serial.println(F("%Power"));
 // Intial display of PreReflow information
 tft.clearScreen();
 // Write display top & bottom lines
 displayTopBottom("Reflow Settings", "> Exit!");
 // Draw profile name
 y = 33;
 tft.setCursor(0, y);
 tft.setTextScale(1);
 tft.setTextColor(WHITE);
 tft.setTextScale(1);
 tft.print("Profile: ");
 tft.setTextColor(CYAN);
 tft.setTextScale(2);
 tft.setCursor(38, y - 7);
 tft.print(pName);
 // Show the current reflow settings
 y += 18; displayReflowInfo("Sec:", YELLOW, profileRunTime, y);
 y += 18; displayReflowInfo("St150:", GREEN, getSecTo150(), y);
 y += 18; displayReflowInfo("TAL:", RED, getTAL(), y);
 // Wait for selection - go to menue ... OR ...
 if (miniMenue("> Start Reflow!", "") == 0) {
 mode = GOTOMENUE;
 return;
 }
 ///////////////////////////
 // ... Start the REFLOW! //
 ///////////////////////////
 // Intial display for Reflow
 tft.clearScreen();
  dx = 250;
 // Write display top & bottom lines
 displayTopBottom("Reflow", "Preheat");
 // Preheat full power for 10 sec
 pwr=100.0; // Inerrupt routine will set power to value of "pwr" variable
 // Print "Preheat..." in middle of the screen
 for (i=0;i>0;i--){
 
 tft.setCursor(74, 50, REL_XY);
 tft.setTextScale(2);
 tft.setTextColor(YELLOW);
 tft.print("Preheat ");
 tft.fillRect(108, 43, 40, 15, BLACK);
 tft.print(i, DEC);
 
 delay(1000);
 }
 // Intial display for Reflow
 tft.clearScreen();
 tft.fillRect(45, 13, 120, 103, RED, BLUE);
 // Write display top & bottom lines
 displayTopBottom("", "Stop!");
 // Write display top line - Special with profile name
 headings("Profile: ", 53, 1, WHITE, BLUE);
 tft.print(pName);
 // Draw temp heading
 headings("Temp", 20, 33, LIGHT_GREY, BLACK);
 // Draw diff heading
 headings("Setpoint", 20, 61, LIGHT_GREY, BLACK);
 // Draw remain heading
 // Now shows remaining time to start cooling!
 headings("Remain", 20, 89, LIGHT_GREY, BLACK);
 // Draw Power heading
 headings("Pwr:", 12, 103, LIGHT_GREY, BLACK);
 // Draw TAL heading
 headings("TAL:", 12, 112, LIGHT_GREY, BLACK);
 // Start reflow from beginning
 nextMillis = 0;
drawTempScale(0, 0, 0, 0);
 // NOW! - Memorize the start moment
 startMillis = millis();
 // LOOP as long as we are in Reflow mode
 while (mode == 0) {
 // Get current elapsed time since start
 nowMillis = millis() - startMillis;
 // 'nextMillis' controls how often profile temperature update will occur
 if (nowMillis > nextMillis) {
 // Calculate the next time to update profile temperature
 nextMillis += intervalMillis;
 // Get the desired profile temperature for the current NOW time (in secs)
 profileTempNow = getReflowProfileTemp(nowMillis / 1000);
 // Adjust the power so that temperature follows the profile
 adjustTemp(profileTempNow);
 // Measure and print current actual temperature and profile temp to serial monitor
 Serial.println("");
 Serial.print(nowMillis / 1000, DEC);
 Serial.write(9);
 Serial.print(profileTempNow, 0);
 Serial.write(9);
 Serial.print(tempNow, 0);
 Serial.write(9);
 Serial.print(tempNow - profileTempNow, 0);
 Serial.write(9);
 Serial.print(pPower, 0);
 Serial.write(9);
 // Update TAL if over 183 C and light the LIQUIDLED!
 if (tempNow > 183) {tal++; digitalWrite(LIQUIDLED, 1);} else { digitalWrite(LIQUIDLED, 0); }
 // Draw temperature diagram on display
 drawTempScale(45, 13, 120, 103);
 // Draw current temp centered
 tft.setTextScale(2);
 tft.setCursor(20, 22, REL_XY);
 tft.setTextColor(WHITE);
 if (tempNow > 183) tft.setTextColor(CYAN);
 tft.fillRect(1, 14, 40, 15, BLACK);
 tft.print(int(tempNow), DEC);
 // Draw diff temp centered with a separate minus sign
 tft.setTextScale(2);
 tft.fillRect(1, 42, 35, 15, BLACK);
// if (int(tempNow - profileTempNow) < 0) {
 //tft. drawLine(1, 50, 17, 10, YELLOW);
// }
 tft.setCursor(20, 50, REL_XY);
 tft.setTextColor(YELLOW); 
//tft.print(abs(int(profileTempNow - tempNow)), DEC);
 tft.print(abs(int(profileTempNow)), DEC);
 // Draw time remaining centered
 tft.setTextScale(2);
 tft.setCursor(20, 78, REL_XY);
 tft.setTextColor(GREEN);
 tft.fillRect(1, 70, 40, 15, BLACK);
 tft.print(profileCoolTime - (nowMillis / 1000), DEC);
 // Draw power centered
 tft.setTextScale(1);
 tft.setCursor(32, 103, REL_XY);
 tft.setTextColor(RED);
 tft.fillRect(22, 99, 18, 8, BLACK);
 tft.print(int(pPower), DEC);
 // Draw TAL centered
 tft.setTextScale(1);
 tft.setCursor(32, 112, REL_XY);
 tft.setTextColor(CYAN);
 tft.fillRect(22, 108, 18, 8, BLACK);
 tft.print(tal, DEC);
 }
 // Keypress detected while in reflow - stop reflow!
 if (digitalRead(ePinSwitch) == 0) {
 stopReflow = true;
 Serial.println("Clicked BREAK!");
 }
 // Profile is FINISHED! - It has played full time - so STOP here!
 if (stopReflow || ((nowMillis / 1000) > profileRunTime)) {
 Serial.println("Profile FINISHED!");
 // Security power off
 pwr = 0;
 // Display PostReflow information
 tft.clearScreen();
 // Write display top & bottom lines
 displayTopBottom("Post Reflow Info", "Exit!");
 // Show info of the last reflow
 // Draw profile name
 y = 33;
 tft.setCursor(0, y);
 tft.setTextScale(1);
 tft.setTextColor(WHITE);
 tft.print("Profile: ");
 tft.setTextColor(CYAN);
 tft.setTextScale(2);
 tft.setCursor(38, y - 7);
 tft.print(pName);
 // Show the current reflow settings
 y += 18; displayReflowInfo("Sec:", YELLOW, profileRunTime, y);
 y += 18; displayReflowInfo("St150:", GREEN, getSecTo150(), y);
 y += 18; displayReflowInfo("TAL:", RED, tal, y);
 // Wait for click
 delay(2000);
 waitForClick();
 // Go back to menue
 mode = GOTOMENUE;
 }
 }
}
void headings(char *text, byte cx, byte cy, int textC, int textB) {
 tft.setCursor(cx, cy, REL_XY);
 tft.setTextScale(1);
 tft.setTextColor(textC, textB);
 tft.print(text);
}
void displayReflowInfo(char *head, int colour, int value, byte y) {
 tft.setCursor(0, y);
 tft.setTextScale(1);
 tft.setTextColor(WHITE);
 tft.setTextScale(1);
 tft.print(head);
 tft.setTextColor(colour);
 tft.setTextScale(2);
 tft.setCursor(38, y - 7);
 tft.print(value);
}
void drawTempScale(int x, int y, int w, int h){
//---------------------------------------------------------------------------
// Draw reflow temperature scale
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN
//----------------------------------------------------------------------------
// Notes:
// Redraws the whole display every time!
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
  byte dy;

if (dx >= 200){ 
  delay(1);
  
  tft.fillRect(x, y, w, h, RED, BLUE);
  tft.drawLine(45, 95, 160, 95, LIGHT_GREY);
  tft.drawLine(45, 75, 160, 75, LIGHT_GREY);
  tft.drawLine(45, 55, 160, 55, LIGHT_GREY);
  tft.drawLine(45, 35, 160, 35, LIGHT_GREY);
  tft.setTextScale(1);
  tft.setTextColor(LIGHT_GREY);
  tft.setCursor(147, 87);
  tft.print("50");
  tft.setCursor(142, 67);
  tft.print("100");
  tft.setCursor(142, 47);
  tft.print("150");
  tft.setCursor(140, 27);
  tft.print("200");
dx = 0;
}

//Target Temperature
dy = (profileTempNow / 2.5);
tft.drawPixel(45 + dx, 115 - dy, YELLOW);

//Actual Temperature
dy = (tempNow / 2.5);
tft.drawPixel(45 + dx, 115 - dy, WHITE);

dx = dx + 1;

if (dx == 115){
 // Draw the background w gradient
 tft.fillRect(x, y, w, h, RED, BLUE);
  tft.drawLine(45, 95, 160, 95, LIGHT_GREY);
  tft.drawLine(45, 75, 160, 75, LIGHT_GREY);
  tft.drawLine(45, 55, 160, 55, LIGHT_GREY);
  tft.drawLine(45, 35, 160, 35, LIGHT_GREY);
  tft.setTextScale(1);
  tft.setTextColor(LIGHT_GREY);
  tft.setCursor(147, 87);
  tft.print("50");
  tft.setCursor(142, 67);
  tft.print("100");
  tft.setCursor(142, 47);
  tft.print("150");
  tft.setCursor(140, 27);
  tft.print("200");
 dx = 0;
}

}
/*
 {

 static int vBuff[34];
 static byte insertPos = 0;
 byte drawPos1, drawPos2, i;
 int vScale = 2, hScale, tm;
 hScale = w / 34;
 if (value == CLEARBUFFER) {
 insertPos = 0;
 for (i = 0; i < 34; i++) vBuff[i] = 0;
 return;
 }
 // Limit value so that it is not drawn outside of "graf area"
 if (value > 25) value = 25;
 if (value < -25) value = -25;
 // Put value into circular buffer
 vBuff[insertPos++] = value; if (insertPos > 33) insertPos = 0;
 // Draw the background w gradient
 tft.fillRect(x, y, w, h, RED, BLUE);
 // Draw title on the scale
 tft.setCursor(x + 3, y + 3);
 tft.setTextScale(1);
 tft.setTextColor(YELLOW);
 tft.print("Diff");
 // Draw ticmarks on scale
 for (tm = -15; tm < 21 ; tm += 5) {
 tft. drawLine(x, y + (h / 2) + (tm * vScale), x + (tm % 10 ? 0 : 3), y + (h / 2) + (tm * vScale),
YELLOW);
 }
 // Draw "zero" line - Maybe wait til last?
 tft. drawLine(x, y + (h / 2), x + w - 1, y + (h / 2), LIGHT_GREY);
 // Draw "diff value" line
 drawPos1 = insertPos;
 drawPos2 = drawPos1 + 1; if (drawPos2 > 33) drawPos2 = 0;
 for (i = 0; i < 33; i++) {
 tft. drawLine(x + (i * hScale), y + (h / 2) - (vBuff[drawPos1]*vScale), x + ((i + 1)*hScale), y + (h /
2) - (vBuff[drawPos2]*vScale), WHITE);
 if (++drawPos1 > 33) drawPos1 = 0;
 if (++drawPos2 > 33) drawPos2 = 0;
 // delay(100);
 }
}
*/
//########################################################################################################
//########################################################################################################
//########################################################################################################
void initializeInterrupts(int ipSec)
//---------------------------------------------------------------------------
// Initialize desired timer interrupt frequency
//----------------------------------------------------------------------------
// Parm Description
// ipSec Interrupts per second (50 recommended)
// RETURN
//----------------------------------------------------------------------------
// Notes:
// The ipSec sets interrupts per seconds - that is the smallest part that the
// power can be controlled! Is used to switch on or off cycles of the 50 Hz
// mains power.
// ipSec max is 100 - every half of the sine wave is controllable
// 50 - controlls a full sine wave (positive and negative)
// 25 - controlls 2 full sine waves
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 // Initialize timer1 for desired interrupt frequency
 noInterrupts(); // Disable all interrupts
 TCCR1A = 0;
 TCCR1B = 0;
 timerPreload = 65536L - (62500 / ipSec); // Value for 256 prescaler
 /*
 timerPreload = 59286; // 10Hz value is: 65536-(16MHz/prescaler 256/10Hz)
 timerPreload = 64286; // 50Hz value is: 65536-(16MHz/prescaler 256/50Hz)
 */
 TCNT1 = timerPreload; // Preload timer
 TCCR1B |= (1 << CS12); // 256 prescaler
 TIMSK1 |= (1 << TOIE1); // Enable timer overflow interrupt
 interrupts(); // Enable all interrupts again
}
int loadCurrentProfile(int sel)
//---------------------------------------------------------------------------
// Load selected profile into memory
//----------------------------------------------------------------------------
// Parm Description
// sel Selected profile to load
//
// RETURN 1=OK, 0=ERROR
//----------------------------------------------------------------------------
// Notes:
// Reads profile into global memory arrays and sets "profileLOaded"
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 // Store current profile number
 profileNo = sel;
 // Load currently selected profile into memory
 if (readProfile(profileNo, &profileSteps, &typeP, pName, profileTemp, profileTime)) {
 Serial.print(F("Loaded profile: ")); Serial.println(pName);
 profileLoaded = 1;
 // Prepare current profile
 prepProfile();
 return (1);
 }
 // Profile NOT found!
 else {
 Serial.print(F("ERROR! Current profile NOT found:")); Serial.println(profileNo);
 profileLoaded = 0;
 return (0);
 }
}
void prepProfile()
//---------------------------------------------------------------------------
// Prepare helper arrays for the current reflow profile
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN temperature
//----------------------------------------------------------------------------
// Notes:
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 byte s;
 // Set up first interval on timeline
 profileSecs[0] = profileTime[1];
 // Set current temperatures in profile (if it is set to "0")
 if (profileTemp[0] == 0) profileTemp[0] = 25; // = readTemp(0); TEST!
 if (profileTemp[profileSteps - 1] == 0) profileTemp[profileSteps - 1] = 25; // = readTemp(0); TEST!
 // For each step in the profile ...
 for (s = 0; s < (profileSteps - 1); s++) {
 // ... calculate the ramp
 profileRamp[s] = (float(profileTemp[s + 1]) - float(profileTemp[s])) / float(profileTime[s + 1]);
 // ... calculate the timeline
 if (s > 0) profileSecs[s] = profileSecs[s - 1] + profileTime[s + 1];
 }
 // Calculate total run time for the reflow profile
 // ASSUMED to be the second last time!
 profileRunTime = profileSecs[profileSteps - 2];
 Serial.print("ProfileRunTime: ");
 Serial.println(profileRunTime);
 // Calculate total time before cooling starts for the reflow profile
 // ASSUMED to be the third last time!
 profileCoolTime = profileSecs[profileSteps - 3];
 Serial.print("ProfileCoolTime: ");
 Serial.println(profileCoolTime);
}
float getReflowProfileTemp(int sec)
//---------------------------------------------------------------------------
// Returns the temperature at a given sec for the current reflow profile
//----------------------------------------------------------------------------
// Parm Description
// sec Secs since start that you want the profile temperature for
//
// RETURN temperature
//----------------------------------------------------------------------------
// Notes:
// Interpolates temps along ramps!
// "rStep" is a global variable that AFTER a call to this function is set to
// the current reflow profile step
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 // Limit to values above zero
 if (sec<0) sec=0;
 // Step through profile until correct interval is found
 rStep = 0;
 while (sec > profileSecs[rStep] && rStep < profileSteps) rStep++;
 if (rStep == 0) return (float(profileTemp[rStep]) + profileRamp[rStep] * (float(sec)));
 else return (float(profileTemp[rStep]) + profileRamp[rStep] * (float(sec) - float(profileSecs[rStep -
1])));
}
int getTAL(void)
//---------------------------------------------------------------------------
// Get Time Above Liquid
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN seconds above 183 C
//----------------------------------------------------------------------------
// Notes:
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int i, tal = 0;
 float temp;
 for (i = 0; i < profileRunTime; i++) {
 if ((temp = getReflowProfileTemp(i)) > 183) tal++;
 }
 return (tal);
}
int getSecTo150(void)
//---------------------------------------------------------------------------
// Seconds to 150 C
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN
//----------------------------------------------------------------------------
// Notes:
// This is the time when it should be safe to move the PCB around
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int i, st150 = 0;
 boolean first = true;
 float temp;
 for (i = 0; i < profileRunTime; i++) {
 temp = getReflowProfileTemp(i);
 if (!first && (temp < 150)) return (st150);
 if (first && (temp > 160)) first = false;
 st150++;
 }
}
float readTemp(int channel)
//---------------------------------------------------------------------------
// Reads the current temperature from the temperature sensor
//----------------------------------------------------------------------------
// Parm Description
// channel Which temperature sensor channel to read
//
// RETURN temperature
//----------------------------------------------------------------------------
// Notes:
// Defined AVGC controlls the Average count
// One reading takes 35 mS.
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{

 float t;
 byte i;
 static byte idx=0;
 static float tavg[AVGC];
 // Read temperature
 tRead = thermocouple.readCelsius();
 // Add measurement to the circular buffer
 tavg[idx++] = tRead; if (idx > (AVGC - 1)) idx = 0;

 // Calculate the sliding average
 t = 0.0;
 for (i = 0; i < AVGC; i++) {
 t += tavg[i];
 }
 t = t / float(AVGC);
/*
 // Just do an average read
 t = 0.0;
 for (i = 0; i < AVGC; i++) {
 t += thermocouple.readCelsius();;
 }
 t = t / float(AVGC);
*/
 return (t);
}
void adjustTemp(float wantTemp)
//---------------------------------------------------------------------------
// Adjustment of temperature by controlling the POWER to the heaters
//----------------------------------------------------------------------------
// Parm Description
// wantTemp The wanted temperature
//
// RETURN void
//----------------------------------------------------------------------------
// Notes:
// Here any extra handling to hold the temperature can be added!
// The actual POWER adjustment is done in the timer interrupt service routine!
// NOTE: Just set the variable "pwr" to the wanted power - ISR will do the rest!
//
// If used manually MUST set rStep=0
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 float change;
 // Memorize currently wanted temp - globaly!
 tempWanted = wantTemp;
 // Calculate the needed change (increase or decrese) to reach wanted temperature
 tempNow=readTemp(0);
 change = wantTemp - tempNow;
 // This is NOT (at the moment) handling the need to cool the oven!
 // Here you can put code to open the door a little or switch on fan to blow cool air into oven
 if (change < 0.0) {
 change = 0.0;
 }
 // Linear proportional POSITIVE power adjustment - How many percent power
 pPower=change*float(PPD);
 // Boost
 if ((profileNo==boostProfile) && ((nowMillis/1000)>boostStart) && ((nowMillis/1000)<boostStop)) {
 pPower=boostPercent;
 }
 // Max 100% power handled - so limit if over!
 if (pPower>100.0) pPower=100.0;
 //////////////
 // FINISHED //
 //////////////
 // Ramp is negative (cooling has started!)
 if (profileRamp[rStep] < 0) {
 // Switch off heating from now on! Should not be needed as the profile ramp SINKS!
 pwr = 0; pPower=0.0;
 // Do something ONCE when oven should start to cool down
 if (doOnceWhenCool) {
 Serial.print(F("Cooling starts!"));

 // Switch on LED to indicate cooling has started!
 digitalWrite(COOLPIN, 1);
 // Change to show remaining time of whole profile
 profileCoolTime=profileRunTime;

 doOnceWhenCool = 0;
 }
 }
 /////////////////////
 // ACTIVE COOLING! //
 /////////////////////
 else if (pPower < 0.0) {
 // <-- Here you can put code to open the door or switch on fan to blow cool air into oven
 }
 //////////////
 // HEATING! //
 //////////////
 else {
 // Convert % power to Cycles ON! (The ISR will handle the power setting!)
 pwr = int(pPower/(100.0/float(IPS)));
 }
}
ISR(TIMER1_OVF_vect)
//---------------------------------------------------------------------------
// Interrupt Service Routine (ISR) to set desired power duty cycle for Triac
//----------------------------------------------------------------------------
// Global var Description
// pwr Number of IPS steps that you want the power ON!
//
// RETURN void
//----------------------------------------------------------------------------
// Notes:
// This is a timer interrupt service routine that is called IPS times/sec
// As such it uses NO PARAMETERS but instead uses a global variable for input!
// External variable changed here so 'volatile' is needed for "pwr".
//
// NOTE! Change to Direct Port Manipulation if needed!
// Uses pin 6 & 7 for Triac and indicator LED!
// Remember to ONLY change pins 6 & 7!!!!!
// Remove digitalWrite.. lines and uncomment "DPM!" lines below!
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 // preload timer
 TCNT1 = timerPreload;
 // Keep track of which cycle it is during each second
 if (++cycle > IPSminus1) cycle = 0;
 // Swich on Triac for the desired power duty cycles in IPS steps
 // Using "SLOW" arduino functions for this - still seems to work OK!
 //if (pwr > cycle) {
 //digitalWrite(OPTOPIN, 1);
 //digitalWrite(LEDPIN, 1);
 //}
 //else {
 //digitalWrite(OPTOPIN, 0);
 //digitalWrite(LEDPIN, 0);
 //}
 // Use very FAST direct Port Manipulation to switch on Triac on pin 6&7 for the
 // desired power duty cycles (0-100% in IPS steps)
 if (pwr>cycle) PORTD |= B11000000; // sets ONLY pins 7 and 6 HIGH DPM!
 else PORTD &= B00111111; // sets ONLY pins 7 and 6 LOW DPM!
}
void printCurrentProfile(void)
//---------------------------------------------------------------------------
// Print out current profile info
//----------------------------------------------------------------------------
// Globals Description
// start Start address (22-800)
// count Count of array elements to store
// pName Name of profile (8 chars)
// temp The end temparature for that interval
// secs The time for that interval
//
// RETURN Next writable EEPROM position
//----------------------------------------------------------------------------
// Notes:
// Byte 20 contains number of profiles stored
// Byte 22 contains next free address to write profile to
// Data stored in EEPROM for each profile as follows:
// Bytes Desc
// 1 Count of array elements
// 8 Name of profile incl blanks to fill 8 chars (Ex: "Lee ")
// 1*count Array with end temperatures for each interval
// 1*count Array with times in secs for each interval
//
// If a profile is deleted ALL remaining profiles will be written to EEPROM!
// (That is to handle garbage collection!)
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int s, i;
 // Profile info
 Serial.println("");
 for (i = 0; i < 60; i++) Serial.print("=");
 Serial.println("");
 Serial.print(F("Name:")); Serial.write(9); Serial.println(pName);
 Serial.print(F("No:")); Serial.write(9); Serial.println(profileNo);
 Serial.print(F("Steps:")); Serial.write(9); Serial.println(profileSteps);
 Serial.print(F("TypeP:")); Serial.write(9);
 Serial.print(typeP);
 if (typeP == 1) {
 Serial.println(F(" = Power"));
 } else {
 Serial.println(F(" = Temp"));
 };
 Serial.print(F("Secs:")); Serial.write(9); Serial.println(profileRunTime);
 Serial.print(F("St150:")); Serial.write(9); Serial.println(getSecTo150());
 Serial.print(F("TAL:")); Serial.write(9); Serial.println(getTAL());
 // Printe headings
 Serial.println("");
 Serial.print(F("Step")); Serial.write(9);
 Serial.print(F("Start")); Serial.write(9);
 Serial.print(F("End")); Serial.write(9);
 Serial.print(F("Secs")); Serial.write(9);
 Serial.print(F("TotSec")); Serial.write(9);
 Serial.println(F("Ramp"));
 // Print out profile
 for (s = 0; s < profileSteps - 1; s++) {
 Serial.print(s); Serial.print("-") ;Serial.print(s + 1);Serial.write(9);
 Serial.print(profileTemp[s]); Serial.write(9);
 Serial.print(profileTemp[s + 1]); Serial.write(9);
 Serial.print(profileTime[s + 1]); Serial.write(9);
 Serial.print(profileSecs[s]); Serial.write(9);
 Serial.print(profileRamp[s], 3); Serial.write(9);
 Serial.println("");
 }
 // Print out profile temperatures for each elapsed second
 Serial.println("");
 Serial.print(F("Sec")); Serial.write(9);
 Serial.println(F("Temp"));
 for (s = 0; s < profileRunTime; s++) {
 Serial.println(s); Serial.write(9);
 Serial.println(getReflowProfileTemp(s));
 }
 // Print a double line
 Serial.println("");
 for (i = 0; i < 60; i++) Serial.print("=");
 Serial.println("");
 Serial.println("");
}
//########################################################################################################
//########################################################################################################
//########################################################################################################
int resetEEPROM(boolean always)
//---------------------------------------------------------------------------
// Writes initial setup values and reflow profiles to EEPROM
//----------------------------------------------------------------------------
// Parm Description
// always 0=First time only (If no ID# found), 1=Always
//
// RETURN First free profile address
//----------------------------------------------------------------------------
// Notes:
// Even though several variables are stored to EEPROM - as there are no function
// to edit the values - we do not (at the moment) bother to read the values to
// save a few bytes!
//
// Two(2) profiles have been predefined! But you can change these profiles below!
// Just make sure you do NOT run out of memory!
// profile?Name[] contains the temperature at each breakpoint
// profile?StepTime[] contains the seconds for the given step
//
// Example:
// char profile0Name[] = "Lee's";
// byte profile0Temp[] = { 25, 179, 185, 215, 30 }; // The temperature for each interval
// byte profile0StepTime[] = { 0, 220, 35, 30, 60 }; // The length in secs for interval
//
// Above temp is set to slightly over room temperature at 25 degrees (Centigrade naturally!)
// (If you set temperature to 0 degrees the CURRENT temperature will be used instead!)
// Time is set to the start of the profile at 0 seconds
// Then a linear ramp is wanted up to 179 degrees during 220 seconds
// After that a short ramp up to 185 degrees should take 35 seconds
// Finally a short ramp up to 215 degrees taking 30 seconds
// Last step is cooling to 30 degrees during 60 seconds
//
// NOTE that no forced cooling is used! YOU will have to CAREFULLY open the oven door to cool it!
// If needed to save memory --> Move this to a special EEPROM Setup program
// to be run on arduino once before Reflow Oven is uploaded to arduino!
// The call can also be commented away after initial run and will same memory!
// Saving: 856 bytes FLASH and 28 bytes RAM
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 unsigned long id;
 int i, j, eeAddr = PROFSTART, iVal;
 byte countTypeP = 0, count, bVal;
 // Kester's Reflow Profile (If temp is "0"=Will be set to current temp)
 char profile1Name[] = "Kester's";
 byte profile1Temp[] = { 25, 150, 175, 225, 225, 30}; // The temperature for each interval
 byte profile1StepTime[] = { 0, 90, 120, 15, 25, 60 }; // The length in secs for interval
 // Lee's Reflow Profile (0=Will be set to current temp)
 char profile0Name[] = "Lee's";
 byte profile0Temp[] = { 25, 180, 185, 220, 30, 25 };
 byte profile0StepTime[] = { 0, 220, 35, 30, 60, 60 };
 // Test Reflow Profile (0=Will be set to current temp)
 // char profile2Name[] = "User1";
 // byte profile2Temp[] = { 25, 150, 175, 225, 225, 30 };
 // byte profile2StepTime[] = { 0, 9, 12, 2, 2, 6 }; // TEST! with 1:10 timing
 // Read ID signature - should be "Ar2R" or read as long=IDSIGNATURE (1379037761)
 EEPROM.get(0, id);
 // If correct ID "Ar2R" is NOT found OR "always"=="true" - reset EEPROM values and profiles!
 if ((id != IDSIGNATURE) || always == true) {
 Serial.println(F("Is resetting EEPROM! All previous data erased!"));
 // Write ID signature "Ar2R" (4 bytes)
 id = IDSIGNATURE;
 EEPROM.put(0, id);
 // Setup values (1 byte)
 bVal = IPS; EEPROM.write(4, bVal); // Interrupts per sec (IPS)
 bVal = PPD; EEPROM.write(5, bVal); // Power Per Degree (PPD)
 bVal = 0; EEPROM.write(6, bVal); // Currently selected profile number
 bVal = 0; EEPROM.write(7, bVal); // Total number of stored profiles - YET!
 // Setup values 2 byte - so EEPROM.put() is used - takes length of variable
 iVal = 256; EEPROM.put(10, iVal); // Start of "last run"
 iVal = 0; EEPROM.put(12, iVal); // Count of "last run"
 // Write 1st temperature profile (#0)
 eeAddr=writeProfile(eeAddr, sizeof(profile0Temp), typeP, profile0Name, profile0Temp, profile0StepTime);
 // Write 2nd temperature profile (#1)
 eeAddr=writeProfile(eeAddr, sizeof(profile1Temp), typeP, profile1Name, profile1Temp, profile1StepTime);
 // Write First FREE after stored reflow profiles
 Serial.print(F("Addr after reset: ")); Serial.println(eeAddr);
 EEPROM.put(8, eeAddr); // First FREE after stored profiles
 }
 // Return First FREE after stored profiles
 return (eeAddr);
}
int writeProfile(int start, byte count, byte typeP, char* profName, byte * aTemp, byte * aTime)
//---------------------------------------------------------------------------
// Writes a reflow profile to EEPROM
//----------------------------------------------------------------------------
// Parm Description
// start Start address
// count Count of array elements stored (7 lower bits = max 127)
// Max length with single read from EEPROM is 32 bytes!
// typeP typeP info about the profile (1 bit)
// Bit 7 Profile type: 0=Tempprofile, 1=Powerprofile
// *profName Name of profile (8 chars incl end marker!)
// *aTemp The end centigrade temparature for that interval
// *aTime The number of secs for that interval
//
// RETURN >0=Next EEPROM position, -1=Count too big!
//----------------------------------------------------------------------------
// Notes:
// Data is stored in EEPROM for each profile as follows:
// Bytes Desc
// 1 count of array elements (7 bits) + typeP (1 bit)
// 8 Name (max 7 chars) incl 0 to mark end (Ex: "Lee\0")
// 1*Count Array with end temperatures for each interval
// 1*Count Array with times in secs for each interval
//
// If one profile is deleted ALL remaining profiles will have to be rewritten
// to EEPROM! (Simple way to handle garbage collection!)
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int i;
 byte countTypeP = 0;
 // Return if number of bytes >32 (Current limit because of writing just once!)
 if (count > 32) return (-1);
 // 7 lowest bits is count and 8th bit is typeP
 countTypeP = (count & 127) + (typeP << 8);
 // Write countTypeP
 EEPROM.write(start++, countTypeP);
 // Write Profile name (NOTE! Max 8 chars - incl end marker!)
 start = EEPROMwriteBytes(start, (byte*)profName, 8);
 // Write profile temperatures
 start = EEPROMwriteBytes(start, aTemp, count);
 // Write profile times
 start = EEPROMwriteBytes(start, aTime, count);
 // Update count
 countTypeP = EEPROM.read(7);
 countTypeP++;
 EEPROM.write(7, countTypeP);
 return (start);
}
int getProfileAddress(byte pNo)
//---------------------------------------------------------------------------
// Reads the profile address from EEPROM
//----------------------------------------------------------------------------
// Parm Description
// pNo Profile number
//
// RETURN >0=Profile address, -1=Does NOT exist!
//----------------------------------------------------------------------------
// Notes:
// Note that all DATA are stored as bytes so values are limited to 0-255!
//
// Data is stored in EEPROM for each profile as follows:
// Bytes Desc
// 1 CountTypeP of array elements
// 8 Name of profile incl blanks to fill 8 chars (Ex: "Lee ")
// 1*Count Array with end temperatures for each interval
// 1*Count Array with times in secs for each interval
//
// If a profile is deleted ALL remaining profiles will have to be rewritten
// to EEPROM! (To handle garbage collection!)
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 byte countTypeP, count, countProfiles, pRead = 0;
 int start;
 start = PROFSTART;
 countProfiles = EEPROM.read(7);
 // Return if profile is too high
 if (pNo > countProfiles - 1) return (-1);
 do {
 // Read array count & typeP bit
 countTypeP = EEPROM.read(start++);
 // Only the 7 lowest bits
 count = countTypeP & 127;
 // Address found so return it!
 if (pRead == pNo) {
 return (start - 1);
 }
 start += 8 + (count * 2);
 } while (pRead++ < pNo);
 return (-2);
}
int readProfileName(byte pNo, char * profName)
//---------------------------------------------------------------------------
// Reads the profile name from EEPROM
//----------------------------------------------------------------------------
// Parm Description
// pNo Profile number
// profName Profile name is RETURNED here!
//
// RETURN 0=Not found, 1=Found
//----------------------------------------------------------------------------
// Notes:
// Note that all DATA is stored as bytes so values are limited to 0-255!
//
// Data is stored in EEPROM for each profile as follows:
// Bytes Desc
// 1 count of array elements (7 bits) + typeP (1 bit)
// 8 Name of profile incl blanks to fill 8 chars (Ex: "Lee ")
// 1*Count Array with end temperatures for each interval
// 1*Count Array with times in secs for each interval
//
// If a profile is deleted ALL remaining profiles will have to be rewritten
// to EEPROM! (To handle garbage collection!)
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 byte countTypeP, count, countProfiles, pRead = 0;
 int start, i = 0;
 start = getProfileAddress(pNo);
 // Return if profile does not exist
 if (start < 0) return (0);
 // Skip count byte)
 start++;
 // Read profile name
 i = 0; while (((profName[i++] = EEPROM.read(start++)) > 0) && i < 8);
 return (1);
}
int readProfile(byte pNo, byte * count, byte * typeP, char* profName, byte * aTemp, byte * aTime)
//---------------------------------------------------------------------------
// Reads a reflow profile from EEPROM
//----------------------------------------------------------------------------
// Parm Description
// pNo Profile Number
// *count Count of array elements stored (7 lower bits = max 127)
// Max length with single read from EEPROM is 32 bytes!
// *TypeP typeP info about the profile (1 bit)
// Bit 7 Profile type: 0=Temperature profile, 1=Power profile
// *profName Name of profile (8 chars)
// *aTemp The end centigrade temparature for that interval
// *aTime The number of secs for that interval
//
// RETURN 1=Found!, 0=NOT found!
//----------------------------------------------------------------------------
// Notes:
// Note that all DATA is stored as bytes so values are limited to 0-255!
//
// Data is stored in EEPROM for each profile as follows:
// Bytes Desc
// 1 count of array elements (7 bits) + typeP (1 bit)
// 8 Name of profile incl blanks to fill 8 chars (Ex: "Lee ")
// 1*Count Array with end temperatures for each interval
// 1*Count Array with times in secs for each interval
//
// If a profile is deleted ALL remaining profiles will have to be rewritten
// to EEPROM! (To handle garbage collection!)
// The power profile function has NOT been implemented due to memory restrictions!
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int i = 0, start;
 byte countTypeP = 0, eeAddr;
 // Save start address of current profile in global variable
 profileAddr = getProfileAddress(pNo);

 start=profileAddr;
 // Return if profile does not exist
 if (start < 0) return (0);
 // Read array count & typeP bit
 countTypeP = EEPROM.read(start++);
 // Only the 7 lowest bits
 *count = countTypeP & 127;
 // Only the 8th bit
 *typeP = countTypeP >> 8;
 // Read Profile name (NOTE! Max 7 chars + end marker!)
 eeAddr = start;
 i = 0; while (((profName[i++] = EEPROM.read(eeAddr++)) > 0) && i < 8);
 start += 8;
 // Read profile temperatures (Max 32 bytes at a time!)
 for (i = 0; i < *count; i++) {
 aTemp[i] = EEPROM.read(start++);
 }
 // Read profile times (Max 32 bytes at a time!)
 for (i = 0; i < *count; i++) aTime[i] = EEPROM.read(start++);
 return (1);
}
int EEPROMwriteBytes(int start, byte * arr, byte count)
//---------------------------------------------------------------------------
// Writes count bytes from address start to EEPROM - but only if byte changed!
//----------------------------------------------------------------------------
// Parm Description
// start Start address
// count Count of array elements stored or ...
// 0=Until end of string marker found (\0)
// *arr Array of bytes to write
//
// RETURN Next EEPROM position
//----------------------------------------------------------------------------
// Notes:
// A read is performed first to see if byte has changed - then write is done!
// This saves on EEPROM write life!
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 byte j = 0, b;
 // Find position of end of string marker
 if (count == 0) {
 while (arr[count++] > 0);
 }
 // Write desired characters to EEPROM - but only if byte is changed - so read first!
 for (j = 0; j < count; j++) {
 if (EEPROM.read(start) != arr[j]) {
 EEPROM.write(start, arr[j]);
 }
 start++;
 }
 return (start);
}
int gefFreeProfileAddr(void)
//----------------------------------------------------------------------------
// Get the first free address after the last stored profile
//----------------------------------------------------------------------------
// Parm Description
// startA Start address (0-
// stopA Stop at address (1-)
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int eeAddr = 0;
 // First FREE after already stored profiles
 EEPROM.get(8, eeAddr);
 return (eeAddr);
}
int miniMenue(char * txt1, char * txt2)
//---------------------------------------------------------------------------
// Draw a minimenue on the last line
//----------------------------------------------------------------------------
// Parm Description
// txt The alternate text to be printed
// RETURN 0="> Exit!", 1=txt1, 2=txt2 - as selected when clicked
//----------------------------------------------------------------------------
// Notes:
// Shows two variable texts txt1 and txt2 and fixed "> Exit!" text
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int sel = 0, ret;
 byte count;
 // Show only two alternatives if some text in txt2
 if (txt2[0] == 0) count = 1; else count = 2;
 while ((ret = readEncoder(sel, 0, count, true)) != CLICKED) {
 sel = ret;
 // EXIT!
 if (sel == 0) {
 showMiniMenueItem("> Exit!", BLUE, YELLOW);
 }
 // Txt1
 else if (sel == 1) {
 showMiniMenueItem(txt1, CYAN, LIGHT_GREY);
 }
 // Txt2
 else if (sel == 2) {
 showMiniMenueItem(txt2, CYAN, LIGHT_GREY);
 }
 }
 return (sel);
}
void waitForClick(void)
//---------------------------------------------------------------------------
// Waits for a click - just as it says! :-)
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN
//----------------------------------------------------------------------------
// Notes:
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 while (readEncoder(0, 1, 2, false) != CLICKED);
}
int readEncoder(int start, int mi, int ma, boolean wrap)
//---------------------------------------------------------------------------
// Read rotary encoder
//----------------------------------------------------------------------------
// Parm Description
// start Start value
// mi Minimum value of selection
// ma Maximum value of selection - Must be <CLICKED (10000)
// wrap 1=Wrap value from max <--> min, 0=Stop at min and max
//
// RETURN CLICKED=Button clicked, All else=New value returned
// With ranges >50 an automatic x10 speedup will be used
// when the encoder is turned quickly!
//----------------------------------------------------------------------------
// Notes:
// Uses three(3) pins as input:
// The ISR uses ePinA. This function also uses ePinB and ePinSwitch.
//
// These MUST have all have their internal pull up resistors enabled!
// It is CRITICAL that a 0.1uF ceramic capacitor is connected between ALL
// switches and ground (GND)! (Total of three(3) capacitors!)
// If capacitors are missing debouncing will NOT work properly!
// Common pin on encoder should be connected to ground (GND)!
//
// Also requires the following: #define CLICKED 10000
//
// The returned values will be forced to stay between min and max
// NOTE! Will return when either the button is clicked or position changed!
// It will return even if min-max forces the RETURN value to be unchanged - as
// long as the position (rotary angel) has been changed!
//
// Typical call example:
// tmpData = readEncoder(result, 1, 5, true);
// if (tmpData != CLICKED) result = tmpData;
// else ...
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int pChange, eStep = 1;
 static unsigned long nowTime, prevTime, gapTime, pPos, prevGapTime;
 int result;
 // Transfer values to global variables for encoder interrupt routine
 pChange = eChange = 1;
 // Loop until the encoder button has been clicked
 while (digitalRead(ePinSwitch)) {
 // Position changed! eDiff will indicate +1 or -1
 if ((pChange != eChange)) {
 // eChange is a global var that is changed via the "doEncoder" interrupt routine
 pChange = eChange;
 // Calculate how long between each change
 prevGapTime = gapTime;
 prevTime = nowTime;
 nowTime = millis();
 gapTime = nowTime - prevTime;
 // If a large range - then use bigger steps if the encoder is turned quicker (shorter gaps)
 if ((ma - mi) > 50 && (gapTime + prevGapTime) < 100) eStep = 10; else eStep = 1;
 // Calculate new resulting position
 result = start + eDiff * eStep;
 // Jump to next higher/lower even 10-step
 if (eStep > 2) {
 if (eDiff == 1) result = result + (10 - (result % 10));
 else if (eDiff == -1) result = result - (result % 10);
 }
 // Keep selected value within limits and act according to wrap setting
 if (result > ma)
 if (wrap) result = mi; else result = ma;
 if (result < mi)
 if (wrap) result = ma; else result = mi;
 // Return
 return (result);
 }
 }
 // Wait till contact bounce is over
 // NOTE that this requires the fitted 0.1uF debounce capacitors!
 delay(300);
 return (CLICKED);
}
void doEncoder()
//----------------------------------------------------------------------------
// INTERRUPT SERVICE ROUTINE! - Reacts on position change for rotary encoder
//----------------------------------------------------------------------------
// Global var Description
// ePinA Encoder pin A - Should be either pin D2(int0) or D3(int1)
// ePinb Encoder pin B
// eDiff Encoder selection - either +1 or -1
// eChange Alternates between 1 and 0
//
// RETURN Value is returned in global var "eDiff"
//----------------------------------------------------------------------------
// Circuit:
// Uses two(2) pins as input: ePinA and ePinB.
// These must have their internal pull up resistors enabled!
// Common pin on encoder should be connected to ground!
// CRITICAL: Should have a 100nF capacitor between all contacts and ground!
//----------------------------------------------------------------------------
// Notes:
// Attatch interrupt on encoder - Arduino pin 2 (=ATmega pin 0) like this:
// attachInterrupt(0, doEncoder, FALLING);
// NOTE! This is interrupt-based so it is ONLY called when position is changed!
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
//----------------------------------------------------------------------------
{
 // Wait for ePinB to stabilize
 delayMicroseconds(20);
 // Interrupt is triggered when pin A goes LOW - so also read pin B
 if (digitalRead(ePinB)) eDiff = -1;
 else eDiff = 1;
 // Flag that we have had an interrupt by alternating the value
 eChange = !eChange;
}
void dumpEEPROM(int count)
//---------------------------------------------------------------------------
// Dumps EEPROM content to serial monitor
//----------------------------------------------------------------------------
// Parm Description
// count Number of memory positions to write (up to count-1)
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
// EEPROM Usage:
// Pos Len Type Description
// 0 4 long ID string "Ar2R" = 1379037761 (65, 114, 50, 82)
// 4 1 byte Interrupts Per Second - Recommended: 50-120
// 5 1 byte IPS-steps per centigrade degree - Recommended: 1-20
// 6 1 byte Current profile number
// 7 1 byte Total number of stored profiles
// 8 2 int First FREE profile address
// 10 2 int Start of last run - Every 5/sec - Store: Goal temp, Actual temp, Power
// 12 2 int Last run - count
// All numbers are stored in lowest byte first order.
//
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 int i;
 byte mem;
 // Separator line
 Serial.println(""); for (i = 0; i < 60; i++) Serial.print("="); Serial.println("");
 // For
 for (i = 0; i < count; i++) {
 mem = EEPROM.read(i);
 if ((i % 10) == 0) {
 Serial.println("");
 if (i<1000) Serial.print(" ");
 threePosPrint(i); Serial.print(" ");
 }
 threePosPrint(mem);
 Serial.print(" ");
 }
 Serial.println("");
 for (i = 0; i < 60; i++) Serial.print("="); Serial.println();
}
void threePosPrint(int i)
//---------------------------------------------------------------------------
// Prints a number rightadjusted in three positions
//----------------------------------------------------------------------------
// Parm Description
// i Number to write
//
// RETURN -
//----------------------------------------------------------------------------
// Notes:
// Handles only positive numbers at the moment!
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
//----------------------------------------------------------------------------
{
 if (i < 10) Serial.print(" ");
 if (i < 100) Serial.print(" ");
 Serial.print(i);
}
//---------------------------------------------------------------------------
// Function Head
//----------------------------------------------------------------------------
// Parm Description
//
// RETURN
//----------------------------------------------------------------------------
// Notes:
//
//
//----------------------------------------------------------------------------
// Date Programmer Action
// 190202 Sfenn ar2uino.wordpress.com original code v1.2
// Modified by KiritoTech 2019-11-18
