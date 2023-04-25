#include <Arduino.h>
#include "Wire.h"
#include "float.h"
#include "FlashAsEEPROM_SAMD.h"

// This project does not use the standard Arduino analog functions which
// number analog inputs A0.. instead we use the def for the muxpos bit
// field in the INPUTCTRL ADC register directly.  See variants.cpp and
// variants.h in the PlatformIO install directory
#define ADC_VREF_PIN                ADC_INPUTCTRL_MUXPOS_PIN1_Val
#define SPECTRA_COLOR_OUT_PIN       ADC_INPUTCTRL_MUXPOS_PIN2_Val
#define SPECTRA_INTENSITY_OUT_PIN   ADC_INPUTCTRL_MUXPOS_PIN3_Val
#define ADC_OVERSAMPLE_COUNT        64
#define ADC_SAMPLING_DELAY          33

#define AT30TS74_I2C_ADDR           72 // 0x48
#define FAST_BLINK_DELAY            200
#define SLOW_BLINK_DELAY            1000
#define CMD_NAME_MAX                12
#define MAX_LINE_SZ                 80
#define OUTBFR_SIZE                 (MAX_LINE_SZ * 3)
#define MAX_DATE_SZ                 30
#define MAX_BOARD_ID_SZ             20

// disable EEPROM/FLASH library driver debug, because it uses Serial 
// not SerialUSB and may hang on startup as a result
#define FLASH_DEBUG               0
#define MAX_EEPROM_ADDR           1024

// possible CLI errors
#define CLI_ERR_NO_ERROR          0
#define CLI_ERR_CMD_NOT_FOUND     1
#define CLI_ERR_TOO_FEW_ARGS      2
#define CLI_ERR_TOO_MANY_ARGS     3
#define MAX_TOKENS                8

// Versions
// 1.1.x was Microchip Studio then MPLAB (2022): both tools unstable/unsuitable
// 1.2+ is PlatformIO/VSCode (2023)
const char      versString[] = "1.3.0";

// CLI Command Table structure
typedef struct {
    char        cmd[CMD_NAME_MAX];
    int         (*func) (int x);
    int         argCount;
    char        help1[MAX_LINE_SZ];
    char        help2[MAX_LINE_SZ];

} cli_entry;

// EEPROM data storage struct
typedef struct {
    uint32_t        sig;      // unique EEPROM signature (see #define)
    float           K;        // K constant for LED equations
    char            boardID[MAX_BOARD_ID_SZ];
    char            calibDate[MAX_DATE_SZ];

    // these were used in ADC driver development, not for production
    bool            enCorrection;
    int             offsetError;
    int             gainError;

} EEPROM_data_t;

// LED measurement
typedef struct {
    uint16_t    colorCounts;
    float       Vout_Color;
    float       wavelength;

    uint16_t    intensityCounts;
    float       Vout_Intensity;
    float       intensity;

} led_meas_t;

// Constant Data
const char      hello[] = "\r\nOCP Vulcan LED Text Fixture (LTF) V ";
const char      cliPrompt[] = "ltf> ";
const int       promptLen = sizeof(cliPrompt);
const float     ADCGain = 2.0;
const float     ADCVrefA = 2.5;
const float     voltsPerCount = ADCVrefA / 4095.0;
const uint32_t  EEPROM_signature = 0xDE110C01;

// calibration data for use with 1P5KH calibration board rev  X00
// that board is marked '2.7' on bottom of small PCB glued to blue board
const float     amberLED_Intensity = 7.53;
const float     amberLED_Wavelength = 584.5;
const float     greenLED_Intensity = 45.8;
const float     greenLED_Wavelength = 526.8;

// Variable data
uint16_t        ADC_resultsArray[ADC_OVERSAMPLE_COUNT+1];
char            outBfr[OUTBFR_SIZE];
bool            calibRequired = false;

// FLASH/EEPROM Data buffer
EEPROM_data_t   EEPROMData;

// --------------------------------------------
// Forward function prototypes
// --------------------------------------------
void EEPROM_Save(void);
void ADC_EnableCorrection(void);
uint16_t ADC_Read(uint8_t ch);
int waitAnyKey(void);
void ledRawRead(led_meas_t *m);

// prototypes for CLI-called functions
// template is func_name(int) because the int arg is the arg
// count from parsing the command line; the arg tokens are
// global in tokens[] with tokens[0] = command entered and
// arg count does not include the command token
int help(int);
int calib(int);
int readCmd(int);
int rawRead(int);
int setCmd(int);
int readLoop(int);
int readTemp(int);
int debug(int);
int calib(int);
int versCmd(int);

// CLI token stack
char                *tokens[MAX_TOKENS];

led_meas_t      currentMeasurement;

// CLI command table
// format is "command", function, required arg count, "help line 1", "help line 2" (2nd line can be NULL)
const cli_entry     cmdTable[] = {
    {"calib",     calib,  0, "Calibrate board LED sensor",                     "Uses LTF Calibration Board LEDs"},
    {"check",  readLoop,  0, "Continuous loop reading raw sensor data.",       "Hit any key to exit loop."},
    {"help",       help,  0, "THIS DOES NOT DISPLAY ON PURPOSE",               " "},
    {"read",    readCmd,  0, "Read LED color temperature and intensity.",      " "},
    {"set",      setCmd,  2, "Sets value of a stored parameter.",              "eg 'set k 1.234' sets K constant to 1.234"},
    {"temp",   readTemp,  0, "Read board (not MCU core) temperature sensor.",  "Reports temperature in degrees C and F."},
    {"vers",    versCmd,  0, "Shows firmware version information.",            " "},
    {"xdebug",    debug, -1, "Debug functions mostly for developer use.",      "'xdebug reset' resets board; 'xdebug eeprom' dumps EEPROM"},
};

#define CLI_ENTRIES     (sizeof(cmdTable) / sizeof(cli_entry))
#define SHOW(x)         (terminalOut((char *) x))
#define SHOWX()         (terminalOut(outBfr))

/**
  * @name   terminalOut
  * @brief  wrapper to SerialUSB.print[ln]
  * @param  msg to output
  * @retval None
  * @note   needed to address missing chars & skittish behavior
  */
void terminalOut(char *msg)
{
    SerialUSB.println(msg);
    SerialUSB.flush();
    delay(50);
}

/**
  * @name   doPrompt
  * @brief  output firmware prompt to terminal
  * @param  None
  * @retval None
  */
void doPrompt(void)
{
    SerialUSB.write(0x0a);
    SerialUSB.write(0x0d);
    SerialUSB.flush();
    SerialUSB.print(cliPrompt);
    SerialUSB.flush();
}

/**
  * @name   cli
  * @brief  command line interpreter
  * @param  raw   pointer to raw input line
  * @retval true if command parsed OK, else false
  */
bool cli(char *raw)
{
    bool         rc = false;
    int         len;
    char        *token;
    const char  delim[] = " ";
    int         tokNdx = 0;
    char        input[80];
    int         error = CLI_ERR_CMD_NOT_FOUND;
    int         argCount;

    strcpy(input, (char *) raw);
    len = strlen(input);

    // initial call, should get and save the command as 0th token
    token = strtok(input, delim);
    if ( token == NULL )
    {
      doPrompt();
      return(true);
    }

    tokens[tokNdx++] = token;

    // subsequent calls should parse any space-separated args into tokens[]
    // note that what's stored in tokens[] are pointers into the
    // input string, not the actual string token itself
    while ( (token = (char *) strtok(NULL, delim)) != NULL && tokNdx < MAX_TOKENS )
    {
        tokens[tokNdx++] = token;
    }

    if ( tokNdx >= MAX_TOKENS )
    {
        SHOW("Too many arguments in command line!");
        doPrompt();
        return(false);
    }

    argCount = tokNdx - 1;

    for ( int i = 0; i < (int) CLI_ENTRIES; i++ )
    {
        if ( strncmp(tokens[0], cmdTable[i].cmd, len) == 0 )
        {
            if ( cmdTable[i].argCount == argCount || cmdTable[i].argCount == -1 )
            {
                // command funcs are passed arg count, tokens are global
                (cmdTable[i].func) (argCount);
                SerialUSB.flush();
                rc = true;
                error = CLI_ERR_NO_ERROR;
                break;
            }
            else if ( cmdTable[i].argCount > argCount )
            {
                error = CLI_ERR_TOO_FEW_ARGS;
                rc = false;
                break;
            }
            else
            {
                error = CLI_ERR_TOO_MANY_ARGS;
                rc = false;
                break;
            }
        }
    }

    if ( rc == false )
    {
        if ( error == CLI_ERR_CMD_NOT_FOUND )
         SHOW("Invalid command");
        else if ( error == CLI_ERR_TOO_FEW_ARGS )
          SHOW("Not enough arguments for this command, check help.");
        else if ( error == CLI_ERR_TOO_MANY_ARGS )
          SHOW("Too many arguments for this command, check help.");
        else
          SHOW("Unknown parser s/w error");
    }

    doPrompt();
    return(rc);

} // cli()

/**
  * @name   waitAnyString
  * @brief  wait for and capture any string entered by user
  * @param  len max length of string
  * @retval pointer to captured string
  * @note   len subject to a max of 79
  */
char *waitAnyString(unsigned len)
{
    int           c;
    static char   buffer[80];

    if ( len > 79 )
        len = 79;

    memset(buffer, 0, len);

    for ( unsigned i = 0; i < len && i < 79; i++ )
    {
        c = waitAnyKey();
        if ( c == 0x0a || c == 0x0d )
        {
            buffer[i] = 0;
            SerialUSB.write((const char *) "\r\n");
            SerialUSB.flush();
            break;
        }

        buffer[i] = (char) c;
        SerialUSB.write((const char) c);
        SerialUSB.flush();
    }

    if ( buffer[0] )
      return(buffer);
    else
      return(NULL);
}

/**
  * @name   calib
  * @brief  calibrate K constant for Spectra probe
  * @param  arg not used
  * @retval None
  */
int calib(int arg)
{
    int           keyPressed;
    led_meas_t    data;
    float         temp_K;
    char          *s;

    SHOW("Power up the LTF Calibration Board and select the GREEN LED now.");
    SHOW("Align the Spectra probe with the GREEN calibration LED.");
    SHOW("Press 'y' to begin calibration, or 'n' to exit.");
    keyPressed = toupper(waitAnyKey());
    if ( keyPressed != 'Y' )
        return(0);

    SerialUSB.print("Today's date (any format): ");
    SerialUSB.flush();

    s = waitAnyString(MAX_DATE_SZ);
    if ( s != NULL )
    {
        strcpy(EEPROMData.calibDate, s);
        EEPROM_Save();
    }

    SHOW("Starting measurements, please wait...");
    ledRawRead(&data);

    sprintf(outBfr, "Spectra intensity: %4d cnts  %5.3f V", data.intensityCounts, data.Vout_Intensity);
    SHOWX();

    // K = lv_constant / Vout**2
    temp_K = greenLED_Intensity / pow(data.Vout_Intensity, 2);

    sprintf(outBfr, "Calculated K = %f", temp_K);
    SHOWX();

    EEPROMData.K = temp_K;
    EEPROM_Save();
    SHOW("K stored in EEPROM");
    calibRequired = false;
    return(0);
}

//===================================================================
//                     DEBUG FUNCTIONS
//===================================================================

/**
  * @name   debug_scan
  * @brief  I2C bus scanner
  * @param  None
  * @retval None
  */
void debug_scan(void)
{
  byte        count = 0;
  int         scanCount = 0;
  uint32_t    startTime = millis();

  SHOW ("Scanning I2C bus...");

  for (byte i = 0; i < 128; i++)
  {
    scanCount++;
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      sprintf(outBfr, "Found device at address %d 0x%2X", i, i);
      SHOWX();
      count++;
      delay(10);  
    } 
  } 

  sprintf(outBfr, "Scan complete, %d addresses scanned in %d msec", scanCount, (int) (millis() - startTime));
  SHOWX();

  if ( count )
  {
    sprintf(outBfr, "Found %d I2C deVout_Intensityce(s)", count);
    SHOWX();
  }
  else
  {
    SHOW("No I2c device found");
  }
}

/**
  * @name   debug_reset
  * @brief  reset board
  * @param  None
  * @retval None
  */
void debug_reset(void)
{
    SHOW("Board reset will disconnect USB-serial connection now.");
    SHOW("Repeat whatever steps you took to connect to the board.");
    delay(1000);
    NVIC_SystemReset();
}

/**
  * @name   debug_dump_eeprom
  * @brief  Dump EEPROM contents
  * @param  None
  * @retval None
  */
void debug_dump_eeprom(void)
{
    SHOW("EEPROM Contents:");
    sprintf(outBfr, "Signature:     %08X", (unsigned int) EEPROMData.sig);
    SHOWX();
    sprintf(outBfr, "K Constant:    %8.4f", EEPROMData.K);
    SHOWX();
    sprintf(outBfr, "Board ID:      %s", EEPROMData.boardID);
    SHOWX();
    sprintf(outBfr, "Calib Date:    %s", EEPROMData.calibDate);
    SHOWX();

    SerialUSB.print("ADC Correct:   ");
    if ( EEPROMData.enCorrection )
    {
      SerialUSB.println("Enabled");
      SerialUSB.print("Gain ERR:   ");
      SerialUSB.println(EEPROMData.gainError, DEC);
      SerialUSB.print("Offset ERR: ");
      SerialUSB.println(EEPROMData.offsetError, DEC);
    }
    else
      SerialUSB.println("Disabled");
}

/**
  * @name   debug_read
  * @brief  read ADC channels 0-5 and display
  * @param  None
  * @retval None
  */
void debug_read(void)
{
    uint16_t            rawCounts;
    float               volts;

    // debug tool that reads ADC channels 0-5 although not all
    // are used by this project
    for ( uint8_t i = 0; i < 6; i++ )
    {
        rawCounts = ADC_Read(i);
        volts = rawCounts * voltsPerCount * ADCGain;
        sprintf(outBfr, "Ch %d %4d %8.3f V ", i, rawCounts, volts);
        SerialUSB.print(outBfr);
        if ( i == 1 )
          SHOW("ARef");
        else if ( i == 2 )
          SHOW("Color");
        else if ( i ==3 )
          SHOW("Intensity");
        else
          SHOW("not used");
    }
}

/**
  * @name   debugHelp
  * @brief  help for debug cmd
  * @param  None
  * @retval None
  */
void debugHelp(void)
{
    SHOW("Debug commands are:");
    SHOW("\ti2c ...... I2C bus scanner");
    SHOW("\treset .... Reset board");
    SHOW("\teeprom ... Dump EEPROM");
    SHOW("\tadc ...... Raw ADC read (channels 0-5)");
}

/**
  * @name   debug
  * @brief  main debug program
  * @param  arg   argument count in cmd line
  * @retval 0=OK 1=error
  */
int debug(int arg)
{
    if ( arg == 0 )
    {
        debugHelp();
        return(0);
    }

    if ( strcmp(tokens[1], "i2c") == 0 )
      debug_scan();
    else if ( strcmp(tokens[1], "reset") == 0 )
      debug_reset();
    else if ( strcmp(tokens[1], "eeprom") == 0 )
      debug_dump_eeprom();
    else if ( strcmp(tokens[1], "adc") == 0 )
      debug_read();
    else
    {
      SHOW("Invalid debug subcommand");
      debugHelp();
      return(1);
    }

    return(0);
}

//===================================================================
//                            ADC Stuff
//===================================================================

/**
  * @name   syncADC
  * @brief  wait for ADC sync complete
  * @param  None
  * @retval None
  * @note   Warning! Blocking call!
  */
static void syncADC() 
{
  while (ADC->STATUS.bit.SYNCBUSY) {};
}

/**
  * @name   ADC_Init
  * @brief  Init ADC
  * @param  None
  * @retval None
  */
void ADC_Init(void)
{
  uint32_t        bias, linearity;

  // enable APB clock
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  // enable GCLK1 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_ADC;
  while ( GCLK->STATUS.bit.SYNCBUSY) {};
  
  // get factory calib data from NVRAM
  bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  syncADC();

  // write factory calibration data to ADC
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
  syncADC();

  // set analog reference - AREFA = pin 4 = 2.5V ref supply
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;

  // enable reference buffer offset compensation
  ADC->REFCTRL.bit.REFCOMP = 1;

  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  // set clock prescalar & resolution
  // this sets ADC to run at 31.25 kHz
  // set to free running
  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4_Val;
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  ADC->CTRLB.bit.FREERUN = 1;

  // adjust sample time for possible input impediance (allow ADC to charge cap)
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(1);
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_1X; // ADC_INPUTCTRL_GAIN_DIV2;
  syncADC();

  if ( EEPROMData.enCorrection )
  {
    ADC_EnableCorrection();
  }

  syncADC();
  ADC->CTRLA.bit.ENABLE = 1;
  syncADC();
}

/**
  * @name   ADC_EnableCorrection
  * @brief  enable ADC auto-correction
  * @param  None
  * @retval None
  * @note   not used, kept for legacy purposes
  */
void ADC_EnableCorrection(void)
{
// set offset and gain correction values 
// see section 33.6.7-10 of the MCU datasheet
// used calculator at https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
// Setup: the "#if" above should be zero to eliminate correction in the ADC for calibration.
// First, set breakpoint in ADC_Read() after both ADC channels have been read, then input the
// 'raw' decimal values from the debugger into the online calculator.  Transferred the calculator's
// results into offset_error and gain_error then re-compiled.  This resulted in a significant
// improvement in accuracy between 0.2V and 3.2V.
// Input: 0.284V ADC: 220
// Input: 3.3V   ADC: 4094


  syncADC();
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(EEPROMData.offsetError);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(EEPROMData.gainError);
  ADC->CTRLB.bit.CORREN = 1;

}

/**
  * @name   ADC_Read
  * @brief  read an ADC channel as 16 bits
  * @param  ch  channel number 0..7
  * @retval uint16_t adc value
  */
uint16_t ADC_Read(uint8_t ch)
{
  uint16_t      result;
  uint32_t      sum = 0;

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = ch;
  ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
  syncADC();

  for ( int i = 0; i <= ADC_OVERSAMPLE_COUNT; i++ )
  {
#if 0
    syncADC();

// switched to free-running mode
    ADC->SWTRIG.bit.START = 1;

    while ( ADC->INTFLAG.bit.RESRDY == 0 ) ;
#endif
    // read result - also clears RESRDY bit
    result = ADC->RESULT.bit.RESULT;

    delay(ADC_SAMPLING_DELAY);
//    delayMicroseconds(250);
    // skip first measurement per datasheet
    if ( i == 0 )
      continue;

    ADC_resultsArray[i - 1] = result;
    sum += result;
  }

  result = (uint16_t) (sum / ADC_OVERSAMPLE_COUNT);
  return(result);
}

/**
  * @name   ledRawRead
  * @brief  read Spectra sensor data for LED
  * @param  m pointer to measurement structure to populate
  * @retval None
  */
void ledRawRead(led_meas_t *m)
{
    // ----------------------
    // Read Intensity ch
    // ----------------------
    m->intensityCounts = ADC_Read(SPECTRA_INTENSITY_OUT_PIN);
    m->Vout_Intensity = m->intensityCounts * voltsPerCount * ADCGain;

    // ----------------------
    // Calculate Intensity
    // lv = Vout^2 * K mcd
    // ----------------------
    m->intensity = pow(m->Vout_Intensity, 2) * EEPROMData.K;

    // ----------------------
    // Read & calculate Color
    // wl = (100(Vout + 4) nm
    // ----------------------
    m->colorCounts = ADC_Read(SPECTRA_COLOR_OUT_PIN);
    m->Vout_Color = m->colorCounts * voltsPerCount * ADCGain;
    m->wavelength = 100.0 * (m->Vout_Color + 4.0);

} // ledRawRead()

/**
  * @name   readCmd
  * @brief  read Spectra probe data
  * @param  arg 15 don't show acquisition msg
  * @retval not used
  */
int readCmd(int arg)
{
    led_meas_t          *m = &currentMeasurement;

    if ( calibRequired == true )
    {
        SHOW("Spectra probe requires calibration before use; use 'calib' command now.");
        return(1);
    }

    if ( arg != 15 )
    {
        SHOW("\r\nAcquiring Spectra sensor data, please wait...");
        SerialUSB.flush();
    }

    ledRawRead(m);

    sprintf(outBfr, "Intensity: %7.3f mcd %4d cnts %5.3f V (K=%5.3f)", m->intensity, m->intensityCounts, m->Vout_Intensity, EEPROMData.K);
    SHOWX();

    sprintf(outBfr, "    Color: %7.3f nm  %4d cnts %5.3f V", m->wavelength, m->colorCounts, m->Vout_Color);
    SHOWX();

    return(0);

} // readCmd()

/**
  * @name   readTemp
  * @brief  read temperature
  * @param  None
  * @retval None
  */
int readTemp(int arg)
{
  signed char         i2cData;
  short int           curTemp;
  float               degF;

  Wire.beginTransmission(AT30TS74_I2C_ADDR);
  Wire.write(0);      // set pointer register
  Wire.endTransmission();
  delay(65);
  Wire.requestFrom(AT30TS74_I2C_ADDR, 1, false);

  // read MSB only - sufficient for this project
  i2cData = Wire.read();

  // data is two's complement so nothing needs be done
  // other than recast the var
  curTemp = (short) i2cData;
  degF = ((curTemp * 9.0) / 5.0) + 32.0;

  sprintf(outBfr, "Board temp: %d C/%d F", curTemp, (int) degF);
  SHOWX();

  return(0);
}

/**
  * @name   readLoop
  * @brief  continuous loop reading LED data and temperature
  * @param  arg not used
  * @retval None
  */
int readLoop(int arg)
{
  if ( calibRequired == true )
  {
      SHOW("Spectra probe requires calibration before use; use 'calib' command now.");
      return(1);
  }

  SHOW("Entering continuous read loop, press any key to stop");

  while ( SerialUSB.available() == 0 )
  {
      readCmd(15);
      readTemp(0);
      SHOW(" ");
      delay(1000);
      SerialUSB.flush();
  }

  // flush any other chars user hit when exiting the loop
  while ( SerialUSB.available() )
    (void) SerialUSB.read();

  SHOW("Loop aborted by user");
  SerialUSB.flush();
  return(0);
}

/**
  * @name   waitAnyKey
  * @brief  Wait for keyboard input
  * @param  None
  * @retval int character pressed
  * @note   This is a blocking call!
  */
int waitAnyKey(void)
{
    int             charIn;

    while ( SerialUSB.available() == 0 )
      ;

    charIn = SerialUSB.read();
    return(charIn);
}

/**
  * @name   help
  * @brief  show command help (all)
  * @param  None
  * @retval None
  */
int help(int arg)
{
    sprintf(outBfr, "%s %s", hello, versString);
    SHOWX();
    SHOW("Enter a command then press ENTER. Some commands require arguments, which must");
    SHOW("be separated from the command and other arguments by a space.");
    SHOW("Up arrow repeats the last command; backspace or delete erases the last");
    SHOW("character entered. Commands available are:");

    for ( int i = 0; i < (int) CLI_ENTRIES; i++ )
    {
      if ( strcmp(cmdTable[i].cmd, "help") == 0 )
        continue;

      sprintf(outBfr, "%s\t%s", cmdTable[i].cmd, cmdTable[i].help1);
      SHOWX();

      if ( cmdTable[i].help2[0] != ' ' )
      {
        sprintf(outBfr, "\t%s", cmdTable[i].help2);
        SHOWX();
      }

      SerialUSB.flush();
    }

    return(0);
}

/**
  * @name   versCmd
  * @brief  show firmware version
  * @param  None
  * @retval None
  */
int versCmd(int argCnt)
{
    sprintf(outBfr, "Firmware version V%s built on %s at %s", versString, __DATE__, __TIME__);
    terminalOut(outBfr);
    return(0);
}

/**
  * @name   setCmdHelp
  * @brief  display help for set cmd
  * @param  None
  * @retval None
  */
void setCmdHelp(void)
{
    SHOW("Set command help; format 'set <param> <value>'");
    SHOW("'set K 12.345' sets LED K constant to 12.345");
    SHOW("'set bid 3' sets board ID to 3 (can also have chars eg 'MY BD 3'");
}

/**
  * @name   setCmd
  * @brief  set a parameter stored in EEPROM
  * @param  argCnt  number of arguments in command line
  * @param  tokens[1] parameter name
  * @param  tokens[2] parameter value (format varies with param)
  * @retval 0 if OK, else 1
  */
int setCmd(int argCnt)
{
    char          *parameter = tokens[1];
    String        userEntry = tokens[2];
    float         fValue;
    int           iValue;
    bool          isDirty = false;

    if ( argCnt == 0 )
    {
        setCmdHelp();
        return(0);
    }

    if ( strcmp(parameter, "k") == 0 )
    {
        fValue = userEntry.toFloat();
        if ( EEPROMData.K != fValue )
        {
          EEPROMData.K = fValue;
          isDirty = true;
        }
    }
    else if ( strcmp(parameter, "gain") == 0 )
    {
        // set ADC gain error
        iValue = userEntry.toInt();
        if (EEPROMData.gainError != iValue )
        {
          isDirty = true;
          EEPROMData.gainError = iValue;
        }
    }
    else if ( strcmp(parameter, "offset") == 0 )
    {
        // set ADC offset error
        iValue = userEntry.toInt();
        if (EEPROMData.offsetError != iValue )
        {
          isDirty = true;
          EEPROMData.offsetError = iValue;
        }
    }
    else if ( strcmp(parameter, "corr") == 0 )
    {
        // set ADC corr on|off
        if ( strcmp(tokens[2], "off") == 0 )
        {
          if ( EEPROMData.enCorrection == true )
          {
              EEPROMData.enCorrection = false;
              SHOW("ADC correction off");
              isDirty = true;
          }
        }
        else if ( strcmp(tokens[2], "on") == 0 )
        {
            if ( EEPROMData.enCorrection == false )
            {
                EEPROMData.enCorrection = true;
                isDirty = true;
            }
        }
        else
        {
          SHOW("Invalid ADC corr argument: must be 'on' or 'off'");
        }
    }
    else if ( strcmp(tokens[1], "bid") == 0 )
    {
        if ( strlen(tokens[2]) > MAX_BOARD_ID_SZ )
            strncpy(EEPROMData.boardID, tokens[2], MAX_BOARD_ID_SZ - 1);
        else
            strcpy(EEPROMData.boardID, tokens[2]);
    }
    else
    {
        SHOW("Invalid parameter name");
        setCmdHelp();
        return(1);
    }

    if ( isDirty )
    {
        EEPROM_Save();
        SHOW("Saved change(s) in EEPROM");
    }

    return(0);
}

//===================================================================
//                      EEPROM/NVM Stuff
//===================================================================

/**
  * @name   EEPROM_Save
  * @brief  write EEPROM structure to EEPROM
  * @param  None
  * @retval None
  */
void EEPROM_Save(void)
{
    uint8_t         *p = (uint8_t *) &EEPROMData;
    uint16_t        eepromAddr = 0;

    for ( int i = 0; i < (int) sizeof(EEPROM_data_t); i++ )
    {
        EEPROM.write(eepromAddr++, *p++);
    }
    
    EEPROM.commit();
}

/**
  * @name   EEPROM_Read
  * @brief  read EEPROM into structure
  * @param  None
  * @retval None
  */
void EEPROM_Read(void)
{
    uint8_t         *p = (uint8_t *) &EEPROMData;
    uint16_t        eepromAddr = 0;

    for ( int i = 0; i < (int) sizeof(EEPROM_data_t); i++ )
    {
        *p++ = EEPROM.read(eepromAddr++);
    }
}

/**
  * @name   EEPROM_Defaults
  * @brief  Set EEPROM defaults
  * @param  None
  * @retval None
  */
void EEPROM_Defaults(void)
{
    EEPROMData.sig = EEPROM_signature;
    EEPROMData.K = 1.0;
    strcpy(EEPROMData.calibDate, "01/01/1970");
    strcpy(EEPROMData.boardID, "Vulcan 1");
    EEPROMData.enCorrection = false;
    EEPROMData.gainError = 1400;
    EEPROMData.offsetError = -69;
}

/**
  * @name   EEPROM_InitLocal
  * @brief  Initialize EEPROM & data
  * @param  None
  * @retval true if EEPROM OK on reset, false if failed
  */
bool EEPROM_InitLocal(void)
{
    bool          rc = true;

    EEPROM_Read();

    if ( EEPROMData.sig != EEPROM_signature )
    {
      // EEPROM failed: either never been used, or real failure
      // initialize the signature and default settings

      // When debugging, EEPROM fails every time, but it
      // is OK over resets and power cycles.  This is because
      // the area in FLASH where the EEPROM is simulated is
      // erased for debugging 
      EEPROM_Defaults();

      // save EEPROM data on the device
      EEPROM_Save();

      rc = false;
      SHOW("EEPROM validation FAILED, EEPROM initialized with defaults");
    }
    else
    {
      SHOW("EEPROM validated OK");
    }

    return(rc);
}

/**
  * @name   setup
  * @brief  POR Initialization
  * @param  None
  * @retval None
  */
void setup() 
{
  bool      LEDstate = false;

  // configure pins AIN1, AIN2 & AIN3 (PA3, PB08 & PB09)
  // NOTE: PA3 = 2.5V ref
  // NOTE: Group 1 = PB
  PORT->Group[0].DIRCLR.reg = PORT_PA02;
  PORT->Group[1].DIRCLR.reg = PORT_PB09 | PORT_PB09;
   
  PORT->Group[0].PINCFG[3].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[1].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[1].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

  // see section 7.1 in SAMD21 datasheet - multiplexed signals
  // this is a bit wonky; datasheet is vague about the odd/even pins partly
  // due to Arduino abstraction, and partly because it's just not clear...
  PORT->Group[1].PMUX[3].reg = PORT_PMUX_PMUXO_B;
  PORT->Group[1].PMUX[4].reg = PORT_PMUX_PMUXO_B;

  // TODO configure ADC AREFA pin

  // configure heartbeat LED pin and turn on which indicates that the
  // board is being initialized
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LEDstate);

  // start serial over USB and wait for a connection
  // NOTE: Baud rate isn't applicable to USB but...
  // NOTE: Many libraries won't init unless Serial
  // is running (or in this case SerialUSB)
  SerialUSB.begin(115200);
  while ( !SerialUSB )
  {
      // fast blink while waiting for a connection
      LEDstate = LEDstate ? 0 : 1;
      digitalWrite(PIN_LED, LEDstate);
      delay(FAST_BLINK_DELAY);
  }

  // initialize system & libraries
  if ( EEPROM_InitLocal() == false )
  {
      calibRequired = true;
  }

  ADC_Init();
  Wire.begin();

  sprintf(outBfr, "%s %s", hello, versString);
  SHOWX();
  doPrompt();

} // setup()

/**
  * @name   loop
  * @brief  main program loop
  * @param  None
  * @retval None
  * @note   blinks LED and handles incoming characters from user 
  */
void loop() 
{
  int             byteIn;
  static char     inBfr[80];
  static int      inCharCount = 0;
  static char     lastCmd[80] = {0};
  const char      bs[4] = {0x1b, '[', '1', 'D'};  // terminal: backspace seq
  static bool     LEDstate = false;
  static uint32_t time = millis();

  // blink heartbeat LED
  if ( millis() - time >= SLOW_BLINK_DELAY )
  {
      time = millis();
      LEDstate = LEDstate ? 0 : 1;
      digitalWrite(PIN_LED, LEDstate);
  }

  // process incoming serial over USB characters
  if ( SerialUSB.available() )
  {
      byteIn = SerialUSB.read();
      if ( byteIn == 0x0a )
      {
          // line feed - echo it
          SerialUSB.write(0x0a);
          SerialUSB.flush();
      }
      else if ( byteIn == 0x0d )
      {
          // carriage return - EOL 
          // save as the last cmd (for up arrow) and call CLI with
          // the completed line less CR/LF
          SHOW(" ");
          inBfr[inCharCount] = 0;
          inCharCount = 0;
          strcpy(lastCmd, inBfr);
          cli(inBfr);
          SerialUSB.flush();
      }
      else if ( byteIn == 0x1b )
      {
          // handle ANSI escape sequence - only UP arrow is supported
          if ( SerialUSB.available() )
          {
              byteIn = SerialUSB.read();
              if ( byteIn == '[' )
              {
                if ( SerialUSB.available() )
                {
                    byteIn = SerialUSB.read();
                    if ( byteIn == 'A' )
                    {
                        SHOW(lastCmd);
                        cli(lastCmd);
                    }
                }
            }
        }
    }
    else if ( byteIn == 127 || byteIn == 8 )
    {
        // delete & backspace do the same thing
        if ( inCharCount )
        {
            inBfr[inCharCount] = 0;
            inCharCount--;
            SerialUSB.write(bs, 4);
            SerialUSB.write(' ');
            SerialUSB.write(bs, 4);
        }
    }
    else
    {
        // all other keys get echoed & stored in buffer
        SerialUSB.write((char) byteIn);
        inBfr[inCharCount++] = byteIn;
    }
  }

} // loop()

/**
  * @name   
  * @brief  
  * @param  None
  * @retval None
  */