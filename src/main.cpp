#include <Arduino.h>
#include "Wire.h"
#include "float.h"
#include "FlashAsEEPROM_SAMD.h"

// This project does not use the standard Arduino analog functions which
// number analog inputs A0.. instead we use the def for the muxpos bit
// field in the INPUTCTRL ADC register directly.  See variants.cpp and
// variants.h in the PlatformIO install directory
#define ADC_VREF_PIN                ADC_INPUTCTRL_MUXPOS_PIN1_Val
#define SPECTRA_INTENSITY_OUT_PIN   ADC_INPUTCTRL_MUXPOS_PIN2_Val
#define SPECTRA_COLOR_OUT_PIN       ADC_INPUTCTRL_MUXPOS_PIN3_Val
#define ADC_OVERSAMPLE_COUNT        32

#define AT30TS74_I2C_ADDR           72 // 0x48
#define FAST_BLINK_DELAY            200
#define SLOW_BLINK_DELAY            1000
#define CMD_NAME_MAX                12
#define MAX_LINE_SZ                 80
#define OUTBFR_SIZE                 (MAX_LINE_SZ * 3)

// disable EEPROM/FLASH library driver debug, because it uses Serial 
// not SerialUSB and may hang on startup as a result
#define FLASH_DEBUG               0

// possible CLI errors
#define CLI_ERR_NO_ERROR          0
#define CLI_ERR_CMD_NOT_FOUND     1
#define CLI_ERR_TOO_FEW_ARGS      2
#define CLI_ERR_TOO_MANY_ARGS     3

// Versions
// 1.1.x was Microchip Studio then MPLAB (2022): both tools unstable/unsuitable
// 1.2.x is PlatformIO/VSCode (2023)
const char      versString[] = "1.2.1";

// Constant Data
const char      hello[] = "Dell Vulcan/OCP LED Text Fixture (LTF) V";
const char      cliPrompt[] = "\r\nltf> ";
const int       promptLen = sizeof(cliPrompt);
const float     ADCGain = 2.0;
const float     ADCVrefA = 2.5;
const uint32_t  EEPROM_signature = 0xDE110C01;  // "DeLL Open Compute 01"

// Variable data
float           voltsPerCount = ADCVrefA / 4095.0;
uint16_t        ADC_resultsArray[ADC_OVERSAMPLE_COUNT];
char            outBfr[OUTBFR_SIZE];

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
    uint32_t        sig;
    float           K;
    bool            enCorrection;
    int             offsetError;
    int             gainError;
} EEPROM_data_t;

// FLASH/EEPROM Data buffer
EEPROM_data_t       EEPROMData;

// --------------------------------------------
// Forward function prototypes
// --------------------------------------------
void EEPROM_Save(void);
void ADC_EnableCorrection(void);
uint16_t ADC_Read(uint8_t ch);

// prototypes for CLI-called functions
// template is func_name(int) because the int arg is the arg
// count from parsing the command line; the arg tokens are
// global in tokens[] with tokens[0] = command entered and
// arg count does not include the command token
int help(int);
int calib(int);
int LEDRawRead(int);
int rawRead(int);
int setK(int);
int readLoop(int);
int readTemp(int);
int debug(int);

// CLI token stack
char                *tokens[8];

// CLI command table
// format is "command", function, required arg count, "help line 1", "help line 2" (2nd line can be NULL)
const cli_entry     cmdTable[] = {
    {"debug",    debug, -1, "Debug functions mostly for developer use.", "'debug reset' resets board; 'debug dump' dumps EEPROM"},
    {"help",       help, 0, "THIS DOES NOT DISPLAY ON PURPOSE", " "},
    {"check",  readLoop, 0, "Continuous loop reading raw sensor data.", "Hit any key to exit loop."},
    {"read", LEDRawRead, 0, "Read LED color temperature and intensity.", " "},
    {"temp",   readTemp, 0, "Read board (not MCU core) temperature sensor.", "Reports temperature in degrees C and F."},
    {"set",        setK, 2, "Sets a stored parameter.", "set k 1.234 sets K constant."}
};

#define CLI_ENTRIES     (sizeof(cmdTable) / sizeof(cli_entry))

// --------------------------------------------
// doPrompt() - Write prompt to terminal
// --------------------------------------------
void doPrompt(void)
{
    SerialUSB.println(" ");
    SerialUSB.print(cliPrompt);
    SerialUSB.flush();
}

// --------------------------------------------
// cli() - Command Line Interpreter
// --------------------------------------------
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
    // note that what's stored in tokens[] are string pointers into the
    // input array not the actual string token itself
    while ( (token = (char *) strtok(NULL, delim))  != NULL )
    {
        tokens[tokNdx++] = token;
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
         SerialUSB.println("Invalid command");
        else if ( error == CLI_ERR_TOO_FEW_ARGS )
          SerialUSB.println("Not enough arguments for this command, check help.");
        else if ( error == CLI_ERR_TOO_MANY_ARGS )
          SerialUSB.println("Too many arguments for this command, check help.");
        else
          SerialUSB.println("Unknown parser s/w error");
    }

    doPrompt();
    return(rc);

} // cli()

//===================================================================
//                     DEBUG FUNCTIONS
//===================================================================

// --------------------------------------------
// debug_scan() - I2C bus scanner
//
// While not associated with the board function,
// this was developed in order to locate the
// temp sensor. Left in for future use.
// --------------------------------------------
void debug_scan(void)
{
  byte        count = 0;
  int         scanCount = 0;
  uint32_t    startTime = millis();

  SerialUSB.println ("Scanning I2C bus...");

  for (byte i = 8; i < 120; i++)
  {
    scanCount++;
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      sprintf(outBfr, "Found device at address %d 0x%2X", i, i);
      SerialUSB.println(outBfr);
      count++;
      delay(10);  
    } 
  } 

  SerialUSB.print("Scan complete, addresses scanned:");
  SerialUSB.print(scanCount);
  SerialUSB.print(" in ");
  SerialUSB.print(millis() - startTime);
  SerialUSB.println(" ms");

  if ( count )
  {
    sprintf(outBfr, "Found %d I2C device(s)", count);
    SerialUSB.println(outBfr);
  }
  else
  {
    SerialUSB.println("No I2c device found");
  }
}

// --------------------------------------------
// debug_reset() - force board reset
// --------------------------------------------
void debug_reset(void)
{
    SerialUSB.println("Board reset will disconnect USB-serial connection.");
    SerialUSB.println("Repeat whatever steps you took to connect to the board.");
    delay(1000);
    NVIC_SystemReset();
}

// --------------------------------------------
// debug_dump_eeprom() - Dump EEPROM contents
// --------------------------------------------
void debug_dump_eeprom(void)
{
    SerialUSB.println("EEPROM Contents:");
    SerialUSB.print("Signature:    ");
    SerialUSB.println(EEPROMData.sig, HEX);
    sprintf(outBfr, "%8.4f", EEPROMData.K);
    SerialUSB.print("K Constant:   ");
    SerialUSB.println(outBfr);
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

void debug_read(void)
{
    float       volts;
    uint16_t    raw;

    for ( uint8_t i = 0; i < 6; i++ )
    {
        raw = ADC_Read(i);
        volts = (raw * voltsPerCount) * 2.0;
        sprintf(outBfr, "Ch %d %4d %8.3f V", i, raw, volts);
        SerialUSB.println(outBfr);
    }
}

// --------------------------------------------
// debug() - Main debug program
// --------------------------------------------
int debug(int arg)
{
    if ( arg == 0 )
    {
        SerialUSB.println("Debug commands are:");
        SerialUSB.println("\tscan ... I2C bus scanner");
        SerialUSB.println("\treset .. Reset board");
        SerialUSB.println("\tdump ... Dump EEPROM");
        SerialUSB.println("\tread ... Raw ADC read (channels 0-3)");
        return(0);
    }

    if ( strcmp(tokens[1], "scan") == 0 )
      debug_scan();
    else if ( strcmp(tokens[1], "reset") == 0 )
      debug_reset();
    else if ( strcmp(tokens[1], "dump") == 0 )
      debug_dump_eeprom();
    else if ( strcmp(tokens[1], "read") == 0 )
      debug_read();
    else
      SerialUSB.println("Invalid debug command");

    return(0);
}

//===================================================================
//                            ADC Stuff
//===================================================================

// --------------------------------------------
// syncADC() - Wait for ADC sync complete
// --------------------------------------------
static void syncADC() 
{
  while (ADC->STATUS.bit.SYNCBUSY) {};
}

// --------------------------------------------
// ADC_Init() - Initialze the ADC
// --------------------------------------------
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

  // set analog reference - AREFA = pin 6 VDDANA of MCU
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_AREFA;

  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  // set clock prescalar & resolution
  // this sets ADC to run at 31.25 kHz
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;

  // adjust sample time for possible input impediance (allow ADC to charge cap)
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(1);

//  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2;
  syncADC();

  if ( EEPROMData.enCorrection )
  {
    ADC_EnableCorrection();
  }

  syncADC();
  ADC->CTRLA.bit.ENABLE = 1;
  syncADC();
}

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

// --------------------------------------------
// ADC_Read() - read ADC channel as 16-bits
// --------------------------------------------
uint16_t ADC_Read(uint8_t ch)
{
  uint16_t      result;
  uint32_t      sum = 0;

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = ch;
  ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;

  for ( int i = 0; i < ADC_OVERSAMPLE_COUNT+1; i++ )
  {
    syncADC();
    ADC->SWTRIG.bit.START = 1;

    while ( ADC->INTFLAG.bit.RESRDY == 0 ) ;

    // read result - also clears RESRDY bit
    result = ADC->RESULT.bit.RESULT;
    ADC_resultsArray[i] = result;

    delay(25);

    if ( i == 0 )
      continue;

    sum += result;
  }

  result = (uint16_t) (sum / ADC_OVERSAMPLE_COUNT);
  return(result);
}

//===================================================================
//                               READ Command
//===================================================================
int LEDRawRead(int arg)
{
    volatile float           vRef, vC, vI, lv;
    volatile uint16_t        colorRaw, intensRaw, intensity;
    uint16_t                 vrefRaw;

    // ----------------------
    // Read ADC Vref (2.5V)
    // ----------------------
    vrefRaw = ADC_Read(ADC_VREF_PIN) * ADCGain;
    vRef = vrefRaw * voltsPerCount;
    voltsPerCount = vRef / 4095;
    sprintf(outBfr, "ADC Vref:           %4d %5.3f V", (int) vrefRaw, vRef);
    SerialUSB.println(outBfr);

    // ----------------------
    // Read both ADC channels
    // ----------------------
    colorRaw = ADC_Read(SPECTRA_INTENSITY_OUT_PIN) * ADCGain;
    intensRaw = ADC_Read(SPECTRA_COLOR_OUT_PIN) * ADCGain;

    // set breakpoint on NOP to obtain raw decimal values for
    // each input channel; plug into calculator online to get
    // offset and gain correction values used in ADC driver
    __NOP();

    // ----------------------
    // Calculate Intensity
    // ----------------------
    vI = intensRaw * voltsPerCount;

    // TODO calculate intensity - from graph in datasheet
    // Says the ADC counts correspond to LED's intensity in mcds
    // since sensor voltage is divided by 2, multiply by 2
    // 4V = 20K mcd
    intensity = intensRaw * 2;
    sprintf(outBfr, "Intensity: %4d mcd %4d %5.3f V", (int) intensity, intensRaw, vI);
    SerialUSB.println(outBfr);

    // ----------------------
    // Calculate Color
    // ----------------------
    vC = colorRaw * voltsPerCount;

    // FIXME - lv formula is incorrect; the equation below
    // is from the Spectra DS datasheet, Color Response
    // section
    lv = (vC + 4) * 100.0;

    sprintf(outBfr, "    Color: %4d nm  %4d %5.3f V", (int) lv, colorRaw, vC);
    SerialUSB.println(outBfr);

    return(0);

} // LEDRawRead()

//===================================================================
//                              TEMP Command
//===================================================================
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

  SerialUSB.print("Board temp: ");
  SerialUSB.print(curTemp);
  SerialUSB.print(" C/");
  SerialUSB.print((int) degF, DEC);
  SerialUSB.println(" F");

  return(0);
}

//===================================================================
//                             LOOP Command
//===================================================================
int readLoop(int arg)
{
  SerialUSB.println("Entering continuous loop, press any key to stop");

  while ( SerialUSB.available() == 0 )
  {
    LEDRawRead(0);
    readTemp(0);
    delay(1000);
    SerialUSB.flush();
  }

  // flush any other chars user hit when exiting the loop
  while ( SerialUSB.available() )
    (void) SerialUSB.read();

  return(0);
}

//===================================================================
//                               HELP Command
//===================================================================
int help(int arg)
{

    SerialUSB.println(" ");
    SerialUSB.print(hello);
    SerialUSB.println(versString);
    SerialUSB.println("Enter a command then press ENTER. Some commands require arguments, which must");
    SerialUSB.println("be separated from the command and other arguments by a space.");
    SerialUSB.println("Up arrow repeats the last command; backspace or delete erases the last");
    SerialUSB.println("character entered. Commands available are:");

    for ( int i = 0; i < (int) CLI_ENTRIES; i++ )
    {
      if ( strcmp(cmdTable[i].cmd, "help") == 0 )
        continue;

      sprintf(outBfr, "%s\t%s", cmdTable[i].cmd, cmdTable[i].help1);
      SerialUSB.println(outBfr);

      if ( cmdTable[i].help2 != NULL )
      {
        sprintf(outBfr, "\t%s", cmdTable[i].help2);
        SerialUSB.println(outBfr);
      }

      SerialUSB.flush();
    }

    return(0);
}

//===================================================================
//                              SET Command
//
// set <parameter> <value>
// 
// Supported parameters: k, gain, offset
//===================================================================
int setK(int arg)
{
    char          *parameter = tokens[1];
    String        userEntry = tokens[2];
    float         fValue;
    int           iValue;
    bool          isDirty = false;

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
              SerialUSB.println("ADC correction off");
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
          SerialUSB.println("Invalid ADC corr argument: must be 'on' or 'off'");
        }
    }
    else
    {
        SerialUSB.println("Invalid parameter name");
        return(1);
    }

    if ( isDirty )
        EEPROM_Save();

    return(0);
}

//===================================================================
//                      EEPROM/NVM Stuff
//===================================================================

// --------------------------------------------
// EEPROM_Save() - write EEPROM structure to
// the EEPROM
// --------------------------------------------
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

// --------------------------------------------
// EEPROM_Read() - Read struct from EEPROM
// --------------------------------------------
void EEPROM_Read(void)
{
    uint8_t         *p = (uint8_t *) &EEPROMData;
    uint16_t        eepromAddr = 0;

    for ( int i = 0; i < (int) sizeof(EEPROM_data_t); i++ )
    {
        *p++ = EEPROM.read(eepromAddr++);
    }
}

// --------------------------------------------
// EEPROM_Defaults() - Set defaults in struct
// --------------------------------------------
void EEPROM_Defaults(void)
{
    EEPROMData.sig = EEPROM_signature;
    EEPROMData.K = 1.0;
    EEPROMData.enCorrection = false;
    EEPROMData.gainError = 1400;
    EEPROMData.offsetError = -69;
}

// --------------------------------------------
// EEPROM_InitLocal() - Initialize EEPROM & Data
//
// Returns false if no error, else true
// --------------------------------------------
bool EEPROM_InitLocal(void)
{
    bool          rc = false;

    EEPROM_Read();

    if ( EEPROMData.sig != EEPROM_signature )
    {
      // EEPROM failed: either never been used, or real failure
      // initialize the signature and settings

      // FIXME: When debugging, EEPROM fails every time, but it
      // is OK over resets and power cycles.  
      EEPROM_Defaults();

      // save EEPROM data on the device
      EEPROM_Save();

      rc = true;
      SerialUSB.println("EEPROM validation FAILED, EEPROM initialized OK");
    }
    else
    {
      SerialUSB.println("EEPROM validated OK");
    }

    return(rc);
}

//===================================================================
//                      setup() - Initialization
//===================================================================
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
  // this is a bit wonky; this is vague about the odd/even pins partly
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
  EEPROM_InitLocal();
  ADC_Init();
  Wire.begin();

  SerialUSB.println(" ");
  SerialUSB.print(hello);
  SerialUSB.println(versString);
  doPrompt();

} // setup()

//===================================================================
//                     loop() - Main Program Loop
//
// FLASHING NOTE: loop() doesn't get called until USB-serial connection
// has been established (ie, SerialUSB = 1).
//
// This loop does two things: blink the heartbeat LED and handle
// incoming characters over SerialUSB connection.  Once a full CR
// terminated input line has been received, it calls the CLI to
// parse and execute any valid command, or sends an error message.
//===================================================================
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
          SerialUSB.println(byteIn);
      }
      else if ( byteIn == 0x0d )
      {
          // carriage return - EOL 
          // save as the last cmd (for up arrow) and call CLI with
          // the completed line less CR/LF
          SerialUSB.println(" ");
          inBfr[inCharCount] = 0;
          inCharCount = 0;
          strcpy(lastCmd, inBfr);
          cli(inBfr);
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
                        doPrompt();
                        SerialUSB.println(lastCmd);
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