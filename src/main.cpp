#include <Arduino.h>
#include "Wire.h"
#include "float.h"
#include "FlashAsEEPROM_SAMD.h"

// Versions
// 1.1.x was Microchip Studio then MPLAB (2022)
// 1.2.x is PlatformIO/VSCode (2023)
const char        versString[] = "1.2.1";

const char hello[] = "\r\nDell Vulcan/OCP LED Text Fixture (LTF) V\r\n";
const char cliPrompt[] = "\r\nltf> ";
const int promptLen = sizeof(cliPrompt);

#define SPECTRA_COLOR_OUT_PIN       ADC_INPUTCTRL_MUXPOS_PIN3_Val
#define SPECTRA_INTENSITY_OUT_PIN   ADC_INPUTCTRL_MUXPOS_PIN2_Val
#define AT30TS74_I2C_ADDR           72 // 0x48

#define CMD_NAME_MAX        12
#define MAX_LINE_SZ         80
#define OUTBFR_SIZE         (MAX_LINE_SZ*3)

// disable EEPROM/FLASH debug, because it uses Serial not SerialUSB
#define FLASH_DEBUG         0

typedef struct {
 uint16_t         pageSize;
 uint32_t         pageCount;
 uint16_t         rowSize;
 uint16_t         EEPROMPages;

} FLASH_DvcCtrl_t;

typedef struct {
    uint32_t        sig;
    float           K;
    bool            enCorrection;
    int             offsetError;
    int             gainError;
} EEPROM_data_t;

// FLASH/EEPROM Device Control Block
static FLASH_DvcCtrl_t  FLASH_DCB;
static const uint32_t   EEPROM_signature = 0xDE110C01;  // "DeLL Open Compute 01"
static EEPROM_data_t    EEPROMData;

// CLI Command Table structure
typedef struct {
    char        cmd[CMD_NAME_MAX];
    int         (*func) (int x);
    int         argCount;
    char        help1[MAX_LINE_SZ];
    char        help2[MAX_LINE_SZ];

} cli_entry;

// --------------------------------------------
// Function prototypes
// --------------------------------------------
void EEPROM_Save(void);
void ADC_EnableCorrection(void);

// prototypes for CLI-called functions
// template is func_name(int) because the int arg is the arg
// count from parsing the command line; the arg tokens are
// global in tokens[] with tokens[0] = command entered
int help(int);
int calib(int);
int LEDRawRead(int);
int rawRead(int);
int setK(int);
int readLoop(int);
int readTemp(int);
int debug(int);

// CLI token stack
static char                *tokens[8];

// CLI command table
// format is "command", function, required arg count, "help line 1", "help line 2" (2nd line can be NULL)
cli_entry           cmdTable[] = {
    {"debug",     debug, -1, "Debug functions for developer use only.", NULL},
    {"help",       help, 0, "THIS DOES NOT DISPLAY ON PURPOSE", NULL},
    {"loop",   readLoop, 0, "Continuous loop reading raw sensor data.", "Hit any key to exit loop."},
    {"read", LEDRawRead, 0, "Read LED color temperature and intensity.", NULL},
    {"temp",   readTemp, 0, "Read board (not MCU core) temperature sensor.", "Reports temperature in degrees C and F."},
    {"set",        setK, 2, "Sets a stored parameter.", "Requires 2 args: parameter and value\r\n\tParameters: k,gain,offset"},
};

#define CLI_ENTRIES     (sizeof(cmdTable) / sizeof(cli_entry))

// --------------------------------------------
// terminalOut() - wrapper to SerialUSB.print
// that differentiates whether or not a CR is
// in the given buffer to print
// --------------------------------------------
void terminalOut(char *buffer)
{
  if ( strchr(buffer, 0x0d) )
      SerialUSB.print(buffer);
  else
      SerialUSB.println(buffer);

  SerialUSB.flush();
}

// --------------------------------------------
// doPrompt() - Write prompt to terminal
// --------------------------------------------
void doPrompt(void)
{
    for ( int i = 0; i < promptLen; i++ )
    {
        SerialUSB.write(cliPrompt[i]);
    }

    SerialUSB.flush();
}

// possible CLI errors
#define CLI_ERR_NO_ERROR          0
#define CLI_ERR_CMD_NOT_FOUND     1
#define CLI_ERR_TOO_FEW_ARGS      2
#define CLI_ERR_TOO_MANY_ARGS     3

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
    char        *s = input;
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

    for ( int i = 0; i < CLI_ENTRIES; i++ )
    {
        if ( strncmp(tokens[0], cmdTable[i].cmd, len) == 0 )
        {
            if ( cmdTable[i].argCount == argCount || cmdTable[i].argCount == -1 )
            {
                // command funcs are passed arg count, tokens are global
                (cmdTable[i].func) (argCount);
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
         terminalOut("Invalid command");
        else if ( error == CLI_ERR_TOO_FEW_ARGS )
          terminalOut("Not enough arguments for this command, check help.");
        else if ( error == CLI_ERR_TOO_MANY_ARGS )
          terminalOut("Too many arguments for this command, check help.");
        else
          terminalOut("Unknown parser s/w error");
    }

    doPrompt();
    return(rc);

} // cli()

//===================================================================
//                     DEBUG FUNCTIONS
//===================================================================

// --------------------------------------------
// debug_scan() - I2C bus scanner
// --------------------------------------------
void debug_scan(void)
{
  byte        count = 0;

  SerialUSB.println ("Scanning I2C bus...");

  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      SerialUSB.print ("Found device at address ");
      SerialUSB.print (i, DEC);
      SerialUSB.print (" (0x");
      SerialUSB.print (i, HEX);
      SerialUSB.println (")");
      count++;
      delay (1);  
    } 
  } 

  SerialUSB.println ("Scan Complete.");
  if ( count )
  {
    SerialUSB.print ("Found ");
    SerialUSB.print (count, DEC);
    SerialUSB.println (" I2C device(s).");
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
    char          outBfr[20];

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

// --------------------------------------------
// debug() - Main debug program
// --------------------------------------------
int debug(int arg)
{
    char        outBfr[80];

#if 0
// uncomment to debug a debug function ;-)
    if ( arg )
    {
      sprintf(outBfr, "Arg: %d Tokens: %s", arg, tokens[1]);
      terminalOut(outBfr);
    }
#endif

    if ( strcmp(tokens[1], "scan") == 0 )
      debug_scan();
    else if ( strcmp(tokens[1], "reset") == 0 )
      debug_reset();
    else if ( strcmp(tokens[1], "dump") == 0 )
        debug_dump_eeprom();
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
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  // set clock prescalar and result to 16 bits (necessary for oversampling)
  // this sets ADC to run at 31.25 kHz
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;

  // adjust sample time for possible input impediance (allow ADC to charge cap)
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(1);

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2;
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

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = ch;
  ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;

  for ( int i = 0; i < 2; i++ )
  {
    syncADC();
    ADC->SWTRIG.bit.START = 1;

    while ( ADC->INTFLAG.bit.RESRDY == 0 ) ;

    // read result - also clears RESRDY bit
    result = ADC->RESULT.bit.RESULT;
  }

  return(result);
}

const float     maxAdcBits = (int) (pow(2,12) - 1);
const float     maxVolts = 3.3f;

volatile float           vC, vI, lv;
volatile uint16_t        colorRaw, intensRaw;

//===================================================================
//                               READ Command
//===================================================================
int LEDRawRead(int arg)
{
    colorRaw = ADC_Read(SPECTRA_INTENSITY_OUT_PIN);
    intensRaw = ADC_Read(SPECTRA_COLOR_OUT_PIN);

    // set breakpoint on NOP to obtain raw decimal values for
    // offset and gain correction calculator
    __NOP();

    vC = maxVolts * ((float) colorRaw / maxAdcBits);

    // FIXME - lv formula is incorrect
    lv = (vC + 4) * 100.0;

    SerialUSB.print("     Color: ");
    SerialUSB.print(lv, 4);
    SerialUSB.print(" nm [raw: 0x");
    SerialUSB.print(colorRaw, HEX);
    SerialUSB.print(" ");
    SerialUSB.print(vC, 3);
    SerialUSB.println(" V]");

    vI = maxVolts * ((float) intensRaw / maxAdcBits);
    SerialUSB.print(" Intensity: ");
    SerialUSB.print(intensRaw);
    SerialUSB.print("         [raw: 0x");
    SerialUSB.print(intensRaw, HEX);
    SerialUSB.print(" ");
    SerialUSB.print(vI, 3);
    SerialUSB.println(" V]");

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
  char                outBfr[80];

  Wire.beginTransmission(AT30TS74_I2C_ADDR);
  Wire.write(0);      // set pointer register
  Wire.endTransmission();
  delay(65);
  Wire.requestFrom(AT30TS74_I2C_ADDR, 1, false);

  // read MSB only - sufficient for this project
  i2cData = Wire.read();

  // data is two's complement so nothing needs be done
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
    char        outBfr[160];
    
    SerialUSB.print("Firmware version: ");
    SerialUSB.println(versString);
    SerialUSB.println("Enter a command then press ENTER. Some commands allow arguments which must");
    SerialUSB.println("be separated from the command and other arguments by a space.");
    SerialUSB.println("Up arrow repeats the last command; backspace or delete erases the last");
    SerialUSB.println("character entered. Commands available are:");

    for ( int i = 0; i < CLI_ENTRIES; i++ )
    {
      if ( strcmp(cmdTable[i].cmd, "help") == 0 )
        continue;

      sprintf(outBfr, "%s\t%s\r\n", cmdTable[i].cmd, cmdTable[i].help1);
      terminalOut(outBfr);

      if ( cmdTable[i].help2 != NULL )
      {
        sprintf(outBfr, "\t%s\r\n", cmdTable[i].help2);
        terminalOut(outBfr);
      }
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
        terminalOut("Invalid parameter name");
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

    for ( int i = 0; i < sizeof(EEPROM_data_t); i++ )
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

    for ( int i = 0; i < sizeof(EEPROM_data_t); i++ )
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
// --------------------------------------------
bool EEPROM_InitLocal(void)
{
    uint16_t        pageSizeField;
    uint32_t        pageCount;
    bool            rc = true, tempBool;
    uint32_t        temp;
    uint8_t         *p = (uint8_t *) &EEPROMData;
    uint16_t        eepromAddr = 0;

    pageSizeField = NVMCTRL->PARAM.bit.PSZ;
    FLASH_DCB.pageSize = pageSizes[pageSizeField];
    FLASH_DCB.pageCount = NVMCTRL->PARAM.bit.NVMP;
    FLASH_DCB.rowSize = FLASH_DCB.pageSize * 4;

    tempBool = EEPROM.isValid();

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

      rc = 1;
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

  // configure pins AIN2 & AIN3 (PB08 & PB09)
  // NOTE: Group 1 = PB
  PORT->Group[1].DIRCLR.reg = PORT_PB09 | PORT_PB09;
  PORT->Group[1].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[1].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

  // see section 7.1 in SAMD21 datasheet - multiplexed signals
  // this is a bit wonky; this is vague about the odd/even pins partly
  // due to Arduino abstraction, and partly because it's just not clear...
  PORT->Group[1].PMUX[3].reg = PORT_PMUX_PMUXO_B;
  PORT->Group[1].PMUX[4].reg = PORT_PMUX_PMUXO_B;

  // configure heartbeat LED pin and turn on which indicates that the
  // board is being initialized
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LEDstate);



  SerialUSB.begin(115200);
  while ( !SerialUSB )
  {
      // fast blink while waiting for a connection
      LEDstate = LEDstate ? 0 : 1;
      digitalWrite(PIN_LED, LEDstate);
      delay(200);
  }

  // initialize EEPROM
  EEPROM_InitLocal();
  ADC_Init();
  Wire.begin();

  SerialUSB.print(hello);
  SerialUSB.println(versString);
  doPrompt();

} // setup()

//===================================================================
//                     loop() - Main Program Loop
//===================================================================
// FLASHING NOTE: loop() doesn't get called until USB-serial connection
// has been established (ie, SerialUSB = 1).
void loop() 
{
  int             byteIn;
  static char     inBfr[80];
  static int      inCharCount = 0;
  static char     lastCmd[80] = {0};
  const char      bs[4] = {0x1b, '[', '1', 'D'};  // terminal: backspace seq
  const char      cl[4] = {0x1b, '[', '2', 'K'};  // terminal: clear line seq
  static bool     LEDstate = false;
  static uint32_t time = millis();

  // blink heartbeat LED
  if ( millis() - time >= 1000  )
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
          SerialUSB.println(byteIn);
      else if ( byteIn == 0x0d )
      {
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
                        terminalOut(lastCmd);
                        cli(lastCmd);
                    }
                }
            }
        }
    }
    else if ( byteIn == 127 || byteIn == 8 )
    {
        // delete & backspace do the same thing
        inBfr[inCharCount] = 0;
        inCharCount--;
        SerialUSB.write(bs, 4);
        SerialUSB.write(' ');
        SerialUSB.write(bs, 4);
    }
    else
    {
        // all other keys get stored in buffer
        SerialUSB.write((char) byteIn);
        inBfr[inCharCount++] = byteIn;
    }
  }

} // loop()