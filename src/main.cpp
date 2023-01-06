#include <Arduino.h>
#include "Wire.h"
#include "float.h"
#include <FlashStorage.h>

const char hello[] = "\r\nDell OCP Led Text Fixture (LTF)\r\n";
const char cliPrompt[] = "\r\nltf> ";
const int promptLen = sizeof(cliPrompt);

#define SPECTRA_COLOR_OUT_PIN       ADC_INPUTCTRL_MUXPOS_PIN3_Val
#define SPECTRA_INTENSITY_OUT_PIN   ADC_INPUTCTRL_MUXPOS_PIN2_Val
#define AT30TS74_I2C_ADDR           72 // 0x48

#define CMD_NAME_MAX        12
#define MAX_LINE_SZ         80
#define OUTBFR_SIZE         (MAX_LINE_SZ*3)

// Versions
// 1.1.x was MPLAB (2022)
// 1.2.x is PlatformIO/VSCode (2023)
#define VERSION             "1.2.1"

typedef struct {
    char        cmd[CMD_NAME_MAX];
    int         (*func) (int x);
    int         argCount;
    char        help1[MAX_LINE_SZ];
    char        help2[MAX_LINE_SZ];

} cli_entry;

int help(int);
int calib(int);
int LEDRawRead(int);
int rawRead(int);
int vers(int);
int readLoop(int);
int readTemp(int);
int debug(int);

static char                *tokens[4];

cli_entry           cmdTable[] = {
    {"debug",     debug, 0, "Debug functions for developer use only.", "Use caution if not a developer!"},
    {"help",       help, 0, "Help for the help command.", NULL},
    {"loop",   readLoop, 0, "Continuous loop reading raw data.", "Hit any key to exit loop."},
    {"read", LEDRawRead, 0, "Read LED color temperature and intensity.", NULL},
    {"temp",   readTemp, 0, "Reads board temperature sensor.", "Reports temperature in degrees C and F."},
    {"vers",       vers, 0, "Displays software version.", NULL},
};

#define CLI_ENTRIES     (sizeof(cmdTable) / sizeof(cli_entry))

void terminalOut(char *buffer)
{
  if ( strchr(buffer, 0x0d) )
    SerialUSB.print(buffer);
  else
    SerialUSB.println(buffer);
}

void doPrompt(void)
{
  for ( int i = 0; i < promptLen; i++ )
  {
    SerialUSB.write(cliPrompt[i]);
  }
}

bool cli(char *raw)
{
    bool         rc = false;
    int         len;
    char        *token;
    const char  delim[] = " ";
    int         tokNdx = 0;
    char        input[80];
    char        *s = input;
    
    strcpy(input, (char *) raw);
    len = strlen(input);

    // initial call, should get and save the command as 0th token
    token = strtok(input, delim);
    tokens[tokNdx++] = token;

    // subsequent calls should parse any space-separated args into tokens[]
    // note that what's stored in tokens[] are string pointers into the
    // input array not the actual string token itself
    while ( (token = (char *) strtok(NULL, delim))  != NULL )
    {
        tokens[tokNdx++] = token;
    }

    for ( int i = 0; i < CLI_ENTRIES; i++ )
    {
        if ( strncmp(tokens[0], cmdTable[i].cmd, len) == 0 )
        {
            // command funcs are passed arg count, tokens are global
            (cmdTable[i].func) (tokNdx);
            rc = true;
            break;
        }
    }

    if ( rc == false )
    {
        terminalOut("Invalid command");
    }

    doPrompt();
    return(rc);

} // cli()

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

void debug_reset(void)
{
  SerialUSB.println("Board reset will disconnect USB-serial connection.");
  SerialUSB.println("Repeat whatever steps you took to connect to the board.");
  delay(1000);
  NVIC_SystemReset();
}

int debug(int arg)
{
    char        outBfr[80];

#if 0
// uncomment to debug a debug function ;-)
    if ( arg )
    {
      sprintf(outBfr, "Arg: %d Token: %s", arg, tokens[1]);
      terminalOut(outBfr);
    }
#endif

    if ( strcmp(tokens[1], "scan") == 0 )
      debug_scan();
    else if ( strcmp(tokens[1], "reset") == 0 )
      debug_reset();

}

static void syncADC() 
{
  while (ADC->STATUS.bit.SYNCBUSY) {};
}

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

#if 0
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
  int16_t       offset_error = -69;
  int16_t       gain_error = 1400;

  syncADC();
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset_error);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain_error);
  ADC->CTRLB.bit.CORREN = 1;
#endif

  syncADC();
  ADC->CTRLA.bit.ENABLE = 1;

}

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

int LEDRawRead(int arg)
{
    colorRaw = ADC_Read(2);
    intensRaw = ADC_Read(3);

    // set breakpoint on NOP to obtain raw decimal values for
    // offset and gain correction calculator
    __NOP();

    vC = maxVolts * ((float) colorRaw / maxAdcBits);
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

  // data is two's complement so nothing need be done
  curTemp = (short) i2cData;
  degF = ((curTemp * 9.0) / 5.0) + 32.0;

  SerialUSB.print("Board temp: ");
  SerialUSB.print(curTemp);
  SerialUSB.print(" C/");
  SerialUSB.print((int) degF, DEC);
  SerialUSB.println(" F");

  return(0);
}

int readLoop(int arg)
{
  SerialUSB.println("Entering continuous loop, press any key to stop");

  while ( SerialUSB.available() == 0 )
  {
    LEDRawRead(0);
    readTemp(0);
    delay(1000);
  }

  while ( SerialUSB.available() )
    (void) SerialUSB.read();

  return(0);
}

int help(int arg)
{
    char        outBfr[80];
    
    SerialUSB.println("Enter a command then press ENTER. Some commands allow arguments");
    SerialUSB.println("which must be separated from the command and other arguments by");
    SerialUSB.println("a space.  Up arrow repeats the last command; backspace or delete");
    SerialUSB.println("erases the last character entered.");
    SerialUSB.println("Commands Available:");

    for ( int i = 0; i < CLI_ENTRIES; i++ )
    {
      if ( strcmp(cmdTable[i].cmd, "help") == 0 )
        continue;

      sprintf(outBfr, "%s\r\n", cmdTable[i].cmd);
      terminalOut(outBfr);
      sprintf(outBfr, "\t%s\r\n", cmdTable[i].help1);
      terminalOut(outBfr);
      if ( cmdTable[i].help2 != NULL )
      {
        sprintf(outBfr, "\t%s\r\n", cmdTable[i].help2);
        terminalOut(outBfr);
      }
    }

    return(0);
}

int vers(int arg)
{
    char        outBfr[80];
    
    sprintf(outBfr, "Version %s has %d commands implemented\r\n", VERSION, (int) CLI_ENTRIES);
    terminalOut(outBfr);
    return(0);
}





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

  // configure heartbeat LED pin and turn on
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LEDstate);

  ADC_Init();
  Wire.begin();

  SerialUSB.begin(115200);
  while ( !SerialUSB )
  {
    LEDstate = LEDstate ? 0 : 1;
    digitalWrite(PIN_LED, LEDstate);
    delay(200);
  }

  SerialUSB.println(hello);
  doPrompt();
}

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
      // delete & backspace
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
}