#include <Arduino.h>
#include "float.h"

const char hello[] = "\r\nDell OCP Led Text Fixture (LTF)\r\n";
const char cliPrompt[] = "\r\nltf> ";
const int promptLen = sizeof(cliPrompt);

#define SPECTRA_COLOR_OUT_PIN       A3
#define SPECTRA_INTENSITY_OUT_PIN   A2
#define CMD_NAME_MAX        12
#define MAX_LINE_SZ         80
#define OUTBFR_SIZE         (MAX_LINE_SZ*3)



// Versions
// 1.1.x was MPLAB
// 1.2.x is PlatformIO/VSCode
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

static char                *tokens[4];

cli_entry           cmdTable[] = {
    {"help",       help, 0, "Help for the help command\r\n", "help line 2\r\n"},
    {"loop",   readLoop, 0, "Continuous loop reading raw data\r\n"},
    {"read", LEDRawRead, 0, "Read LED temperature.\r\n", "Reports temp in degrees C.\r\n"},
    {"vers",       vers, 0, "Displays software version\r\n", "vers line 2\r\n"},
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

    // initial call, should get command else table lookup below will fail
    token = strtok(input, delim);
    tokens[tokNdx++] = token;

    // subsequent calls, should parse any args
    while ( (token = (char *) strtok(NULL, delim))  != NULL )
    {
        tokens[tokNdx++] = token;
    }

    for ( int i = 0; i < CLI_ENTRIES; i++ )
    {
        if ( strncmp(s, cmdTable[i].cmd, len) == 0 )
        {
            // command funcs are passed arg count, tokens are global
            (cmdTable[i].func) (tokNdx);
            rc = true;
            break;
        }
    }

    doPrompt();
    return(rc);

} // cli()

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
    SerialUSB.print("    Color: ");
    SerialUSB.print(lv, 4);
    SerialUSB.print(" nm [raw: 0x");
    SerialUSB.print(colorRaw, HEX);
    SerialUSB.print(" ");
    SerialUSB.print(vC, 3);
    SerialUSB.println(" V]");

    vI = maxVolts * ((float) intensRaw / maxAdcBits);
    SerialUSB.print("Intensity: ");
    SerialUSB.print(intensRaw);
    SerialUSB.print("         [raw: 0x");
    SerialUSB.print(intensRaw, HEX);
    SerialUSB.print(" ");
    SerialUSB.print(vI, 3);
    SerialUSB.println(" V]");

    return(0);

} // LEDRawRead()

int readLoop(int arg)
{
  int       byteIn;
  bool      loopControl = true;

  SerialUSB.println("Entering continuous loop, hit any key to stop");

  while ( loopControl == true )
  {
    if ( SerialUSB.available() )
    {
      byteIn = SerialUSB.read();
      loopControl = false;
    }
    else
    {
      LEDRawRead(0);
      delay(1000);
    }
  }

  return(0);
}

int help(int arg)
{
    char        outBfr[80];
    
    sprintf(outBfr, "Commands Available:\r\n");
    terminalOut(outBfr);

    for ( int i = 0; i < CLI_ENTRIES; i++ )
    {
        sprintf(outBfr, "%s\r\n", cmdTable[i].cmd);
        terminalOut(outBfr);
        terminalOut(cmdTable[i].help1);
        terminalOut(cmdTable[i].help2);
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

  ADC_Init();

  SerialUSB.begin(115200);
  while ( !SerialUSB )
  {
    ; // wait for USBSerial()
  }

  SerialUSB.println(hello);
  doPrompt();
}

void loop() 
{
  int             byteIn;
  static char     inBfr[80];
  static int      inCharCount = 0;

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
      cli(inBfr);
    }
    else
    {
      SerialUSB.write((char) byteIn);
      inBfr[inCharCount++] = byteIn;
    }
  }
}