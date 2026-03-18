/***************************************************************************//**
 * @file app.c
 * @brief IADC Scan Example - EFR32BG21, Gecko SDK 4.1.0
 *        Channels: PC1 (battery), PC4 (motor1), PD1 (motor2)
 *   only created by AI, not verified!
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"

// IADC input clock (from FSRCO or HFXO, must be <= 40 MHz for Series 2)
#define CLK_SRC_ADC_FREQ    20000000  // 20 MHz
#define CLK_ADC_FREQ        10000000  // 10 MHz ADC clock

// Number of scan channels
#define NUM_CHANNELS        3

// Scan result storage
volatile IADC_Result_t scanResult[NUM_CHANNELS];
volatile bool conversionDone = false;

/***************************************************************************//**
 * @brief  IADC scan complete IRQ handler
 ******************************************************************************/
void IADC_IRQHandler(void)
{
  // Read results for all 3 scan table entries
  for (int i = 0; i < NUM_CHANNELS; i++) {
    scanResult[i] = IADC_pullScanFifoResult(IADC0);
  }
  conversionDone = true;
  IADC_clearInt(IADC0, IADC_IF_SCANTABLEDONE);
}

/***************************************************************************//**
 * @brief  Initialize IADC for scan mode on PC1, PC4, PD1
 ******************************************************************************/
void initIADC(void)
{
  // Enable clocks
  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Set PC1, PC4, PD1 as analog inputs (disabled output, no pull)
  GPIO_PinModeSet(gpioPortC, 1, gpioModeDisabled, 0);
  GPIO_PinModeSet(gpioPortC, 4, gpioModeDisabled, 0);
  GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 0);

  // IADC init structure - use defaults
  IADC_Init_t init = IADC_INIT_DEFAULT;
  // Set ADC clock prescaler: CLK_ADC = CLK_SRC_ADC / (prescale + 1)
  // With FSRCO at 20 MHz, prescale=1 gives 10 MHz ADC clock
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // IADC allConfigs - use internal 1.2V bandgap reference
  IADC_AllConfigs_t allConfigs = IADC_ALLCONFIGS_DEFAULT;
  allConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
  allConfigs.configs[0].vRef = 1210; // mV
  allConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(
      IADC0,
      CLK_ADC_FREQ,
      0,
      iadcCfgModeNormal,
      init.srcClkPrescale);

  // Scan table init
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  initScan.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID3; // trigger when 3 results ready
  initScan.showId = true; // include channel ID in result

  // Scan table entries
  IADC_ScanTable_t scanTable = IADC_SCANTABLE_DEFAULT;

  // Entry 0: PC1 - Battery (single-ended, negative = GND)
  scanTable.entries[0].posInput = iadcPosInputPortCPin1;
  scanTable.entries[0].negInput = iadcNegInputGnd;
  scanTable.entries[0].includeInScan = true;

  // Entry 1: PC4 - Motor1 current (single-ended, negative = GND)
  scanTable.entries[1].posInput = iadcPosInputPortCPin4;
  scanTable.entries[1].negInput = iadcNegInputGnd;
  scanTable.entries[1].includeInScan = true;

  // Entry 2: PD1 - Motor2 current (single-ended, negative = GND)
  scanTable.entries[2].posInput = iadcPosInputPortDPin1;
  scanTable.entries[2].negInput = iadcNegInputGnd;
  scanTable.entries[2].includeInScan = true;

  // Initialize IADC
  IADC_init(IADC0, &init, &allConfigs);
  IADC_initScan(IADC0, &initScan, &scanTable);

  // Enable scan table done interrupt
  IADC_enableInt(IADC0, IADC_IEN_SCANTABLEDONE);
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

/***************************************************************************//**
 * @brief  Convert raw IADC result to millivolts
 *         Using internal 1.21V reference, 12-bit, single-ended
 ******************************************************************************/
uint32_t convertToMillivolts(uint32_t rawData)
{
  // VFS = VREF = 1210 mV (single-ended, gain=1)
  // mV = rawData * 1210 / 4095
  return (rawData * 1210) / 4095;
}

/***************************************************************************//**
 * @brief  Trigger a scan conversion and return results
 ******************************************************************************/
void readAnalogChannels(uint32_t *batteryMv, uint32_t *motor1Mv, uint32_t *motor2Mv)
{
  conversionDone = false;

  // Start scan conversion
  IADC_command(IADC0, iadcCmdStartScan);

  // Wait for conversion to complete
  while (!conversionDone);

  // Convert results to mV
  *batteryMv = convertToMillivolts(scanResult[0].data);
  *motor1Mv  = convertToMillivolts(scanResult[1].data);
  *motor2Mv  = convertToMillivolts(scanResult[2].data);
}

/***************************************************************************//**
 * @brief  Main application entry
 ******************************************************************************/
int main(void)
{
  CHIP_Init();

  initIADC();

  uint32_t batteryMv, motor1Mv, motor2Mv;

  while (1) {
    readAnalogChannels(&batteryMv, &motor1Mv, &motor2Mv);

    // Use the values as needed:
    // batteryMv  -> battery voltage in mV (on PC1)
    // motor1Mv   -> motor1 current sense voltage in mV (on PC4)
    // motor2Mv   -> motor2 current sense voltage in mV (on PD1)

    // Add delay or sleep as needed
  }
}
