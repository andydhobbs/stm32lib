/**
  ******************************************************************************
  * @file    stm32u5xx_hal_pwr_ex.c
  * @author  MCD Application Team
  * @brief   Extended PWR HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Power Controller extension peripheral :
  *           + Power Supply Control Functions
  *           + Low Power Control Functions
  *           + Voltage Monitoring Functions
  *           + Memories Retention Functions
  *           + I/O Pull-Up Pull-Down Configuration Functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) Call HAL_PWREx_ControlVoltageScaling() and HAL_PWREx_GetVoltageRange() to
       set / get the voltage scaling range.
      (+) Voltage scaling can be one of the following values :
             (++) voltage output scale 1 : 1V2
                  => Used when system clock frequency is up to 160 MHz
             (++) voltage output scale 2 : 1V1
                  => Used when system clock frequency is up to 100 MHz
             (++) voltage output scale 3 : 1V0
                  => Used when system clock frequency is up to 50 MHz
             (++) voltage output scale 4 : 0V9
                  => Used when system clock frequency is up to 24 MHz

   (#) Call HAL_PWREx_EnableFastSoftStart() and HAL_PWREx_DisableFastSoftStart()
       to enable / disable the fast software startup for the current regulator.

   (#) Call HAL_PWREx_EnterSTOP1Mode() function to enter the whole system to
       Stop 1 mode. Wake-up from Stop 1 mode could be following to an event or
       an interrupt according to low power mode intrinsic request called
       (__WFI() or __WFE()). (Regulator state on U5 devices is managed
       internally but regulator parameter is kept for product compatibility).

   (#) Call HAL_PWREx_EnterSTOP2Mode() function to enter the whole system to
       Stop 2 mode. Wake-up from Stop 2 mode could be following to an event or
       an interrupt according to low power mode intrinsic request called
       (__WFI() or __WFE()). (Regulator state on U5 devices is managed
       internally but regulator parameter is kept for product compatibility).

   (#) Call HAL_PWREx_EnterSTOP3Mode() function to enter the whole system to
       Stop 3 mode. Wake-up from Stop 3 mode could be following to an event or
       an interrupt according to low power mode intrinsic request called
       (__WFI() or __WFE()). (Regulator state on U5 devices is managed
       internally but regulator parameter is kept for product compatibility).

   (#) Call HAL_PWREx_EnterSHUTDOWNMode() function to enter the whole system in
       Shutdown mode. Wake-up from Shutdown mode can be following to an external
       reset (NRST pin), a WKUP pin event (configurable rising or falling edge),
       or an RTC event occurs (alarm, periodic wakeup, timestamp), or a tamper
       detection.

   (#) Call HAL_PWREx_ConfigSRDDomain() to force in Run mode or to enter in Stop
       mode Smart Run Domain when the system enters Stop mode (Stop 0/1/2).

   (#) Call HAL_PWREx_EnableUltraLowPowerMode() and
       HAL_PWREx_DisableUltraLowPowerMode() to enable / disable the BOR ultra
       low power mode.

   (#) Call HAL_PWREx_S3WU_IRQHandler() function to handle the PWR Stop 3 wake
       up interrupt request.

   (#) Call HAL_PWREx_EnableBatteryCharging() and
       HAL_PWREx_DisableBatteryCharging() to enable / disable the battery
       charging capability when VDD alimentation is available.

   (#) Call HAL_PWREx_EnableVddUSB(), HAL_PWREx_EnableVddIO2() and
       HAL_PWREx_EnableVddA() to enable respectively VDDUSB, VDDIO2 and VDDA
       electrical and logical isolation.
       It is recommanded to disable VDDUSB, VDDIO2 and VDDA electrical and
       logical isolation through HAL_PWREx_DisableVddUSB(),
       HAL_PWREx_DisableVddIO2() and HAL_PWREx_DisableVddA().

   (#) Call HAL_PWREx_ConfigPVM() after setting parameters to be configured
       (event mode and PVD type) in order to set up the Peripheral Voltage
       Monitor, then use HAL_PWREx_EnableUVM(), HAL_PWREx_EnableIO2VM(),
       HAL_PWREx_EnableAVM1() and HAL_PWREx_EnableAVM2() functions to start the
       PVM VDDx monitoring and use HAL_PWREx_DisableUVM(),
       HAL_PWREx_DisableIO2VM(), HAL_PWREx_DisableAVM1() and
       HAL_PWREx_DisableAVM2() to stop the PVM VDDx monitoring.
       (+) PVM monitored voltages are :
             (++) VDDUSB versus 1V2
             (++) VDDIO2 versus 0V9
             (++) VDDA versus 1V6
             (++) VDDA versus 1V8

   (#) Call HAL_PWREx_EnableMonitoring() and HAL_PWREx_DisableMonitoring() to
       enable / disable the VBAT and temperature monitoring.

   (#) Call HAL_PWREx_EnableUCPDStandbyMode() and
       HAL_PWREx_DisableUCPDStandbyMode() to enable / disable the UCPD
       configuration memorization in Standby mode.

   (#) Call HAL_PWREx_EnableUCPDDeadBattery() and
       HAL_PWREx_DisableUCPDDeadBattery() to enable / disable the dead battery
       behavior.

   (#) Call HAL_PWREx_PVD_PVM_IRQHandler() function to handle the PWR PVD and
       PVM interrupt request.

   (#) Call HAL_PWREx_EnableSRAM2ContentStandbyRetention() and
       HAL_PWREx_DisableSRAM2ContentStandbyRetention() to
       enable / disable the SRAM2 content retention in Stop 3 and Standby low
       power modes.

   (#) Call HAL_PWREx_EnableRAMsContentStopRetention() and
       HAL_PWREx_DisableRAMsContentStopRetention() to
       enable / disable the RAMs content retention in Stop mode (Stop 0/1/2/3).
       (+) Retained RAM can be one of the following RAMs :
             (++) SRAM1
             (++) SRAM2
             (++) SRAM3
             (++) SRAM4
             (++) ICACHE
             (++) DMA2DRAM
             (++) PKA32RAM
             (++) DCACHE1
             (++) FMAC
             (++) FDCAN
             (++) USB

   (#) Call HAL_PWREx_EnableRAMsContentRunRetention() and
       HAL_PWREx_DisableRAMsContentRunRetention() to
       enable / disable the RAMs content retention in Run mode.
       (+) Retained RAM can be one of the following RAMs :
             (++) SRAM1
             (++) SRAM2
             (++) SRAM3
             (++) SRAM4

   (#) Call HAL_PWREx_EnableFlashFastWakeUp() and
       HAL_PWREx_DisableFlashFastWakeUp() to enable / disable the flash memory
       fast wakeup from Stop mode (Stop 0/1).

   (#) Call HAL_PWREx_EnableSRAM4FastWakeUp() and
       HAL_PWREx_DisableSRAM4FastWakeUp() to enable / disable the SRAM4 memory
       fast wakeup from Stop mode (Stop 0/1/2).

   (#) Call HAL_PWREx_EnableBkupRAMRetention() and
       HAL_PWREx_DisableBkupRAMRetention() to enable / disable the Backup RAM
       content retention in Standby, Shutdown and VBAT modes.

   (#) Call HAL_PWREx_EnablePullUpPullDownConfig() and
       HAL_PWREx_DisablePullUpPullDownConfig() to I/O enable / disable pull-up
       and pull-down configuration.

   (#) Call HAL_PWREx_EnableGPIOPullUp() and HAL_PWREx_EnableGPIOPullDown() to
       apply repectively pull-up and pull-down to selected I/O.
       Call HAL_PWREx_DisableGPIOPullUp() and HAL_PWREx_DisableGPIOPullDown() to
       disable applied repectively pull-up and pull-down to selected I/O.

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/** @addtogroup STM32U5xx_HAL_Driver
  * @{
  */

/** @defgroup PWREx PWREx
  * @brief    PWR Extended HAL module driver
  * @{
  */

#if defined (HAL_PWR_MODULE_ENABLED)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup PWR_Extended_Private_Defines PWR Extended Private Defines
  * @{
  */
/*!< PORTI pins mask */
#define PWR_PORTI_AVAILABLE_PINS  (0xFFU)
/*!< Time out value of flags setting */
#define PWR_FLAG_SETTING_DELAY    (0x32U)

/** @defgroup PWR_PVM_Mode_Mask PWR PVM Mode Mask
  * @{
  */
#define PVM_RISING_EDGE  (0x01U)  /*!< Mask for rising edge set as PVM trigger                 */
#define PVM_FALLING_EDGE (0x02U)  /*!< Mask for falling edge set as PVM trigger                */
#define PVM_MODE_IT      (0x04U)  /*!< Mask for interruption yielded by PVM threshold crossing */
#define PVM_MODE_EVT     (0x08U)  /*!< Mask for event yielded by PVM threshold crossing        */
/**
  * @}
  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup PWREx_Exported_Functions PWR Extended Exported Functions
  * @{
  */

/** @defgroup PWREx_Exported_Functions_Group1 Power Supply Control Functions
  * @brief    Power supply control functions
  *
@verbatim
 ===============================================================================
                  ##### Power supply control functions #####
 ===============================================================================
    [..]
      This section provides functions allowing to control power supply.

    [..]
      (+) The STM32U5 Series devices embed two regulators : one LDO (linear
          voltage regulator) and one SMPS (step down converter) in parallel to
          provide the VCORE supply for digital peripherals, SRAM1, SRAM2, SRAM3
          and SRAM4 and embedded Flash memory.

      (+) The SMPS allows the power consumption to be reduced but some
          peripherals can be perturbed by the noise generated by the SMPS,
          requiring the application to switch to LDO when running this
          peripheral in order to reach the best performances.

      (+) The LDO and the SMPS regulators have two modes: Main regulator mode
          (used when performance is needed), and Low-power regulator mode. LDO
          or SMPS can be used in all voltage scaling ranges, and in all Stop
          modes.

      (+) After reset, the regulator is the LDO, in Range 4. Switching to SMPS
          provides lower consumption in particular at high VDD voltage. It is
          possible to switch from LDO to SMPS, or from SMPS to LDO on the fly in
          any range, by configuring the REGSEL bit. It is recommended to switch
          first to SMPS before changing the voltage range.

      (+) When exiting the Stop or Standby modes, the regulator is the same than
          when entering low power modes. The voltage range is the Range 4.

      (+) Both regulators can provide four different voltages (voltage scaling)
          and can operate in Stop modes.
          Voltage scaling ranges can be one of the following values :
             (++) voltage output scale 1 : 1V2
                  => Used when system clock frequency is up to 160 MHz
             (++) voltage output scale 2 : 1V1
                  => Used when system clock frequency is up to 100 MHz
             (++) voltage output scale 3 : 1V0
                  => Used when system clock frequency is up to 50 MHz
             (++) voltage output scale 4 : 0V9
                  => Used when system clock frequency is up to 24 MHz

@endverbatim
  * @{
  */

/**
  * @brief  Configure the main internal regulator output voltage to achieve
  *         a tradeoff between performance and power consumption.
  * @param  VoltageScaling : Specifies the regulator output voltage scale.
  *                          This parameter can be one of the following values :
  *                          @arg @ref PWR_REGULATOR_VOLTAGE_SCALE1 : Regulator voltage output scale 1.
  *                                                                   Provides a typical output voltage at 1.2 V.
  *                                                                   Used when system clock frequency is up to 160 MHz.
  *                          @arg @ref PWR_REGULATOR_VOLTAGE_SCALE2 : Regulator voltage output scale 2.
  *                                                                   Provides a typical output voltage at 1.1 V.
  *                                                                   Used when system clock frequency is up to 100 MHz.
  *                          @arg @ref PWR_REGULATOR_VOLTAGE_SCALE3 : Regulator voltage output scale 3.
  *                                                                   Provides a typical output voltage at 1.0 V.
  *                                                                   Used when system clock frequency is up to 50 MHz.
  *                          @arg @ref PWR_REGULATOR_VOLTAGE_SCALE4 : Regulator voltage output scale 4.
  *                                                                   Provides a typical output voltage at 0.9 V.
  *                                                                   Used when system clock frequency is up to 24 MHz.
  * @note  Before moving to voltage scaling 2, it is mandatory to ensure that
  *        the system frequency is between 50 MHz and 100 MHz.
  * @note  Before moving to voltage scaling 3, it is mandatory to ensure that
  *        the system frequency is between 24 MHz and 50 MHz.
  * @note  Before moving to voltage scaling 4, it is mandatory to ensure that
  *        the system frequency is below 24 MHz.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling)
{
  uint32_t timeout;
  uint32_t vos_old;

  /* Check the parameter */
  assert_param(IS_PWR_VOLTAGE_SCALING_RANGE(VoltageScaling));

  /* Get the current voltage scale applied */
  vos_old = READ_BIT(PWR->SVMSR, PWR_SVMSR_ACTVOS);

  /* No change, nothing to do */
  if (vos_old == VoltageScaling)
  {
    return HAL_OK;
  }

  /* Check voltage scaling level */
  /*
   *  The Embedded power distribution (EPOD) must be enabled before switching to
   *  voltage scale 1 / 2 from voltage scale lower.
   */
  if (VoltageScaling > PWR_REGULATOR_VOLTAGE_SCALE3)
  {
    MODIFY_REG(PWR->VOSR, (PWR_VOSR_VOS | PWR_VOSR_BOOSTEN), (VoltageScaling | PWR_VOSR_BOOSTEN));
  }
  else
  {
    MODIFY_REG(PWR->VOSR, (PWR_VOSR_VOS | PWR_VOSR_BOOSTEN), VoltageScaling);
  }

  /* Wait until VOSRDY is rised */
  timeout = ((PWR_FLAG_SETTING_DELAY * (SystemCoreClock / 1000U)) / 1000U) + 1U;
  while (HAL_IS_BIT_CLR(PWR->VOSR, PWR_VOSR_VOSRDY) && (timeout != 0U))
  {
    timeout--;
  }

  /* Check time out */
  if (timeout != 0U)
  {
    /* Wait until ACTVOSRDY is rised */
    timeout = ((PWR_FLAG_SETTING_DELAY * (SystemCoreClock / 1000U)) / 1000U) + 1U;
    while ((HAL_IS_BIT_CLR(PWR->SVMSR, PWR_SVMSR_ACTVOSRDY)) && (timeout != 0U))
    {
      timeout--;
    }
  }

  /* Check time out */
  if (timeout == 0U)
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

/**
  * @brief  Return Voltage Scaling Range.
  * @retval Applied voltage scaling value.
  */
uint32_t HAL_PWREx_GetVoltageRange(void)
{
  return (PWR->SVMSR & PWR_SVMSR_ACTVOS);
}

/**
  * @brief  Configure the system Power Supply.
  * @param  SupplySource : Specifies the Power Supply source to set after a
  *                        system startup.
  *                        This parameter can be one of the following values :
  *                        @arg PWR_LDO_SUPPLY  : The LDO regulator supplies the Vcore Power Domains.
  *                        @arg PWR_SMPS_SUPPLY : The SMPS regulator supplies the Vcore Power Domains.
  * @retval HAL Status.
  */
HAL_StatusTypeDef HAL_PWREx_ConfigSupply(uint32_t SupplySource)
{
  uint32_t timeout;

  /* Check the parameter */
  assert_param(IS_PWR_SUPPLY(SupplySource));

  /* Set maximum time out */
  timeout = ((PWR_FLAG_SETTING_DELAY * (SystemCoreClock / 1000U)) / 1000U) + 1U;

  /* Configure the LDO as system regulator supply */
  if (SupplySource == PWR_LDO_SUPPLY)
  {
    /* Set the power supply configuration */
    CLEAR_BIT(PWR->CR3, PWR_CR3_REGSEL);

    /* Wait until system switch on new regulator */
    while (HAL_IS_BIT_SET(PWR->SVMSR, PWR_SVMSR_REGS) && (timeout != 0U))
    {
      timeout--;
    }
  }
  /* Configure the SMPS as system regulator supply */
  else
  {
    /* Set the power supply configuration */
    SET_BIT(PWR->CR3, PWR_CR3_REGSEL);

    /* Wait until system switch on new regulator */
    while (HAL_IS_BIT_CLR(PWR->SVMSR, PWR_SVMSR_REGS) && (timeout != 0U))
    {
      timeout--;
    }
  }

  /* Check time out */
  if (timeout == 0U)
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

/**
  * @brief  Get the power supply configuration.
  * @retval The supply configured.
  */
uint32_t  HAL_PWREx_GetSupplyConfig(void)
{
  return (PWR->SVMSR & PWR_SVMSR_REGS);
}

/**
  * @brief  Enable fast soft start for the current regulator.
  * @retval None.
  */
void HAL_PWREx_EnableFastSoftStart(void)
{
  SET_BIT(PWR->CR3, PWR_CR3_FSTEN);
}

/**
  * @brief  Disable fast soft start for the current regulator.
  * @retval None.
  */
void HAL_PWREx_DisableFastSoftStart(void)
{
  CLEAR_BIT(PWR->CR3, PWR_CR3_FSTEN);
}
/**
  * @}
  */


/** @defgroup PWREx_Exported_Functions_Group2 Low Power Control Functions
  * @brief    Low power control functions
  *
@verbatim
 ===============================================================================
                     ##### Low power control functions #####
 ===============================================================================
    [..]
      This section provides functions allowing to control low power modes.

    *** Low Power modes configuration ***
    =====================================
    [..]
      This section presents 4 principles low power modes :
      (+) Stop 1 mode   : Cortex-M33 is stopped, clocks are stopped and the
                          regulator is in low power mode. Several peripheral can
                          operate in this mode.

      (+) Stop 2 mode   : Cortex-M33 is stopped, clocks are stopped and the
                          regulator is in low power mode. Only autonomous
                          peripherals can operate in this mode.

      (+) Stop 3 mode   : Cortex-M33 is stopped, clocks are stopped and the
                          regulator is in low power mode. No peripheral can
                          operate in this mode. Only RAMs content is preserved.

      (+) Shutdown mode : All PWR domains enter Shutdown mode and the VCORE
                          supply regulator is powered off. The SRAMs and
                          register contents are lost except for registers in the
                          Backup domain.

   *** Stop 1 mode ***
   ===================
    [..]
      The Stop 1 mode is based on the Cortex-M33 Deepsleep mode combined with
      the peripheral clock gating. The voltage regulator is configured in low
      power mode. In Stop 1 mode, all clocks in the VCORE domain are stopped.
      The PLL, MSIS, MSIK, HSI16 and HSE oscillators are disabled.
      Some peripherals with the LPBAM capability can switch on HSI16 or MSIS or
      MSIK for transferring data. All SRAMs and register contents are preserved,
      but the SRAMs can be totally or partially switched off to further reduced
      consumption.
      The BOR is always available in Stop 1 mode.

      (+) Entry:
          The Stop 1 mode is entered by using the HAL_PWREx_EnterSTOP1Mode()
          function.

          (++) PWR_STOPENTRY_WFI: enter Stop 1 mode with WFI instruction.
          (++) PWR_STOPENTRY_WFE: enter Stop 1 mode with WFE instruction.

      (+) Exit:
          Any EXTI line configured in interrupt mode (the corresponding EXTI
          interrupt vector must be enabled in the NVIC). The interrupt source
          can be external interrupts or peripherals with wakeup capability.
          Any peripheral interrupt occurring when the AHB/APB clocks are present
          due to an autonomous peripheral clock request (the peripheral vector
          must be enabled in the NVIC)
          Any EXTI line configured in event mode.

   *** Stop 2 mode ***
   ===================
    [..]
      The Stop 2 mode is based on the Cortex-M33 Deepsleep mode combined with
      peripheral clock gating. In Stop 2 mode, all clocks in the VCORE domain
      are stopped.
      The PLL, MSIS, MSIK, HSI16 and HSE oscillators are disabled.
      Some peripherals with the LPBAM capability can switch on HSI16 or MSIS or
      MSIK for transferring data. All SRAMs and register contents are preserved,
      but the SRAMs can be totally or partially switched off to further reduce
      consumption.
      The BOR is always available in Stop 2 mode.

      (+) Entry:
          The Stop 2 mode is entered by using the HAL_PWREx_EnterSTOP2Mode()
          function.

          (++) PWR_STOPENTRY_WFI: enter Stop 2 mode with WFI instruction.
          (++) PWR_STOPENTRY_WFE: enter Stop 2 mode with WFE instruction.

      (+) Exit:
          Any EXTI line configured in interrupt mode (the corresponding EXTI
          interrupt vector must be enabled in the NVIC). The interrupt source
          can be external interrupts or peripherals with wakeup capability.
          Any peripheral interrupt occurring when the AHB/APB clocks are present
          due to an autonomous peripheral clock request (the peripheral vector
          must be enabled in the NVIC)
          Any EXTI line configured in event mode.

   *** Stop 3 mode ***
   ===================
    [..]
      The Stop 3 mode is based on the Cortex