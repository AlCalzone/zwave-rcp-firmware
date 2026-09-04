/// Z-Wave RCP application on the Trident IoT SDK
///
/// Startup, the application task and the glue the prebuilt ZPAL library
/// expects from an application.

#include <stdint.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <zpal_init.h>
#include <zpal_misc.h>
#include <zpal_power_manager.h>
#include <zpal_radio.h>
#include <zpal_watchdog.h>
#include <zpal_defs.h>
#include <Assert.h>

#include "common.h"
#include "serial_link.h"
#include "rcp_app.h"

// -----------------------------------------------------------------------------
// Firmware properties the bootloader and ZPAL read from the image
// -----------------------------------------------------------------------------

/// Placed in the .fw_properties section the linker script reserves right after
/// .text, where the bootloader looks for the application version.
const __attribute__((__used__, section(".fw_properties"))) app_version_info_t app_version_info = {
    .magic_word = MAGIC_WORD_STR,
    .app_version = {
        .app_version_major = FIRMWARE_VERSION_MAJOR,
        .app_version_minor = FIRMWARE_VERSION_MINOR,
        .app_version_patch = FIRMWARE_VERSION_PATCH,
    },
    // This firmware is not a Z-Wave certified product, so it carries no
    // manufacturer or product identity
    .manufacturer_id = 0,
    .product_type_id = 0,
    .product_id = 0,
};

// zpal_get_product_id() reads these. The RCP has no Z-Wave device class.
const uint16_t ZPAL_PRODUCT_ID_INSTALLER_ICON_TYPE = 0;
const uint8_t ZPAL_PRODUCT_ID_GENERIC_TYPE = 0;
const uint8_t ZPAL_PRODUCT_ID_SPECIFIC_TYPE = 0;
const uint8_t ZPAL_PRODUCT_ID_REQUESTED_SECURITY_KEYS = 0;

// -----------------------------------------------------------------------------
// Task
// -----------------------------------------------------------------------------

/// Stack in words. Frame handling copies a few frames of up to 256 bytes onto
/// the stack, and the ZPAL radio driver runs its callbacks on this task
#define RCP_TASK_STACK_WORDS (4 * 1024)
/// Same priority the Z-Wave stack task uses, radio events are time critical
#define RCP_TASK_PRIORITY (TASK_PRIORITY_MAX - 10)

static StackType_t rcp_task_stack[RCP_TASK_STACK_WORDS] __attribute__((aligned(8)));
static StaticTask_t rcp_task_buffer;
static TaskHandle_t rcp_task_handle = NULL;

void rcp_notify(uint32_t events)
{
  if (rcp_task_handle == NULL)
  {
    return;
  }
  if (zpal_in_isr())
  {
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(rcp_task_handle, events, eSetBits, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
  else
  {
    xTaskNotify(rcp_task_handle, events, eSetBits);
  }
}

static void rcp_task(void *unused)
{
  (void)unused;

  zpal_watchdog_init();
  zpal_enable_watchdog(true);

  // Keep the radio powered. Without a lock the power manager may sleep the
  // radio between events
  zpal_pm_handle_t radio_power_lock = zpal_pm_register(ZPAL_PM_TYPE_USE_RADIO);
  zpal_pm_stay_awake(radio_power_lock, 0);

  uart_zpal_init();
  radio_zpal_init();

  while (true)
  {
    uint32_t events = 0;
    xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);

    if (events & RCP_EVENT_RF_RX)
    {
      radio_zpal_handle_rx();
    }
    if (events & RCP_EVENT_RF_TX_DONE)
    {
      radio_zpal_handle_tx_done();
    }
    if (events & RCP_EVENT_BEAM_TIMER)
    {
      radio_zpal_handle_beam_timer();
    }
    if (events & RCP_EVENT_UART_RX)
    {
      uart_zpal_pump_rx();
    }

    // The serial link parser asks for another pass when the FIFO still holds
    // data after a frame was consumed
    while (uart_rx_done)
    {
      uart_rx_done = false;
      serial_link_process_rx();
    }
  }
}

/// Called by main() in the ZPAL library after the hardware is set up and
/// before the FreeRTOS scheduler starts
void zpal_system_startup(zpal_reset_reason_t reset_reason)
{
  (void)reset_reason;

  rcp_task_handle = xTaskCreateStatic(
      rcp_task,
      "RCP",
      RCP_TASK_STACK_WORDS,
      NULL,
      RCP_TASK_PRIORITY,
      rcp_task_stack,
      &rcp_task_buffer);
  ASSERT(rcp_task_handle != NULL);
}

// -----------------------------------------------------------------------------
// Hooks the ZPAL library requires from the application
// -----------------------------------------------------------------------------

/// The power manager reports mode changes here. The RCP never sleeps the
/// radio, so there is nothing to do.
void zpal_zw_pm_event_handler(zpal_pm_mode_t from, zpal_pm_mode_t to)
{
  (void)from;
  (void)to;
}

void enterPowerDown(uint32_t millis)
{
  (void)millis;
}

void exitPowerDown(uint32_t millis)
{
  (void)millis;
}

/// Assertion failure inside the ZPAL library or this application. Restart the
/// chip, so the host sees a dropped link instead of a silent hang.
void Assert(const char *file_name, int line_number)
{
  (void)file_name;
  (void)line_number;
  zpal_reboot_with_info(0, ZPAL_RESET_ASSERT_PTR);
  for (;;)
  {
  }
}

void Assert_SetCb(AssertCb_t cb)
{
  (void)cb;
}

const void *AssertPtr(const void *ptr, const char *message)
{
  (void)message;
  if (ptr == NULL)
  {
    Assert(__FILE__, __LINE__);
  }
  return ptr;
}

/// The ZPAL radio treats the requested transmit power on classic channels as
/// an offset from the configured maximum when this returns true. The host
/// hands over absolute dBm values, so disable that translation.
bool zpal_radio_classic_tx_power_is_enabled(void)
{
  return false;
}
