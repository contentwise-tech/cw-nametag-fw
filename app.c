/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "drivers/system/system.h"
#include "drivers/display/display.h"
#include "drivers/imu/imu.h"
#include "stdio.h"

#include "gatt_db.h"
#include "sl_bluetooth.h"

#include "drivers/display/arial_13x13.h"
#include "drivers/display/steelfish-10.h"
#include "drivers/display/fixedsys10x19.h"
#include "drivers/display/intThinPix17x27.h"

#define QR_CODE_BUFFER_SIZE 0xFF
#define SCORE_SIZE  20

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static uint8_t firstName[16] = {0};
static uint8_t lastName[16] = {0};

static uint8_t connection = 0;

static uint8_t dataUpdated = 0;
static size_t data_recv_len;

static uint8_t score[SCORE_SIZE] = {0};

static fontHeaderStr arial13x13 = {.firstChar = arialData13x13[2], .lastChar = arialData13x13[4], .fontSize = arialData13x13[6], .fontData = arialData13x13};
static fontHeaderStr steelfish7x14 = {.firstChar = Steelfish_Rg7x14[2], .lastChar = Steelfish_Rg7x14[4], .fontSize = Steelfish_Rg7x14[6], .fontData = Steelfish_Rg7x14};
static fontHeaderStr inkyThinPixels17x27 = {.firstChar = Inky_Thin_Pixels17x27[2], .lastChar = Inky_Thin_Pixels17x27[4], .fontSize = Inky_Thin_Pixels17x27[6], .fontData = Inky_Thin_Pixels17x27};
static fontHeaderStr fixedsys10x19 = {.firstChar = Fixedsys10x19[2], .lastChar = Fixedsys10x19[4], .fontSize = Fixedsys10x19[6], .fontData = Fixedsys10x19};

static uint8_t qrCodeBuffer[QR_CODE_BUFFER_SIZE];

static void app_gatt_update(void);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
    /////////////////////////////////////////////////////////////////////////////
    // Put your additional application init code here!                         //
    // This is called once during start-up.                                    //
    /////////////////////////////////////////////////////////////////////////////

    uint8_t strData[50];
    //initSystem();
    initDisplay();
    initDispayDriver(20);
    initIMU();

    clearDisplayRam();
    sendQrCode();

    uint8_t strLen = sprintf((char*)strData, "Battlefield");
    displaySendString(7, 104-inkyThinPixels17x27.fontSize - 16 - fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    strLen = sprintf((char*)strData, "of");
    displaySendString(50, 104-inkyThinPixels17x27.fontSize - 16 - 2*fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    strLen = sprintf((char*)strData, "Things");
    displaySendString(30, 104-inkyThinPixels17x27.fontSize - 16 - 3*fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    flushImageRam();
}

void sendScreen(void){
    uint8_t strData[50];

    clearDisplayRam();
    sendQrCode();

    uint8_t strLen = sprintf((char*)strData, "%s %s", firstName, lastName);
    displaySendString(0, 104-inkyThinPixels17x27.fontSize - 0, strData, strLen, &inkyThinPixels17x27);

    strLen = sprintf((char*)strData, "%s", score);
    displaySendString(7, 104-inkyThinPixels17x27.fontSize - 1 - fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    strLen = sprintf((char*)strData, "Battlefield");
    displaySendString(7, 104-inkyThinPixels17x27.fontSize - 16 - fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    strLen = sprintf((char*)strData, "of");
    displaySendString(50, 104-inkyThinPixels17x27.fontSize - 16 - 2*fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    strLen = sprintf((char*)strData, "Things");
    displaySendString(30, 104-inkyThinPixels17x27.fontSize - 16 - 3*fixedsys10x19.fontSize, strData, strLen, &fixedsys10x19);

    flushImageRam();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
    /////////////////////////////////////////////////////////////////////////////
    // Put your additional application code here!                              //
    // This is called infinitely.                                              //
    // Do not call blocking functions from here!                               //
    /////////////////////////////////////////////////////////////////////////////

    if (connection == false){
        if (dataUpdated){
            dataUpdated = false;
            sendScreen();
        }
    } else {
        float t = inuReadTemperature();

        uint16_t T = (t + 128) * 100;

        sl_status_t sc;
        sc = sl_bt_gatt_server_send_notification(connection, gattdb_temperature_measurement, sizeof(uint16_t), (uint8_t*)&T);
    }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      connection = evt->data.evt_connection_opened.connection;

      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);

      connection = false;
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.
    case sl_bt_evt_gatt_server_attribute_value_id:
      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_first_name){
          sc = sl_bt_gatt_server_read_attribute_value(gattdb_first_name, 0, sizeof(firstName), &data_recv_len, firstName);
          if (sc == SL_STATUS_OK) {
              if (data_recv_len != 0){
                  firstName[data_recv_len] = 0;
                  dataUpdated = true;
              }
          }
      }

      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_last_name){
          sc = sl_bt_gatt_server_read_attribute_value(gattdb_last_name, 0, sizeof(lastName), &data_recv_len, lastName);
          if (sc == SL_STATUS_OK) {
              if (data_recv_len != 0){
                  lastName[data_recv_len] = 0;
                  dataUpdated = true;
              }
          }
      }

      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_qr_code){
          sc = sl_bt_gatt_server_read_attribute_value(gattdb_qr_code, 0, sizeof(qrCodeBuffer), &data_recv_len, qrCodeBuffer);
          if (sc == SL_STATUS_OK) {
              if (data_recv_len != 0){

              }
          }
      }

      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_score){
          uint8_t ansStr[SCORE_SIZE];
          sc = sl_bt_gatt_server_read_attribute_value(gattdb_score, 0, sizeof(ansStr), &data_recv_len, ansStr);
          if (sc == SL_STATUS_OK) {
              if (data_recv_len > 3){
                  if ((ansStr[0] == '#') && (ansStr[1] == '/') && (ansStr[2] == '@')){
                      memcpy(score, &ansStr[3], data_recv_len - 3);
                      score[data_recv_len - 3] = 0;
                      dataUpdated = true;
                  }
              }
          }
      }

      break;

      case sl_bt_evt_system_external_signal_id:
          app_gatt_update();
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

static void app_gatt_update(void){
    sl_status_t sc;

    sc = sl_bt_gatt_server_write_attribute_value(gattdb_first_name, 0, strlen(firstName), firstName);
    sc = sl_bt_gatt_server_write_attribute_value(gattdb_last_name, 0, strlen(lastName), lastName);

}
