/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_srv_bms Biopotential Measurement Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Biopotential Measurement Service module.
 *
 * @details This module implements the Biopotential Measurement Service with the Voltage characteristic.
 *          During initialization it adds the Biopotential Measurement Service and Voltage characteristic
 *          to the BLE stack dataBISe. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the Biopotential Measurement Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_bms_battery_level_update() function.
 *          If an event handler is supplied by the application, the Biopotential Measurement Service will
 *          generate Biopotential Measurement Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Biopotential Measurement Service module by calling
 *       ble_bms_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_BMS_H__
#define BLE_BMS_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
//#include "ads1291-2.h"

// Base UUID
#define BMS_UUID_BASE {0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39, 0x5F, 0x87, 0xFD, 0x35, 0x00, 0x00, 0x8B, 0x22};

// Service UUID
//#define BLE_UUID_BIOPOTENTIAL_MEASUREMENT_SERVICE	0x3260

	#define BLE_UUID_BIOPOTENTIAL_MEASUREMENT_SERVICE	0xEEF0
// Characteristic UUIDs
#define BLE_UUID_EEG_CH1_CHAR		0xEEF1
#define BLE_UUID_EEG_CH2_CHAR		0xEEF2
#define BLE_UUID_EEG_CH3_CHAR		0xEEF3
#define BLE_UUID_EEG_CH4_CHAR		0xEEF4

// Maximum number of body voltage measurement bytes buffered by the application
#define BLE_BMS_MAX_BUFFERED_MEASUREMENTS				8
//30


/**@brief Biopotential Measurement Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    uint16_t        							conn_handle;						/**< Event handler to be called for handling events in the Biopotential Measurement Service. */
    uint16_t											service_handle; 				/**< Handle of ble Service (as provided by the BLE stack). */
	
		ble_gatts_char_handles_t			bvm_handles;						/**< Handles related to the our body V measure characteristic. */
		uint32_t										 	bvm_buffer[BLE_BMS_MAX_BUFFERED_MEASUREMENTS];
		uint8_t											 	bvm_count;
		
		ble_gatts_char_handles_t			eeg_ch2_handles;
		uint32_t										 	eeg_ch2_buffer[BLE_BMS_MAX_BUFFERED_MEASUREMENTS];
		uint8_t											 	eeg_ch2_count;
	
		ble_gatts_char_handles_t			eeg_ch3_handles;
		uint32_t										 	eeg_ch3_buffer[BLE_BMS_MAX_BUFFERED_MEASUREMENTS];
		uint8_t											 	eeg_ch3_count;
	
		ble_gatts_char_handles_t			eeg_ch4_handles;
		uint32_t										 	eeg_ch4_buffer[BLE_BMS_MAX_BUFFERED_MEASUREMENTS];
		uint8_t											 	eeg_ch4_count;
	
} ble_bms_t;

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_ecg_service_init(ble_bms_t *p_bms);

/**@brief Biopotential Measurement Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the Biopotential Measurement Service.
 *
 * @param[in]   p_bms      Biopotential Measurement Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_bms_on_ble_evt(ble_bms_t * p_bms, ble_evt_t * p_ble_evt);

/**@brief Function for sending body voltage measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a voltage measurement.
 *          If notification has been enabled, the voltage measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_bms                    Biopotential Measurement Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 
uint32_t ble_bms_body_voltage_measurement_send(ble_bms_t * p_bms);
*/
/**@brief Function for adding a Body Voltage Measurement to the buffer.
 *
 * @details All buffered voltage measurements will be included in the next biopotential
 *          measurement message, up to the maximum number of measurements that will fit into the
 *          message. If the buffer is full, the oldest measurement in the buffer will be deleted.
 *
 * @param[in]   p_bms        Biopotential Measurement Service structure.
 * @param[in]   bvm_val 	   New voltage measurement (will be buffered until the next
 *                           connection interval).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 
uint32_t ble_bms_bvm_add(ble_bms_t * p_bms, int16_t bvm_val);
*/
/**@brief Function for checking if Body Voltage Measurement buffer is full.
 *
 * @param[in]   p_bms        Biopotential Measurement Service structure.
 *
 * @return      true if Body Voltage Measurement buffer is full, false otherwise.
 
bool ble_bms_bvm_buffer_is_full(ble_bms_t * p_bms);
*/
/**@brief function for updating/notifying BLE of new value.
*
*/
//void ble_bms_update (ble_bms_t *p_bms, int16_t *body_voltage);

void ble_bms_update_24 (ble_bms_t *p_bms, int32_t *eeg1, int32_t *eeg2, int32_t *eeg3, int32_t *eeg4);

uint32_t ble_bms_send_24_ch1 (ble_bms_t *p_bms);

uint32_t ble_bms_send_24_ch2 (ble_bms_t *p_bms);

uint32_t ble_bms_send_24_ch3 (ble_bms_t *p_bms);

uint32_t ble_bms_send_24_ch4 (ble_bms_t *p_bms);

#endif // BLE_BMS_H__
