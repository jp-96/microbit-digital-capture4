/*
MIT License

Copyright (c) 2021 jp-rad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MICROBIT_CUSTOM_H
#define MICROBIT_CUSTOM_H

// The base of custom Event Bus ID.
static const uint16_t MICROBIT_CUSTOM_ID_BASE = 32768;

/*
 * MicroBitIndoorBikeStepSensor
 */

// Event Bus ID for IndoorBike step sensor
#ifndef MICROBIT_INDOORBIKE_STEP_SENSOR_ID
#define MICROBIT_INDOORBIKE_STEP_SENSOR_ID (MICROBIT_CUSTOM_ID_BASE+1)
#endif /* #ifndef MICROBIT_INDOORBIKE_STEP_SENSOR_ID */

// Event value
#define MICROBIT_INDOOR_BIKE_STEP_SENSOR_EVT_DATA_UPDATE 0b0000000000000001

/*
 * MicroBitIndoorBikeStepService
 */

#ifndef BLE_DEVICE_LOCAL_NAME_CHENGE
#define BLE_DEVICE_LOCAL_NAME_CHENGE 1
#endif /* #ifndef BLE_DEVICE_LOCAL_NAME_CHENGE */

#ifndef BLE_DEVICE_LOCAL_NAME
#define BLE_DEVICE_LOCAL_NAME "STEP:BIT"
#endif /* #ifndef BLE_DEVICE_LOCAL_NAME */

// Event Bus ID for IndoorBike step sensor
#ifndef MICROBIT_INDOORBIKE_STEP_SERVICE_ID
#define MICROBIT_INDOORBIKE_STEP_SERVICE_ID (MICROBIT_CUSTOM_ID_BASE+2)
#endif /* #ifndef MICROBIT_INDOORBIKE_STEP_SERVICE_ID */

// FTMP Event Val
// # 0x00 M Request Control
#define FTMP_EVENT_VAL_OP_CODE_CPPR_00_REQUEST_CONTROL                      0b0000000000000001
// # 0x01 M Reset
#define FTMP_EVENT_VAL_OP_CODE_CPPR_01_RESET                                0b0000000000000010
// # 0x07 M Start or Resume
#define FTMP_EVENT_VAL_OP_CODE_CPPR_07_START_RESUME                         0b0000000000000100
// # 0x08 M Stop or Pause [UINT8, 0x01-STOP, 0x02-PAUSE]
#define FTMP_EVENT_VAL_OP_CODE_CPPR_08_STOP_PAUSE                           0b0000000000001000

#endif /* #ifndef MICROBIT_CUSTOM_H */