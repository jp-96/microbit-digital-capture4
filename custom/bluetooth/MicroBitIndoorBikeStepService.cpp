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
#include "MicroBitIndoorBikeStepService.h"
#include "struct.h"
#include "inttypes.h" // for Debug

MicroBitIndoorBikeStepService::MicroBitIndoorBikeStepService(MicroBit &_uBit, MicroBitIndoorBikeStepSensor &_indoorBike, uint16_t id)
    : uBit(_uBit), indoorBike(_indoorBike)
{
    this->id = id;
    this->stopOrPause=0;

    // BLE Appearance and LOCAL_NAME
    uBit.ble->gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_CYCLING);
    if (BLE_DEVICE_LOCAL_NAME_CHENGE)
    {
        uBit.ble->gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME
            , (const uint8_t *)BLE_DEVICE_LOCAL_NAME, sizeof(BLE_DEVICE_LOCAL_NAME)-1);
    }
    
    // FTMS - Service Advertising Data
    const uint8_t FTMS_UUID[sizeof(UUID::ShortUUIDBytes_t)] = {0x26, 0x18};
    uBit.ble->accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, FTMS_UUID, sizeof(FTMS_UUID));
    uint8_t serviceData[2+1+2];
    struct_pack(serviceData, "<HBH", 0x1826, 0x01, 1<<5);
    uBit.ble->accumulateAdvertisingPayload(GapAdvertisingData::SERVICE_DATA, serviceData, sizeof(serviceData));

    // Caractieristic
    GattCharacteristic  indoorBikeDataCharacteristic(
        UUID(0x2AD2)
        , (uint8_t *)&indoorBikeDataCharacteristicBuffer, 0, indoorBikeDataCharacteristicBufferSize
        , GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    GattCharacteristic  fitnessMachineControlPointCharacteristic(
        UUID(0x2AD9)
        , (uint8_t *)&fitnessMachineControlPointCharacteristicBuffer, 0, fitnessMachineControlPointCharacteristicBufferSize
        , GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE|GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE
    );
    GattCharacteristic  fitnessMachineFeatureCharacteristic(
        UUID(0x2ACC)
        , (uint8_t *)&fitnessMachineFeatureCharacteristicBuffer, 0, fitnessMachineFeatureCharacteristicBufferSize
        , GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
    );
    GattCharacteristic  fitnessMachineStatusCharacteristic(
        UUID(0x2ADA)
        , (uint8_t *)&fitnessMachineStatusCharacteristicBuffer, 0, fitnessMachineStatusCharacteristicBufferSize
        , GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    GattCharacteristic  fitnessTrainingStatusCharacteristic(
        UUID(0x2AD3)
        , (uint8_t *)&fitnessTrainingStatusCharacteristicBuffer, 0, fitnessTrainingStatusCharacteristicBufferSize
        , GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    
    // Set default security requirements
    indoorBikeDataCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);
    fitnessMachineControlPointCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);
    fitnessMachineFeatureCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);
    fitnessMachineStatusCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);
    fitnessTrainingStatusCharacteristic.requireSecurity(SecurityManager::MICROBIT_BLE_SECURITY_LEVEL);

    // Service
    GattCharacteristic *characteristics[] = {
        &indoorBikeDataCharacteristic,
        &fitnessMachineControlPointCharacteristic,
        &fitnessMachineFeatureCharacteristic,
        &fitnessMachineStatusCharacteristic,
        &fitnessTrainingStatusCharacteristic,
    };
    GattService service(
        UUID(0x1826), characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *)
    );
    uBit.ble->addService(service);
    
    // Characteristic Handle
    indoorBikeDataCharacteristicHandle = indoorBikeDataCharacteristic.getValueHandle();
    fitnessMachineControlPointCharacteristicHandle = fitnessMachineControlPointCharacteristic.getValueHandle();
    fitnessMachineFeatureCharacteristicHandle = fitnessMachineFeatureCharacteristic.getValueHandle();
    fitnessMachineStatusCharacteristicHandle = fitnessMachineStatusCharacteristic.getValueHandle();
    fitnessTrainingStatusCharacteristicHandle = fitnessTrainingStatusCharacteristic.getValueHandle();
    
    // GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
    uint8_t fitnessMachineFeatureBuff[fitnessMachineFeatureCharacteristicBufferSize];
    struct_pack(fitnessMachineFeatureBuff
        , "<II"
        , FTMP_FLAGS_FITNESS_MACINE_FEATURES_FIELD
        , FTMP_FLAGS_TARGET_SETTING_FEATURES_FIELD
    );
    uBit.ble->gattServer().write(fitnessMachineFeatureCharacteristicHandle
        ,(uint8_t *)&fitnessMachineFeatureBuff, fitnessMachineFeatureCharacteristicBufferSize);
    uint8_t fitnessTrainingStatusBuff[fitnessTrainingStatusCharacteristicBufferSize];
    struct_pack(fitnessTrainingStatusBuff
        , "<BB"
        , FTMP_FLAGS_TRAINING_STATUS_FIELD_00_STATUS_ONLY
        , FTMP_VAL_TRAINING_STATUS_01_IDEL
    );
    uBit.ble->gattServer().write(fitnessTrainingStatusCharacteristicHandle
        ,(uint8_t *)&fitnessTrainingStatusBuff, fitnessTrainingStatusCharacteristicBufferSize);
    
    // GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE - Fitness Machine Control Point Characteristic
    uBit.ble->onDataWritten(this, &MicroBitIndoorBikeStepService::onDataWritten);
    
    // Microbit Event listen
    if (EventModel::defaultEventBus)
    {
        EventModel::defaultEventBus->listen(this->indoorBike.getId(), MICROBIT_INDOOR_BIKE_STEP_SENSOR_EVT_DATA_UPDATE
            , this, &MicroBitIndoorBikeStepService::indoorBikeUpdate, MESSAGE_BUS_LISTENER_IMMEDIATE);
        EventModel::defaultEventBus->listen(this->id, MICROBIT_EVT_ANY
            , this, &MicroBitIndoorBikeStepService::onFitnessMachineControlPoint, MESSAGE_BUS_LISTENER_IMMEDIATE);
    }

}

void MicroBitIndoorBikeStepService::onDataWritten(const GattWriteCallbackParams *params)
{
    if (params->handle == fitnessMachineControlPointCharacteristicHandle && params->len >= 1)
    {
        uint8_t responseBuffer[3];
        responseBuffer[0] = FTMP_OP_CODE_CPPR_80_RESPONSE_CODE;
        uint8_t *opCode=&responseBuffer[1];
        opCode[0]=params->data[0];
        uint8_t *result=&responseBuffer[2];
        result[0] = FTMP_RESULT_CODE_CPPR_03_INVALID_PARAMETER;
        uint16_t eventValue = MICROBIT_EVT_ANY;
        switch (opCode[0])
        {
        case FTMP_OP_CODE_CPPR_00_REQUEST_CONTROL:
            eventValue=FTMP_EVENT_VAL_OP_CODE_CPPR_00_REQUEST_CONTROL;
            if (params->len == 1)
            {
                result[0] = FTMP_RESULT_CODE_CPPR_01_SUCCESS;
            }
            break;

        case FTMP_OP_CODE_CPPR_01_RESET:
            eventValue=FTMP_EVENT_VAL_OP_CODE_CPPR_01_RESET;
            if (params->len == 1)
            {
                result[0] = FTMP_RESULT_CODE_CPPR_01_SUCCESS;
            }
            break;

        case FTMP_OP_CODE_CPPR_07_START_RESUME:
            eventValue=FTMP_EVENT_VAL_OP_CODE_CPPR_07_START_RESUME;
            if (params->len == 1)
            {
                result[0] = FTMP_RESULT_CODE_CPPR_01_SUCCESS;
            }
            break;

        case FTMP_OP_CODE_CPPR_08_STOP_PAUSE:
            eventValue=FTMP_EVENT_VAL_OP_CODE_CPPR_08_STOP_PAUSE;
            if (params->len == 2)
            {
                this->stopOrPause = params->data[1];
                result[0] = FTMP_RESULT_CODE_CPPR_01_SUCCESS;
            }
            break;
            
        default:
            result[0] = FTMP_RESULT_CODE_CPPR_02_NOT_SUPORTED;
            break;

        }

        // Response - Fitness Machine Control Point
        uBit.ble->gattServer().write(fitnessMachineControlPointCharacteristicHandle
                , (const uint8_t *)&responseBuffer, sizeof(responseBuffer));
        
        // Fire MicroBit Event
        if (result[0]==FTMP_RESULT_CODE_CPPR_01_SUCCESS)
        {
            new MicroBitEvent(this->id, eventValue);
        }

        // Debug - USB Serial
        uBit.serial.printf("CP:%" PRIu32 ", eventValue[%d], opCode[0x%02X], result[0x%02X], data", (uint32_t)system_timer_current_time(), eventValue, opCode[0], result[0]);
        for (int i=0; i<params->len; i++)
        {
            uBit.serial.printf(", 0x%02X", params->data[i]);
        }
        uBit.serial.printf("\r\n");

    }
}

void MicroBitIndoorBikeStepService::indoorBikeUpdate(MicroBitEvent e)
{
    if (uBit.ble->getGapState().connected)
    {
        uint8_t buff[indoorBikeDataCharacteristicBufferSize];
        struct_pack(buff, "<HHH",
            FTMP_FLAGS_INDOOR_BIKE_DATA_CHAR,
            this->indoorBike.getSpeed100(),
            this->indoorBike.getCadence2()
        );
        uBit.ble->gattServer().notify(indoorBikeDataCharacteristicHandle
            , (uint8_t *)&buff, indoorBikeDataCharacteristicBufferSize);
    }
}

uint8_t MicroBitIndoorBikeStepService::getStopOrPause()
{
    return this->stopOrPause;
}

void MicroBitIndoorBikeStepService::sendTrainingStatusIdle(void)
{
    static const uint8_t buff[]={FTMP_FLAGS_TRAINING_STATUS_FIELD_00_STATUS_ONLY, FTMP_VAL_TRAINING_STATUS_01_IDEL};
    uBit.ble->gattServer().notify(this->fitnessTrainingStatusCharacteristicHandle
        , (uint8_t *)&buff, sizeof(buff));
}

void MicroBitIndoorBikeStepService::sendTrainingStatusManualMode(void)
{
    static const uint8_t buff[]={FTMP_FLAGS_TRAINING_STATUS_FIELD_00_STATUS_ONLY, FTMP_VAL_TRAINING_STATUS_0D_MANUAL_MODE};
    uBit.ble->gattServer().notify(this->fitnessTrainingStatusCharacteristicHandle
        , (uint8_t *)&buff, sizeof(buff));
}
    
void MicroBitIndoorBikeStepService::sendFitnessMachineStatusReset(void)
{
    static const uint8_t buff[]={FTMP_OP_CODE_FITNESS_MACHINE_STATUS_01_RESET};
    uBit.ble->gattServer().notify(this->fitnessMachineStatusCharacteristicHandle
        , (uint8_t *)&buff, sizeof(buff));
}

void MicroBitIndoorBikeStepService::onFitnessMachineControlPoint(MicroBitEvent e)
{
    switch (e.value)
    {
    case FTMP_EVENT_VAL_OP_CODE_CPPR_00_REQUEST_CONTROL:
        // # 0x00 M Request Control
        // #define FTMP_EVENT_VAL_OP_CODE_CPPR_00_REQUEST_CONTROL
        // (NOP)
        break;
    case FTMP_EVENT_VAL_OP_CODE_CPPR_01_RESET:
        // # 0x01 M Reset
        // #define FTMP_EVENT_VAL_OP_CODE_CPPR_01_RESET
        this->sendTrainingStatusManualMode();
        break;
    case FTMP_EVENT_VAL_OP_CODE_CPPR_07_START_RESUME:
        // # 0x07 M Start or Resume
        // #define FTMP_EVENT_VAL_OP_CODE_CPPR_07_START_RESUME
        this->sendTrainingStatusManualMode();
        break;
    case FTMP_EVENT_VAL_OP_CODE_CPPR_08_STOP_PAUSE:
        // # 0x08 M Stop or Pause [UINT8, 0x01-STOP, 0x02-PAUSE]
        // #define FTMP_EVENT_VAL_OP_CODE_CPPR_08_STOP_PAUSE
        this->sendTrainingStatusIdle();
        break;
    default:
        break;
    }
    
}
