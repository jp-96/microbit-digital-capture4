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

#include "MicroBitIndoorBikeStepSensor.h"

void calcIndoorBikeData(uint32_t crankIntervalTime, uint32_t* cadence2, uint32_t* speed100)
{
    
    if (crankIntervalTime==0)
    {
        *cadence2 = 0;
        *speed100 = 0;
    }
    else
    {
        *cadence2 = (uint32_t)( (uint64_t)K_STEP_CADENCE / crankIntervalTime );
        *speed100 = (uint32_t)( (uint64_t)K_STEP_SPEED   / crankIntervalTime );
    }
}

MicroBitIndoorBikeStepSensor::MicroBitIndoorBikeStepSensor(MicroBit &_uBit, MicrobitIndoorBikeStepSensorPin pin, uint16_t id)
    : uBit(_uBit)
{
    this->id = id;
    this->lastIntervalTime=0;
    this->lastCadence2=0;
    this->lastSpeed100=0;
    this->updateSampleTimestamp=0;
    
    if (EventModel::defaultEventBus)
        EventModel::defaultEventBus->listen(MICROBIT_INDOOR_BIKE_STEP_SENSOR_EVENT_IDs[pin], MICROBIT_PIN_EVT_FALL
            , this, &MicroBitIndoorBikeStepSensor::onStepSensor);
    switch (pin)
    {
    case EDGE_P0:
        uBit.io.P0.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
        break;
    case EDGE_P1:
        uBit.io.P1.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
        break;
    default:    // EDGE_P2
        uBit.io.P2.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
        break;
    }
    
}

void MicroBitIndoorBikeStepSensor::idleTick()
{

    if(!(status & MICROBIT_INDOOR_BIKE_STEP_SENSOR_ADDED_TO_IDLE))
    {
        // If we're running under a fiber scheduer, register ourselves for a periodic callback to keep our data up to date.
        // Otherwise, we do just do this on demand, when polled through our read() interface.
        fiber_add_idle_component(this);
        status |= MICROBIT_INDOOR_BIKE_STEP_SENSOR_ADDED_TO_IDLE;
    }
    
    this->update();
}

uint32_t MicroBitIndoorBikeStepSensor::getIntervalTime(void)
{
    return this->lastIntervalTime;
}

uint32_t MicroBitIndoorBikeStepSensor::getCadence2(void)
{
    return this->lastCadence2;
}

uint32_t MicroBitIndoorBikeStepSensor::getSpeed100(void)
{
    return this->lastSpeed100;
}

void MicroBitIndoorBikeStepSensor::update(void)
{
    uint64_t currentTime = system_timer_current_time_us();

    if (currentTime >= this->updateSampleTimestamp)
    {
        this->updateSampleTimestamp = currentTime + this->SENSOR_UPDATE_PERIOD_US;
        
        if ((this->intervalList.size()>0) && ((currentTime - this->intervalList.back())>=this->MAX_STEPS_INTERVAL_TIME_US))
        {
            while (this->intervalList.size()>0)
            {
                this->intervalList.pop();
            }
        }
        
        if (this->intervalList.size() < 2)
        {
            this->lastIntervalTime = 0;
        }
        else
        {
            uint64_t intervalNum = this->intervalList.size() - 1;
            uint64_t periodTime = this->intervalList.back() - this->intervalList.front();
            this->lastIntervalTime = periodTime / intervalNum;
        }
        
        calcIndoorBikeData(this->lastIntervalTime, &this->lastCadence2, &this->lastSpeed100);
        
        MicroBitEvent e(id, MICROBIT_INDOOR_BIKE_STEP_SENSOR_EVT_DATA_UPDATE);
    }
}

void MicroBitIndoorBikeStepSensor::onStepSensor(MicroBitEvent e) 
{
    uint64_t currentTime = e.timestamp;
    
    if (this->intervalList.size()==0)
    {
        // 初回から、回転数とスピードを算出する。
        this->intervalList.push(currentTime-MAX_STEPS_INTERVAL_TIME_US);
    }
    this->intervalList.push(currentTime);
    if ((uint32_t)this->intervalList.size()>(this->INTERVAL_LIST_SIZE))
    {
        this->intervalList.pop();
    }
}
