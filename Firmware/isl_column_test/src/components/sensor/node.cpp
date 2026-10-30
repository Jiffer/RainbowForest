/* 
// This file is subject to the terms and conditions defined in
// file 'LICENSE.md', which is part of this source code package.
*/

#include "sensor.h"

SensorNode::SensorNode( size_t index, CRGB* control, SFE_ISL29125* sensor ) :
_index(index),
_control(control),
_sensor(sensor)
{
    _reading.r = 0;
    _reading.g = 0;
    _reading.b = 0;
}

SensorStatus_e SensorNode::_report(SensorStatus_e s){
    status = s;
    return status;
}

SensorStatus_e SensorNode::power(bool on){
    if(!_control){ return SENSOR_ERR_CONFIG; }
    _control[(2*_index) + 0] = (on) ? CRGB(0, 0, 0) : CRGB(255, 0, 0);
    FastLED.show();
    if(on){
        delay(2);   // time for sensor to wake up
    }
    return _report(SENSOR_OK);
}

void idle_for(uint32_t ms, void (*idle_fn)(void)){
    uint32_t now = millis();
    while((millis() - now) < ms){
        if(idle_fn){
            (*idle_fn)();
        }
    }
}

isl_readint_t SensorNode::read(void (*idle_fn)(void)){
    if((!_control) || (!_sensor)){
        _report(SENSOR_ERR_CONFIG);
        return _reading;
    }
    _lockLed(true);                             // lock led to prevent interruption of the illumination color
    //                      G  R  B
    _setIllumination(CRGB(255, 255, 255));      // illuminate the target (adjust white balance)
    power(true);                                // turn on sensor power
    
    _sensor->init();

    // sensor continuously runs ADC at ~ 10 hz so to be sure wait 0.2 seconds before reading
    idle_for(200, idle_fn);
    
    idle_for(300, idle_fn); // delay to combat voltage sag from turning on all the leds...
                            // i've experimentally determined that while there is no LED brightness that completely 
                            // eliminates noise in detected color there is a minimum total delay between turning on
                            // the leds and taking a sample that gets darn close. Its approx 500 ms total (including
                            // time dedicated to letting the sensor read)

                            // the final product may as well turn on all the leds, wait half a second, and then sample
                            // all of the color sensors rapidly. 

    // read the sensor
    _reading.r = _sensor->readRed();
    _reading.g = _sensor->readGreen();
    _reading.b = _sensor->readBlue();

    power(false);           // turn off sensor power
    _lockLed(false);        // unlock the led
    _setLed(_led);          // return led to desired color

    _report(SENSOR_OK);
    return _reading;
}

SensorStatus_e SensorNode::_setIllumination(const CRGB& c){
    if(!_control){ return _report(SENSOR_ERR_CONFIG); }
    _control[(2*_index) + 1] = c;
    FastLED.show();
    return _report(SENSOR_OK);
}

SensorStatus_e SensorNode::_setLed(const CRGB& c){
    if(!_control){ return _report(SENSOR_ERR_CONFIG); }
    _led = c;
    if(!_locked){
        _control[(2*_index) + 1] = c;
    }
    return _report(SENSOR_OK);
}

SensorStatus_e SensorNode::_lockLed(bool lock){
    _locked = lock;
    return _report(SENSOR_OK);
}
