/* 
// This file is subject to the terms and conditions defined in
// file 'LICENSE.md', which is part of this source code package.
*/

#include "sensor.h"

SensorString::SensorString(size_t len, uint32_t data_pin, TwoWire* i2c ) : 
_len(len),
_data_pin(data_pin),
_sensor(i2c)
{
    _num_control_elements = 2*_len;
    _control = new CRGB[_num_control_elements];
    if(!_control){
        _report(SENSOR_ERR_NOMEM);
        return;
    }

    _nodes = new SensorNode*[_len];
    if(!_nodes){
        _report(SENSOR_ERR_NOMEM);
        return;
    }

    for(size_t idx = 0; idx < _len; idx++){
        SensorNode* node = new SensorNode(idx, _control, &_sensor);
        if(!node){
            _report(SENSOR_ERR_NOMEM);
            return;
        }
        *(_nodes + idx) = node;
    }
}

SensorString::~SensorString(){
    if(_control){ delete _control; }
    if(_nodes){
        for(size_t idx = 0; idx < _len; idx++){
            SensorNode* node = *(_nodes + idx);
            if(node){ delete node; }
        }
        delete _nodes;
    }
}

SensorStatus_e SensorString::_report(SensorStatus_e s){
    status = s;
    return status;
}


SensorStatus_e SensorString::begin( void ){
    if(!_control){ return _report(SENSOR_ERR_CONFIG); }
    if(!_nodes){ return _report(SENSOR_ERR_CONFIG); }

    LEDS.setBrightness(255);

    // make sure all sensors are off and not showing colors
    for(size_t idx = 0; idx < _len; idx++){
        SensorNode* node = (*this)[idx];
        if(status != SENSOR_OK){ continue; }
        node->power(false);
        *node = CRGB(0, 0, 0);
    }

    return _report(SENSOR_OK);
}
