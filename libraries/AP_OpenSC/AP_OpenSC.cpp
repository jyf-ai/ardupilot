/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   OpenSC library
*/


#define AP_SERIALMANAGER_OPEN_SC_BAUD        115200
#define AP_SERIALMANAGER_OPENSC_BUFSIZE_RX          64
#define AP_SERIALMANAGER_OPENSC_BUFSIZE_TX          64

#include "AP_OpenSC.h"

extern const AP_HAL::HAL& hal;

AP_OpenSC::AP_OpenSC(void)
{
    _port = NULL;
    _step = 0;
}


void AP_OpenSC::init(const AP_SerialManager& serial_manager)
{
        // check for SC_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_SC, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_OPEN_SC_BAUD, AP_SERIALMANAGER_OPENSC_BUFSIZE_RX, AP_SERIALMANAGER_OPENSC_BUFSIZE_TX);    
    }
}

bool AP_OpenSC::update()
{
    if(_port == NULL)
    return false;

    int16_t numc = _port->available();
    uint8_t data;
    // if no data available, return 0
    if (numc == 0)
    {
        data = 0;
        cx = 0;
        cy = 0;
    }
    
    uint8_t receive_buf[numc];
    // uint8_t *receive_buf = new uint8_t[numc];
    uint8_t *ptr_cx = (uint8_t*)& cx; // 指向cx的指针
    uint8_t *ptr_cy = (uint8_t*)& cy; // 指向cy的指针

    for (int16_t i = 0; i < numc; i++) {
        data = _port->read();
        receive_buf[i] = data;

        switch (i){
            case 0:break;
            case 1:break;
            case 2:break;
            case 3:               
                    for (uint8_t t = 0; t < 4; t++) {  
                        *(ptr_cx + t) = receive_buf[t]; 
                    }               
                break;
            case 4:break;
            case 5:break;
            case 6:break;
            case 7:
                    for (uint8_t t = 4; t < 8; t++) {  
                        *(ptr_cy + t - 4) = receive_buf[t]; 
                    }
                    break;
            default:
                break;       
        }
    }        
    return true; 
}