/*
   Pod library
*/

#define AP_SERIALMANAGER_POD_BAUD         115200
#define AP_SERIALMANAGER_POD_BUFSIZE_RX        64
#define AP_SERIALMANAGER_POD_BUFSIZE_TX        64

#include "AP_Pod.h"

extern const AP_HAL::HAL& hal;

//constructor
AP_Pod::AP_Pod(void)
{
    _port = NULL;
    _step = 0;
}

// init - perform require initialisation including detecting which protocol to use
void AP_Pod::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Pod, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_POD_BAUD, AP_SERIALMANAGER_POD_BUFSIZE_RX, AP_SERIALMANAGER_POD_BUFSIZE_TX);
    }
}

void AP_Pod::update()
{
    if(_port != NULL){
        int16_t numc = _port->available();
        uint8_t data;
        uint8_t checksum = 0;

        for (int16_t i = 0; i < numc; i++) {
            data = _port->read();

            switch(_step) {
            case 0:
                if(data == 0xEE)
                    _step = 1;
                break;

            case 1:
                if(data == 0x90)
                    _step = 2;
            else
                _step = 0;
            break;

            case 16:
                checksum += data;
                angle_yaw_temp = data;
                _step = 17;
                break;

            case 17:
                checksum += data;
                angle_yaw_temp |= data << 8;
                _step = 18;
                break;

            case 18:
                checksum += data;
                angle_pitch_temp = data;
                _step = 19;
                break;

            case 19:
                checksum += data;
                angle_pitch_temp |= data << 8;
                _step = 20;
                break;
            
            
            case 2:case 3:case 4:case 5:case 6:case 7:case 8:case 9:case 10:
            case 11:case 12:case 13:case 14:case 15:case 20:case 21:case 22:
            case 23:case 24:case 25:case 26:case 27:case 28:case 29:case 30:
                checksum += data;
                _step += 1;
                break;
                
            case 31:
                if(checksum == data) {
                    angle_pitch = angle_pitch_temp;
                    angle_yaw = angle_yaw_temp;
                    last_frame_ms = AP_HAL::millis();
                    // return true;
                }
                break;

            default:
                _step = 0;
            }
        }
    }
}
