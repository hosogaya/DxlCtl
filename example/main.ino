#include <dxl_ctl/motor_driver.h>

#include <cmath>
#include <vector>

DxlCtl dxl3;

std::vector<uint8_t> id{1};
std::vector<int32_t> origin{2048};

void setup() {
    Serial.begin(115200);

    // setup
    dxl3.attach(Serial3, 2000000);
    if (!dxl3.setModel(id, origin, arduino::dynamixel::Model::X)) {
        Serial.println("Sizes of id and origin are different");
        exit(1);
    }

    // boot
    dxl3.syncEnableTorque();
}

std::vector<float> rad_obs;
std::vector<float> rad_tar;
unsigned long t = 0;
int sign = 1;

void loop() {
    if (millis() - t > 1e3f) {
        sign *= -1;
    }        
    if (!dxl3.syncReadPos(rad_obs)) {
        Serial.println("failed to read current states of motors");
        exit(1);
    }
    rad_tar[0] = sign*M_PI/2.0f;
    if (!dxl3.syncWritePos(rad_tar)) {
        Serial.println("failed to write velocity instructions");
        exit(1);
    }
}