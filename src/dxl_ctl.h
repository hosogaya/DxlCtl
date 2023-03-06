#include <Arduino.h>

#include "Dynamixel-0.2.0/Dynamixel.h"
#include <algorithm>
#include <vector>
#include <memory>

class DxlCtl {
    private:
        std::unique_ptr<Dynamixel> dxif_;
        std::vector<uint8_t> id_;
        std::vector<int32_t> origin_;
        std::vector<int32_t> buff32_;
        std::vector<uint32_t> buffu32_;
        std::vector<int16_t> buff16_;
        std::vector<uint16_t> buffu16_;
        std::vector<uint8_t> buffu8_;

        const float kDxl2Rad_ = M_PI/2048; // [dxl]->[rad]
        const float kDxl2Vel_ = 0.229f*2.0f*M_PI/60.0f; // [dxl]->[rad/s]
        const float kDxl2Cur_ = 2.69f; // [dxl]->[mA]
        const float kRad2Dxl_ = 2048/M_PI;
        const float kVel2Dxl_ = 60.0f/(2.0f*M_PI*0.229f);
        const float kCur2Dxl_ = 1.0f/2.69f;

        enum Reg {
            HardwareErrorStatus = 70,
            PresentCurrent = 126, 
            PresentVelocity = 128,
            PresentPosition = 132
        };

        enum OperatingMode {
            CurrentControl = 0,
            VelocityControl = 1,
            PositionControl = 3,
            ExetendedPositionControl = 4,
            CurrentBasePositionControl = 5,
            PWMControl = 16
        };

    public:
        DxlCtl(const uint8_t pin = 0);

        bool find(const uint8_t id);
        size_t getSize();

        const std::vector<uint8_t>& getID();
        
        void attach(HardwareSerial& serial, const size_t baudrate);
        bool addModel(const uint8_t id, const int32_t origin);
        bool addModel(const std::vector<uint8_t>& id, const std::vector<int32_t>& origin);

        // operating mode
        bool syncWriteOperatingMode(const uint8_t mode=3);
        // gain Velocity Control
        bool syncWriteVelocityPgain(const uint16_t gain=100);
        bool syncWriteVelocityIgain(const uint16_t gain=1920);

        bool ping();
        bool syncEnableTorque();
        bool syncDisableTorque();

        // transform
        float dxl2rad(const int32_t pos, const int32_t origin);
        float dxl2vel(const int32_t vel);
        float dxl2cur(const int16_t cur);
        int32_t rad2dxl(const float rad, const int32_t origin);
        int32_t vel2dxl(const float vel);
        int16_t cur2dxl(const float cur);

        // sync read
        bool syncReadPos(std::vector<float>& rad);
        bool syncReadCur(std::vector<float>& cur);
        bool syncReadPosVelCur(std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur);
        // sync write
        bool syncWritePos(std::vector<float>& rad);
        bool syncWriteVel(std::vector<float>& vel);
        bool syncWriteCur(std::vector<float>& cur);
};