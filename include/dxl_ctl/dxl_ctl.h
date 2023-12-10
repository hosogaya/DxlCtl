#include <Arduino.h>

#include "./Dynamixel-0.2.0/Dynamixel.h"
#include <algorithm>
#include <vector>
#include <memory>
#include <TeensyThreads.h>

class DxlCtl {
    protected:
        std::unique_ptr<Dynamixel> dxif_;
        std::vector<uint8_t> id_;
        std::vector<int32_t> origin_;
        std::vector<int32_t> buff32_;
        std::vector<uint32_t> buffu32_;
        std::vector<int16_t> buff16_;
        std::vector<uint16_t> buffu16_;
        std::vector<uint8_t> buffu8_;

        uint8_t indirect_data_num_ = 0;
        std::vector<uint16_t> indirect_data_size_;
        uint8_t shutdown_ = 0b00111100; // other than over voltage error

        const float kDxl2Rad_ = M_PI/2048; // [dxl]->[rad]
        const float kDxl2Vel_ = 0.229f*2.0f*M_PI/60.0f; // [dxl]->[rad/s]
        const float kDxl2Cur_ = 2.69f; // [dxl]->[mA]
        const float kRad2Dxl_ = 2048/M_PI;
        const float kVel2Dxl_ = 60.0f/(2.0f*M_PI*0.229f);
        const float kCur2Dxl_ = 1.0f/2.69f;

    public:
        enum Reg {
            TorqueEnable = 64,
            HardwareErrorStatus = 70,
            PresentCurrent = 126, 
            PresentVelocity = 128,
            PresentPosition = 132,
            IndirectAddr1 = 168,
            IndirectData1 = 224
        };

        enum OperatingMode {
            CurrentControl = 0,
            VelocityControl = 1,
            PositionControl = 3,
            ExetendedPositionControl = 4,
            CurrentBasePositionControl = 5,
            PWMControl = 16
        };

        enum HardwareError {
            InputVoltagetError = 0x01,
            OverheatingError = 0x04,
            MotionEncoderError=0x08,
            ElectricalShocError = 0x10,
            OverloadError = 0x20
        };

    public:
        DxlCtl(const uint8_t pin = 0);

        bool find(const uint8_t id);
        size_t getSize() const;

        const std::vector<uint8_t>& getID();
        
        void attach(HardwareSerial& serial, const size_t baudrate);
        bool addModel(const uint8_t id, const int32_t origin, const arduino::dynamixel::Model model);
        bool setModel(const std::vector<uint8_t>& id, const std::vector<int32_t>& origin, const arduino::dynamixel::Model model);

        // operating mode
        bool syncWriteOperatingMode(const uint8_t mode=3);
        // gain Velocity Control
        bool syncWriteVelocityPgain(const uint16_t gain=100);
        bool syncWriteVelocityIgain(const uint16_t gain=1920);

        bool ping();
        bool syncEnableTorque();
        bool syncDisableTorque();

        // error 
        // if shutdown_& <hardware error status> > 0, the motor shutdowns. 
        bool addShutdonwStatus(const HardwareError err);
        bool removeShutdownStatus(const HardwareError err);

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
        bool syncReadPosVelCur(std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur, Threads::Mutex& mx);
        // sync write
        bool syncWritePos(std::vector<float>& rad);
        bool syncWriteVel(std::vector<float>& vel);
        bool syncWriteCur(std::vector<float>& cur);
        bool syncWritePos(std::vector<float>& rad, Threads::Mutex& mx);
        bool syncWriteVel(std::vector<float>& vel, Threads::Mutex& mx);
        bool syncWriteCur(std::vector<float>& cur, Threads::Mutex& mx);



        // indirect Address
        bool addIndirectData(const uint16_t addr, const uint8_t data_size);
        bool syncReadPosVelCurErr(std::vector<uint8_t>& err, std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur);
        bool syncReadPosVelCurErrTor(std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur, std::vector<uint8_t>& err, std::vector<uint8_t>& tor);
};