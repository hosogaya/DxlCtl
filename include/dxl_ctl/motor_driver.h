#pragma once
#include <dxl_ctl/dxl_ctl.h>
#include <TeensyThreads.h>

namespace motor_interface 
{
class MotorDriver
{
public:
    MotorDriver();
    ~MotorDriver();

    void addMotorChain(const arduino::dynamixel::Model model, const DxlCtl::OperatingMode mode, std::vector<u_int8_t>& ids, std::vector<int32_t>& origin, HardwareSerial& serial);
    bool ready(const long baudrate, const float period);
    bool start();
    bool stop();
    void getState(std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor);
    bool read(std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor);
    bool write(std::vector<float>& input);
    std::vector<float> getPos(const int chain_id);
    std::vector<float> getVel(const int chain_id);
    std::vector<float> getTor(const int chain_id);
    std::shared_ptr<DxlCtl>& getDriver(const int chain_id);
    int size() const;
    // return the number of motors connected with the motor chain i. 
    size_t size(const int i) const {return ids_[i].size();} 
    void setInput(const int chain_id, std::vector<float>& value);
    void setInput(const std::vector<float>& value);

    struct Info
    {
        std::vector<float> pos_;
        std::vector<float> vel_;
        std::vector<float> tor_;
        std::vector<float> input_;
        std::shared_ptr<DxlCtl> dxl_;
        bool work_;
        Threads::Mutex mx_in_;
        Threads::Mutex mx_out_;
        DxlCtl::OperatingMode mode_;
        size_t num_;
        long period_;
        long elapsed_time_;
    };

private:
    // std::vector<Info> info_;
    // chain*motor
    bool addMotorChain(std::vector<uint8_t> motor_ids, std::vector<int32_t> origin,  std::shared_ptr<HardwareSerial> serial);

    int size_ = 0;
    std::vector<std::vector<uint8_t>> ids_;
    std::vector<std::vector<int32_t>> origin_;

    std::vector<std::shared_ptr<HardwareSerial>> serial_;
    std::vector<arduino::dynamixel::Model> models_;
    // std::vector<std::shared_ptr<std::thread>> thread_;

    std::vector<DxlCtl::OperatingMode> modes_;
};

void read(MotorDriver& driver, std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor);
}