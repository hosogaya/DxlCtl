#include <dxl_ctl/motor_driver.h>


extern Threads threads;
namespace motor_interface
{

void read(MotorDriver& driver, std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor)
{
    int index = 0;
    for (int i=0; i<driver.size(); ++i)
    {   
        // position
        std::vector<float> buff = driver.getPos(i);
        for (size_t j=0; j<buff.size(); ++j)
        {
            pos[index+j] = buff[j];
        }
        // velocity
        buff = driver.getVel(i);
        for (size_t j=0; j<buff.size(); ++j)
        {
            vel[index+j] = buff[j];
        }
        // torque
        buff = driver.getTor(i);
        for (size_t j=0; j<buff.size(); ++j)
        {
            tor[index+j] = buff[j];
        }
        index += buff.size();
    }
}

std::vector<MotorDriver::Info> info_;
void threadRW(int i)
{
    MotorDriver::Info& info = info_[i];
    long start;
    while (info.work_)
    {
        start = micros();
        if (!info.dxl_->syncReadPosVelCur(info.pos_, info.vel_, info.tor_, info.mx_out_)) info.work_ = false;  

        if (info.mode_ == DxlCtl::PositionControl)
        {
            if (!info.dxl_->syncWritePos(info.input_, info.mx_in_)) info.work_ = false;
        }
        else if (info.mode_ == DxlCtl::VelocityControl)
        {
            if (!info.dxl_->syncWriteVel(info.input_, info.mx_in_)) info.work_ = false;
        }
        info_[i].elapsed_time_ = micros() - start;
        while (micros() - start) threads.yield();
    }
}

void threadRead(int i)
{
    MotorDriver::Info& info = info_[i];
    long start;
    while (info.work_)
    {
        start = micros();
        if (!info.dxl_->syncReadPosVelCur(info.pos_, info.vel_, info.tor_, info.mx_out_)) info.work_ = false;
        info_[i].elapsed_time_ = micros() - start;
        while (micros() - start) threads.yield();
    }
}

void threadWrite(int i)
{
    MotorDriver::Info& info = info_[i];
    long start;
    while (info.work_)
    {
        start = micros();
        if (info.mode_ == DxlCtl::PositionControl)
        {
            if (!info.dxl_->syncWritePos(info.input_, info.mx_in_)) info.work_ = false;
        }
        else if (info.mode_ == DxlCtl::VelocityControl)
        {
            if (!info.dxl_->syncWriteVel(info.input_, info.mx_in_)) info.work_ = false;
        }
        info_[i].elapsed_time_ = micros() - start;
        while (micros() - start) threads.yield();
    }
}

void MotorDriver::addMotorChain(const arduino::dynamixel::Model model, const DxlCtl::OperatingMode mode, std::vector<u_int8_t>& ids, std::vector<int32_t>& origin, HardwareSerial& serial)
{
    this->addMotorChain(ids, origin, std::make_shared<HardwareSerial>(serial));
    models_.push_back(model);
    modes_.push_back(mode);
}

MotorDriver::MotorDriver()
{}

MotorDriver::~MotorDriver()
{
    info_.clear();
}

bool MotorDriver::addMotorChain(std::vector<uint8_t> _ids, std::vector<int32_t> _origin,  std::shared_ptr<HardwareSerial> _serial)
{
    auto itr = std::find_if(serial_.begin(), serial_.end(), [_serial](const std::shared_ptr<HardwareSerial>& s){return s == _serial;});
    if (itr == serial_.end()) 
    {
        serial_.push_back(_serial);
        ids_.push_back(_ids);
        origin_.push_back(_origin);
        info_.resize(ids_.size());
        ++size_;
    }
    else {
        // insert ids
        int index = std::distance(serial_.begin(), itr);
        ids_[index].insert(ids_[index].end(), _ids.begin(), _ids.end());
        origin_[index].insert(origin_[index].end(), _origin.begin(), _origin.end());
    }
    // check chain num
    if (serial_.size() != ids_.size()) return false;
    if (serial_.size() != origin_.size()) return false;
    if (info_.size() != serial_.size()) return false;
    
    // check unique ness
    auto serial_itr = std::unique(serial_.begin(), serial_.end());
    if (serial_itr != serial_.end()) return false;
    for (size_t i=0; i<ids_.size(); ++i)
    {
        // check size
        if (origin_[i].size() != ids_.size()) return false;

        auto ids_itr = std::unique(ids_[i].begin(), ids_[i].end());
        if (ids_itr != ids_[i].end()) return false;
    }

    return true;
}

bool MotorDriver::ready(const long baudrate, const float period) /// milli second
{
    for (size_t i=0; i<serial_.size(); ++i)
    {
        serial_[i]->begin(baudrate);
        info_[i].dxl_ = std::make_shared<DxlCtl>();
        info_[i].dxl_->attach(*serial_[i], baudrate);
        if (info_[i].dxl_->setModel(ids_[i], origin_[i], models_[i])) return false;

        info_[i].dxl_->syncEnableTorque();
        info_[i].work_ = true;
        info_[i].mode_ = modes_[i];
        info_[i].period_ = period*1e3;
        info_[i].num_ = ids_[i].size();
        info_[i].input_.resize(info_[i].num_);
        info_[i].pos_.resize(info_[i].num_);
        info_[i].vel_.resize(info_[i].num_);
        info_[i].tor_.resize(info_[i].num_);
    } 

    return true;
}

bool MotorDriver::start()
{
    for (size_t i=0; i<info_.size(); ++i)
    {
        threads.addThread(threadRW, i);
    }
    return true;
}

bool MotorDriver::stop() 
{
    for (Info& i : info_) i.work_ = false;
    return true;
}

void MotorDriver::getState(std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor)
{
    int index = 0;
    for (int i=0; i<size(); ++i)
    {   
        {
            Threads::Scope lock(info_[i].mx_out_);
            for (size_t j=0; j<info_[i].num_; ++j)
            {
                pos[index+j] = info_[i].pos_[j];
                vel[index+j] = info_[i].vel_[j];
                tor[index+j] = info_[i].tor_[j];
            }
        }
        index += info_[i].num_;
    }
}

bool MotorDriver::read(std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor)
{
    int index = 0;
    std::vector<float> t_pos, t_vel, t_tor;
    for (int i=0; i<size(); ++i)
    {   
        if (info_[i].work_) return false;
        if (!info_[i].dxl_->syncReadPosVelCur(t_pos, t_vel, t_tor)) return false;
        {
            for (size_t j=0; j<info_[i].num_; ++j)
            {
                pos[index+j] = t_pos[j];
                vel[index+j] = t_vel[j];
                tor[index+j] = t_tor[j];
            }
        }
        index += info_[i].num_;
    }
    return true;
}

bool MotorDriver::write(std::vector<float>& input)
{
    int index = 0;
    std::vector<float> t_input;
    for (const Info& info: info_)
    {
        if (info.work_) return false;
        t_input.resize(info.num_);
        for (size_t j=0; j<info.num_; ++j)
        {
            t_input[j] = input[index+j];
        }
        if (info.mode_ == DxlCtl::PositionControl)
        {
            if (!info.dxl_->syncWritePos(t_input)) return false;
        }
        else if (info.mode_ == DxlCtl::VelocityControl)
        {
            if (!info.dxl_->syncWriteVel(t_input)) return false;
        }
        index += info.num_;
    }
    return true;
}

std::vector<float> MotorDriver::getPos(const int chain_id) 
{
    Threads::Scope lock(info_[chain_id].mx_out_);
    return info_[chain_id].pos_;
}

std::vector<float> MotorDriver::getVel(const int chain_id) 
{
    Threads::Scope lock(info_[chain_id].mx_out_);
    return info_[chain_id].vel_;
}

std::vector<float> MotorDriver::getTor(const int chain_id) 
{
    Threads::Scope lock(info_[chain_id].mx_out_);
    return info_[chain_id].tor_;
}

void MotorDriver::setInput(const int chain_id, std::vector<float>& value)
{
    Threads::Scope lock(info_[chain_id].mx_in_);
    info_[chain_id].input_ = value;
}

void MotorDriver::setInput(const std::vector<float>& value)
{
    size_t index = 0;
    for (size_t i=0; i<size(); ++i)
    {
        Threads::Scope lock(info_[i].mx_in_);
        for (size_t j=0; j<info_[i].num_; ++j)
        {
            info_[i].input_[j] = value[index+j];
        }
        index += info_[i].num_;
    }
}

std::shared_ptr<DxlCtl>& MotorDriver::getDriver(const int chain_id)
{
    return info_[chain_id].dxl_;
}

int MotorDriver::size() const {return size_;}


}// end motor_contorl