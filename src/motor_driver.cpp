#include <dxl_ctl/motor_driver.h>


extern Threads threads;
namespace motor_interface
{
// void setup(MotorDriver& driver, const arduino::dynamixel::Model model)
// {
// // setup motor config
//   std::vector<uint8_t> ids1{11, 12, 13, 21, 22, 23, 31, 32, 33};
//   std::vector<uint8_t> ids2{41, 42, 43, 51, 52, 53, 61, 62, 63};
//   std::vector<int32_t> origin1{0, 0, 0, 0, 0, 0, 0, 0, 0};
//   std::vector<int32_t> origin2{0, 0, 0, 0, 0, 0, 0, 0, 0};
//   std::vector<float> input{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   driver.addMotorChain(ids1, origin1, std::make_shared<HardwareSerial>(Serial3));
//   driver.addMotorChain(ids2, origin2, std::make_shared<HardwareSerial>(Serial4));
//   const long baudrate = 1e6;
//   const float period = 10; // milli seond
//   driver.ready(baudrate, period, model); 

//   // set initial state
//   driver.setInput(0, input);
//   driver.setInput(1, input);

//   // start communication with motor
//   driver.start();
// }

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
        info.dxl_->syncReadPosVelCur(info.pos_, info.vel_, info.tor_, info.mx_out_);  

        if (info.mode_ == DxlCtl::PositionControl)
        {
            info.dxl_->syncWritePos(info.pos_, info.mx_in_);
        }
        else if (info.mode_ == DxlCtl::VelocityControl)
        {
            info.dxl_->syncWriteVel(info.pos_, info.mx_in_);
        }
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

void MotorDriver::read(std::vector<float>& pos, std::vector<float>& vel, std::vector<float>& tor)
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

// void MotorDriver::getData(Robot& robot)
// {
//     std::vector<float> pos, vel, tor;
//     std::array<float, 3> t_pos, t_vel, t_tor;
//     int leg_ind = 0;
//     for (int i=0; i<size_; ++i)
//     {
//         pos = getPos(i);
//         vel = getVel(i);
//         tor = getTor(i);
        
//         for (size_t j=0; j<info_[i].num_; ++leg_ind)
//         {
//             for (size_t k=0; k<3; ++k, ++j)
//             {
//                 t_pos[k] = pos[j];
//                 t_vel[k] = vel[j];
//                 t_tor[k] = tor[j];
//             }
//             robot.legs_[leg_ind].setObsAngle(t_pos);
//             robot.legs_[leg_ind].observed_angular_velocity_ = t_vel;
//             robot.legs_[leg_ind].observed_torque_ = t_tor;
//         }
//     }
// }

// void MotorDriver::setData(Robot& robot, const std::vector<float>& pos, const std::vector<float>& vel,  const std::vector<float>& tor)
// {
//     std::array<float, 3> t_pos, t_vel, t_tor;
//     int leg_ind = 0;

//     for (size_t j=0; j<pos.size();)
//     {
//         for (size_t k=0; k<3; ++k, ++j)
//         {
//             t_pos[k] = pos[j];
//             t_vel[k] = vel[j];
//             t_tor[k] = tor[j];
//         }
//         robot.legs_[leg_ind].setObsAngle(t_pos);
//         robot.legs_[leg_ind].observed_angular_velocity_ = t_vel;
//         robot.legs_[leg_ind].observed_torque_ = t_tor;
//         ++leg_ind;
//     }
// }


void MotorDriver::setInput(const int chain_id, std::vector<float>& value)
{
    Threads::Scope lock(info_[chain_id].mx_in_);
    info_[chain_id].input_ = value;
}

std::shared_ptr<DxlCtl>& MotorDriver::getDriver(const int chain_id)
{
    return info_[chain_id].dxl_;
}

int MotorDriver::size() const {return size_;}


}// end motor_contorl