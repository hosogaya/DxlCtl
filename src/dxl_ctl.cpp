#include <dxl_ctl.h>

DxlCtl::DxlCtl(const uint8_t pin) {
    dxif_ = std::make_unique<Dynamixel>(pin);
}

bool DxlCtl::find(const uint8_t id) {
    auto itr = std::find(id_.begin(), id_.end(), id);
    if (itr != id_.end()) return true;
    return false;
}

size_t DxlCtl::getSize() const {return id_.size();}

void DxlCtl::attach(HardwareSerial& serial, const size_t baudrate) {
    serial.begin(baudrate);
    while(!serial);
    dxif_->attach(serial, baudrate);
    // Serial3.begin(baudrate);
    // while(!Serial3);
    // dxif_->attach(Serial3, baudrate);
}


const std::vector<uint8_t>& DxlCtl::getID() {return id_;}

bool DxlCtl::addModel(const uint8_t id, const int32_t origin) {
    if (find(id)) return false; // already exisit

    id_.push_back(id);
    origin_.push_back(origin);
    dxif_->addModel<DxlModel::MX>(id);
    return true;
}

bool DxlCtl::addModel(const std::vector<uint8_t>& id, const std::vector<int32_t>& origin) {
    if (id.size() != origin.size()) return false;
    id_.resize(id.size());
    origin_.resize(id.size());
    for (size_t i=0; i<id.size(); ++i) {
        id_[i] = id[i];
        origin_[i] = origin[i];
        dxif_->addModel<DxlModel::MX>(id_[i]);
    }
    return true;
}

bool DxlCtl::syncWriteOperatingMode(const uint8_t mode) {
    // check size
    if (this->buffu8_.size() != id_.size()) buffu8_.resize(id_.size());

    // check vaild mode
    if (mode == OperatingMode::CurrentControl 
     || mode == OperatingMode::VelocityControl
     || mode == OperatingMode::PositionControl
     || mode == OperatingMode::ExetendedPositionControl
     || mode == OperatingMode::CurrentBasePositionControl
     || mode == OperatingMode::PWMControl)
    {
        for (size_t i=0; i<buffu8_.size(); ++i) buffu8_[i] = mode;
        return dxif_->syncwriteoperatingMode(id_.data(), buffu8_.data(), buffu8_.size());
    }
    else return false;
}

bool DxlCtl::syncWriteVelocityPgain(const uint16_t gain) {
    // check size
    if (this->buffu16_.size() != this->id_.size()) buffu16_.resize(id_.size());
    for (size_t i=0; i<buffu16_.size(); ++i) buffu16_[i] = gain;
    return dxif_->syncwritevelocityPGain(id_.data(), buffu16_.data(), id_.size());
}

bool DxlCtl::syncWriteVelocityIgain(const uint16_t gain) {
    // check size
    if (this->buffu16_.size() != this->id_.size()) buffu16_.resize(id_.size());
    for (size_t i=0; i<buffu16_.size(); ++i) buffu16_[i] = gain;
    return dxif_->syncwritevelocityIGain(id_.data(), buffu16_.data(), id_.size());
}

bool DxlCtl::ping() {
    for (auto d: id_) {
        if (!dxif_->ping(d)) return false;
    }
    return true;
}

bool DxlCtl::syncEnableTorque() {
    std::unique_ptr<bool[]> data = std::make_unique<bool[]>(id_.size());
    for (size_t i=0; i<id_.size(); ++i) data[i] = true;
    return dxif_->syncwritetorqueEnable(id_.data(), data.get(), id_.size());
    // for (auto id: id_) while(!dxif_->torqueEnable(id, true));
    // return true;
}

bool DxlCtl::syncDisableTorque() {
    std::unique_ptr<bool[]> data = std::make_unique<bool[]>(id_.size());
    for (size_t i=0; i<id_.size(); ++i) data[i] = false;
    return dxif_->syncwritetorqueEnable(id_.data(), data.get(), id_.size());
}

float DxlCtl::dxl2rad(const int32_t pos, const int32_t origin) {
    return static_cast<float>(pos - origin)*kDxl2Rad_;
}

float DxlCtl::dxl2vel(const int32_t vel) {
    return static_cast<float>(vel)*kDxl2Vel_;
}

float DxlCtl::dxl2cur(const int16_t cur) {
    return static_cast<float>(cur)*kDxl2Cur_;
}

int32_t DxlCtl::rad2dxl(const float rad, const int32_t origin) {
    return static_cast<int32_t>(rad*kRad2Dxl_) + origin;
}

int32_t DxlCtl::vel2dxl(const float vel) {
    return static_cast<int32_t>(vel*kVel2Dxl_);
}

int16_t DxlCtl::cur2dxl(const float cur) {
    return static_cast<int16_t>(cur*kCur2Dxl_);
}

bool DxlCtl::syncReadPos(std::vector<float>& rad) {
    // check size
    if (id_.size() != rad.size()) rad.resize(id_.size());
    if(buff32_.size() != id_.size()) buff32_.resize(id_.size());

    // read
    if(!dxif_->syncreadpresentPosition(id_.data(), buff32_.data(), id_.size())) return false;
    
    // transform
    for (size_t i=0; i<id_.size(); ++i) rad[i] = dxl2rad(buff32_[i], origin_[i]);
    return true;
}

bool DxlCtl::syncReadCur(std::vector<float>& cur) {
    if (id_.size() != cur.size()) cur.resize(id_.size());
    if (id_.size() != buff16_.size()) buff16_.resize(id_.size());

    if (!dxif_->syncreadpresentCurrent(id_.data(), buff16_.data(), id_.size())) return false;
    
    for (size_t i=0; i<id_.size(); ++i) cur[i] = dxl2cur(buff16_[i]);
    return true;
}


bool DxlCtl::syncReadPosVelCur(std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur) {
    // check size 
    if (id_.size() != rad.size()) rad.resize(id_.size());
    if (id_.size() != vel.size()) vel.resize(id_.size());
    if (id_.size() != cur.size()) cur.resize(id_.size());
    if (buffu32_.size() != 3*id_.size()) buffu32_.resize(3*id_.size());

    uint16_t bytes[] = {2,4,4};
    if (!dxif_->syncreadmultipledata(id_.data(), buffu32_.data(), Reg::PresentCurrent, bytes, 3, id_.size())) return false;

    for (size_t i=0; i<id_.size(); ++i) {
        rad[i] = dxl2rad(static_cast<int32_t>(buffu32_[i+2]), origin_[i]);
        vel[i] = dxl2vel(static_cast<int32_t>(buffu32_[i+1]));
        cur[i] = dxl2cur(static_cast<int16_t>(buffu32_[i]));
    }
    return true;
}

bool DxlCtl::syncWritePos(std::vector<float>& rad) {
    // check size
    if (id_.size() != rad.size()) return false;
    if (id_.size() != buff32_.size()) buff32_.resize(id_.size());

    // transform 
    for (size_t i=0; i<id_.size(); ++i) buff32_[i] = rad2dxl(rad[i], origin_[i]);

    return dxif_->syncwritegoalPosition(id_.data(), buff32_.data(), id_.size());
}

bool DxlCtl::syncWriteVel(std::vector<float>& vel) {
    // check size
    if (id_.size() != vel.size()) return false;
    if (id_.size() != buff32_.size()) buff32_.resize(id_.size());

    // transform 
    for (size_t i=0; i<id_.size(); ++i) buff32_[i] = vel2dxl(vel[i]);

    return dxif_->syncwritegoalVelocity(id_.data(), buff32_.data(), id_.size());
}


bool DxlCtl::syncWriteCur(std::vector<float>& cur) {
    // check size
    if (id_.size() != cur.size()) cur.resize(id_.size());
    if (id_.size() != buff16_.size()) buff16_.resize(id_.size());

    // transform
    for (size_t i=0; i<id_.size(); ++i) buff16_[i] = cur2dxl(cur[i]);

    return dxif_->syncwritegoalCurrent(id_.data(), buff16_.data(), id_.size());
}

bool DxlCtl::addIndirectData(const uint16_t addr, const uint8_t data_size) {
    for (size_t i=0; i<id_.size(); ++i) {
        for (uint8_t j=0; j<data_size; ++j) {
            dxif_->setIndirectAddress(id_[i], indirect_data_num_+j, addr+j);
        }
    }
    indirect_data_num_ += data_size;
    indirect_data_size_.push_back(data_size);
}

bool DxlCtl::syncReadPosVelCurErr(std::vector<uint8_t>& err, std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur) {
    // check size 
    if (id_.size() != rad.size()) rad.resize(id_.size());
    if (id_.size() != vel.size()) vel.resize(id_.size());
    if (id_.size() != cur.size()) cur.resize(id_.size());
    if (id_.size() != err.size()) err.resize(id_.size());
    if (buffu32_.size() != indirect_data_size_.size()*id_.size()) buffu32_.resize(indirect_data_size_.size()*id_.size());

    if (!dxif_->syncreadmultipledata(id_.data(), buffu32_.data(), Reg::IndirectData1, indirect_data_size_.data(), indirect_data_size_.size(), id_.size())) return false;

    for (size_t i=0; i<id_.size(); ++i) {
        rad[i] = dxl2rad(static_cast<int32_t>(buffu32_[i+3]), origin_[i]);
        vel[i] = dxl2vel(static_cast<int32_t>(buffu32_[i+2]));
        cur[i] = dxl2cur(static_cast<int16_t>(buffu32_[i+1]));
        err[i] = static_cast<uint8_t>(buffu32_[i]);
    }
    return true;
}

bool DxlCtl::syncReadPosVelCurErrTor(std::vector<float>& rad, std::vector<float>& vel, std::vector<float>& cur, std::vector<uint8_t>& err, std::vector<uint8_t>& tor) {
    // check size 
    if (id_.size() != rad.size()) rad.resize(id_.size());
    if (id_.size() != vel.size()) vel.resize(id_.size());
    if (id_.size() != cur.size()) cur.resize(id_.size());
    if (id_.size() != err.size()) err.resize(id_.size());
    if (id_.size() != tor.size()) tor.resize(id_.size());
    if (buffu32_.size() != indirect_data_size_.size()*id_.size()) buffu32_.resize(indirect_data_size_.size()*id_.size());

    if (!dxif_->syncreadmultipledata(id_.data(), buffu32_.data(), Reg::IndirectData1, indirect_data_size_.data(), indirect_data_size_.size(), id_.size())) return false;

    for (size_t i=0; i<id_.size(); ++i) {
        rad[i] = dxl2rad(static_cast<int32_t>(buffu32_[i+4]), origin_[i]);
        vel[i] = dxl2vel(static_cast<int32_t>(buffu32_[i+3]));
        cur[i] = dxl2cur(static_cast<int16_t>(buffu32_[i+2]));
        err[i] = static_cast<uint8_t>(buffu32_[i+1]);
        tor[i] = static_cast<uint8_t>(buffu32_[i]);
    }
    return true;
}