#pragma once
#ifndef ARDUINO_DYNAMIXEL_IMPL_H
#define ARDUINO_DYNAMIXEL_IMPL_H

#include "lib/DynamixelSDK/include/dynamixel_sdk.h"
#include "util/ArxSmartPtr/ArxSmartPtr.h"
#include "DynamixelControlTable.h"
#include <stdio.h>
namespace arduino {
namespace dynamixel {

enum class ProtocolVersion {
    V1,
    V2
};

const uint8_t ID_BROADCAST = 0xFE;
const uint8_t ID_LIMIT = 0xFC;

class Dynamixel {
    // TODO: do not use new, make instance inside
    ::dynamixel::PortHandler* port_handler;
    ::dynamixel::PacketHandler* packet_handler;
//#if 0
        ::dynamixel::GroupSyncWrite sync_writer;
        ::dynamixel::GroupSyncRead sync_reader;
        ::dynamixel::GroupBulkWrite bulk_writer;
        ::dynamixel::GroupBulkRead bulk_reader;
//#endif

    struct Info {
        std::shared_ptr<ControlTable> ct;
        int result;
        uint8_t error;
        uint16_t model;
    };
    Map<uint8_t, Info> info;

public:
    using Models = Vec<uint8_t>;

    // TODO: do not use new, make instance inside
    Dynamixel(uint8_t pin_rts_enable, ProtocolVersion ver = ProtocolVersion::V2)
        : port_handler(new ::dynamixel::PortHandler(pin_rts_enable)), packet_handler((ver == ProtocolVersion::V2) ? (::dynamixel::PacketHandler*)new ::dynamixel::Protocol2PacketHandler() : (::dynamixel::PacketHandler*)new ::dynamixel::Protocol1PacketHandler())
//#if 0
        , sync_writer(port_handler, packet_handler)
        , sync_reader(port_handler, packet_handler)
        , bulk_writer(port_handler, packet_handler)
        , bulk_reader(port_handler, packet_handler)
//#endif
    {
    }

    ~Dynamixel() {
        delete port_handler;
        delete packet_handler;
    }

    template <Model>
    inline void addModel(uint8_t id) {
        std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::OTHER>::instance();
        info.emplace(id, Info{sp, 0, 0, 0});
    }

    void attach(Stream& s, size_t baud) {
        port_handler->attach(s, baud);
        packet_handler->attach(port_handler);
    }

    uint8_t size() const { return info.size(); }

    // wrapper for instructions

    bool write(uint8_t id, Reg reg, uint32_t data) {
        if (info.find(id) == info.end()) return false;

        uint16_t addr = info[id].ct->ct[reg].addr;
        uint8_t size = info[id].ct->ct[reg].size;
        uint8_t error;
        int result = packet_handler->writeBytesTxRx(id, addr, data, size, &error);
        return handleResult(id, result, error);
    }

    uint32_t read(uint8_t id, Reg reg) {
        if (info.find(id) == info.end()) return 0xFFFFFFFF;

        uint32_t value = 0;
        uint16_t addr = info[id].ct->ct[reg].addr;
        uint8_t size = info[id].ct->ct[reg].size;
        uint8_t error = 0;
        int result = packet_handler->readBytesTxRx(id, addr, (uint8_t*)&value, size, &error);
        bool b = handleResult(id, result, error);
        return b ? value : 0xFFFFFFFF;
    }

    bool ping(uint8_t id) {
        if (id > ID_LIMIT) return false;
        uint8_t error = 0;
        uint16_t model = 0;
        int result = packet_handler->ping(id, &model, &error);
        return handleResult(id, result, error, model);
    }

    Models ping()  // broadcast
    {
        Models ids;
        int result = packet_handler->broadcastPing(ids);
        if (result != COMM_SUCCESS) return Models();
        return ids;
    }

    bool factoryReset(uint8_t id, ResetMode mode = ResetMode::EXC_ID_BAUD) {
        uint8_t error = 0;
        int result = packet_handler->factoryReset(id, (uint8_t)mode, &error);
        return handleResult(id, result, error);
    }

    bool factoryReset(ResetMode mode = ResetMode::EXC_ID_BAUD) {
        return factoryReset(ID_BROADCAST, mode);
    }

    bool reboot(uint8_t id) {
        uint8_t error = 0;
        int result = packet_handler->reboot(id, &error);
        return handleResult(id, result, error);
    }

    void verbose(uint8_t id) {
        if (info.find(id) == info.end()) return;
        verboseResult(id);
        verboseError(id);
    }

    // TODO: if there is no such ID
    int lastCommResult(uint8_t id) { return (info.find(id) != info.end()) ? info[id].result : -1; }
    uint8_t lastError(uint8_t id) { return (info.find(id) != info.end()) ? info[id].error : 0; }
    uint16_t lastModelNo(uint8_t id) { return (info.find(id) != info.end()) ? info[id].model : 0; }

    // wrappers for control table

    // read values
    uint16_t modelNumber(uint8_t id) { return (uint16_t)read(id, Reg::MODEL_NUMBER); }
    uint32_t modelInformation(uint8_t id) { return (uint32_t)read(id, Reg::MODEL_INFORMATION); }
    uint8_t versionOfFirmware(uint8_t id) { return (uint8_t)read(id, Reg::VERSION_OF_FIRMWARE); }
    uint8_t id(uint8_t id) { return (uint8_t)read(id, Reg::ID); }
    uint8_t baudrate(uint8_t id) { return (uint8_t)read(id, Reg::BAUDRATE); }
    uint8_t returnDelayTime(uint8_t id) { return (uint8_t)read(id, Reg::RETURN_DELAY_TIME); }
    uint8_t driveMode(uint8_t id) { return (uint8_t)read(id, Reg::DRIVE_MODE); }
    uint8_t operatingMode(uint8_t id) { return (uint8_t)read(id, Reg::OPERATING_MODE); }
    uint8_t secondaryId(uint8_t id) { return (uint8_t)read(id, Reg::SECONDARY_ID); }
    uint8_t protocolVersion(uint8_t id) { return (uint8_t)read(id, Reg::PROTOCOL_VERSION); }
    int32_t homingOffset(uint8_t id) { return (int32_t)read(id, Reg::HOMING_OFFSET); }
    uint32_t movingThreshold(uint8_t id) { return (uint32_t)read(id, Reg::MOVING_THRESHOLD); }
    uint8_t temperatureLimit(uint8_t id) { return (uint8_t)read(id, Reg::TEMPERATURE_LIMIT); }
    uint16_t maxVoltageLimit(uint8_t id) { return (uint16_t)read(id, Reg::MAX_VOLTAGE_LIMIT); }
    uint16_t minVoltageLimit(uint8_t id) { return (uint16_t)read(id, Reg::MIN_VOLTAGE_LIMIT); }
    uint16_t pwmLimit(uint8_t id) { return (uint16_t)read(id, Reg::PWM_LIMIT); }
    uint16_t currentLimit(uint8_t id) { return (uint16_t)read(id, Reg::CURRENT_LIMIT); }
    uint32_t accelerationLimit(uint8_t id) { return (uint32_t)read(id, Reg::ACCELERATION_LIMIT); }
    uint32_t velocityLimit(uint8_t id) { return (uint32_t)read(id, Reg::VELOCITY_LIMIT); }
    uint32_t maxPositionLimit(uint8_t id) { return (uint32_t)read(id, Reg::MAX_POSITION_LIMIT); }
    uint32_t minPositionLimit(uint8_t id) { return (uint32_t)read(id, Reg::MIN_POSITION_LIMIT); }
    uint8_t shutdown(uint8_t id) { return (uint8_t)read(id, Reg::SHUTDOWN); }
    bool torqueEnable(uint8_t id) { return (bool)read(id, Reg::TORQUE_ENABLE); }
    uint8_t led(uint8_t id) { return (uint8_t)read(id, Reg::LED); }
    uint8_t statusReturnLevel(uint8_t id) { return (uint8_t)read(id, Reg::STATUS_RETURN_LEVEL); }
    uint8_t registerdInstruction(uint8_t id) { return (uint8_t)read(id, Reg::REGISTERED_INSTRUCTION); }
    uint8_t hardwareErrorStatus(uint8_t id) { return (uint8_t)read(id, Reg::HARDWARE_ERROR_STATUS); }
    uint16_t velocityIGain(uint8_t id) { return (uint16_t)read(id, Reg::VELOCITY_I_GAIN); }
    uint16_t velocityPGain(uint8_t id) { return (uint16_t)read(id, Reg::VELOCITY_P_GAIN); }
    uint16_t positionDGain(uint8_t id) { return (uint16_t)read(id, Reg::POSITION_D_GAIN); }
    uint16_t positionIGain(uint8_t id) { return (uint16_t)read(id, Reg::POSITION_I_GAIN); }
    uint16_t positionPGain(uint8_t id) { return (uint16_t)read(id, Reg::POSITION_P_GAIN); }
    uint16_t feedForwardAccelerationGain(uint8_t id) { return (uint16_t)read(id, Reg::FEEDFORWARD_ACCELERATION_GAIN); }
    uint16_t feedForwardVelocityGain(uint8_t id) { return (uint16_t)read(id, Reg::FEEDFORWARD_VELOCITY_GAIN); }
    int8_t busWatchdog(uint8_t id) { return (int8_t)read(id, Reg::BUS_WATCHDOG); }
    int16_t goalPwm(uint8_t id) { return (int16_t)read(id, Reg::GOAL_PWM); }
    int16_t goalCurrent(uint8_t id) { return (int16_t)read(id, Reg::GOAL_CURRENT); }
    int32_t goalVelocity(uint8_t id) { return (int32_t)read(id, Reg::GOAL_VELOCITY); }
    uint32_t profileAcceleration(uint8_t id) { return (uint32_t)read(id, Reg::PROFILE_ACCELERATION); }
    uint32_t profileVelocity(uint8_t id) { return (uint32_t)read(id, Reg::PROFILE_VELOCITY); }
    int32_t goalPosition(uint8_t id) { return (int32_t)read(id, Reg::GOAL_POSITION); }
    uint16_t realTimeTick(uint8_t id) { return (uint16_t)read(id, Reg::REALTIME_TICK); }
    uint8_t moving(uint8_t id) { return (uint8_t)read(id, Reg::MOVING); }
    uint8_t movingStatus(uint8_t id) { return (uint8_t)read(id, Reg::MOVING_STATUS); }
    int16_t presentPwm(uint8_t id) { return (int16_t)read(id, Reg::PRESENT_PWM); }
    int16_t presentCurrent(uint8_t id) { return (int16_t)read(id, Reg::PRESENT_CURRENT); }
    int32_t presentVelocity(uint8_t id) { return (int32_t)read(id, Reg::PRESENT_VELOCITY); }
    int32_t presentPosition(uint8_t id) { return (int32_t)read(id, Reg::PRESENT_POSITION); }
    uint32_t velocityTrajectory(uint8_t id) { return (uint32_t)read(id, Reg::VELOCITY_TRAJECTORY); }
    uint32_t positionTrajectory(uint8_t id) { return (uint32_t)read(id, Reg::POSITION_TRAJECTORY); }
    uint16_t presentInputVoltage(uint8_t id) { return (uint16_t)read(id, Reg::PRESENT_INPUT_VOLTAGE); }
    uint8_t presentTemperature(uint8_t id) { return (uint8_t)read(id, Reg::PRESENT_TEMPERATURE); }
	
	// syncread values
	
	bool syncreadmodelNumber         (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::MODEL_NUMBER,          x,count); }
    bool syncreadmodelInformation    (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::MODEL_INFORMATION,     x,count); }
    bool syncreadversionOfFirmware   (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::VERSION_OF_FIRMWARE,   x,count); }
    bool syncreadid                  (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::ID,                    x,count); }
    bool syncreadbaudrate            (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::BAUDRATE,              x,count); }
    bool syncreadreturnDelayTime     (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::RETURN_DELAY_TIME,     x,count); }
    bool syncreaddriveMode           (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::DRIVE_MODE,            x,count); }
    bool syncreadoperatingMode       (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::OPERATING_MODE,        x,count); }
    bool syncreadsecondaryId         (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::SECONDARY_ID,          x,count); }
    bool syncreadprotocolVersion     (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::PROTOCOL_VERSION,      x,count); }
    bool syncreadhomingOffset        (uint8_t *id, int32_t *x,int count){ return syncread(id, Reg::HOMING_OFFSET,         x,count); }
    bool syncreadmovingThreshold     (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::MOVING_THRESHOLD,      x,count); }
    bool syncreadtemperatureLimit    (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::TEMPERATURE_LIMIT,     x,count); }
    bool syncreadmaxVoltageLimit     (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::MAX_VOLTAGE_LIMIT,     x,count); }
    bool syncreadminVoltageLimit     (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::MIN_VOLTAGE_LIMIT,     x,count); }
    bool syncreadpwmLimit            (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::PWM_LIMIT,             x,count); }
    bool syncreadcurrentLimit        (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::CURRENT_LIMIT,         x,count); }
    bool syncreadaccelerationLimit   (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::ACCELERATION_LIMIT,    x,count); }
    bool syncreadvelocityLimit       (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::VELOCITY_LIMIT,        x,count); }
    bool syncreadmaxPositionLimit    (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::MAX_POSITION_LIMIT,    x,count); }
    bool syncreadminPositionLimit    (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::MIN_POSITION_LIMIT,    x,count); }
    bool syncreadshutdown            (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::SHUTDOWN,              x,count); }
    bool syncreadtorqueEnable        (uint8_t *id,    bool *x,int count){ return syncread(id, Reg::TORQUE_ENABLE,         x,count); }
    bool syncreadled                 (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::LED,                   x,count); }
    bool syncreadstatusReturnLevel   (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::STATUS_RETURN_LEVEL,   x,count); }
    bool syncreadregisterdInstruction(uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::REGISTERED_INSTRUCTION,x,count); }
    bool syncreadhardwareErrorStatus (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::HARDWARE_ERROR_STATUS, x,count); }
    bool syncreadvelocityIGain       (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::VELOCITY_I_GAIN,       x,count); }
    bool syncreadvelocityPGain       (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::VELOCITY_P_GAIN,       x,count); }
    bool syncreadpositionDGain       (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::POSITION_D_GAIN,       x,count); }
    bool syncreadpositionIGain       (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::POSITION_I_GAIN,       x,count); }
    bool syncreadpositionPGain       (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::POSITION_P_GAIN,       x,count); }
    bool syncreadfeedForwardAccelerationGain(uint8_t *id, uint16_t *x,int count) { return syncread(id, Reg::FEEDFORWARD_ACCELERATION_GAIN,x,count); }
    bool syncreadfeedForwardVelocityGain(uint8_t *id, uint16_t *x,int count) { return syncread(id, Reg::FEEDFORWARD_VELOCITY_GAIN,x,count); }
    bool syncreadbusWatchdog         (uint8_t *id,  int8_t *x,int count){ return syncread(id, Reg::BUS_WATCHDOG,          x,count); }
    bool syncreadgoalPwm             (uint8_t *id, int16_t *x,int count){ return syncread(id, Reg::GOAL_PWM,              x,count); }
    bool syncreadgoalCurrent         (uint8_t *id, int16_t *x,int count){ return syncread(id, Reg::GOAL_CURRENT,          x,count); }
    bool syncreadgoalVelocity        (uint8_t *id, int32_t *x,int count){ return syncread(id, Reg::GOAL_VELOCITY,         x,count); }
    bool syncreadprofileAcceleration (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::PROFILE_ACCELERATION,  x,count); }
    bool syncreadprofileVelocity     (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::PROFILE_VELOCITY,      x,count); }
    bool syncreadgoalPosition        (uint8_t *id, int32_t *x,int count){ return syncread(id, Reg::GOAL_POSITION,         x,count); }
    bool syncreadrealTimeTick        (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::REALTIME_TICK,         x,count); }
    bool syncreadmoving              (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::MOVING,                x,count); }
    bool syncreadmovingStatus        (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::MOVING_STATUS,         x,count); }
	bool syncreadpresentPwm          (uint8_t *id, int16_t *x,int count){ return syncread(id, Reg::PRESENT_PWM,           x,count); }
    bool syncreadpresentCurrent      (uint8_t *id, int16_t *x,int count){ return syncread(id, Reg::PRESENT_CURRENT,       x,count); }
    bool syncreadpresentVelocity     (uint8_t *id, int32_t *x,int count){ return syncread(id, Reg::PRESENT_VELOCITY,      x,count); }
	bool syncreadpresentPosition     (uint8_t *id, int32_t *x,int count){ return syncread(id, Reg::PRESENT_POSITION,      x,count); }
	bool syncreadvelocityTrajectory  (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::VELOCITY_TRAJECTORY,   x,count); }
    bool syncreadpositionTrajectory  (uint8_t *id,uint32_t *x,int count){ return syncread(id, Reg::POSITION_TRAJECTORY,   x,count); }
    bool syncreadpresentInputVoltage (uint8_t *id,uint16_t *x,int count){ return syncread(id, Reg::PRESENT_INPUT_VOLTAGE, x,count); }
    bool syncreadpresentTemperature  (uint8_t *id, uint8_t *x,int count){ return syncread(id, Reg::PRESENT_TEMPERATURE,   x,count); }
	
    // write values
    bool id(uint8_t id, uint8_t x) { return write(id, Reg::ID, x); }
    bool baudrate(uint8_t id, uint8_t x) { return write(id, Reg::BAUDRATE, x); }
    bool returnDelayTime(uint8_t id, uint8_t x) { return write(id, Reg::RETURN_DELAY_TIME, x); }
    bool driveMode(uint8_t id, uint8_t x) { return write(id, Reg::DRIVE_MODE, x); }
    bool operatingMode(uint8_t id, uint8_t x) { return write(id, Reg::OPERATING_MODE, x); }
    bool secondaryId(uint8_t id, uint8_t x) { return write(id, Reg::SECONDARY_ID, x); }
    bool protocolVersion(uint8_t id, uint8_t x) { return write(id, Reg::PROTOCOL_VERSION, x); }
    bool homingOffset(uint8_t id, int32_t x) { return write(id, Reg::HOMING_OFFSET, x); }
    bool movingThreshold(uint8_t id, uint32_t x) { return write(id, Reg::MOVING_THRESHOLD, x); }
    bool temperatureLimit(uint8_t id, uint8_t x) { return write(id, Reg::TEMPERATURE_LIMIT, x); }
    bool maxVoltageLimit(uint8_t id, uint16_t x) { return write(id, Reg::MAX_VOLTAGE_LIMIT, x); }
    bool minVoltageLimit(uint8_t id, uint16_t x) { return write(id, Reg::MIN_VOLTAGE_LIMIT, x); }
    bool pwmLimit(uint8_t id, uint16_t x) { return write(id, Reg::PWM_LIMIT, x); }
    bool currentLimit(uint8_t id, uint16_t x) { return write(id, Reg::CURRENT_LIMIT, x); }
    bool accelerationLimit(uint8_t id, uint32_t x) { return write(id, Reg::ACCELERATION_LIMIT, x); }
    bool velocityLimit(uint8_t id, uint32_t x) { return write(id, Reg::VELOCITY_LIMIT, x); }
    bool maxPositionLimit(uint8_t id, uint32_t x) { return write(id, Reg::MAX_POSITION_LIMIT, x); }
    bool minPositionLimit(uint8_t id, uint32_t x) { return write(id, Reg::MIN_POSITION_LIMIT, x); }
    bool shutdown(uint8_t id, uint8_t x) { return write(id, Reg::SHUTDOWN, x); }
    bool torqueEnable(uint8_t id, bool x) { return write(id, Reg::TORQUE_ENABLE, x); }
    bool led(uint8_t id, bool x) { return write(id, Reg::LED, x); }
    bool statusReturnLevel(uint8_t id, uint8_t x) { return write(id, Reg::STATUS_RETURN_LEVEL, x); }
    bool velocityIGain(uint8_t id, uint16_t x) { return write(id, Reg::VELOCITY_I_GAIN, x); }
    bool velocityPGain(uint8_t id, uint16_t x) { return write(id, Reg::VELOCITY_P_GAIN, x); }
    bool positionDGain(uint8_t id, uint16_t x) { return write(id, Reg::POSITION_D_GAIN, x); }
    bool positionIGain(uint8_t id, uint16_t x) { return write(id, Reg::POSITION_I_GAIN, x); }
    bool positionPGain(uint8_t id, uint16_t x) { return write(id, Reg::POSITION_P_GAIN, x); }
    bool feedForwardAccelerationGain(uint8_t id, uint16_t x) { return write(id, Reg::FEEDFORWARD_ACCELERATION_GAIN, x); }
    bool feedForwardVelocityGain(uint8_t id, uint16_t x) { return write(id, Reg::FEEDFORWARD_VELOCITY_GAIN, x); }
    bool busWatchdog(uint8_t id, int8_t x) { return write(id, Reg::BUS_WATCHDOG, x); }
    bool goalPwm(uint8_t id, int16_t x) { return write(id, Reg::GOAL_PWM, x); }
    bool goalCurrent(uint8_t id, int16_t x) { return write(id, Reg::GOAL_CURRENT, x); }
    bool goalVelocity(uint8_t id, int32_t x) { return write(id, Reg::GOAL_VELOCITY, x); }
    bool profileAcceleration(uint8_t id, uint32_t x) { return write(id, Reg::PROFILE_ACCELERATION, x); }
    bool profileVelocity(uint8_t id, uint32_t x) { return write(id, Reg::PROFILE_VELOCITY, x); }
    bool goalPosition(uint8_t id, int32_t x) { return write(id, Reg::GOAL_POSITION, x); }
	
	// syncwrite values
	bool syncwritecid                (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::ID,                  x, count); }
    bool syncwritebaudrate           (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::BAUDRATE,            x, count); }
    bool syncwritereturnDelayTime    (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::RETURN_DELAY_TIME,   x, count); }
    bool syncwritedriveMode          (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::DRIVE_MODE,          x, count); }
    bool syncwriteoperatingMode      (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::OPERATING_MODE,      x, count); }
    bool syncwritesecondaryId        (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::SECONDARY_ID,        x, count); }
    bool syncwriteprotocolVersion    (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::PROTOCOL_VERSION,    x, count); }
    bool syncwritehomingOffset       (uint8_t *id, int32_t *x,int count){ return syncwrite(id, Reg::HOMING_OFFSET,       x, count); }
    bool syncwritemovingThreshold    (uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::MOVING_THRESHOLD,    x, count); }
    bool syncwritetemperatureLimit   (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::TEMPERATURE_LIMIT,   x, count); }
    bool syncwritemaxVoltageLimit    (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::MAX_VOLTAGE_LIMIT,   x, count); }
    bool syncwriteminVoltageLimit    (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::MIN_VOLTAGE_LIMIT,   x, count); }
    bool syncwritepwmLimit           (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::PWM_LIMIT,           x, count); }
	bool syncwritecurrentLimit       (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::CURRENT_LIMIT,       x, count); }
    bool syncwriteaccelerationLimit  (uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::ACCELERATION_LIMIT,  x, count); }
    bool syncwritevelocityLimit      (uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::VELOCITY_LIMIT,      x, count); }
    bool syncwritemaxPositionLimit   (uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::MAX_POSITION_LIMIT,  x, count); }
    bool syncwriteminPositionLimit   (uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::MIN_POSITION_LIMIT,  x, count); }
    bool syncwriteshutdown           (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::SHUTDOWN,            x, count); }
	bool syncwritetorqueEnable       (uint8_t *id,    bool *x,int count){ return syncwrite(id, Reg::TORQUE_ENABLE,       x, count); }
	bool syncwriteled                (uint8_t *id,    bool *x,int count){ return syncwrite(id, Reg::LED,                 x, count); }
    bool syncwritestatusReturnLevel  (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::STATUS_RETURN_LEVEL, x, count); }
    bool syncwritevelocityIGain      (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::VELOCITY_I_GAIN,     x, count); }
    bool syncwritevelocityPGain      (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::VELOCITY_P_GAIN,     x, count); }
    bool syncwritepositionDGain      (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::POSITION_D_GAIN,     x, count); }
    bool syncwritepositionIGain      (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::POSITION_I_GAIN,     x, count); }
    bool syncwritepositionPGain      (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::POSITION_P_GAIN,     x, count); }
    bool syncwritefeedForwardAccelerationGain(uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::FEEDFORWARD_ACCELERATION_GAIN, x, count); }
    bool syncwritefeedForwardVelocityGain    (uint8_t *id,uint16_t *x,int count){ return syncwrite(id, Reg::FEEDFORWARD_VELOCITY_GAIN,     x, count); }
    bool syncwritebusWatchdog        (uint8_t *id, uint8_t *x,int count){ return syncwrite(id, Reg::BUS_WATCHDOG,        x, count); }
    bool syncwritegoalPwm            (uint8_t *id, int16_t *x,int count){ return syncwrite(id, Reg::GOAL_PWM,            x, count); }
	bool syncwritegoalCurrent        (uint8_t *id, int16_t *x,int count){ return syncwrite(id, Reg::GOAL_CURRENT,        x, count); }	
	bool syncwritegoalVelocity       (uint8_t *id, int32_t *x,int count){ return syncwrite(id, Reg::GOAL_VELOCITY,       x, count); }
	bool syncwriteprofileAcceleration(uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::PROFILE_ACCELERATION,x, count); }
    bool syncwriteprofileVelocity    (uint8_t *id,uint32_t *x,int count){ return syncwrite(id, Reg::PROFILE_VELOCITY,    x, count); }
	bool syncwritegoalPosition       (uint8_t *id, int32_t *x,int count){ return syncwrite(id, Reg::GOAL_POSITION,       x, count); }
	
	bool syncwrite(uint8_t *id, Reg reg,bool *x,int count){
		uint32_t data32[count];
		for(int label=0;label<count;label++) data32[label]=(uint32_t)x[label];
		return syncwrite(id,reg,data32,count);
	}
	bool syncwrite(uint8_t *id, Reg reg,uint8_t *x,int count){
		uint32_t data32[count];
		for(int label=0;label<count;label++) data32[label]=(uint32_t)x[label];
		return syncwrite(id,reg,data32,count);
	}
	bool syncwrite(uint8_t *id, Reg reg,uint16_t *x,int count){
		uint32_t data32[count];
		for(int label=0;label<count;label++) data32[label]=(uint32_t)x[label];
		return syncwrite(id,reg,data32,count);
	}
	bool syncwrite(uint8_t *id, Reg reg,int16_t *x,int count){
		uint32_t data32[count];
		for(int label=0;label<count;label++) data32[label]=(uint32_t)x[label];
		return syncwrite(id,reg,data32,count);
	}
	bool syncwrite(uint8_t *id, Reg reg,int32_t *x,int count){
		uint32_t data32[count];
		for(int label=0;label<count;label++) data32[label]=(uint32_t)x[label];
		return syncwrite(id,reg,data32,count);
	}
	
	bool syncwrite(uint8_t *id, Reg reg,uint32_t *x,int count){
		uint16_t addr = info[id[0]].ct->ct[reg].addr; //Address
		uint16_t size = info[id[0]].ct->ct[reg].size; //data_length
		uint8_t err=0; 
		set_address(addr,size);
		// int sizeof_array = count;//(sizeof(id)/sizeof(id[0])<=sizeof(x)/sizeof(x[0])) ? sizeof(id)/sizeof(id[0])-1 : sizeof(x)/sizeof(x[0])-1;
		for(int label=0;label<count;label++){
			if (info.find(id[label]) == info.end()){
				return false;
			}else{
				uint8_t param_write[4];
				param_write[0] = DXL_LOBYTE(DXL_LOWORD(x[label]));
				param_write[1] = DXL_HIBYTE(DXL_LOWORD(x[label]));
				param_write[2] = DXL_LOBYTE(DXL_HIWORD(x[label]));
				param_write[3] = DXL_HIBYTE(DXL_HIWORD(x[label]));
				if(add(id[label],param_write)==false) err++; 
			}
		}
		if(err==0){
			send();
			return true;
		}else{
			return false;
		}
		//return (err==0)? true : false;		
	}

	bool syncread(uint8_t *id, Reg reg, bool *x,int count){
		uint32_t xx[count];
		if(syncread(id,reg,xx,count)==true){
			for(int label=0;label<count;label++)
				x[label] = (bool)xx[label];
			return true;
		}else{
			return false;
		}
	}
	bool syncread(uint8_t *id, Reg reg, int8_t *x,int count){
		uint32_t xx[count];
		if(syncread(id,reg,xx,count)==true){
			for(int label=0;label<count;label++)
				x[label] = (int8_t)xx[label];
			return true;
		}else{
			return false;
		}
	}
	bool syncread(uint8_t *id, Reg reg, uint8_t *x,int count){
		uint32_t xx[count];
		if(syncread(id,reg,xx,count)==true){
			for(int label=0;label<count;label++)
				x[label] = (uint8_t)xx[label];
			return true;
		}else{
			return false;
		}
	}
	bool syncread(uint8_t *id, Reg reg, int16_t *x,int count){
		uint32_t xx[count];
		if(syncread(id,reg,xx,count)==true){
			for(int label=0;label<count;label++)
				x[label] = (int16_t)xx[label];
			return true;
		}else{
			return false;
		}
	}
	bool syncread(uint8_t *id, Reg reg, uint16_t *x,int count){
		uint32_t xx[count];
		if(syncread(id,reg,xx,count)==true){
			for(int label=0;label<count;label++)
				x[label] = (uint16_t)xx[label];
			return true;
		}else{
			return false;
		}
	}
	bool syncread(uint8_t *id, Reg reg, int32_t *x,int count){
		uint32_t xx[count];
		if(syncread(id,reg,xx,count)==true){
			for(int label=0;label<count;label++)
				x[label] = (int32_t)xx[label];
			return true;
		}else{
			return false;
		}
	}
	
	bool syncread(uint8_t *id, Reg reg, uint32_t *x,int count){
		uint16_t addr = info[id[0]].ct->ct[reg].addr; //Address
		uint16_t size = info[id[0]].ct->ct[reg].size; //data_length
		// uint8_t counter=0;
		set_target(addr,size);
		for(int label=0;label<count;label++){
			if (info.find(id[label]) == info.end()){
				return false;
			}else{
				add_id(id[label]);
			}
		}
		request();
		for(int label=0;label<count;label++){
			if(available(id[label])){
				x[label] = data(id[label]);
			}else{
				return false;
			}
		}
		return true;
	}
		
	int syncreadmultipledata(uint8_t *id, uint32_t *x,const uint16_t start_addr,const uint16_t *bytes ,const uint8_t data_count ,const uint8_t id_count){
		uint16_t sum_bytes=0;
		for(int index=0;index<data_count;index++)
			sum_bytes +=bytes[index];
		
		set_target(start_addr, sum_bytes);		
		for(int index=0;index<id_count;index++){
			if (info.find(id[index]) == info.end()){
				return false;
			}else{
				add_id(id[index]);
			}
		}
		request();
		for(int index=0;index<id_count;index++){
			if(available(id[index])){
				uint16_t sbyte=0;
				for(int index2=0;index2<data_count;index2++){
					x[index*data_count+index2]= sync_reader.getByteData(id[index],sbyte,bytes[index2]);
					sbyte +=bytes[index2];
				}
			}else{
				return id[index];
			}
		}
		return -1;
	}
	
		
private:
    bool handleResult(uint8_t id, uint8_t result, uint8_t error = 0, uint16_t model = 0) {
        bool b = true;
        if ((result != COMM_SUCCESS) || (error != 0)) b = false;
        saveResult(id, result, error, model);
        return b;
    }

    void saveResult(uint8_t id, int result, uint8_t error = 0, uint16_t model = 0) {
        if (id > ID_LIMIT) return;

        auto it = info.find(id);
        if (it == info.end()) return;

        it->second.result = result;
        it->second.error = error;
        if (model != 0) it->second.model = model;
    }

    void verboseResult(uint8_t id) {
        if (info[id].result != COMM_SUCCESS)
            Serial.println(packet_handler->getTxRxResult(info[id].result));
    }
    void verboseError(uint8_t id) {
        if (info[id].error != 0)
            Serial.println(packet_handler->getRxPacketError(info[id].error));
    }

//#if 0   // TODO: reg_write & action, indirect, sync, bulk
        bool reg_write(uint8_t id, uint16_t reg, uint16_t size, uint8_t* data)
        {
            // TODO: clamp value
            uint8_t error = 0;
            int result = packet_handler->regWriteTxRx(id, reg, size, data, &error);
            return handleResult(id, result, error);
        }
        bool action(uint8_t id)
        {
            int result = packet_handler->action(id);
            return handleResult(id, result);
        }
public:
        // for indirect address
        bool setIndirectAddress(uint8_t id, uint8_t nth, uint16_t addr)
        {
            if (nth > 128) return false;
            if (info.find(id) == info.end()) return false;
            uint16_t indirect_addr = info[id].ct->ct[Reg::INDIRECT_ADDR_1].addr + 2 * nth;
            uint8_t indirect_size = info[id].ct->ct[Reg::INDIRECT_ADDR_1].size;
            uint8_t error = 0;
            // int result = packet_handler->write2ByteTxRx(id, info[id].ct->ct[Reg::INDIRECT_ADDR_1].addr + 2 * nth, addr, &error);
            int result = packet_handler->writeBytesTxRx(id, indirect_addr, addr, indirect_size, &error);
            return handleResult(id, result, error);
        }
        bool setIndirectData(uint8_t id, uint8_t nth, uint8_t data)
        {
            uint8_t error = 0;
            // TODO: ayashii data size
            // int result = packet_handler->write2ByteTxRx(id, info[id].ct->ct[Reg::INDIRECT_DATA_1].addr + nth, data, &error);
            int result = packet_handler->writeBytesTxRx(id, info[id].ct->ct[Reg::INDIRECT_DATA_1].addr + nth, data, info[id].ct->ct[Reg::INDIRECT_DATA_1].size, &error);
            return handleResult(id, result, error);
        }
private:
        // for sync write
        void set_address(uint16_t addr, uint16_t size)
        {
            sync_writer.setAddress(addr, size);
        }
        bool add(uint8_t id, uint8_t* data)
        {
            // TODO: check if endian is ok or not
            int result = sync_writer.addParam(id, data);
            if (result != true) return false;
            return true;
        }
        bool send()
        {
            int result = sync_writer.txPacket();
            sync_writer.clearParam();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        // for sync read
        // TODO: duplicate name with sync write
        void set_target(uint16_t addr, uint16_t size)
        {
            sync_reader.setAddress(addr, size);
        }
        // TODO: varidic arguments...
        bool add_id(uint8_t id)
        {
            int result = sync_reader.addParam(id);
            if (result != true) return false;
            return true;
        }
        bool request()
        {
            int result = sync_reader.txRxPacket();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        bool available(uint8_t id)
        {
            return sync_reader.isAvailable(id);
        }
        uint32_t data(uint8_t id)
        {
            return sync_reader.getData(id);
        }		
        // for bulk write
        bool add_bulk_target(uint8_t id, uint16_t addr, uint16_t size, uint8_t* data)
        {
            return bulk_writer.addParam(id, addr, size, data);
        }
        bool bulk_write()
        {
            int result = bulk_writer.txPacket();
            bulk_writer.clearParam();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        // for bulk read
        bool add_bulk_read_target(uint8_t id, uint16_t addr, uint16_t size)
        {
            return bulk_reader.addParam(id, addr, size);
        }
        bool bulk_read_request()
        {
            int result = bulk_reader.txRxPacket();
            if (result != COMM_SUCCESS) return false;
            return true;
        }
        bool bulk_available(uint8_t id)
        {
            return bulk_reader.isAvailable(id);
        }
        uint32_t bulk_read_data(uint8_t id)
        {
            return bulk_reader.getData(id);
        }
//#endif  // TODO:
};

template <>
inline void Dynamixel::addModel<Model::PRO>(uint8_t id) {
    std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::PRO>::instance();
    info.emplace(id, Dynamixel::Info{sp, 0, 0, 0});
}
template <>
inline void Dynamixel::addModel<Model::X>(uint8_t id) {
    std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::X>::instance();
    info.emplace(id, Dynamixel::Info{sp, 0, 0, 0});
}
template <>
inline void Dynamixel::addModel<Model::MX>(uint8_t id) {
    std::shared_ptr<ControlTable> sp = ControlTableOfModel<Model::MX>::instance();
    info.emplace(id, Dynamixel::Info{sp, 0, 0, 0});
}

}  // namespace dynamixel
}  // namespace arduino

using Dynamixel = arduino::dynamixel::Dynamixel;
using DxlModel = arduino::dynamixel::Model;
using DxlReg = arduino::dynamixel::Reg;

#endif  // ARDUINO_DYNAMIXEL_IMPL_H
