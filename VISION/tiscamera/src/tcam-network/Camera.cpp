/*
 * Copyright 2013 The Imaging Source Europe GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Camera.h"

#include "Firmware.h"
#include "utils.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <errno.h>
#include <future>
#include <ifaddrs.h>
#include <iostream>
#include <linux/if.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>

namespace tis
{

std::shared_ptr<Camera> getCameraFromList(const camera_list cameras,
                                          const std::string& identifier,
                                          camera_ident id_type)
{
    std::shared_ptr<Camera> rval;

    if (identifier.empty())
    {
        return nullptr;
    }

    std::function<bool(std::shared_ptr<Camera>)> f = [&identifier,
                                                      &id_type](std::shared_ptr<Camera> cam) {
        if (id_type == CAMERA_MAC)
        {
            if (cam->getMAC().compare(identifier) == 0)
            {
                return true;
            }
        }
        else if (id_type == CAMERA_IP)
        {
            if (cam->getCurrentIP().compare(identifier) == 0)
            {
                return true;
            }
        }
        else if (id_type == CAMERA_NAME)
        {
            if (cam->getUserDefinedName().compare(identifier) == 0)
            {
                return true;
            }
        }
        else
        {
            if (cam->getSerialNumber().compare(identifier) == 0)
            {
                return true;
            }
        }
        return false;
    };

    auto iter = find_if(cameras.begin(), cameras.end(), f);

    if (iter != cameras.end())
    {
        rval = *iter;
    }
    return rval;
}


Camera::Camera(const Packet::ACK_DISCOVERY& _packet,
               std::shared_ptr<NetworkInterface> _interface,
               int timeoutIntervals)
    : packet(_packet), interface(_interface), timeoutCounter(timeoutIntervals),
      timeoutCounterDefault(timeoutIntervals)
{
    this->socket = interface->createSocket();

    this->requestID = 1;
    this->isControlled = false;
}

Camera::~Camera()
{
    if (this->isControlled)
    {
        abandonControl();
    }
}


std::shared_ptr<Socket> Camera::getSocket()
{
    return this->socket;
}


int Camera::generateRequestID()
{
    this->requestID++;
    if (this->requestID == 0)
    {
        this->requestID++;
    }

    return this->requestID;
}


int Camera::reduceCounter()
{
    timeoutCounter--;
    return timeoutCounter;
}


int Camera::resetCounter()
{
    timeoutCounter = timeoutCounterDefault;
    return timeoutCounter;
}


std::string Camera::getNetworkInterfaceName()
{
    return interface->getInterfaceName();
}


void Camera::updateCamera(std::shared_ptr<Camera> cam)
{
    this->packet = cam->packet;
}


std::string Camera::getModelName()
{
    return this->packet.ModelName;
}


std::string Camera::getSerialNumber()
{
    return this->packet.Serialnumber;
}


std::string Camera::getVendorName()
{
    return this->packet.ManufacturerName;
}


std::string Camera::getUserDefinedName()
{
    return this->packet.UserDefinedName;
}


bool Camera::setUserDefinedName(const std::string& name)
{
    if (getControl())
    {
        char buf[16];
        strcpy(buf, name.substr(0, 15).c_str());
        return sendWriteMemory(Register::USER_DEFINED_NAME_REGISTER, sizeof(buf), &buf);
    }
    else
    {
        return false;
    }
}


const std::string Camera::getMAC()
{
    return int2mac(((uint64_t)ntohs(this->packet.DeviceMACHigh)) << 32
                   | ntohl(this->packet.DeviceMACLow));
}


const std::string Camera::getCurrentIP()
{
    return int2ip(this->packet.CurrentIP);
}


const std::string Camera::getCurrentSubnet()
{
    return int2ip(this->packet.CurrentSubnetMask);
}


const std::string Camera::getCurrentGateway()
{
    return int2ip(this->packet.DefaultGateway);
}


bool Camera::isStaticIPactive()
{
    uint32_t i = 0;
    sendReadMemory(Register::CURRENT_IPCFG_REGISTER, 4, &i);

    const int static_ip_bit = 0x0;

    if (ntohl(i) & (1 << (static_ip_bit)))
    {
        return true;
    }
    return false;
}


bool Camera::setStaticIPstate(const bool on)
{
    bool retv = false;
    if (getControl())
    {
        const int static_ip_bit = 0x00;
        const int dhcp_bit = 0x01;
        const int llaBit = 0x02;

        uint32_t data = ntohl(this->packet.IPConfigCurrent);
        if (on)
        {
            // set lla ip bit
            data |= (0x01 << llaBit);

            // deal with incompetent cameras
            if (isDHCPactive())
            {
                data |= (0x01 << dhcp_bit);
            }
            else
            {
                data &= ~(0x01 << dhcp_bit);
            }
            data |= (0x01 << static_ip_bit);
            data = ntohl(data);


            retv = this->sendWriteMemory(Register::CURRENT_IPCFG_REGISTER, 4, &data);
        }
        else
        {
            data |= (0x01 << llaBit);
            data &= ~(0x01 << static_ip_bit);
            if (isDHCPactive())
            {
                data |= (0x01 << dhcp_bit);
            }
            else
            {
                data &= ~(0x01 << dhcp_bit);
            }
            data = ntohl(data);

            retv = this->sendWriteMemory(Register::CURRENT_IPCFG_REGISTER, 4, &data);
        }
        abandonControl();
    }
    return retv;
}


bool Camera::isDHCPactive()
{
    uint32_t i = 0;
    sendReadMemory(Register::CURRENT_IPCFG_REGISTER, 4, &i);

    const int dhcpBit = 0x01;
    if (htonl(i) & (1 << (dhcpBit)))
    {
        return true;
    }

    return false;
}


bool Camera::setDHCPstate(const bool on)
{
    bool retv = false;
    if (getControl())
    {
        const int static_ip_bit = 0x00;
        const int dhcpBit = 0x01;
        const int llaBit = 0x02;
        uint32_t data = ntohl(this->packet.IPConfigCurrent);

        if (on)
        {
            data |= (0x01 << dhcpBit);
            data |= (0x01 << llaBit);
            if (isStaticIPactive())
            {
                data |= (0x01 << static_ip_bit);
            }
            else
            {
                data &= ~(0x01 << static_ip_bit);
            }
            data = htonl(data);
            retv = this->sendWriteMemory(Register::CURRENT_IPCFG_REGISTER, 4, &data);
        }
        else
        {
            data &= ~(0x01 << dhcpBit);
            data |= (0x01 << llaBit);
            if (isStaticIPactive())
            {
                data |= (0x01 << static_ip_bit);
            }
            else
            {
                data &= ~(0x01 << static_ip_bit);
            }
            data = htonl(data);
            retv = this->sendWriteMemory(Register::CURRENT_IPCFG_REGISTER, 4, &data);
        }
        abandonControl();
    }
    return retv;
}


bool Camera::setIPconfigState(const bool dhcp, const bool staticIP)
{
    bool retv = false;
    if (getControl())
    {
        const int static_ip_bit = 0x00;
        const int dhcpBit = 0x01;
        const int llaBit = 0x02;
        uint32_t data = ntohl(this->packet.IPConfigCurrent);

        data |= (0x01 << llaBit); // always on

        if (dhcp)
        {
            data |= (0x01 << dhcpBit);
        }
        else
        {
            data &= ~(0x01 << dhcpBit);
        }

        if (staticIP)
        {
            data |= (0x01 << static_ip_bit);
        }
        else
        {
            data &= ~(0x01 << static_ip_bit);
        }

        data = htonl(data);
        retv = this->sendWriteMemory(Register::CURRENT_IPCFG_REGISTER, 4, &data);

        abandonControl();
    }
    return retv;
}


const std::string Camera::getPersistentIP()
{
    uint32_t ip = 0;

    if (sendReadMemory(Register::PERSISTANT_IPADDRESS_REGISTER, 4, &ip))
    {
        return int2ip(ip);
    }
    else
    {
        return "";
    }
}


bool Camera::setPersistentIP(const std::string& ip)
{
    bool retv = false;
    if (getControl())
    {
        uint32_t data = ip2int(ip);
        retv = this->sendWriteMemory(Register::PERSISTANT_IPADDRESS_REGISTER, 4, &data);
        abandonControl();
    }
    return retv;
}


const std::string Camera::getPersistentSubnet()
{
    uint32_t ip;
    if (sendReadMemory(Register::PERSISTANT_SUBNETMASK_REGISTER, 4, &ip))
    {
        return int2ip(ip);
    }
    else
    {
        return "";
    }
}


bool Camera::setPersistentSubnet(const std::string& subnet)
{
    bool retv = false;
    if (getControl())
    {
        uint32_t data = ip2int(subnet);
        retv = this->sendWriteMemory(Register::PERSISTANT_SUBNETMASK_REGISTER, 4, &data);
        abandonControl();
    }
    return retv;
}


const std::string Camera::getPersistentGateway()
{
    uint32_t ip;
    if (sendReadMemory(Register::PERSISTANT_DEFAULTGATEWAY_REGISTER, 4, &ip))
    {
        return int2ip(ip);
    }
    else
    {
        return "";
    }
}


bool Camera::setPersistentGateway(const std::string& gateway)
{
    bool retv = false;
    if (getControl())
    {
        uint32_t data = ip2int(gateway);
        retv = this->sendWriteMemory(Register::PERSISTANT_DEFAULTGATEWAY_REGISTER, 4, &data);
        abandonControl();
    }
    return retv;
}


bool Camera::forceIP(const std::string& ip, const std::string& subnet, const std::string& gateway)
{
    uint32_t _ip = ip2int(ip);
    uint32_t _subnet = ip2int(subnet);
    uint32_t _gateway = ip2int(gateway);

    bool returnVal = true;

    this->sendForceIP(_ip, _subnet, _gateway);

    // we want to reset to ip configuration
    // and will not get an ack paket
    // assume everything is ok
    if (_ip == 0 && _subnet == 0 && _gateway == 0)
    {
        returnVal = true;
    }

    return returnVal;
}


std::string Camera::getFirmwareVersion()
{
    return this->packet.DeviceVersion;
}


int Camera::uploadFirmware(const std::string& filename,
                           const std::string& overrideModelName,
                           std::function<void(int, const std::string&)> progressFunc)
{
    FirmwareUpdate::Status retv = FirmwareUpdate::Status::DeviceAccessFailed;
    FwdFirmwareWriter writer = FwdFirmwareWriter(*this);

    if (getControl())
    {
        // retrieve current heartbeat and set our own
        // some cameras run into timeouts
        auto dev_timeout = getHeartbeatTimeout();

        if (dev_timeout == -1)
        {
            abandonControl();
            return -1;
        }

        if (!setHeartbeatTimeout(10000)) // 10 seconds
        {
            abandonControl();
            return -1;
        }

        // actual writing
        retv = upgradeFirmware(writer, this->packet, filename, overrideModelName, progressFunc);

        // reset heartbeat
        setHeartbeatTimeout(dev_timeout);

        abandonControl();
    }

    return (int)retv;
}


std::string Camera::getInterfaceName()
{
    return interface->getInterfaceName();
}


int Camera::getHeartbeatTimeout()
{
    bool retv = false;
    uint32_t data;
    try
    {
        retv = this->sendReadRegister(Register::HEARTBEAT_TIMEOUT_REGISTER, &data);
    }
    catch (const std::exception& exc)
    {
        std::cerr << exc.what() << std::endl;
    }

    if (retv)
    {
        return data;
    }
    return -1;
}


bool Camera::setHeartbeatTimeout(uint32_t timeout)
{
    bool retv = false;
    if (!isControlled)
    {
        if (!this->getControl())
        {
            return false;
        }
    }

    uint32_t data = timeout;
    try
    {
        retv = this->sendWriteRegister(Register::HEARTBEAT_TIMEOUT_REGISTER, data);
    }
    catch (const std::exception& exc)
    {
        std::cerr << exc.what() << std::endl;
    }

    return retv;
}


bool Camera::getControl()
{
    bool retv = false;
    if (!isControlled)
    {
        try
        {
            uint32_t data = 2;
            retv = this->sendWriteRegister(Register::CONTROLCHANNEL_PRIVELEGE_REGISTER, data);
        }
        catch (const std::exception& exc)
        {
            std::cerr << exc.what() << std::endl;
        }

        if (retv)
        {
            retv = true;
            this->isControlled = true;
        }
    }
    else
    {
        return this->isControlled;
    }

    return retv;
}


bool Camera::abandonControl()
{
    bool retv = false;
    retv = this->sendWriteRegister(Register::CONTROLCHANNEL_PRIVELEGE_REGISTER, 0);
    if (retv)
    {
        this->isControlled = false;
    }
    return retv;
}

bool Camera::getIsBusy()
{
    if (isControlled)
        return false;

    bool retv = true;
    if (getControl())
    {
        retv = false;
        abandonControl();
    }
    return retv;
}


void Camera::resetIP()
{
    abandonControl();
    this->sendForceIP(0, 0, 0);
}


bool Camera::isReachable()
{
    bool retv = false;

    uint32_t pc_mask = interface->getInterfaceNetmask();
    uint32_t pc_addr = interface->getInterfaceIP();
    uint32_t cam_mask = this->packet.CurrentSubnetMask;
    uint32_t cam_ip = this->packet.CurrentIP;

    if ((pc_addr & pc_mask) == (cam_ip & cam_mask))
    {
        retv = true;
    }
    return retv;
}


bool Camera::sendWriteRegister(const uint32_t address, uint32_t value)
{
    unsigned int response = Status::TIMEOUT;

    unsigned short id = generateRequestID();
    size_t real_size = sizeof(Packet::CMD_WRITEREG);

    std::vector<uint8_t> packet_(real_size);
    auto _packet = (Packet::CMD_WRITEREG*)packet_.data();

    _packet->header.magic = 0x42;
    _packet->header.flag = Flags::NEEDACK;
    _packet->header.command = htons(Commands::WRITEREG_CMD);
    _packet->header.length = htons(sizeof(Packet::CMD_WRITEREG) - sizeof(Packet::COMMAND_HEADER));
    _packet->header.req_id = htons(id);

    _packet->ops[0].address = htonl(address);
    _packet->ops[0].value = htonl(value);

    auto callback_function = [id, &response](void* msg) -> int {
        Packet::ACK_WRITEREG* ack = (Packet::ACK_WRITEREG*)msg;
        response = Status::FAILURE;

        if (ntohs(ack->header.ack_id) == id)
        {
            response = ntohs(ack->header.status);
            return Socket::SendAndReceiveSignals::END;
        }
        return Socket::SendAndReceiveSignals::CONTINUE;
    };

    try
    {
        int retries = tis::PACKET_RETRY_COUNT;
        while (retries && (response == Status::TIMEOUT))
        {
            socket->sendAndReceive(getCurrentIP(), _packet, real_size, callback_function);
            retries--;
        }
    }
    catch (SocketSendToException& exc)
    {
        std::cerr << exc.what() << std::endl;
    }

    return response == Status::SUCCESS;
}


bool Camera::sendReadRegister(const uint32_t address, uint32_t* value)
{

    if (!value)
    {
        return false;
    }

    unsigned int response = Status::TIMEOUT;

    unsigned short id = generateRequestID();

    Packet::CMD_READREG _packet = Packet::CMD_READREG();

    _packet.header.magic = 0x42;
    _packet.header.flag = Flags::NEEDACK;
    _packet.header.command = htons(Commands::READREG_CMD);
    _packet.header.length = htons(sizeof(Packet::CMD_READREG) - sizeof(Packet::COMMAND_HEADER));
    _packet.header.req_id = ntohs(id);

    _packet.address[0] = htonl(address);

    auto callback_function = [&id, &value, &response](void* msg) -> int {
        Packet::ACK_READREG* ack = (Packet::ACK_READREG*)msg;

        response = Status::FAILURE;

        if (ntohs(ack->header.ack_id) == id)
        {
            if (ack->header.status == Status::SUCCESS)
            {
                memcpy(value, ack->data, sizeof(uint32_t));
            }
            response = ntohs(ack->header.status);
            *value = ntohl(*value);
            return Socket::SendAndReceiveSignals::END;
        }
        return Socket::SendAndReceiveSignals::CONTINUE;
    };

    try
    {
        int retries = tis::PACKET_RETRY_COUNT;
        while (retries && (response == Status::TIMEOUT))
        {
            socket->sendAndReceive(getCurrentIP(), &_packet, sizeof(_packet), callback_function);
            retries--;
        }
    }
    catch (SocketSendToException& exc)
    {
        std::cerr << exc.what() << std::endl;
    }

    return response == Status::SUCCESS;
}


bool Camera::sendReadMemory(const uint32_t address, const uint32_t size, void* data)
{
    if ((size % 4) != 0)
    {
        return false;
    }

    unsigned int response = Status::TIMEOUT;

    unsigned short id = generateRequestID();

    Packet::CMD_READMEM _packet = Packet::CMD_READMEM();

    _packet.header.magic = 0x42;
    _packet.header.flag = Flags::NEEDACK;
    _packet.header.command = htons(Commands::READMEM_CMD);
    _packet.header.length = htons(sizeof(Packet::CMD_READMEM) - sizeof(Packet::COMMAND_HEADER));
    _packet.header.req_id = ntohs(id);

    _packet.count = htons(size);
    _packet.address = htonl(address);

    auto callback_function = [&data, &id, &size, &response](void* msg) -> int {
        Packet::ACK_READMEM* ack = (Packet::ACK_READMEM*)msg;

        response = Status::FAILURE;

        if (ntohs(ack->header.ack_id) == id)
        {
            if (ack->header.status == Status::SUCCESS)
            {
                memcpy(data, ack->data, size);
            }
            response = ntohs(ack->header.status);
            return Socket::SendAndReceiveSignals::END;
        }
        return Socket::SendAndReceiveSignals::CONTINUE;
    };

    try
    {
        int retries = tis::PACKET_RETRY_COUNT;
        while (retries && (response == Status::TIMEOUT))
        {
            socket->sendAndReceive(getCurrentIP(), &_packet, sizeof(_packet), callback_function);
            retries--;
        }
    }
    catch (SocketSendToException& exc)
    {
        std::cerr << exc.what() << std::endl;
    }

    return response == Status::SUCCESS;
}


bool Camera::sendWriteMemory(const uint32_t address, const size_t size, void* data)
{
    if ((size % 4) != 0)
    {
        return false;
    }

    unsigned int response = Status::TIMEOUT;

    unsigned short id = generateRequestID();
    size_t real_size = sizeof(Packet::CMD_WRITEMEM) + (size - sizeof(uint32_t));

    std::vector<uint8_t> packet_(real_size);
    auto _packet = (Packet::CMD_WRITEMEM*)packet_.data();

    _packet->header.magic = 0x42;
    _packet->header.flag = Flags::NEEDACK;
    _packet->header.command = htons(Commands::WRITEMEM_CMD);
    _packet->header.length = htons(size + sizeof(uint32_t));
    _packet->header.req_id = htons(id);

    memcpy(&_packet->data, data, size);
    _packet->address = htonl(address);

    auto callback_function = [id, &response](void* msg) -> int {
        Packet::ACK_WRITEMEM* ack = (Packet::ACK_WRITEMEM*)msg;
        response = Status::FAILURE;

        if (ntohs(ack->header.ack_id) == id)
        {
            response = ntohs(ack->header.status);
            return Socket::SendAndReceiveSignals::END;
        }
        return Socket::SendAndReceiveSignals::CONTINUE;
    };

    try
    {
        int retries = tis::PACKET_RETRY_COUNT;
        while (retries && (response == Status::TIMEOUT))
        {
            socket->sendAndReceive(getCurrentIP(), _packet, real_size, callback_function);
            retries--;
        }
    }
    catch (SocketSendToException& exc)
    {
        std::cerr << exc.what() << std::endl;
    }

    if (response == Status::ACCESS_DENIED)
    {
        std::cout << "Unable to write. Access Denied." << std::endl;
    }

    return response == Status::SUCCESS;
}


void Camera::sendForceIP(const uint32_t ip, const uint32_t subnet, const uint32_t gateway)
{
    unsigned short id = generateRequestID();
    auto s = getSocket();

    Packet::CMD_FORCEIP _packet;

    _packet.header.magic = 0x42;
    _packet.header.flag = Flags::NEEDACK;
    _packet.header.command = htons(Commands::FORCEIP_CMD);
    _packet.header.length = htons(sizeof(Packet::CMD_FORCEIP) - sizeof(Packet::COMMAND_HEADER));
    _packet.header.req_id = id;

    _packet.DeviceMACHigh = this->packet.DeviceMACHigh;
    _packet.DeviceMACLow = this->packet.DeviceMACLow;

    _packet.StaticIP = ip;
    _packet.StaticSubnetMask = subnet;
    _packet.StaticGateway = gateway;

    try
    {
        s->sendAndReceive("255.255.255.255", &_packet, sizeof(_packet), NULL, true);
    }
    catch (SocketSendToException& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

} /* namespace tis */
