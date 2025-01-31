#pragma once
#include <vector>
#include <stdint.h>
#include <string>

namespace tam::network {

class BaseReceiver {
public:
    BaseReceiver() = default;
    virtual ~BaseReceiver() = default;
    BaseReceiver(BaseReceiver&&) = default; // required due to user-defined dtor
    BaseReceiver& operator=(BaseReceiver&&) = default;
    BaseReceiver(const BaseReceiver&) = default; // required due to user-defined move ctor
    BaseReceiver& operator=(const BaseReceiver&) = default;

    virtual std::vector<uint8_t> receive() = 0;
    virtual bool waiting_for_client_connect() {return true;};
    virtual void disconnect() { };
private:
};


class BaseSender {
public:
    BaseSender() = default;
    virtual ~BaseSender() = default;
    BaseSender(BaseSender&&) = default; // required due to user-defined dtor
    BaseSender& operator=(BaseSender&&) = default;
    BaseSender(const BaseSender&) = default; // required due to user-defined move ctor
    BaseSender& operator=(const BaseSender&) = default;

    virtual int send_data(std::vector<uint8_t>& data) = 0;
    virtual void change_destination(const std::string& destIPAddress, const int destPort = -1)= 0;
    virtual void change_destination(const std::vector<std::string>& destIPAddresses, const int destPort = -1)= 0;
    virtual std::string get_destination_ip() = 0;
private:

};
}; //namespace tam::network
