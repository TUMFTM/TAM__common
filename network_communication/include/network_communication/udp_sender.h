// Copyright 2024 Simon Hoffmann
#pragma once
#include <arpa/inet.h>
#include "connection_configs.h"
#include <unistd.h>
#include <rclcpp/serialization.hpp>
#include "base_protocol.h"
#include <vector>
#include <string>

namespace tam::network {

class UdpSender : public BaseSender{
public:
    explicit UdpSender(const int destPort, const std::string& destIPAddress = "127.0.0.1");
    UdpSender(const int destPort, const std::vector<std::string> destIPAddresses);
    ~UdpSender() { close(sockfd); }
    UdpSender(UdpSender&&) = default; // required due to user-defined dtor
    UdpSender& operator=(UdpSender&&) = default;
    UdpSender(const UdpSender&) = default; // required due to user-defined move ctor
    UdpSender& operator=(const UdpSender&) = default;

    int send_data(std::vector<uint8_t>& data) override {
        int res = 0;
        for(const auto & addr : servaddrs)
        {
            res = sendto(sockfd, data.data(), data.size(), 0,
                      (struct sockaddr *)&addr, sizeof(addr));
            // https://linux.die.net/man/2/sendto
            if(res == -1)
            {
                break;
            }
        }
        return res;
    }

    void change_destination(const std::string& destIPAddress, const int destPort = -1) override;
    void change_destination(const std::vector<std::string>& destIPAddresses, const int destPort = -1) override;

    int send(const char *msg, size_t size) {
        int res = 0;
        for(const auto & addr : servaddrs)
        {
            res = sendto(sockfd, msg, size, 0, (struct sockaddr *)&addr, sizeof(addr));
            // https://linux.die.net/man/2/sendto
            if(res == -1)
            {
                break;
            }
        }
        return res;
    }

    void print_connection_specs();
    std::string get_destination_ip() override {
        std::string list = "[";
        for (size_t i = 0; i < servaddrs.size(); i++)
        {
            list += std::string(inet_ntoa(servaddrs.at(i).sin_addr));
            if(i < servaddrs.size() - 1)
            {
                list += ",";
            }
        }
        return list + "]";
    }

private:
    std::vector<sockaddr_in> servaddrs;
    int sockfd;
    in_port_t oldPort;
};
}; // namespace tod_network
