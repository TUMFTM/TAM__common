// Copyright 2024 Simon Hoffmann
#pragma once
#include <arpa/inet.h>
#include <thread>
#include "udp_sender.h"
#include <cstring> // std::memcpy
#include <iostream>
#include <vector>
#include "base_protocol.h"

#define MAXLINE 100*1024

namespace tam::network {

class UdpReceiver : public BaseReceiver{
public:
    explicit UdpReceiver(const int destPort);
    ~UdpReceiver() { close(sockfd); }

    std::vector<uint8_t> receive() override {
        uint8_t buffer [MAXLINE];
        int num_bytes = recv(sockfd, &buffer, MAXLINE, MSG_WAITALL);
        std::vector<uint8_t> vec(buffer, buffer + num_bytes);
        return vec;
    }
    int wait_for_udp_receiver_to_close();

private:
    struct sockaddr_in servaddr;
    int sockfd;
    std::thread anti_block_thread;
    UdpSender anti_block_sender;

    void send_data_for_anti_block();
    };
}; //namespace tod_network
