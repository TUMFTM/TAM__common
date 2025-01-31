// Copyright 2024 Simon Hoffmann
#include <rclcpp/rclcpp.hpp>
#include "network_communication/udp_receiver.h"

namespace tam::network {

UdpReceiver::UdpReceiver(const int destPort)
    : anti_block_thread{}, anti_block_sender(destPort) {
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    servaddr.sin_addr.s_addr = INADDR_ANY;
    // creating socket file descriptor
    if ( (sockfd = socket(servaddr.sin_family, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }
    // bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) {
        perror("bind failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }
    anti_block_thread = std::thread(&UdpReceiver::send_data_for_anti_block, this);
}

void UdpReceiver::send_data_for_anti_block() {
    if (!rclcpp::ok()) {
        printf("\033[31m ERROR @ UdpReceiver::send_data_for_anti_block() port %i: "
               " UDP-RECEIVER NEEDS TO BE initialized AFTER ros::init() \033[0m \n", servaddr.sin_port);
    }

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // free block of receiver
    int integer { 0 };
    for (int it = 0; it != 20; ++it)
        anti_block_sender.send((char *) &integer, sizeof(integer));
}

int UdpReceiver::wait_for_udp_receiver_to_close() {
    anti_block_thread.join();
    return 0;
}
}; // namespace tod_network
