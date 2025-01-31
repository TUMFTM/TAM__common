// Copyright 2024 Simon Hoffmann
#include "network_communication/udp_sender.h"

namespace tam::network {

UdpSender::UdpSender(const int destPort, const std::string& destIPAddress) {
    oldPort = ntohs(destPort);
    change_destination(destIPAddress, destPort);
    // create sender socket file descriptor
    if ( (sockfd = socket(servaddrs.at(0).sin_family, SOCK_DGRAM, 0)) < 0 ) {
        perror("cannot create socket");
        printf("%s:%i", destIPAddress.c_str(), destPort);
        exit(EXIT_FAILURE);
    }
}

UdpSender::UdpSender(const int destPort, const std::vector<std::string> destIPAddresses) {
    change_destination(destIPAddresses, destPort);
    auto addr_family = servaddrs.at(0).sin_family;
    auto dest = destIPAddresses.at(0);
    if ((sockfd = socket(addr_family, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket");
        printf("%s:%i", dest.c_str(), destPort);
        exit(EXIT_FAILURE);
    }
}

void UdpSender::change_destination(const std::string &destIPAddress, const int destPort) {
    servaddrs.clear();
    sockaddr_in servaddr;
    // fill servaddr with values
    inet_aton(destIPAddress.c_str(), &(servaddr.sin_addr)); // ip
    if (destPort >= 0) {
        servaddr.sin_port = htons(destPort); // port
        oldPort = servaddr.sin_port;
    }
    else
    {
        servaddr.sin_port = oldPort; // port
    }
    servaddr.sin_family = AF_INET;

    servaddrs.emplace_back(servaddr);
}

void UdpSender::change_destination(const std::vector<std::string>& destIPAddresses, const int destPort)
{
    servaddrs.clear();
    for(auto & destIPAddress : destIPAddresses)
    {
        sockaddr_in servaddr;
        // fill servaddr with values
        inet_aton(destIPAddress.c_str(), &(servaddr.sin_addr)); // ip
        if (destPort >= 0) {
            servaddr.sin_port = htons(destPort); // port
            oldPort = servaddr.sin_port;
        }
        else
        {
            servaddr.sin_port = oldPort; // port
        }
        servaddr.sin_family = AF_INET;

        servaddrs.emplace_back(servaddr);
    }
}

void UdpSender::print_connection_specs() {
    auto addr = servaddrs.at(0);
    printf("Destination IP Address %s\n", inet_ntoa(addr.sin_addr));
    auto port = ntohs(addr.sin_port);
    printf("Destination Port %i\n", port);
}
}; //namespace tam::network
