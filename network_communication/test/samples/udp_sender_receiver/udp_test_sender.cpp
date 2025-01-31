// Server side implementation of UDP client-server model
#include <iostream>
#include "network_communication/udp_sender.h"

#define PORT    8080

int main() {

// --------------------- Example udpsender.sendCfgMsg --------------------------
    std::string IP{"127.0.0.1"};
    tam::network::UdpSender udpSender(PORT, IP);
    udpSender.print_connection_specs();

    while (true) {
        std::vector<uint8_t> data{1,2,3,4,5,6,7,8,9, 10, 11};

        std::cout << "Data sent: " << std::endl;
        for (auto ele : data) {std::cout << int(ele) << " ";}; std::cout << std::endl;

        int sent = udpSender.send_data(data);

        usleep(1000000);
    }

    return 0;

}