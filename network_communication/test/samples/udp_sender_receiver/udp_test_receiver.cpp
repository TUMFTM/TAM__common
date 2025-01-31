// UDP Receiver for ROS messages
#include <iostream>
#include "network_communication/udp_receiver.h"

#define PORT    8080

int main() {

// --------------------- Example receiveROSMsg --------------------------
    tam::network::UdpReceiver udpReceiver(PORT);

    while (true) {
        std::vector<uint8_t> rec = udpReceiver.receive();
        std::cout << "nofBytes: " << rec.size() << std::endl;
        std::cout << "Data out: " << (int)rec.at(2) << std::endl;
    }

    return 0;
}
