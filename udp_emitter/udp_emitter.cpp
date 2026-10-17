#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#define IP_TO_SEND "192.168.50.196"

int main() {
    int sock;
    struct sockaddr_in server_addr;
    const char *message = "Hello World";

    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return 1;
    }

    // Set up the server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8080);  // Port number
    server_addr.sin_addr.s_addr = inet_addr(IP_TO_SEND);  // Receiver IP

    // Send the message
    while(1){
    if (sendto(sock, message, strlen(message), 0,
               (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Send failed");
        close(sock);
        return 1;
    }
	}
    std::cout << "Message sent: " << message << std::endl;

    close(sock);
    return 0;
}
