#pragma once

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <vector>

#include <netinet/in.h>
#include <arpa/inet.h>

template <typename Type>
class UDP_server
{
private:
    int port_num;
    int sockfd;
    socklen_t addrlen;
    struct sockaddr_in addr, cliaddr;
    Type data_to_be_exchanged;
public:
    UDP_server(int PORT)
    {
        this->port_num = PORT;

        if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        {
            return;
        }

        memset((void *)&addr, 0x00, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(this->port_num);

        addrlen = sizeof(addr);
        if(bind(sockfd, (struct sockaddr *)&addr, addrlen) == -1)
        {
            return;
        }

    }

    int ReceiveData()
    {
        // char buf[BUFSIZ];
        int n;
        n = recvfrom(sockfd, (void *)&data_to_be_exchanged, sizeof(data_to_be_exchanged), 0, (struct sockaddr *)&cliaddr, &addrlen);
        if(n == -1)
        {
            printf("recvfrom error\n");
            return -1;
        }
        return 0;
    }

    int SendData(const Type &data_to_be_exchanged_args)
    {
        data_to_be_exchanged = data_to_be_exchanged_args;
        int n;
        n = sendto(sockfd, (void *)&data_to_be_exchanged, sizeof(data_to_be_exchanged), 0, (struct sockaddr *)&cliaddr, addrlen);
        if(n == -1)
        {
            printf("sendto error\n");
            return -1;
        }
        return 0;
    }

    Type GetData()
    {
        return data_to_be_exchanged;
    }
    
    Type ReceiveAndGetData()
    {
    	ReceiveData();
    	Type data = GetData();
    	return data;
    }

    int ReceiveAndGetArray(double* buffer, size_t size_of_buffer)
    {
        int n = recvfrom(sockfd, buffer, size_of_buffer, 0, (struct sockaddr *)&cliaddr, &addrlen);
        if(n == -1)
        {
            printf("recvfrom error\n");
            return -1;
        }
        return n; // 받은 바이트 수
    }

    int SendArray(const double* buffer, size_t size_of_buffer)
    {
        int n = sendto(sockfd, buffer, size_of_buffer, 0, (struct sockaddr *)&cliaddr, addrlen);
        if(n == -1)
        {
            printf("sendto error\n");
            return -1;
        }
        return n; // 보낸 바이트 수
    }

    // 클라이언트로부터 문자열 수신
    int ReceiveString(std::string &s, size_t max_len) {
        std::vector<char> buf(max_len);
        addrlen = sizeof(cliaddr);
        int n = recvfrom(
            sockfd,
            buf.data(), buf.size(),
            0,
            (struct sockaddr *)&cliaddr, &addrlen
        );
        if(n == -1) { perror("recvfrom"); return -1; }
        s.assign(buf.data(), n);
        return n;  // 받은 바이트 수
    }

    // 클라이언트로 문자열 전송
    int SendString(const std::string &s) {
        addrlen = sizeof(cliaddr);
        int n = sendto(
            sockfd,
            s.data(), s.size(),
            0,
            (struct sockaddr *)&cliaddr, addrlen
        );
        if(n == -1) { perror("sendto"); return -1; }
        return n;  // 보낸 바이트 수
    }


};
