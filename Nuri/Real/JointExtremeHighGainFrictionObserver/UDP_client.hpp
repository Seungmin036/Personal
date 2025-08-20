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
class UDP_client
{
private:
    int port_num;
    int sockfd;
    socklen_t addrlen;
    struct sockaddr_in addr;
    std::string ipaddress;
    Type data_to_be_exchanged;
    Type recvaddr;
public:
    UDP_client(int PORT, std::string IP)
    {
        this->port_num = PORT;
        this->ipaddress = IP;
        this->addrlen = 0;

        if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1 )
        {
            return;
        }

        memset((void *)&addr, 0x00, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(this->ipaddress.c_str());
        addr.sin_port = htons(this->port_num);
    }

    int ReceiveData()
    {
        // char buf[BUFSIZ];
        int n;
        n = recvfrom(sockfd, (void *)&data_to_be_exchanged, sizeof(data_to_be_exchanged), 0, (struct sockaddr *)&recvaddr, &addrlen);
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
        addrlen = sizeof(addr);
        int n;
        n = sendto(sockfd, (void *)&data_to_be_exchanged, sizeof(data_to_be_exchanged), 0, (struct sockaddr *)&addr, addrlen);
        if(n == -1)
        {
            printf("sendto error\n");
            return -1;
        }
        return 0;
    }

    int ReceiveArray(double* buffer, size_t size_of_buffer) {
        int n = recvfrom(sockfd, buffer, size_of_buffer, 0, (struct sockaddr *)&recvaddr, addrlen);
        if (n == -1) {
            printf("recvfrom error\n");
            return -1;
        }
        return n; // 받은 바이트 수
    }

    int SendArray(const double* buffer, size_t size_of_buffer) {
        addrlen = sizeof(addr);
        int n = sendto(sockfd, buffer, size_of_buffer, 0, (struct sockaddr *)&addr, addrlen);
        if (n == -1) {
            printf("sendto error\n");
            return -1;
        }
        return n; // 보낸 바이트 수
    }

    // Header 내부에 추가
    int SendString(const std::string &s) {
        addrlen = sizeof(addr);
        // s.data(), s.size() 만큼 전송 → 문자열 전체 전송
        int n = sendto(sockfd,
                       s.data(),
                       s.size(),
                       0,
                       (struct sockaddr *)&addr,
                       addrlen);
        if(n == -1) { perror("sendto"); return -1; }
        return n;  // 보낸 바이트 수
    }

    int ReceiveString(std::string &s, size_t max_len) {
        std::vector<char> buf(max_len);
        addrlen = sizeof(addr);
        int n = recvfrom(sockfd,
                         buf.data(),
                         buf.size(),
                         0,
                         (struct sockaddr *)&recvaddr,
                         &addrlen);
        if(n == -1) { perror("recvfrom"); return -1; }
        s.assign(buf.data(), n);
        return n;  // 받은 바이트 수
    }

    Type GetData()
    {
        return data_to_be_exchanged;
    }

    void GetDataUsingReference(Type& data_to_be_exchanged)
    {
        data_to_be_exchanged = this->data_to_be_exchanged;
    }

    Type ReceiveAndGetData()
    {
    	ReceiveData();
    	Type data = GetData();
    	return data;
    }

};
