#include "service.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <lwip/api.h>
#include <lwip/arch.h>
#include <lwip/opt.h>
#include <lwip/inet.h>
#include <lwip/errno.h>
#include <netdb.h>

#include "cam.h"
#include "microphone.h"
#include "utils_getopt.h"
#include "bflb_mtimer.h"


uint8_t recv_buf[512] = {0};
uint64_t recv_len = 0;
int sock = -1;

void service_close(int sig)
{
    if (sock) {
        closesocket(sock);
    }
    if (recv_len > 0) {
        printf("Total send data=%lld\r\n", recv_len);
    }
}
void service_init()
{
    printf("udp server task start ...\r\n");

    char *port = "9876";
    struct sockaddr_in udp_addr, remote_addr;
    socklen_t addr_len;

    memset(&recv_buf[0], 0, sizeof(recv_buf));

    while (1) {
        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("udp create socket error\r\n");
            return;
        }
        udp_addr.sin_family = AF_INET;
        udp_addr.sin_port = htons(atoi(port));
        udp_addr.sin_addr.s_addr = INADDR_ANY;
        memset(&(udp_addr.sin_zero), 0, sizeof(udp_addr.sin_zero));

        printf("Server ip Address : %s:%s\r\n", udp_addr.sin_addr.s_addr, port);

        if (bind(sock, (struct sockaddr *)&udp_addr, sizeof(struct sockaddr)) != 0) {
            printf("udp bind falied!\r\n");
            closesocket(sock);
            return;
        }

        recv_len = 0;
        while (1) {
            recv_len = recvfrom(sock, recv_buf, 1024, 0, (struct sockaddr *)&remote_addr, &addr_len);
            // printf("recv from %s\r\n", inet_ntoa(remote_addr.sin_addr));
            // printf("recv:%s \r\n", recv_buf);
            if(recv_len > 0)
            {
                cam_set_status(0);
                while (cam_status()== 0);
                int len = cam_data_length();
                char *msg_count = (char *)malloc(64);
                sprintf(msg_count,"len:%d\n\0", len);
                sendto(sock, msg_count, strlen(msg_count), 0, (struct sockaddr *)&remote_addr, addr_len);
                recv_len = recvfrom(sock, recv_buf, 1024, 0, (struct sockaddr *)&remote_addr, &addr_len);
                free(msg_count);
                int count = (int)(len/1024)+1;
                for (int i = 0; i < count; i++)
                {
                    sendto(sock, cam_data() + i * 1024, len < 1024 ? len : 1024, 0, (struct sockaddr *)&remote_addr, addr_len);
                    // bflb_mtimer_delay_ms(1);
                    len -= 1024;
                    if(len<0)
                    {
                        break;
                    }
                    else
                    {
                        recv_len = recvfrom(sock, recv_buf, 1024, 0, (struct sockaddr *)&remote_addr, &addr_len);
                    }
                }
                #if 0
                audio_set_status(0);
                while (audio_status() == 0);
                int len = 8*1000;
                char *msg_count = (char *)malloc(64);
                sprintf(msg_count,"len:%d\n\0", len);
                sendto(sock, msg_count, strlen(msg_count), 0, (struct sockaddr *)&remote_addr, addr_len);
                recv_len = recvfrom(sock, recv_buf, 1024, 0, (struct sockaddr *)&remote_addr, &addr_len);
                free(msg_count);
                int count = (int)(len/1024)+1;
                for (int i = 0; i < count; i++)
                {
                    sendto(sock, audio_data() + i * 1024, len < 1024 ? len : 1024, 0, (struct sockaddr *)&remote_addr, addr_len);
                    // bflb_mtimer_delay_ms(1);
                    len -= 1024;
                    if(len<0)
                    {
                        break;
                    }
                    else
                    {
                        recv_len = recvfrom(sock, recv_buf, 1024, 0, (struct sockaddr *)&remote_addr, &addr_len);
                    }
                }
                #endif
            }
        }
        closesocket(sock);
        return;
    }
}