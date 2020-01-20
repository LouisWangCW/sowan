
#ifndef __UDP_SERVER__
#define __UDP_SERVER__

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#ifdef _WIN32
#include <ws2tcpip.h>
#include <time.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")
#endif

#define SERVER_PORT 8888
#define BUFF_LEN 8

//#define SERVER_IP "192.168.100.161"
//#define SERVER_IP "10.42.0.1"
//#define SERVER_IP "10.1.1.12"


class UDP_SERVER {

public:
	bool Ether_Work = true;

	char buf[BUFF_LEN];
	int server_fd, sListen, ret;
	struct sockaddr_in ser_addr;

	int SwitchSel = 0;
	int RoomSel = -1;

	char HitCount = 0;;

public:
	UDP_SERVER()
	{
		
#ifdef _WIN32
		int r;
		WSAData wsaData;
		WORD DLLVSERION;
		DLLVSERION = 0x102; // MAKEWORD(2, 1);
		r = WSAStartup(DLLVSERION, &wsaData);
#endif


		server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
		if (server_fd < 0)
		{
			printf("create socket fail!\n");
			//return;
		}
		printf("create socket susessful!\n");

		memset(&ser_addr, 0, sizeof(ser_addr));
		ser_addr.sin_family = AF_INET;
		//ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
		ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
		ser_addr.sin_port = htons(SERVER_PORT);  //端口号，需要网络序转换

		//sListen = socket(AF_INET, SOCK_DGRAM, NULL);
		//ret = bind(sListen, (SOCKADDR*)&ser_addr, sizeof(ser_addr));

		ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));

		if (ret < 0)
		{
			printf("socket bind fail!\n");
			//system("pause");
			//return;
		}
		printf("socket bind susessful!\n");

		//return;
	}
	int handle_udp_msg(void)
	{
		socklen_t len;
		int count;
		struct sockaddr_in clent_addr;
		char tmpsave = 0;
		int getCmd = 0;

		while (Ether_Work)
		{
			printf("recieve start! \n");
			memset(buf, 0, BUFF_LEN);
			len = sizeof(clent_addr);
			count = recvfrom(server_fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);
			if (count == -1)
			{
				printf("recieve data fail!\n");
				return 0;
			}

			if (buf[0] != 0) {
				printf("recieve data success! [");
				for (int i = 0; i < BUFF_LEN; i++) {
					printf("%d_", buf[i]);

				}
				printf("]___%d\n", count);
			}

			getCmd = buf[0] & 0x0ff;
			char roominput = (buf[1] & 0x0ff);

			if (getCmd == 'R' && roominput > '0' ) {
				RoomSel = roominput + 100 - '1';
				printf("recieve a room ID [ %c , %d ]! \n", buf[1], RoomSel-100);
			}
			else if (getCmd != 10 && getCmd >= 'a' && getCmd >= 'Z') {
				printf("recieve a Cmd [ %c , %d ]! \n", getCmd, getCmd);
				SwitchSel = getCmd;
			}

			char buf2[BUFF_LEN];
			memset(buf2, 0, BUFF_LEN);
			buf2[0] = 'R';
			buf2[1] = HitCount;
			//sprintf(buf, "01234567", count);
			//printf("server send: [");
			//for (int i = 0; i < BUFF_LEN; i++) {
			//	printf("%d_", buf2[i]);
			//}
			//printf("]___%d\n", count);
			sendto(server_fd, buf2, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);
		}

		close(server_fd);
		return 1;
	}



};

#endif
