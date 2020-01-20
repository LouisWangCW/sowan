#pragma once

#ifndef __CAR_TECO_MOTOR__
#define __CAR_TECO_MOTOR__

#include     <stdio.h>      /*夹非块块﹚竡*/
#include     <stdlib.h>     /*夹非ㄧ计畐﹚竡*/
#include     <unistd.h>     /*Unix 夹非ㄧ计﹚竡*/
#include     <sys/types.h> 
#include     <sys/stat.h>  
#include     <fcntl.h>      /*郎北﹚竡*/
#include     <termios.h>    /*PPSIX 沧狠北﹚竡*/
#include     <errno.h>      /*岿粇腹﹚竡*/
#include	<sys/time.h>

#define MotorID(N) N
#define JOG_PLUS 0x10
#define JOG_MINUS 0x20
#define MOTOR_MOVE_FWD 0x40
#define MOTOR_MOVE_REV 0x80
#define MOTOR_DELAY_TIME 18000
#define MOTOR_DELAY_TIME_CAST 19000

#define FALSE false
#define TRUE true

#define Rbuffer_SIZE 64



namespace RS485_TECO_Motor {

	class Motor_Control {

	public:
		int Speed_Multiple;
		bool WR_Working;

		int Pos_1, Pos_2;
		bool Debug_ShowReadError = false;

	private:
		int failCounter = 3;
		int Status;
		int fd;
		const char ErrorString[8] = "COMERR2";
		struct timeval start, end;
		long mtime, seconds, useconds;
		int Serial_Input_Fail_Count = 0;
		int Serial_Input_timeout_Count = 0;
		bool WR_SWITCH;
		

	public:
		Motor_Control(std::string Dev) {
			Speed_Multiple = 1;

			//SerialPortInitial(ComName);
			fd = OpenDev(Dev);
			printf("Serial Port:OpenDev sucessful\n");
			set_speed(fd, B115200);
			printf("Serial Port:set_speed sucessful\n");
			if (set_Parity(fd, 8, 1, 'n') == FALSE) {
				printf("Serial Port:Set Parity Error\n");
				//exit(0);
			}
			else {
				printf("Serial Port:set_Parity sucessful\n");
			}

			WR_Working = false;
		}

	public:
		~Motor_Control(void) {
			//CloseHandle(serialPort);
		}

	private:
		int OpenDev(std::string Dev) {
			int fd = open(Dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);         //| O_NOCTTY | O_NDELAY 
			if (-1 == fd) {
				perror("Serial Port:Can't Open Serial Port");
				return -1;
			}
			else {
				return fd;
			}

		}
		void set_speed(int fd, int speed) {
			int   i;
			int   status;
			struct termios   Opt;
			tcgetattr(fd, &Opt);

			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed);
			cfsetospeed(&Opt, speed);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if (status != 0)
				perror("tcsetattr fd1");
			return;
			tcflush(fd, TCIOFLUSH);

		}

		int set_Parity(int fd, int databits, int stopbits, int parity) {
			struct termios options;
			if (tcgetattr(fd, &options) != 0) {
				perror("SetupSerial 1");
				return(FALSE);
			}

			options.c_cflag &= ~CSIZE;
			switch (databits) /*砞竚计沮じ计*/
			{
			case 7:
				options.c_cflag |= CS7;
				break;
			case 8:
				options.c_cflag |= CS8;
				break;
			default:
				fprintf(stderr, "Unsupported data size\n"); return (FALSE);
			}
			switch (parity)
			{
			case 'n':
			case 'N':
				options.c_cflag &= ~PARENB;   /* Clear parity enable */
				options.c_iflag &= ~INPCK;     /* Enable parity checking */
				break;
			case 'o':
			case 'O':
				options.c_cflag |= (PARODD | PARENB); /* 砞竚喷*/
				options.c_iflag |= INPCK;             /* Disnable parity checking */
				break;
			case 'e':
			case 'E':
				options.c_cflag |= PARENB;     /* Enable parity */
				options.c_cflag &= ~PARODD;   /* 锣传案喷*/
				options.c_iflag |= INPCK;       /* Disnable parity checking */
				break;
			case 'S':
			case 's':  /*as no parity*/
				options.c_cflag &= ~PARENB;
				options.c_cflag &= ~CSTOPB; break;
			default:
				fprintf(stderr, "Unsupported parity\n");
				return (FALSE);
			}

			/* 砞竚氨ゎ*/
			switch (stopbits)
			{
			case 1:
				options.c_cflag &= ~CSTOPB;
				break;
			case 2:
				options.c_cflag |= CSTOPB;
				break;
			default:
				fprintf(stderr, "Unsupported stop bits\n");
				return (FALSE);
			}

			/* Set input parity option */
			if (parity != 'n')
				options.c_iflag |= INPCK;

			tcflush(fd, TCIFLUSH);
			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //Input
			options.c_oflag &= ~OPOST;   //Output
			//options.c_oflag |= ~OPOST;
			//options.c_lflag |= ~(ICANON | ECHO | ECHOE | ISIG);

			options.c_iflag |= CREAD;
			options.c_cflag |= (CLOCAL | CREAD);
			options.c_cc[VTIME] = 30; // 砞竚禬3 seconds
			options.c_cc[VMIN] = 30; // Update the options and do it NOW

			if (tcsetattr(fd, TCSANOW, &options) != 0)
			{
				perror("SetupSerial 3");
				return (FALSE);
			}
			return (TRUE);
		}

		void Serial_Input(char *Wdata, int size, char *Rdata) {
			failCounter =6;
			bool ReadCorrect = true;
			bool SerialRead_Success = true;
			do {
				write(fd, Wdata, size);
				SerialRead_Success = SerialRead(fd, Rdata);
				ReadCorrect = CheckReadCorrect(Rdata);
				if (ReadCorrect && SerialRead_Success) {
					failCounter = 0;
				}
				else {
					if (!ReadCorrect) {
						Serial_Input_Fail_Count++;
					}					
					if (!SerialRead_Success) {
						Serial_Input_timeout_Count++;
					}
					if (Debug_ShowReadError) {
						std::cout << "[";
						for (int gg = 0; Wdata[gg] != 0x0a && Wdata[gg] != 0x0d; gg++) {
							printf("%c", Wdata[gg]);
						}
						printf("]_COMERR(%d)_Timeout(%d)_FailCount(%d) \n", Serial_Input_Fail_Count, Serial_Input_timeout_Count, failCounter);
					}
					if (failCounter%2 == 0) {
						usleep(int(1000000 * 0.1));
						RGB_READ(  ((Wdata[2]  - '0'))%2  );
						//printf("RGB_READ,failCounter = %d, %c\n", failCounter, RGB_READ(Wdata[2] - '0'));
					}
					failCounter--;
					if (failCounter == 0) {
						printf("Serial_Input ,failCounter = %d ,Timeout = %d,Input_Fail = %d\n", failCounter, Serial_Input_timeout_Count, Serial_Input_Fail_Count);
						for (int i = 0; Wdata[i] != 0x0a; i++) {
							std::cout << (Wdata[i]);
						}
						std::cout << std::endl;
					}
				}
			} while (failCounter);
		}

		bool SerialRead(int fd, char *Rdata) {
			int res = 0;
			int trytime = 0;
			//gettimeofday(&start, NULL);
			char tmp;

			int count = 0;
			bool startw = false;
			int outcount = 0;

			do {
				res = read(fd, &tmp, 1);
				if ((tmp == 0x0a && count !=0) || count > Rbuffer_SIZE - 2) {
					Rdata[count] = 0x0a;
					break;
				}
				if (tmp == '#') {
					startw = true;
				}
				if (startw) {
					Rdata[count] = tmp;
					//std::cout <<(int) tmp << ",";
					count++;
				}
				trytime++;
			} while (trytime < 20000);

			if (trytime < 20000) {
				//std::cout << "Sucessful , trytime = " << trytime << std::endl;
				return true;
			}
			else {
				if (failCounter  == 0) {
					std::cout << "Fault , trytime = " << trytime << ",count = " << count << std::endl;
				}
				
				return false;
			}
				
		}

		void Serial_Cast(char *Wdata, int size) {
			write(fd, Wdata, size);

			//for (int i = 0; Wdata[i] != 0x0a; i++) {
			//	std::cout << (Wdata[i]);
			//}
			//std::cout << std::endl;
		};

		bool CheckReadCorrect(char *Rdata) {
			int hitcount = 0;
			bool res = true;
			for (int i = 0; i < 7;i++) {
				if (Rdata[i + 4] == ErrorString[i])
					hitcount++;
			}
			if (hitcount > 1) {
				res = false;
			}
			return res;
		}
		int RGB_READ(char Motor_ID) {
			char Wbuffer[] = "@00:RGB\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			write(fd, Wbuffer, sizeof(Wbuffer));
			bool SerialRead_Success = SerialRead(fd, Rbuffer);
			bool ReadCorrect = CheckReadCorrect(Rbuffer);
			if (!ReadCorrect) {
				//Serial_Input_Fail_Count++;
				std::cout << "[";
				for (int gg = 0; Wbuffer[gg] != 0x0a && Wbuffer[gg] != 0x0d; gg++) {
					printf("%c", Wbuffer[gg]);
				}
				printf("]_%d \n", Serial_Input_Fail_Count);
			}

			return Rbuffer[8];
		}


	public:

		int Motor_Pos_Set(char Motor_ID, int Value) {
			char Wbuffer[] = "@00:EX=00000000000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			int tmpValue = Value;
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			if (Value < 0) {
				Value = -Value;
				Wbuffer[7] = '-';
			}
			for (int i = 0; i < 10; i++) {
				Wbuffer[17 - i] = Value % 10 + '0';
				Value /= 10;
				if (Value == 0)
					break;
			}

			if (Motor_ID == 1 ) {
				Pos_1 = tmpValue;
			}
			else if (Motor_ID == 2) {
				Pos_2 = tmpValue;
			}

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			//std::cout<<(Rbuffer) << std::endl;
			//std::cout << "Set Pos = "<< tmpValue<<std::endl << std::endl;
		}

		int Motor_Pos_Read(char Motor_ID) {
			char Wbuffer[] = "@00:EX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			int tmpValue = 0;
			bool startNum = false;
			for (int i = 0; i < Rbuffer_SIZE; i++) {
				if (Rbuffer[i] == '=') {
					startNum = true;
					i++;
				}
				else if (Rbuffer[i] == ';' || Rbuffer[i] == 0x0a || Rbuffer[i] == 0x0d)
					break;
				if (startNum) {
					if (Rbuffer[i] == '-') {
						i++;
						tmpValue = -(Rbuffer[i] - '0');
					}
					else {
						if (tmpValue >= 0)
							tmpValue = (10 * tmpValue) + (Rbuffer[i] - '0');
						else
							tmpValue = (10 * tmpValue) - (Rbuffer[i] - '0');
					}
				}

			}
			if (!CheckReadCorrect(Rbuffer)) {
				//for (int gg = 0; gg < Rbuffer_SIZE; gg++) {
				//	if (Rbuffer[gg] == 0x0a) {
				//		break;
				//	}
				//	printf("%c[%d] ", Rbuffer[gg], Rbuffer[gg]);
				//}
				//printf("\n");
				tmpValue = Motor_Pos_Read(Motor_ID);
			}
			return tmpValue;
		}

		double Motor_Speed_RPM_Read(char Motor_ID) {
			char Wbuffer[] = "@00:VX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);

			double tmpValue = 0;
			bool startNum = false;
			bool minus = false;
			bool dotAdd = false;
			float dotNum = 1;
			for (int i = 0; i < Rbuffer_SIZE; i++) {
				if (Rbuffer[i] == '=') {
					startNum = true;
					i++;
				}
				else if (Rbuffer[i] == ';' || Rbuffer[i] == 0x0a || Rbuffer[i] == 0x0d)
					break;
				if (startNum) {
					if (Rbuffer[i] == '-') {
						minus = true;
						continue;
					}
					if (Rbuffer[i] == '.') {
						dotAdd = true;
						continue;
					}
					else {
						tmpValue = (10 * tmpValue) + (Rbuffer[i] - '0');
						if (dotAdd) {
							dotNum *= 0.1;
						}
					}
				}

			}

			tmpValue = tmpValue * dotNum;
			if (minus)
				tmpValue = -tmpValue;
			//std::cout << "SPD3 = " << tmpValue << "__" << (Rbuffer) << std::endl;
			return tmpValue;

		}

		double Motor_Speed_Plus_Read(char Motor_ID) {
			return (Motor_Speed_RPM_Read(Motor_ID) * 10000.0 / 60.0);
		}
		int Motor_State_Read(char Motor_ID) {
			char Wbuffer[] = "@00:MST;FLT\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			int startNum = 0;

			int mst = 0, flt = 0;

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);


			for (int i = 0; i < Rbuffer_SIZE; i++) {
				if (Rbuffer[i] == 'x') {
					startNum++;
					i++;
				}
				else if (Rbuffer[i] == 0x0a || Rbuffer[i] == 0x0d)
					break;

				if (startNum == 1) {
					if (Rbuffer[i] < ('9' + 1))
						mst = (Rbuffer[i] - '0');
					else if (Rbuffer[i] >= 'A' &&  Rbuffer[i] <= 'F')
						mst = 10 + (Rbuffer[i] - 'A');
					else if (Rbuffer[i] >= 'a' &&  Rbuffer[i] <= 'f')
						mst = 10 + (Rbuffer[i] - 'a');
					startNum++;
				}
				else if (startNum == 3) {
					int tmp;
					if (Rbuffer[i] < ('9' + 1))
						tmp = (Rbuffer[i] - '0');
					else if (Rbuffer[i] >= 'A' &&  Rbuffer[i] <= 'F')
						tmp = 10 + (Rbuffer[i] - 'A');
					else if (Rbuffer[i] >= 'a' &&  Rbuffer[i] <= 'f')
						tmp = 10 + (Rbuffer[i] - 'a');

					flt = (flt << 4) + tmp;
				}

			}


			//std::cout << (Rbuffer) << std::endl;
			return mst + (flt * 100);
			//std::cout << "State = " << tmpValue << std::endl;
		}

		void ServoOn(char Motor_ID) {
			char Wbuffer[] = "@00:SVON\r\n";
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
		}
		void ServoOFF(char Motor_ID) {
			char Wbuffer[] = "@00:SVOFF\r\n";
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}

		void JOG_Setting(double speed, double acc, char Motor_ID) {
			char Wbuffer[] = "@00:ACC=00000;HSPD=00000.000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			int speed_int = speed * 1000;
			int acc_int = acc;

			for (int i = 0; i < 5; i++) {
				Wbuffer[12 - i] = acc_int % 10 + '0';
				acc_int /= 10;
			}
			for (int i = 0; i < 9; i++) {
				if (i == 3) {
					i++;
				}
				Wbuffer[27 - i] = speed_int % 10 + '0';
				speed_int /= 10;
			}
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}
		void JOG_Move(char Motor_ID, double acc, double speed) {

			//printf("%d,acc=%f,spd=%f \n", Motor_ID, acc, speed);

			char Wbuffer[] = "@00:ACC=00000;JOGV=00000.000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			int speed_int = speed * 1000;
			int acc_int = acc;

			for (int i = 0; i < 5; i++) {
				Wbuffer[12 - i] = acc_int % 10 + '0';
				acc_int /= 10;
			}
			if (speed < 0) {
				Wbuffer[19] = '-';
				speed_int = -speed_int;
			}

			for (int i = 0; i < 8; i++) {
				if (i == 3) {
					i++;
				}
				Wbuffer[27 - i] = speed_int % 10 + '0';
				speed_int /= 10;
			}
			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}

		void Motor_JOGX(char Motor_ID, char dir) {
			char Wbuffer[] = "@00:JOGXN\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			if (dir == 'P') {
				Wbuffer[8] = dir;
			}

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}

		void Motor_Step_Move(char Motor_ID, int Value) {
			int now = Motor_Pos_Read(Motor_ID);
			int tag = now + Value;

			//std::cout << tag << "," << now << "," << Value<<std::endl;

			char Wbuffer[] = "@00:X=00000000000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			if (tag < 0) {
				Wbuffer[6] = '-';
				tag = -tag;
			}
			for (int i = 0; tag; i++) {
				Wbuffer[16 - i] = tag % 10 + '0';
				tag /= 10;
			}
			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}
		void Motor_Step_Move_Both(char Motor_ID_1, int Value1, char Motor_ID_2, int Value2) {

			int tag1 = Motor_Pos_Read(Motor_ID_1) + Value1;
			int tag2 = Motor_Pos_Read(Motor_ID_2) + Value2;
			char Wbuffer1[] = "@00:X=00000000000\r\n";
			char Wbuffer2[] = "@00:X=00000000000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Wbuffer1[1] = (Motor_ID_1 / 10) + '0';
			Wbuffer1[2] = (Motor_ID_1 % 10) + '0';
			Wbuffer2[1] = (Motor_ID_2 / 10) + '0';
			Wbuffer2[2] = (Motor_ID_2 % 10) + '0';
			if (tag1 < 0) {
				Wbuffer1[6] = '-';
				tag1 = -tag1;
			}
			for (int i = 0; tag1; i++) {
				Wbuffer1[16 - i] = tag1 % 10 + '0';
				tag1 /= 10;
			}
			if (tag2 < 0) {
				Wbuffer2[6] = '-';
				tag2 = -tag2;
			}
			for (int i = 0; tag2; i++) {
				Wbuffer2[16 - i] = tag2 % 10 + '0';
				tag2 /= 10;
			}
			Serial_Input(Wbuffer1, sizeof(Wbuffer1), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
			Serial_Input(Wbuffer2, sizeof(Wbuffer2), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}

		void Motor_STOP(char Motor_ID) {
			char Wbuffer[] = "@00:STOPX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Wbuffer[1] = (Motor_ID / 10) + '0';
			Wbuffer[2] = (Motor_ID % 10) + '0';

			Serial_Input(Wbuffer, sizeof(Wbuffer), Rbuffer);
			//std::cout << (Rbuffer) << std::endl;
		}
		void Motor_Error_Limit() {
			char Wbuffer[] = "@00:CEMS=3000;CEVAL=3000\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };
			Serial_Cast(Wbuffer, sizeof(Wbuffer));
			//std::cout << (Rbuffer) << std::endl;
		}

		//Group
		void ServoOn_Cast() {
			char Wbuffer[] = "@00:SVON\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			Serial_Cast(Wbuffer, sizeof(Wbuffer));
			//printf(Rbuffer);
			//printf("ServoOn_Cast \n");

		}

		void Motor_STOP_Group() {
			char Wbuffer[] = "@00:STOPX\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };


			Serial_Cast(Wbuffer, sizeof(Wbuffer));
			//std::cout << (Rbuffer) << std::endl;
		}
		void Motor_JOGX_Group(char dir) {
			char Wbuffer[] = "@00:JOGXN\r\n";
			char Rbuffer[Rbuffer_SIZE] = { 0 };

			if (dir == 'P') {
				Wbuffer[8] = dir;
			}

			Serial_Cast(Wbuffer, sizeof(Wbuffer));
		}





		/*
		void JOG_Input_AR(int speed, int value, char direction, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x06, 0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			speed = speed * Speed_Multiple;
			if (direction == 'P')
				direction = JOG_PLUS;
			else if
				(direction == 'N')
				direction = JOG_MINUS;


			///JOG speed 忋埵
			Wbuffer[2] = 0x02; Wbuffer[3] = 0x86;
			Wbuffer[4] = speed >> 24 & 0xff; Wbuffer[5] = speed >> 16 & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			///JOG speed 壓埵
			Wbuffer[2] = 0x02; Wbuffer[3] = 0x87;
			Wbuffer[4] = speed >> 8 & 0xff; Wbuffer[5] = speed & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			///JOG step 忋埵
			Wbuffer[2] = 0x10; Wbuffer[3] = 0x48;
			Wbuffer[4] = value >> 24 & 0xff; Wbuffer[5] = value >> 16 & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			///JOG step 壓埵
			Wbuffer[2] = 0x10; Wbuffer[3] = 0x49;
			Wbuffer[4] = value >> 8 & 0xff; Wbuffer[5] = value & 0xff;
			Serial_Input(Wbuffer, Rbuffer);

			/// [4]JOG+-摦嶌
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d; Wbuffer[4] = direction; Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d; Wbuffer[4] = 0x00; Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);
		}
		void JOG_Acc_Setting_AR(int acc, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0, 0, 0x00, 0x02, 0x04, ///[0]ID ///address
				0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x02; Wbuffer[3] = 0x88;

			Wbuffer[7] = acc >> 24 & 0xff;; Wbuffer[8] = acc >> 16 & 0xff;
			Wbuffer[9] = acc >> 8 & 0xff; Wbuffer[10] = acc & 0xff;
			Serial_Input_mul(Wbuffer, Rbuffer, 11);

		}
		/////

		void RUN_Input(int speed, char direction, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, ///[0]ID
				0x04, 0x80,  ///address
				0x00, 0x04, ///Number
				0x08,   ///Numberdata bytes // Number*2
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			speed = speed * Speed_Multiple;

			Wbuffer[7] = speed >> 24 & 0xff;; Wbuffer[8] = speed >> 16 & 0xff;
			Wbuffer[9] = speed >> 8 & 0xff; Wbuffer[10] = speed & 0xff;

			Wbuffer[11] = speed >> 24 & 0xff;; Wbuffer[12] = speed >> 16 & 0xff;
			Wbuffer[13] = speed >> 8 & 0xff; Wbuffer[14] = speed & 0xff;

			Serial_Input_mul(Wbuffer, Rbuffer, 15);

			if (direction == 'F')
				direction = 0x40;
			else if (direction == 'B')
				direction = 0x80;

			Wbuffer[1] = 0x06;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;				///address
			Wbuffer[4] = direction; Wbuffer[5] = 0x00;			///[0]ID
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[4] = direction; Wbuffer[5] = 0x01;			///[0]ID
			Serial_Input(Wbuffer, Rbuffer);
		}
		void RUN_Acc_Setting(int acc, char direction, char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x10, 0x06, 0, 0x00, 0x04, 0x08, ///[0]ID ///address
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			if (direction == 'P')
				Wbuffer[3] = 0x00;
			else if (direction == 'N')
				Wbuffer[3] = 0x80;

			if (Motor_ID == 7) {
				acc = 1000000000 / acc;
			}

			Wbuffer[7] = acc >> 24 & 0xff;; Wbuffer[8] = acc >> 16 & 0xff;
			Wbuffer[9] = acc >> 8 & 0xff; Wbuffer[10] = acc & 0xff;

			Wbuffer[11] = acc >> 24 & 0xff;; Wbuffer[12] = acc >> 16 & 0xff;;
			Wbuffer[13] = acc >> 8 & 0xff; Wbuffer[14] = acc & 0xff;

			Serial_Input_mul(Wbuffer, Rbuffer, 15);

		}

		void Motor_Stop(char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x06, 0x00, 0x7d, 0x00, 0x20, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);


			Wbuffer[4] = 0x00; Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);
		}

		void Motor_Error_Reset(char Motor_ID) {
			Motor_Stop(Motor_ID);

			char Wbuffer[] = { Motor_ID, 0x06, 0x01, 0x81, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			Wbuffer[2] = 0x01; Wbuffer[3] = 0x81;
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x01;
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);
		}

		void Pos_Reset(char Motor_ID) {
			Motor_Stop(Motor_ID);

			char Wbuffer[] = { Motor_ID, 0x06, 0x01, 0x8b, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[5] = 0x00;
			Serial_Input(Wbuffer, Rbuffer);
		}

		bool Motor_Free(char Motor_ID) {
			//Motor_Stop(Motor_ID);
			bool tmp;

			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0x7d, 0x00, 0x01 , 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

			Serial_Input_read(Wbuffer, Rbuffer, 1);

			if (Rbuffer[4] & 0x40) {
				tmp = true;
				Wbuffer[5] = 0x00;
			}
			else {
				Wbuffer[5] = 0x40;
				tmp = false;
			}

			Wbuffer[1] = 0x06;
			Serial_Input(Wbuffer, Rbuffer);

			if (Wbuffer[5] & 0x40)
				return true;
			else
				return false;

		}
		//////

		int Motor_Pos_Return(char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0xc6, 0x00, 0x02, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			Int32 tmp = 0;
			Serial_Input_read(Wbuffer, Rbuffer, 2);
			tmp = ((Rbuffer[3] & 0xff) << 24) + ((Rbuffer[4] & 0xff) << 16) + ((Rbuffer[5] & 0xff) << 8) + (Rbuffer[6] & 0xff);
			return tmp;
		}
		bool Motor_State_Return(char Motor_ID, int bit_address) {
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0x7f, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input_read(Wbuffer, Rbuffer, 1);
			int tmp = (((Rbuffer[3] & 0xff) << 8) + ((Rbuffer[4] & 0xff))) & (0x01 << bit_address);
			return (tmp != 0);
		}
		bool Motor_Err_Detector(char Motor_ID) {
			return Motor_State_Return(Motor_ID, 7);
		}
		bool Motor_Ready_Detector(char Motor_ID) {
			return Motor_State_Return(Motor_ID, 5);
		}
		int Motor_Speed_Return(char Motor_ID) {
			char Wbuffer[] = { Motor_ID, 0x03, 0x00, 0xc8, 0x00, 0x02, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			Int32 tmp = 0;
			Serial_Input_read(Wbuffer, Rbuffer, 2);
			tmp = ((Rbuffer[3] & 0xff) << 24) + ((Rbuffer[4] & 0xff) << 16) + ((Rbuffer[5] & 0xff) << 8) + (Rbuffer[6] & 0xff);
			return tmp;
		}


		void Motor_Pos_Move(int speed, int value, char Motor_ID) {
			Int32 tmp = Motor_Pos_Return(Motor_ID);
			tmp = value - tmp;
			if (tmp > 0)
				JOG_Input(speed, tmp, JOG_PLUS, Motor_ID);
			else if (tmp < 0)
				JOG_Input(speed, -tmp, JOG_MINUS, Motor_ID);

		}
		void Motor_Breaker_Lock() {

			char Wbuffer[] = { 0x02, 0x06, 0x02, 0x01, 0x00, 0x03, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[1] = 0x06;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;			///address
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x20;			///[0]ID
			Serial_Input(Wbuffer, Rbuffer);
		}
		void Motor_Breaker_Free() {

			char Wbuffer[] = { 0x02, 0x06, 0x02, 0x01, 0x00, 0x01, 0, 0 };
			char Rbuffer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			Serial_Input(Wbuffer, Rbuffer);

			Wbuffer[1] = 0x06;
			Wbuffer[2] = 0x00; Wbuffer[3] = 0x7d;				///address
			Wbuffer[4] = 0x00; Wbuffer[5] = 0x00;			///[0]ID
			Serial_Input(Wbuffer, Rbuffer);
		}
		*/


	};
}

#endif