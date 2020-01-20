#include <stdio.h>
#include <wiringPi.h>

#define GPIO_OUT_NUM 10
#define GPIO_IN_NUM 3
class RaspberryGPIO {
private:
	int g_in_[3] = {26,27,28};
	//
	//int g_out_[GPIO_OUT_NUM] = { 15,16,1,4,5,6,10,11,31}; //wiring pi
	int g_out_[GPIO_OUT_NUM] = { 8,16,1,4,5,6,10,11,31,15 }; //wiring pi
	//
	bool g_out_value[GPIO_OUT_NUM] = { 0,0,0,0,0,0,0,0,0,0 };

	int g_in_buff[3] = { 0,0,0 };

	int input_buff_size = 3;


public:
	RaspberryGPIO() {
		if (wiringPiSetup() == -1)
			printf("wiringPiSetup Sucessful\n");

		for (int i = 0; i < GPIO_OUT_NUM; i++) {
			pinMode(g_out_[i], OUTPUT);
			digitalWrite(g_out_[i], 0);
		}
		//pinMode (g_out, OUTPUT) ;         // wiring 7 = bcm gpio 4
		//pinMode(g_out_2 , OUTPUT);		  // 1 = gpio 18
		for (int i = 0; i < 3; i++) {
			pinMode(g_in_[i], INPUT);
		}
		//pinMode (g_in , INPUT) ;          // 3 = gpio 22
	}
	void GPIO_Set_Value(int pin,bool value) {
		int outpin = g_out_[pin];
		g_out_value[pin] = value;
	
		if (value) {
			digitalWrite (outpin, 1) ;
		}
		else {
			digitalWrite (outpin, 0) ;
		}
	}
	void GPIO_inv_Value(int pin) {
		int outpin = g_out_[pin];
		g_out_value[pin] = !g_out_value[pin];
		digitalWrite(outpin, g_out_value[pin]);

		for (int i = 0; i < GPIO_OUT_NUM; i++) {
			printf("(i=%d), ", g_out_value[i]);		
		}
		printf("\n");
	}
	int GPIO_Read_Value(int pin) {
		return digitalRead(g_in_[pin]) ;
	}
	void GPIO_Input_refresh(int pin) {
		GPIO_Read_Value_Buff(0);
		GPIO_Read_Value_Buff(1);
		GPIO_Read_Value_Buff(2);
	}
	int GPIO_Read_Value_Buff(int pin) {
		int tmp = digitalRead(g_in_[pin]);
		if (tmp) {
			g_in_buff[pin]++;
			if (g_in_buff[pin] > input_buff_size) {
				g_in_buff[pin] = input_buff_size;
			}
		}
		else {
			g_in_buff[pin] --;
			if (g_in_buff[pin] < 0) {
				g_in_buff[pin] = 0;
			}
		}
		return g_in_buff[pin];
	}

	float GPIO_Read_Float(int pin) {
		float tmp = (float)GPIO_Read_Value_Buff(pin);

		return (float) (tmp / input_buff_size);
	}

private:
};

