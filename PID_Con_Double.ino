/*
 Name:		PID_Con_Double.ino
 Created:	2017/5/24 15:08:41
 Author:	wooden
*/

// works pretty good, two servo can be control

// the setup function runs once when you press reset or power the board
#include <Servo.h>
#include <PID_v1.h>
#include <MsTimer2.h>
#include <MLX90316.h>
#include <Metro.h>
//======================================================================
// define the variable

// define PID
double Input1, Setpoint1, Output1;
double Kp1 = 2, Ki1 = 2, Kd1 = 0.1;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, REVERSE);

double Input2, Setpoint2, Output2;
double Kp2 = 2, Ki2 = 4, Kd2 = 0.1;
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, REVERSE);

Servo myservo1;
Servo myservo2;

//int pinSS = 5, pinSCK = 3, pinMOSI = 4;
int PINSS[] = { 4, 7 };
int PINSCK[] = { 2, 5 };
int PINMOSI[] = { 3, 6 };
int pinSS, pinSCK, pinMOSI;
int angle_temp[2], angle_real[2], angle_continue[2], angle_delta[2];
Metro mlxMetro = Metro(5);
MLX90316 mlx_1 = MLX90316();

int times = 20;
int limit = 3000;
int angle_initial[2];
double temp1;
double temp2;
/*int k;
int aim;
int flag = 1;*/  // valiable for swap

void setup() {
	Serial.begin(56000);
	Serial.setTimeout(200);
	//Serial.println("10");
	//=====================================
	//mlx_1.attach(pinSS, pinSCK, pinMOSI);    // need put in subfun
	initialization_angle(1);
	initialization_angle(2);
	Setpoint1 = angle_continue[1-1];
	Setpoint2 = angle_continue[2 - 1];

	//=====================================
	myservo1.attach(9);
	myservo1.write(90);
	myservo2.attach(10);
	myservo2.write(90);
	//Serial.println("1");
	myPID1.SetOutputLimits(-10000, 10000);
	myPID1.SetSampleTime(times);
	myPID1.SetMode(AUTOMATIC);
	myPID2.SetOutputLimits(-10000, 10000);
	myPID2.SetSampleTime(times);
	myPID2.SetMode(AUTOMATIC);
	//=====================================
	//Serial.println("2");
	MsTimer2::set(times, sig);
	MsTimer2::start();
	//Serial.println("3");
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (Serial.available() > 0) {
		MsTimer2::stop();
		Setpoint1 = Serial.parseInt();
		Setpoint2 = Serial.parseInt();
		temp1 = Serial.parseInt();
		temp2 = Serial.parseInt();
		while (temp1 != Setpoint1||temp2!=Setpoint2||temp1==0||temp2==0) {
			Setpoint1 = Serial.parseInt();
			Serial.println(Setpoint1);
			Setpoint2 = Serial.parseInt();
			temp1 = Serial.parseInt();
			temp2 = Serial.parseInt();
			Serial.flush();
		}
		/*Serial.print(Setpoint1);
		Serial.print(" ");
		Serial.println(Setpoint2);
		delay(3000);
		while (Serial.parseInt()!=1)
		{
			Setpoint1 = Serial.parseInt();
			Setpoint2 = Serial.parseInt();
			Serial.print(Setpoint1);
			Serial.print(" ");
			Serial.println(Setpoint2);
			delay(3000);
		}*/
	}
	MsTimer2::start();
	delay(times);
}

// define user function
void sig() {
	Refresh_angle(1);
	Refresh_angle(2);
	servo_pid();
}

void initialization_angle(int i) {
	
	int flag = 0;
	int j;
	j = i - 1;

	pinSS = PINSS[j];
	pinSCK = PINSCK[j];
	pinMOSI = PINMOSI[j];
	mlx_1.attach(pinSS, pinSCK, pinMOSI);

	while (flag != 1) {
		if (mlxMetro.check() == 1) {

			angle_temp[j] = mlx_1.readAngle();
			Serial.println(angle_temp[j]);
			if (angle_temp[j] >= 0) {
				angle_delta[j] = angle_temp[j] - angle_real[j];
				angle_continue[j] = angle_temp[j];
				angle_real[j] = angle_temp[j];
				flag = 1;
			}
		}
	}
	angle_initial[i - 1] = angle_continue[i - 1];
	//Serial.print(angle_continue);
	//Serial.println("");
	return;

}

void Refresh_angle(int i) {
	
	int j;
	int flag = 0;
	j = i - 1;
	pinSS = PINSS[j];
	pinSCK = PINSCK[j];
	pinMOSI = PINMOSI[j];
	mlx_1.attach(pinSS, pinSCK, pinMOSI);

	while (flag != 1) {
		if (mlxMetro.check() == 1) {

			angle_temp[j] = mlx_1.readAngle();
			//Serial.println(angle_temp);
			if (angle_temp[j] >= 0) {
				angle_delta[j] = angle_temp[j] - angle_real[j];
				if (angle_delta[j]>3000) angle_delta[j] = angle_delta[j] - 3600;
				if (angle_delta[j]<-3000) angle_delta[j] = angle_delta[j] + 3600;
				angle_continue[j] = angle_continue[j] + angle_delta[j];
				angle_real[j] = angle_temp[j];
				flag = 1;
			}
		}
	}
	//Serial.print(angle_continue);
	//Serial.println("");
	return;
}

void servo_pid() {
	double temp;
	int PWM;
	//temp = Output;
	//=======================================
	Input1 = angle_continue[1-1];
	myPID1.Compute();
	//temp = temp - Output;
	temp = Output1;
	PWM = int(temp*0.15) + 90;
	if (PWM > 180) PWM = 180;
	if (PWM < 0) PWM = 0;
	//if (angle_continue[1 - 1] > (angle_initial[1 - 1] + limit)) PWM = 90;
	myservo1.write(PWM);

	//=======================================
	Input2 = angle_continue[2-1];
	myPID2.Compute();
	//temp = temp - Output;
	temp = Output2;
	PWM = int(temp*0.15) + 90;
	if (PWM > 180) PWM = 180;
	if (PWM < 0) PWM = 0;
	//if (angle_continue[2 - 1] >(angle_initial[2 - 1] + limit)) PWM = 90;
	myservo2.write(PWM);

	/*Serial.print("IN   ");
	Serial.println(Input);
	Serial.print("OUT ");
	Serial.println(Output);
	Serial.print("SET ");
	Serial.println(Setpoint);
	Serial.print("PWM ");
	Serial.println(PWM);*/
	Serial.print(int(Input1));
	Serial.print(" ");
	Serial.print(int(Input2));
	Serial.print(" ");
	Serial.print(int(Output1));
	Serial.print(" ");
	Serial.print(int(Output2));
	Serial.print(" ");
	//Serial.print(PWM);
	//Serial.print(" ");
	Serial.print(int(Setpoint1));
	Serial.print(" ");
	Serial.print(int(Setpoint2));
	Serial.println(" ");

}