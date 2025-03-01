//global variables to stop the robot
bool end_game = false;
int time;


//moves a motor to pull back an elastic band to shoot a ping pong ball
void pull_back()
{
	nMotorEncoder[motorB] = 0;
	motor[motorB] = -10;
	while(abs(nMotorEncoder[motorB]) < 340)
	{}
	motor[motorB] = 0;
}


//resets the first motor in the shooting mechanism
void reset_1()
{
	nMotorEncoder[motorB] = 0;
	motor[motorB] = 30;
	while(abs(nMotorEncoder[motorB]) < 340)
	{}
	motor[motorB] = 0;
}


//resets the second motor in the shooting mechanism
void reset_2()
{
	nMotorEncoder[motorC] = 0;
	motor[motorC] = -30;
	while(abs(nMotorEncoder[motorC]) < 100)
	{}
	motor[motorC] = 0;
}


//shoots the ping pong ball by letting go of an elastic band
void shoot()
{
	time = time1[T1] / 1000;
	nMotorEncoder[motorC] = 0;
	motor[motorC] = 100;
	while(abs(nMotorEncoder[motorC]) < 100)
	{}
	motor[motorC] = 0;
	reset_1();
	reset_2();
}


//callibrates all sensors
void calibrate_all()
{
	SensorType[S2] = sensorEV3_Gyro ;
	wait1Msec(50);
	SensorMode [S2] = modeEV3Gyro_Calibration ;
	wait1Msec(50);
	SensorMode [S2] = modeEV3Gyro_RateAndAngle ;
	wait1Msec(50);
	SensorType[S1]= sensorEV3_Touch;
	wait1Msec(50);
	SensorType[S4]= sensorEV3_Ultrasonic;
	wait1Msec(50);
}


//resets the readings of all sensors
void zero_all()
{
	nMotorEncoder[motorA] = 0;
	wait1Msec(50);
	resetGyro(S2);
	wait1Msec(50);
}


//updates the x-y coordinates of the robot
void update_coordinates (int total_degrees, int distance, float &x, float &y, string movement_type)
{
	if(movement_type == "forward")
	{
		y += sinDegrees(total_degrees % 360) * distance;
  		x += cosDegrees(total_degrees % 360) * distance;
	}

	else
	{
		y -= sinDegrees(total_degrees % 360) * distance;
  		x -= cosDegrees(total_degrees % 360) * distance;
  }
}


//determines if a movement is valid by making sure the robot will not move beyond the boundaries
bool prediction (int total_degrees, int distance, float x, float y, int &x_playfield, int &y_playfield, string movement_type)
{

	if(movement_type == "forward")
	{
		y += sinDegrees(total_degrees % 360) * distance;
  		x += cosDegrees(total_degrees % 360) * distance;
	}

	else
	{
		y -= sinDegrees(total_degrees % 360) * distance;
  		x -= cosDegrees(total_degrees % 360) * distance;
	}

	if ( x > x_playfield || x < -1 || y > y_playfield|| y < -1)
	{
		return false;
	}

	else
	{
		return true;
	}
}


//uses ultraosnic sensor to determine if someone is within 100cm
void detect(int total_degrees, float x, float y, int &x_playfield, int &y_playfield)
{
	string movement_type = "forward";
	int value = SensorValue[S4];
	if(value > 10 && value <100)
	{
		if(end_game != true && prediction(total_degrees, value, x, y, x_playfield, y_playfield, movement_type))
		{
			shoot();
			end_game = true;
		}
	}
}


//accelerates the robot for movement
void accel(string direction, int total_degrees, float x, float y, int x_playfield, int y_playfield)
{
	int speed = 0;

	nMotorEncoder[motorA] = 0;
	while(abs(nMotorEncoder[motorA])*8*PI/360 < 30)
	{
		speed += 2;
		if(direction == "forward")
		{
			motor[motorA] = -speed;
			motor[motorD] = -speed;
		}
		else
		{
			motor[motorA] = speed;
			motor[motorD] = speed;
		}
		if(end_game != true)
		{
			detect(total_degrees, x, y, x_playfield, y_playfield);
		}
		wait1Msec(30);
	}
}


//slows down the robot for stopping
void decel(string direction, int total_degrees, float x, float y, int x_playfield, int y_playfield)
{
	int speed = 80;

	nMotorEncoder[motorA] = 0;
	while(abs(nMotorEncoder[motorA])*8*PI/360 < 15)
	{
		speed -= 5;
		if(direction == "forward")
		{
			motor[motorA] = -speed;
			motor[motorD] = -speed;
		}
		else
		{
			motor[motorA] = speed;
			motor[motorD] = speed;
		}
		if(end_game != true)
		{
			detect(total_degrees, x, y, x_playfield, y_playfield);
		}
		wait1Msec(30);
	}
}


//initiates a random movement
void movement(float &x, float &y, int &total_degrees, int &x_playfield, int &y_playfield)
{
		string movement_types[4] = {"forward", "back", "right", "left"};
		int movement_magnitudes[4] = {50, 60, 75, 90};
		int spin_magnitudes[7] = {120, 150, 180, 210, 240, 270, 300};

		int movement_magnitude;
		int spin_magnitude;

		zero_all();

		string movement_type = movement_types[random(3)];

		if(movement_type == "forward" || movement_type == "back")
		{
			movement_magnitude = movement_magnitudes[random(3)];
			if(prediction(total_degrees, movement_magnitude, x, y, x_playfield, y_playfield, movement_type))
			{
				if(movement_type == "forward")
				{
					accel(movement_type, total_degrees, x, y, x_playfield, y_playfield);
					nMotorEncoder[motorA] = 0;
					while((abs(nMotorEncoder[motorA])*8*PI/360)<movement_magnitude-45 && end_game != true)
					{
						detect(total_degrees, x, y, x_playfield, y_playfield);
					}
					decel(movement_type, total_degrees, x, y, x_playfield, y_playfield);
					motor[motorA] = motor[motorD] = 0;
				}
				else
				{
					accel(movement_type, total_degrees, x, y, x_playfield, y_playfield);
					nMotorEncoder[motorA] = 0;
					while((abs(nMotorEncoder[motorA])*8*PI/360)<movement_magnitude-45 && end_game != true)
					{
						detect(total_degrees, x, y, x_playfield, y_playfield);
					}
					decel(movement_type, total_degrees, x, y, x_playfield, y_playfield);
					motor[motorA] = motor[motorD] = 0;
				}
				update_coordinates(total_degrees, movement_magnitude, x, y, movement_type);
			}
		}

		else
		{
			spin_magnitude = spin_magnitudes[random(6)];
			if(movement_type == "right")
			{
				motor[motorA] = -10;
				motor[motorD] = 10;
				while(abs(getGyroDegrees(S2))< spin_magnitude && end_game != true)
				{
					detect(total_degrees, x, y, x_playfield, y_playfield);
				}
				motor[motorA] = motor[motorD] = 0;
				total_degrees -= spin_magnitude;
			}
			else
			{
				motor[motorA] = 10;
				motor[motorD] = -10;
				while(abs(getGyroDegrees(S2))<spin_magnitude && end_game != true)
				{
					detect(total_degrees, x, y, x_playfield, y_playfield);
				}
				motor[motorA] = motor[motorD] = 0;
				total_degrees += spin_magnitude;
			}
		}
}


//multitasking for a timer, making sure the robot stops after 20 seconds
task timer()
{
	clearTimer(T1);
	while(end_game != true)
	{
		int playtime = 20000;
		if(time1[T1] > playtime)
		{
			end_game = true;
			time = time1[T1] / 1000;
			reset_1();
		}
	}
}


//multitasking to ensure the robot stops if the touch sensor is pressed
task touch_sensor()
{
	while(end_game != true)
	{
		if (SensorValue[S1] == 1)
		{
			end_game = true;
			time = time1[T1] / 1000;
			reset_1();
		}
	}
}


//initiates the program
task main()
{

	while(SensorValue[S1]==0)
	{}
	while(SensorValue[S1]==1)
	{}

	float x = 100;
	float y = 100;
	int total_degrees = 90;

	int x_playfield = 200;
	int y_playfield = 200;

	calibrate_all();
	zero_all();

	wait1Msec(1000);

	pull_back();

	wait1Msec(1000);

	startTask(touch_sensor);

	wait1Msec(50);

	startTask(timer);

	wait1Msec(50);

	while(end_game != true)
	{
		movement(x, y, total_degrees, x_playfield, y_playfield);
	}
	motor[motorA] = motor[motorD] = 0;

	displayTextLine(7, "Time: %d", time);
	while(!getButtonPress(buttonAny))
	{}
	while(getButtonPress(buttonAny))
	{}
}
