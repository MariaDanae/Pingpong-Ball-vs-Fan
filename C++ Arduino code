int x, x0, index;
int calibrate;
const int num = 30;					// was 10
double height, actual_height, max_height, previous_height;
double previous_error, previous_time, current_time, change_in_time;
double error, kp, integral, ki, derivative, kd;
double total, average_height;
double readings[num];
long duration;			//try int**********
bool start;

void two()
{							// Because the Ultrasonic sensor is opposite to the fan
	height = 80.0; //70.0;  //(8 / 10)*(max_height);		// 20% of max height is 80% from the top down	
}

void four()
{
	height = 60.0; //52.0; // (6 / 10)*max_height;		// 40% of max height is 60% from the top down	
}

void six()
{
	height = 40.0; // 35.0; // (4 / 10)*max_height;		// 60% of max height is 40% from the top down	
}

void eight()
{
	height = 20.0; //17.0;  // (2 / 10)*max_height; 		// 80% of max height is 20% from the top down
}

void setup()
{
	pinMode(10, OUTPUT);     		// pwn output   

	attachInterrupt(2, two, RISING);   	// pin 21               
	attachInterrupt(3, four, RISING);    	// pin 20            
	attachInterrupt(4, six, RISING);    	// pin 19            
	attachInterrupt(5, eight, RISING);    	// pin 18            

	height = 40.0;
	actual_height = 0.0;
	max_height = 95.0;			//range in cm
	x = 0;		
	x0 = 170;
	start = true;
	calibrate = 0;

	kp = 0.25;			// was 0.25 .. and before that was 0.1
	ki = 0.1;
	kd = 200.0;			// was 200

	error = 0.0;
	integral = 0.0;
	derivative = 0.0;

	previous_error = 0.0;
	previous_time = micros();

	//arduino.cc/en/Tutorial/Smoothing

	index = 0;			//index of current reading
	total = 0.0;			//current total
	average_height = 0.0;
	previous_height = 0.0;

	for (int reading = 0; reading < num; reading++) readings[reading] = 0;	 //initialises all 10 samples to 0

	Serial.begin(9600);    		// helps you keep track of the values
}

void loop()
{
	
	if (calibrate <= 9)
	{
		while (start == true)
		{
			x = 255;	//initial input
			Serial.println("        Initializing        ");
			analogWrite(10, x);
			delay(2000);

			x = x0;
			start = false;
		}

		if ((previous_height < 35) )
		{
			analogWrite(10, 250);
			delay(500);
			x++;
			calibrate=0;
	
		}
		else if ((previous_height > 80) )
		{
			analogWrite(10, 80);
			delay(200);
			x--;
			calibrate=0;
		}
		else if (average_height <= previous_height+1 && average_height >= previous_height-1) calibrate++;
	
		if (calibrate == 10)
		{
			x0 = x;
			Serial.print("        Calibrated        ");
			Serial.println(x0);
		}
	}
	
	previous_height = average_height;

	analogWrite(10, x);

	
	while (index < num)	//0-9
	{
		pinMode(50, OUTPUT);
		digitalWrite(50, LOW);		// Give a short LOW pulse beforehand to ensure a clean HIGH pulse
		delayMicroseconds(2);
		digitalWrite(50, HIGH);		// The PING is triggered by a HIGH pulse of 2 or more microseconds
		delayMicroseconds(5);
		digitalWrite(50, LOW);

		pinMode(50, INPUT);		// The same pin is used to read the signal from the PING
		duration = pulseIn(50, HIGH);
		actual_height = (max_height - (duration / 29 / 2));

		delay(50);
		readings[index] = actual_height;
		total += readings[index];
		index++;
	}

	average_height = (total / num);		//calculates average of 10 heights
	total = 0.0;
	index = 0;
	// time delay might hinder time based response, majority kp control
	current_time = micros();
	change_in_time = (current_time - previous_time);
	previous_time = current_time;

	error = (height - average_height);
	integral += (error / change_in_time);
	derivative = ((error - previous_error) / change_in_time);
	previous_error = error;

	if (calibrate >= 10)
	{

		if ((error <= 10) && (error >= -10))			// for steady state
		{
			x += kp*error + ki*integral + kd*derivative;	//pwm can only have 0-255 (int)
		}


		else if (error < -10)		// if the ball needs to go down
		{
			//analogWrite(10, (x0-40));		//want to make smoother 
			analogWrite(10, 80);
			delay(average_height*1.5);			// was 150
			x = x0;
		}

		else if (error > 10)	// if ball needs to go up
		{
			//analogWrite(10, (x0 + 50));			//want to make smoother 
			analogWrite(10, 255);
			delay(150+average_height*1.5);
			x = x0;
		}
if (average_height >91)
		{
		analogWrite(10, 80);
		delay(2500);
		}
		if (average_height <12)
		{
		analogWrite(10, 255);
		delay(1000);
		}
	}
	
		

	if (x > 255)
	{
		x = 255;
	}
	if (x < 1)
	{
		x = 1;
	}
	
	if (calibrate < 10)
	{
		Serial.print(actual_height);
		Serial.print("  cm  act      pwm: ");
		Serial.print(x);
		Serial.print("     Calibrate: ");
		Serial.println(calibrate);
	}
	
	if (calibrate >= 10)
	{
		Serial.print(actual_height);
		Serial.print("  cm act        ");
		Serial.print(height);
		Serial.print("  cm  set      pwm: ");
		Serial.println(x);
	}

	delay(10);

	//Serial.println (x);   		// control + shift + m 
}
