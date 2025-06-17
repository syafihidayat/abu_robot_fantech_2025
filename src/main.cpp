#include <Arduino.h>
#include <config.h>
#include <Servo.h>
#include <pid.h>
#include <kinematic.h>
#include <odometry.h>
#include <imu.h>
#include <USBHost_t36.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>

void launcher();
void moveBase();
void setMotor(int cwPin, int ccwPin, float pwmVal);
void setMOTOR(int cwPin, int ccwPin, float pwmPin, float pwmVal);
template <int j>
void readEncoder();
void linearGo(float speed_go, float speed_break);
void upperRobot();
void parseJoystickData(String data);
void calculate_smooth_vel(double &current_output, double input_value, double deltaT, double max_acceleration_);
void dribble_pneumatic();
void button_list(uint32_t buttons, int joy_axis);
void Motor_UpDown(float speed_go, float speed_break);
void moveMotor(float angle, float pwm);
void shooterMotor();
void readEncoderA();
void readEncoderB();

void MotorJump();
void slinderGo();

USBHost usb_joy;
USBHub joy_hub(usb_joy);
USBHIDParser hid_joy(usb_joy);

#define COUNT_JOYSTICKS 4

JoystickController joy_control[COUNT_JOYSTICKS]{
	JoystickController(usb_joy), JoystickController(usb_joy),
	JoystickController(usb_joy), JoystickController(usb_joy)};
int user_axis[64];
uint32_t buttons_prev = 0;

USBDriver *drivers[] = {&joy_hub, &joy_control[0], &joy_control[1], &joy_control[2], &joy_control[3], &hid_joy};
#define CNT_DEVICES (sizeof(drivers) / sizeof(drivers[0]))
const char *driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]", "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joy_control[0], &joy_control[1], &joy_control[2], &joy_control[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers) / sizeof(hiddrivers[0]))
const char *hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};
bool show_changed_only = false;

uint8_t joystick_left_trigger_value[COUNT_JOYSTICKS] = {0};
uint8_t joystick_right_trigger_value[COUNT_JOYSTICKS] = {0};
uint64_t joystick_full_notify_mask = (uint64_t)-1;

Servo esc_first;
Servo esc_second;

int sign;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long prevT = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);														// try  another i2c wire/wire1
const int enca[5] = {MOTOR1_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A, launcher_up_A, launcher_down_A}; // MotorDrib_enca};
const int encb[5] = {MOTOR1_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B, launcher_up_B, launcher_down_B}; // MotorDrib_encb};
volatile long pos[5];
volatile long poseEnc_up;
volatile long poseEnc_down;

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID dribble(PWM_MIN, PWM_MAX, drib_kp, drib_ki, drib_kd);
PID launcher_up(0, 180, ESC_UP_KP, ESC_UP_KI, ESC_UP_KD);
PID launcher_down(0, 180, ESC_DOWN_KP, ESC_DOWN_KI, ESC_DOWN_KD);

Kinematic kinematic(
	Kinematic::LINO_BASE,
	MOTOR_MAX_RPS,
	MAX_RPS_RATIO,
	MOTOR_OPERATING_VOLTAGE,
	MOTOR_POWER_MAX_VOLTAGE,
	WHEEL_DIAMETER,
	ROBOT_DIAMETER);

Odometry odometry;

IMU imu_sensor;

void setup()
{
	Serial.begin(115200);
	usb_joy.begin();

	bno.begin();
	bno.setExtCrystalUse(true);

	esc_first.attach(esc_up, 1000, 2000);
	esc_second.attach(esc_down, 1000, 2000);

	for (int i = 0; i < 5; i++)
	{
		pinMode(cw[i], OUTPUT);
		pinMode(ccw[i], OUTPUT);

		pinMode(MOTOR_Up, OUTPUT);
		pinMode(MOTOR_Down, OUTPUT);

		analogWriteFrequency(cw[i], PWM_FREQUENCY);
		analogWriteFrequency(ccw[i], PWM_FREQUENCY);

		analogWriteFrequency(MOTOR_Up, PWM_FREQUENCY);
		analogWriteFrequency(MOTOR_Down, PWM_FREQUENCY);

		pinMode(MOTOR_RELOAD_INA, OUTPUT);
		pinMode(MOTOR_RELOAD_INB, OUTPUT);

		analogWriteFrequency(MOTOR_RELOAD_INA, PWM_FREQUENCY);
		analogWriteFrequency(MOTOR_RELOAD_INB, PWM_FREQUENCY);

		// pinMode(MotorDrib_CW, OUTPUT);
		// pinMode(MotorDrib_CCW, OUTPUT);

		// analogWriteFrequency(MotorDrib_CW, PWM_FREQUENCY);
		// analogWriteFrequency(MotorDrib_CCW, PWM_FREQUENCY);

		// pinMode(MotorDrib_enca, INPUT);
		// pinMode(MotorDrib_encb, INPUT);

		analogWriteResolution(PWM_BITS);
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);

		pinMode(enca[i], INPUT);
		pinMode(encb[i], INPUT);

		pinMode(launcher_up_A, INPUT);
		pinMode(launcher_up_B, INPUT);
		pinMode(launcher_down_A, INPUT);
		pinMode(launcher_down_B, INPUT);
		pinMode(enca[i], INPUT);
		pinMode(encb[i], INPUT);
	}

	pinMode(limitTop, INPUT_PULLUP);
	pinMode(limitBottom, INPUT_PULLUP);

	pinMode(solShoot, OUTPUT);

	digitalWrite(solShoot, LOW);

	// pinMode(proxi_Front, INPUT_PULLUP);
	// pinMode(proxi_Behind, INPUT_PULLUP);

	// pinMode(solDrib, OUTPUT);
	// pinMode(solGrip, OUTPUT);

	// pinMode(pwm_pin, OUTPUT);

	wheel1.ppr_total(COUNTS_PER_REV1);
	wheel2.ppr_total(COUNTS_PER_REV2);
	wheel3.ppr_total(COUNTS_PER_REV3);

	launcher_up.ppr_total(1024);
	launcher_down.ppr_total(1024);

	attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);

	attachInterrupt(digitalPinToInterrupt(launcher_up_A), readEncoderA, RISING);
	attachInterrupt(digitalPinToInterrupt(launcher_down_A), readEncoderB, RISING);
	// attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);

	esc_first.write(0);
	esc_second.write(0);

	pinMode(LED_PIN, OUTPUT);

	// setMotor(MOTOR_RELOAD_INA,MOTOR_RELOAD_INB, 0);
}

float toDeg(float rad)
{
	return rad * 360 / (11204);
}

int numbers[8]; // adjust size as needed
double value_upper_launcher = 0;
double value_lower_launcher = 0;
unsigned long launch_prevT = 0;

bool turn_on_roller = false;

float upper_target = 0;
float lower_target = 0;

String inputString;

int applyDeadzone(int value, int deadzone = 20)
{
	if (abs(value) < deadzone)
	{
		return 0;
	}
	return value;
}

// int sign= 0;
void loop()
{
	usb_joy.Task();

	unsigned long launch_currT = micros();
	float launch_dt = ((float)(launch_currT - launch_prevT)) / 1.0e6;

	int limit_A = digitalRead(limitBottom);
	int limit_B = digitalRead(limitTop);

	if (((limit_A == 1 && limit_B == 0) || (limit_A == 0 && limit_B == 1)) &&
		!(button.LB == 1))
	{
		setMotor(MOTOR_Up, MOTOR_Down, 0);
	}

	if (joy_control[0].available())
	{
		uint32_t buttons = joy_control[0].getButtons();
		button_list(buttons, joy_control[0].getAxis(9));

		joystick.axis1_x = applyDeadzone(joy_control[0].getAxis(1) - 128);
		joystick.axis1_y = applyDeadzone(joy_control[0].getAxis(0) - 128);
		joystick.axis0_x = applyDeadzone(joy_control[0].getAxis(5) - 128);
		joystick.axis0_y = applyDeadzone(joy_control[0].getAxis(2) - 128);

		MotorJump();
		moveBase();
		shooterMotor();
		slinderGo();
	}

	if (turn_on_roller)
	{
		upper_target = 50;
		lower_target = 50;
	}
	else
	{

		upper_target = 0;
		lower_target = 0;
	}

	float launcher_upper_controlled = launcher_up.control_speed(upper_target, -poseEnc_up, launch_dt);
	float launcher_lower_controlled = launcher_down.control_speed(lower_target, poseEnc_down, launch_dt);

	if (fabs(upper_target) < 0.02)
	{
		launcher_upper_controlled = 0.0;
	}
	if (fabs(lower_target) < 0.02)
	{
		launcher_lower_controlled = 0.0;
	}

	esc_first.write(abs(launcher_upper_controlled));
	esc_second.write(abs(launcher_lower_controlled));
	// Serial.print(200);
	// Serial.print(" , ");
	Serial.print(launcher_up.get_filt_vel());
	Serial.print(" , ");
	Serial.print(launcher_down.get_filt_vel());
	Serial.print(" , ");
	Serial.println(0);

	// Serial.print(poseEnc_up);
	// Serial.print(" , ");
	// Serial.println(poseEnc_down);

	launch_prevT = launch_currT;

	// launch_prevT = launch_currT;

	// setMotor( MOTOR_RELOAD_INA,MOTOR_RELOAD_INB,40);

	// int sensorValue2 = digitalRead(proxi_Behind);
	// int sensorValue1 = digitalRead(proxi_Front);

	// if (sensorValue1 == 0 && sign == 0){
	// 	Serial.print(sensorValue1);
	// 	setMotor( MOTOR_RELOAD_INA,MOTOR_RELOAD_INB, 170);
	// }
	// else if(sensorValue1 == 1 ){
	// 	sign = 1;

	// 	Serial.print(sensorValue1);
	// }

	// else if(sign == 1)
	// {
	// 	setMotor(MOTOR_RELOAD_INA,MOTOR_RELOAD_INB, -170);
	// 	if(sensorValue2 == 0 )
	// 	{
	// 		sign = 2;

	// 	}
	// }

	// else if (sign == 2)
	// {
	// 	setMotor(MOTOR_RELOAD_INA,MOTOR_RELOAD_INB, 0);

	// }

	// Serial.print(sensorValue1);
	// Serial.print(sensorValue2);
	// Serial.println();

	// if (digitalRead(proxi_Behind) == LOW) {
	// 	Serial.println("1");
	// }
	// else{
	// 	Serial.println("0");
	// }

	// digitalWrite(MOTOR_Up, -80);
	// delay(1000);
	// digitalWrite(MOTOR_Down, 80);
	// delay(1000);

	// if (digitalRead(limitBottom) == LOW){

	// 	Serial.print("1");
	// }else{

	// 	Serial.print("0");
	// }

	// Serial.println("moving");
	// setMotor(MotorDrib_CW,MotorDrib_CCW, 100);
	// digitalWrite(MotorDrib_CW,1);
	// digitalWrite(MotorDrib_CCW,0);
	// analogWrite()
	// delay(1000);
	// digitalWrite(MotorDrib_CW,0);
	// digitalWrite(MotorDrib_CCW,1);
	// delay(1000);
	// moveMotor(-1.5, 100);

	// Serial.println("motor");
	// Serial.println(moveMotor);

	// setMotor(22,23, 225 * -1);

	// setMotor(3,4, 225 * -1 );

	// setMotor(1,0, 225 * -1);

	// setMotor(22,23, 225);
}

bool buttonPressed = false;

// void moveMotor(float angle, float pwm)
// {
// 	while (true)
// 	{
// 		unsigned long currT = micros();
// 		float deltaT = ((float)(currT - prevT)) / 1.0e6;

// 		// float control_motor = dribble.control_angle(angle, pos[3], pwm, deltaT);
// 		float control_motor = dribble.control_speed(angle, pos[3], deltaT);

// 		// 184 range 170 - 185
// 		if (toDeg(pos[3]) < -170)
// 		{
// 			control_motor = dribble.control_speed(0, pos[3], deltaT);
// 			break;
// 		}

// 		setMotor(MotorDrib_CW, MotorDrib_CCW, control_motor);
// 		prevT = currT;
// 		// if (fabs(dribble.get_error()) < 7)
// 		// {
// 		// 	break;
// 		// }
// 		Serial.print("target ");
// 		Serial.print(angle);
// 		Serial.print(" pos ");
// 		Serial.print(toDeg(pos[3]));
// 		Serial.print(" pos2 ");
// 		Serial.print(dribble.get_filt_vel());
// 		Serial.print(" motor ");
// 		Serial.print(control_motor);
// 		Serial.print(" Error ");
// 		Serial.println(dribble.get_error());
// 	}

// 	setMotor(MotorDrib_CW, MotorDrib_CCW, 0);
// }

// void move2Motor(float angle, float pwm)
// {
// 	while (true)
// 	{
// 		unsigned long currT = micros();
// 		float deltaT = ((float)(currT - prevT)) / 1.0e6;

// 		float controlled_motor = dribble.control_angle(angle, toDeg(pos[3]), pwm, deltaT);
// 		setMotor(MotorDrib_CW, MotorDrib_CCW, controlled_motor);
// 		prevT = currT;

// 		if (fabs(dribble.get_error()) < 7)
// 		{

// 			break;
// 		}
// 	}
// }

void Motor_UpDown(float speed_go, float speed_break)
{

	int limit_A = digitalRead(limitBottom);
	int limit_B = digitalRead(limitTop);

	if (speed_go > 0)
	{
		if (limit_A == 0 && limit_B == 1)
		{

			// breakMotor
		}
		else
		{
			setMotor(MOTOR_Up, MOTOR_Down, speed_go);
		}
	}
	else if (speed_go < 0)
	{
		if (limit_A == 1 && limit_B == 0)
		{

			// breakMotor
		}
		else
		{

			setMotor(MOTOR_Up, MOTOR_Down, speed_go);
		}
	}
	else
	{
		setMotor(MOTOR_Up, MOTOR_Down, 0);
	}
}

// void LinearMotor(float speed_go, float speed_break)
// {
// 	int proxi_A = digitalRead(proxi_Front);
// 	int proxi_B = digitalRead(proxi_Behind);

// 	if(speed_go > 0)
// 	{
// 		if(proxi_A == 1 && proxi_B == 0)
// 		{
// 			//break motor

// 		}
// 		else
// 		{
// 			setMotor(MOTOR_RELOAD_INA,MOTOR_RELOAD_INB, speed_go);

// 		}
// 	}else if(speed_go < 0)
// 	{
// 		if(proxi_A == 0 && proxi_B == 1)
// 		{
// 			//break motor
// 		}
// 		else
// 		{
// 			setMotor(MOTOR_RELOAD_INA, MOTOR_RELOAD_INB,speed_go);
// 		}
// 	}
// 	else
// 	{
// 		setMotor(MOTOR_RELOAD_INA,MOTOR_RELOAD_INB,0);

// 	}
// }

// bool dribble_once = false;
bool buttonRTPressed = false;
bool buttonLTPressed = false;
bool solGripOn = false;

enum PneumaticState
{
	IDLE,
	STEP1,
	STEP2,
	STEP3,
	STEP4,
	STEP_WAIT,
	DONE
};
PneumaticState state = IDLE;
unsigned long stateTime = 0;

// void dribble_pneumatic()
// {

// 	if (state == IDLE)
// 	{
// 		digitalWrite(solDrib, HIGH);
// 	}

// 	if (state == IDLE || state == STEP1)
// 	{
// 		digitalWrite(solGrip, solGripOn ? LOW : HIGH);
// 	}

// 	if (button.RT == 1 && buttonRTPressed == false)
// 	{
// 		Serial.print("|button RT|");
// 		// digitalWrite(solGrip, LOW);
// 		// delay(90);
// 		solGripOn = !solGripOn;
// 		buttonRTPressed = true;
// 	}
// 	else if (button.RT == 0 && buttonRTPressed == true)
// 	{
// 		buttonRTPressed = false;
// 	}

// 	if (button.LT == 1 && buttonLTPressed == false && state == IDLE)
// 	{
// 		Serial.print("| button LT|");

// 		moveMotor(-1.5, 90);
// 		// digitalWrite(solDrib, LOW);
// 		// digitalWrite(solGrip, HIGH);
// 		// solGripOn = false;
// 		state = STEP_WAIT;
// 		stateTime = millis();
// 		buttonLTPressed = true;
// 	}

// 	else if (button.LT == 0 && buttonLTPressed == true)
// 	{
// 		buttonLTPressed = false;
// 	}

// 	if (state == STEP_WAIT && millis() - stateTime >= 1000)
// 	{
// 		digitalWrite(solDrib, LOW);
// 		digitalWrite(solGrip, HIGH);
// 		solGripOn = false;
// 		state = STEP1;
// 		stateTime = millis();
// 	}

// 	if (state == STEP1 && millis() - stateTime >= 150)
// 	{
// 		digitalWrite(solDrib, HIGH);
// 		state = STEP2;
// 		stateTime = millis();
// 	}

// 	if (state == STEP2 && millis() - stateTime >= 70)
// 	{
// 		digitalWrite(solGrip, HIGH);
// 		move2Motor(-90, 90);
// 		solGripOn = true;
// 		state = STEP3;
// 		stateTime = millis();
// 	}

// 	if (state == STEP3 && millis() - stateTime >= 230)
// 	{
// 		// digitalWrite(solGrip, solGripOn ? LOW : HIGH);
// 		state = IDLE;
// 	}

// if(button.LT == 1 && buttonLTPressed == false){

// 	moveMotor(180,150); //moveMotor(180,150);
// 	digitalWrite(solDrib, HIGH);
// 	delay(20);
// 	digitalWrite(solGrip, HIGH);
// 	delay(70);
// 	digitalWrite(solDrib, LOW);
// 	delay(20);

// 	buttonLTPressed = true;

// }else if(button.LT == 0){

// 	buttonLTPressed = false;

// }

// if (button.RT == 1 && buttonPressed == false){

// 	Serial.print(" | button RT |");
// 	// moveMotor(180, 150);
// 	digitalWrite(solGrip, LOW);
// 	delay(10);
// 	buttonPressed = true;

// }else if (button.RT == 0 && buttonPressed == true){

// 	buttonPressed = false;

// }
// if(button.LT == 1 && cmd_to_dribble == 4){

// 	Serial.print("| button LT |");
// 	digitalWrite(solDrib,LOW);
// 	moveBase();
// 	delay(20);
// 	cmd_to_dribble = 0;

// }else if (cmd_to_dribble == 1){

// 	moveMotor(180, 150); //moveMotor(-180, 150);
// 	Serial.print("| cmd2dribble 1 |");
// 	// if (trig)

// }
// }

// enum PneumaticState{IDLE, STEP1, STEP2, STEP3, DONE};
// PneumaticState state = IDLE;
// unsigned long stateTime = 0;

// void dribble_pnematic(){

// 	if (button.RT == 1 && buttonRTPressed == false)
// 	{
// 		Serial.print("| buttonRT |");
// 		digitalWrite(solGrip, LOW);
// 		delay(10);
// 		buttonRTPressed = true;
// 	}
// 	if (button.RT == 0)
// 	{
// 		buttonRTPressed = false;
// 	}
// 	if(button.LT == 1 && state == IDLE)
// 	{
// 		Serial.print("| buttonLT |");
// 		moveMotor(180, 150);
// 		digitalWrite(solDrib, LOW);
// 		state = STEP1;
// 		stateTime = millis();
// 	}
// 	if (state == STEP1 && millis() - stateTime >= 20)
// 	{
// 		digitalWrite(solGrip, LOW);
// 		state = STEP2;
// 		stateTime = millis();

// 	}
// 	if (state == STEP2 && millis() - stateTime >= 70)
// 	{
// 		digitalWrite(solDrib, HIGH);
// 		state = DONE;
// 		stateTime = millis();
// 	}
// 	if (state == DONE && millis() - stateTime >= 20)
// 	{
// 		state = IDLE;

// 	}

// }

bool RBpressed = false;
bool prevRBpressed = false;

bool LTpressed = false;
bool prevLTpressed = false;

void shooterMotor()
{

	if (button.RB)
	{
		turn_on_roller = true;
	}
	else if (button.LT)
	{
		turn_on_roller = false;
	}
	// 	if (button.RB == 1 && prevRBpressed == false && button.LT == 0 && prevLTpressed == false)
	// 	{
	// 		if (RBpressed == false)
	// 		{
	// 			turn_on_roller = true;
	// 			RBpressed = true;
	// 			LTpressed = false;
	// 		}
	// 		else if(button.RB == 0 && prevRBpressed == true && buttonLTPressed == 1 && prevLTpressed == false);
	// 		{
	// 			turn_on_roller = false;
	// 			LTpressed = true;
	// 			RBpressed = false;
	// 		}
}

// esc_first.write(value_upper_launcher);
// esc_second.write(value_lower_launcher);
// prevRBpressed = button.RB == 1;
// prevLTpressed = button.LT == 1;
// }

bool LBpressed = false;
bool prevLBpressed = false;

void MotorJump()
{
	if (button.LB == 1 && prevLBpressed == false)
	{
		if (LBpressed == false)
		{
			Motor_UpDown(-140, 0);
			LBpressed = true;
		}
		else
		{
			Motor_UpDown(100, 0);
			LBpressed = false;
		}
	}

	prevLBpressed = button.LB == 1;
}

bool Xpressed = false;
bool prevXpressed = false;

bool solenoidActive = false;
unsigned long solenoidStartTime = 0;
const unsigned long solenoidDuration = 500;

void slinderGo()
{
	if (button.X == 1 && prevXpressed == false && solenoidActive == false)
	{
		digitalWrite(solShoot, HIGH);
		solenoidStartTime = millis();
		solenoidActive = true;
		Xpressed = true;
	}

	if (solenoidActive && millis() - solenoidStartTime >= solenoidDuration)
	{
		digitalWrite(solShoot, LOW);

		solenoidActive = false;
		Xpressed = false;
	}

	prevXpressed = button.X;
}

// void motorGo()
// {
// 	if(button.X == 1 && prevXpressed == false)
// 	{
// 		LinearMotor(80, 0);
// 		Xpressed = true;
// 	}
// 	else
// 	{
// 		LinearMotor(-80,0);
// 		Xpressed = false;
// 	}

// 	prevXpressed = button.X;
// }

joy joySmoothed;
void moveBase()
{
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

	// Serial.print("imu");
	// Serial.print(event.orientation.x);
	// Serial.println(",");

	unsigned long currT = micros();
	float deltaT = ((float)(currT - prevT)) / 1.0e6;

	joystick.axis1_x = map(joystick.axis1_x, 0, 128, 0, 2);
	joystick.axis1_y = map(joystick.axis1_y, 0, 128, 0, 2);
	joystick.axis0_y = map(joystick.axis0_y, 0, 128, 0, 2);

	calculate_smooth_vel(joySmoothed.axis1_x, joystick.axis1_x, deltaT, 1.5);
	calculate_smooth_vel(joySmoothed.axis1_y, joystick.axis1_y, deltaT, 1.5);
	calculate_smooth_vel(joySmoothed.axis0_y, joystick.axis0_y, deltaT, 4.0); // 5.0
	// Serial.print("robot cmdVel=>");
	// Serial.print(joystick.axis1_x);
	// Serial.print(",");
	// Serial.print(joystick.axis1_y);
	// Serial.print(",");
	// Serial.print(joystick.axis0_y);
	// Serial.println();

	// Kinematic::rps req_rps;
	// req_rps = kinematic.getRPS(
	// 	-joySmoothed.axis1_x,
	// 	-joySmoothed.axis1_y,
	// 	joySmoothed.axis0_y,
	// 	-event.orientation.x);

	Kinematic::rps req_rps;
	req_rps = kinematic.getRPS(
		joySmoothed.axis1_x * -1,
		joySmoothed.axis1_y,
		joySmoothed.axis0_y,
		event.orientation.x);

	float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
	float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
	float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);

	// Serial.print(req_rps.motor1 * 25);
	// Serial.println("");
	// Serial.print(req_rps.motor2 * 25);
	// Serial.println("");
	// Serial.print(req_rps.motor3 * 25);
	// Serial.println("");

	// Serial.print("r");
	// Serial.print(req_rps.motor1);
	// Serial.print(",");
	// Serial.print(req_rps.motor2);
	// Serial.print(",");
	// Serial.print(req_rps.motor3);
	// Serial.println(",");

	// Serial.print(pos[0]);
	// Serial.print(" , ");
	// Serial.print(pos[1]);
	// Serial.print(" , ");
	// Serial.print(pos[2]);
	// Serial.print(" , ");
	// Serial.print(poseEnc_up);
	// Serial.print(" , ");
	// Serial.print(poseEnc_down);
	// Serial.println("  ");

	// Serial.print(pos[0]);
	// Serial.println(" ");
	// Serial.print(pos[1]);
	// Serial.println(" ");
	// Serial.print(pos[2]);
	// Serial.println(" ");

	float current_rps1 = wheel1.get_filt_vel();
	float current_rps2 = wheel2.get_filt_vel();
	float current_rps3 = wheel3.get_filt_vel();

	// Serial.print("c");
	// Serial.print(current_rps1);
	// Serial.print(",");
	// Serial.print(current_rps2);
	// Serial.print(",");
	// Serial.print(current_rps3);
	// Serial.println(",");

	if (fabs(req_rps.motor1) < 0.02)
	{
		controlled_motor1 = 0.0;
	}
	if (fabs(req_rps.motor2) < 0.02)
	{
		controlled_motor2 = 0.0;
	}
	if (fabs(req_rps.motor3) < 0.02)
	{
		controlled_motor3 = 0.0;
	}

	// setMotor(cw[0], ccw[0], req_rps.motor1 * 25);
	// setMotor(cw[1], ccw[1], req_rps.motor2 * 25);
	// setMotor(cw[2], ccw[2], req_rps.motor3 * 25);

	setMotor(cw[0], ccw[0], controlled_motor1);
	setMotor(cw[1], ccw[1], controlled_motor2);
	setMotor(cw[2], ccw[2], controlled_motor3);

	Kinematic::velocities vel = kinematic.getVelocities(
		current_rps1,
		current_rps2,
		current_rps3);

	unsigned long now = millis();
	float vel_dt = (now - prev_odom_update) / 1000.0;
	prev_odom_update = now;
	odometry.update(
		vel_dt,
		vel.linear_x,
		vel.linear_y,
		vel.angular_z);

	prevT = currT;
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
	if (pwmVal > 0)
	{
		analogWrite(cwPin, fabs(pwmVal));
		analogWrite(ccwPin, 0);
	}
	else if (pwmVal < 0)
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, fabs(pwmVal));
	}
	else
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, 0);
	}
}

// void setMOTOR(int cwPin, int ccwPin, float pwmPin, float pwmVal)
// {
// 	if(pwmVal > 0){
// 		digitalWrite(cwPin, 1);
// 		digitalWrite(ccwPin, 0);
// 		analogWrite(pwmPin, fabs(pwmVal));
// 	}else if (pwmVal < 0){
// 		digitalWrite(cwPin, 0);
// 		digitalWrite(ccwPin, 1);
// 		analogWrite(pwmPin, fabs(pwmVal));
// 	}else{
// 		digitalWrite(cwPin, 0);
// 		digitalWrite(cwPin, 0);
// 		analogWrite(pwmPin, 0);
// 	}

// }

template <int j>
void readEncoder()
{
	int b = digitalRead(encb[j]);
	if (b > 0)
	{
		pos[j]++;
	}
	else
	{
		pos[j]--;
	}
}

void readEncoderA()
{
	int b = digitalRead(launcher_up_B);
	if (b > 0)
	{
		poseEnc_up--;
	}
	else
	{
		poseEnc_up++;
	}
}

void readEncoderB()
{
	int b = digitalRead(launcher_down_B);
	if (b > 0)
	{
		poseEnc_down++;
	}
	else
	{
		poseEnc_down--;
	}
}

void calculate_smooth_vel(double &current_output, double input_value, double deltaT, double max_acceleration_)
{
	double target = input_value;

	if (input_value == 0.0 && std::abs(current_output) > 1e-6)
	{
		target = 0.0;
	}
	double max_delta = max_acceleration_ * deltaT;

	double delta_target = target - current_output;

	if (std::abs(delta_target) > max_delta)
	{
		delta_target = (delta_target > 0.0) ? max_delta : -max_delta;
	}

	current_output += delta_target;
}
void button_list(uint32_t buttons, int joy_axis)
{
	/*
	A = 2
	B = 4
	X = 1
	Y = 8
	RB = 20
	Lb = 10
	LT = 40
	RT = 80
	select = 100
	home  = 1000
	start = 200
	*/
	switch (buttons)
	{
	case 2:
		button.A = 1;
		break;

	case 4:
		button.B = 1;
		break;

	case 1:
		button.X = 1;
		break;

	case 8:
		button.Y = 1;
		break;

	case 32:
		button.RB = 1;
		break;

	case 16:
		button.LB = 1;
		break;

	case 64:
		button.LT = 1;
		break;

	case 128:
		button.RT = 1;
		break;

	case 256:
		button.select = 1;
		break;

	case 4096:
		button.home = 1;
		break;

	case 512:
		button.start = 1;
		break;
	default:
		button.A = 0;
		button.B = 0;
		button.X = 0;
		button.Y = 0;
		button.RB = 0;
		button.LB = 0;
		button.LT = 0;
		button.RT = 0;
		button.select = 0;
		button.home = 0;
		button.start = 0;
		break;
	}
	switch (joy_axis)
	{
	case 0:
		button.up = 1;
		break;
	case 2:
		button.right = 1;
		break;
	case 4:
		button.down = 1;
		break;
	case 6:
		button.left = 1;
		break;
	case 15:
		button.up = 0;
		button.down = 0;
		button.left = 0;
		button.right = 0;
		break;
	}
}