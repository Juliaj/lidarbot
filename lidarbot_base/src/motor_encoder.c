// This file calculate encoder ticks and sets the speed each motor should run at according to
// the commands received from the Waveshare Motor Driver system hardware component

#include "lidarbot_base/motor_encoder.h"
#include <math.h>

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

// Initialize wheel directions
// 1 - forward, 0 - backward
int left_wheel_direction = 1;
int right_wheel_direction = 1;

// Read wheel encoder values
void read_encoder_values(int *left_encoder_value, int *right_encoder_value) {
  *left_encoder_value = left_wheel_pulse_count;
  *right_encoder_value = right_wheel_pulse_count;
}

// Left wheel callback function
void left_wheel_pulse() {
  // left wheel direction
  // 1 - forward
  // 0 - backward

  // Read encoder direction value for left wheel
  left_wheel_direction = digitalRead(LEFT_WHL_ENC_DIR);
  // printf("Left wheel direction: %d, pin: %d\r\n", left_wheel_direction, LEFT_WHL_ENC_DIR);

  if (left_wheel_direction == 1)
    left_wheel_pulse_count++;
  else
    left_wheel_pulse_count--;

  // printf("Left wheel pulse count: %d\r\n", left_wheel_pulse_count);
}

// Right wheel callback function
void right_wheel_pulse() {
  // right wheel direction
  // 1 - forward,
  // 0 - backward

  // Read encoder direction value for right wheel
  right_wheel_direction = digitalRead(RIGHT_WHL_ENC_DIR);
  // printf("Right wheel direction: %d, pin: %d\r\n", right_wheel_direction, RIGHT_WHL_ENC_DIR);

  // for whatever the reason, the right wheel direction is reversed
  if (right_wheel_direction == 0)
    right_wheel_pulse_count++;
  else
    right_wheel_pulse_count--;
  // printf("Right wheel pulse count: %d\r\n", right_wheel_pulse_count);
}

// Set each motor speed from the respective velocity command interface
void set_motor_speeds(double left_wheel_command, double right_wheel_command) {
  printf("Left wheel command: %f, Right wheel command: %f\r\n", left_wheel_command, right_wheel_command);
  
  // Initialize DIR enum variables
  DIR left_motor_direction;
  DIR right_motor_direction;

  // Tune motor speeds by adjusting the command coefficients. These are
  // dependent on the number of encoder ticks. 3000 ticks and above work well
  // with coefficients of 1.0
  double left_motor_speed = ceil(left_wheel_command * 1.65);
  double right_motor_speed = ceil(right_wheel_command * 1.65);

  // Set motor directions
  if (left_motor_speed > 0)
    left_motor_direction = FORWARD;
  else
    left_motor_direction = BACKWARD;

  if (right_motor_speed > 0)
    right_motor_direction = FORWARD;
  else
    right_motor_direction = BACKWARD;

  // Run motors with specified direction and speeds
  Motor_Run(MOTORA, left_motor_direction, (int)abs(left_motor_speed));
  Motor_Run(MOTORB, right_motor_direction, (int)abs(right_motor_speed));
  printf("Left motor speed: %f, Right motor speed: %f\r\n", left_motor_speed, right_motor_speed);
  printf("Left motor direction: %d, Right motor direction: %d\r\n", left_motor_direction, right_motor_direction);
  
}

void handler(int signo) {
  Motor_Stop(MOTORA);
  Motor_Stop(MOTORB);

  exit(0);
}
