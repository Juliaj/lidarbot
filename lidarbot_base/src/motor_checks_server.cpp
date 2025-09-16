// Server node runs motor tests to confirm that the motor is working correctly by moving both motors forward

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lidarbot_base/motor_encoder.h"

// Reset pulse counters
void reset_pulse_counters()
{
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
}

// Move a specified motor forward and return pulse count
int move_motor(int motor_id, int duration_seconds)
{
    // Motor_id
    // MOTORA - 0 (left motor)
    // MOTORB - 1 (right motor)
    reset_pulse_counters();
    sleep(1);  // Brief delay to ensure clean start

    printf("Running motor %s for %d seconds at 50%% speed...\n",
           motor_id == MOTORA ? "LEFT" : "RIGHT", duration_seconds);

    // Move motor FORWARD for 10 seconds at 50% speed
    Motor_Run(motor_id, FORWARD, 50);
    sleep(duration_seconds);
    int left_wheel_pulse_count_tested = left_wheel_pulse_count;
    int right_wheel_pulse_count_tested = right_wheel_pulse_count;
    Motor_Stop(motor_id);

    // Return the pulse count for the specified motor
    if (motor_id == MOTORA) {
        printf("Left motor pulse count: %d\n", left_wheel_pulse_count_tested);
        return left_wheel_pulse_count_tested;
    }
    if (motor_id == MOTORB) {
        printf("Right motor pulse count: %d\n", right_wheel_pulse_count_tested);
        return right_wheel_pulse_count_tested;
    }
    
    return 0;  // Should never reach here
}

// DDS helps pass the request and response between client and server
void checkMotors(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    int duration_seconds = 2;
    // Prepare response
    response->success = true;
    response->message = "";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to check motors...");

    printf("\n=== Motor Speed Comparison Test ===\n");

    // Left motor check
    printf("Checking left motor...\n");
    int left_pulse_count_tested = move_motor(MOTORA, duration_seconds);
    if (left_pulse_count_tested <= 0) {
        response->success = false;
        response->message += "Left motor check failed, confirm motor wiring. ";
        printf("❌ Left motor did not move forward, pulse count: %d\n", left_pulse_count_tested);
    }
    else {
        printf("✅ Left motor moved forward, pulse count: %d\n", left_pulse_count_tested);
    }

    printf("\n");  // Add spacing between tests

    // Right motor check
    printf("Checking right motor...\n");
    int right_pulse_count_tested = move_motor(MOTORB, duration_seconds);
    if (right_pulse_count_tested <= 0) {
        response->success = false;
        response->message += "Right motor check failed, confirm motor wiring. ";
        printf("❌ Right motor did not move forward, pulse count: %d\n", right_pulse_count_tested);
    }
    else {
        printf("✅ Right motor moved forward, pulse count: %d\n", right_pulse_count_tested);
    }

    // Compare motor speeds if both motors moved
    if (left_pulse_count_tested > 0 && right_pulse_count_tested > 0) {
        printf("\n=== Speed Comparison Results ===\n");
        printf("Left motor:  %d pulses\n", left_pulse_count_tested);
        printf("Right motor: %d pulses\n", right_pulse_count_tested);
        
        int speed_difference = abs(left_pulse_count_tested - right_pulse_count_tested);
        double speed_ratio = (double)speed_difference / ((left_pulse_count_tested + right_pulse_count_tested) / 2.0) * 100.0;
        
        printf("Speed difference: %d pulses\n", speed_difference);
        printf("Speed variance: %.1f%%\n", speed_ratio);
        
        if (speed_ratio > 15.0) {
            printf("⚠️  WARNING: Significant speed difference detected!\n");
            if (left_pulse_count_tested > right_pulse_count_tested) {
                printf("   Left motor is %.1f%% faster than right motor\n", 
                       ((double)left_pulse_count_tested / right_pulse_count_tested - 1.0) * 100.0);
            } else {
                printf("   Right motor is %.1f%% faster than left motor\n", 
                       ((double)right_pulse_count_tested / left_pulse_count_tested - 1.0) * 100.0);
            }
            printf("   Consider calibrating motor speeds\n");
            response->message += "Speed difference exceeds 15% threshold. ";
        } else {
            printf("✅ Motors running at similar speeds (%.1f%% variance)\n", speed_ratio);
        }
    }

    if (response->success) {
        response->message += "Motor test completed successfully.";
        printf("\n✅ OVERALL TEST: PASSED\n");
    } else {
        printf("\n❌ OVERALL TEST: FAILED\n");
    }

    printf("\n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response...");
}

int main(int argc, char **argv)
{
    // Initialize motor driver
    Motor_Init();

    // Initialize wiringPi using GPIO BCM pin numbers
    wiringPiSetupGpio();

    // Setup GPIO encoder interrupt and direction pins
    pinMode(LEFT_WHL_ENC_INT, INPUT);
    pinMode(RIGHT_WHL_ENC_INT, INPUT);
    pinMode(LEFT_WHL_ENC_DIR, INPUT);
    pinMode(RIGHT_WHL_ENC_DIR, INPUT);

    // Setup pull up resistors on encoder pins
    pullUpDnControl(LEFT_WHL_ENC_INT, PUD_UP);
    pullUpDnControl(RIGHT_WHL_ENC_INT, PUD_UP);

    // Initialize encoder interrupts for falling signal states
    wiringPiISR(LEFT_WHL_ENC_INT, INT_EDGE_FALLING, left_wheel_pulse);
    wiringPiISR(RIGHT_WHL_ENC_INT, INT_EDGE_FALLING, right_wheel_pulse);

    // Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // Create a shared pointer to a Node type and name it "motor_checks_server"
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motor_checks_server");

    // Create a "checks" service with a checkMotors callback
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
        node->create_service<std_srvs::srv::Trigger>("checks", &checkMotors);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to check motors");

    // Spin the node until it's terminated
    rclcpp::spin(node);
    rclcpp::shutdown();
}
