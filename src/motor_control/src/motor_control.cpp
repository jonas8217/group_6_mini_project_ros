#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

#include <chrono>

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

#define MOTOR_CENTER    512
#define CAMERA_CENTER   695
#define MOUNT_DIFF      MOTOR_CENTER - CAMERA_CENTER;



uint8_t *inp_buff;
uint8_t *out_buff;

int sign(float n1, float n2)
{
    return (n1 > n2) * 2 - 1;
}

class MotorControl : public rclcpp::Node
{
	public:
		MotorControl() : Node("motor_control") {
			RCLCPP_INFO(this->get_logger(), "Initializing MotorControl node");

			RCLCPP_INFO(this->get_logger(), "Starting angle and screw type subscription");

			angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
                "/screw_angle",
                10,
                std::bind(&MotorControl::onAngleMsg, this, std::placeholders::_1)
			);

            screw_type_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
                "/screw_type",
                10,
                std::bind(&MotorControl::ontypeMsg, this, std::placeholders::_1)
			);

            motor_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
				"/set_position",
				10
			);

            updater = this->create_wall_timer(
                std::chrono::microseconds(1000000), 
                std::bind(&MotorControl::run_motor_controller, this)
            );
            
            // reset motors to middle
            commandMotor(0,0,true);
            commandMotor(1,0,true);
		}

	private:
		rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr motor_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr screw_type_subscription_;

        rclcpp::Duration duration = rclcpp::Duration(1,100000);
        rclcpp::TimerBase::SharedPtr updater;


        float angle = 0.0;
        uint8_t screw_type = 0;
        bool new_angle = false; 
        bool new_type = false;

        void onAngleMsg(const std_msgs::msg::Float32::SharedPtr msg) {
        	RCLCPP_INFO(this->get_logger(), "Received angle from angle detector");
            angle = msg->data;
            new_angle = true;
        }

        void ontypeMsg(const std_msgs::msg::Int16::SharedPtr msg) {
        	RCLCPP_INFO(this->get_logger(), "Received screw type from CNN");
            screw_type = msg->data;
            new_type = true;
        }

        void commandMotor(int id, float motor_angle, bool step = false)
        {
            auto msg = dynamixel_sdk_custom_interfaces::msg::SetPosition::SharedPtr();
            msg->id = id;
            if (step) {
                msg->position = motor_angle;
            }
            else
            {
                msg->position = 512 + motor_angle * 3.4;
            }


            motor_publisher_->publish(*msg.get());
            return;
        }

        int get_approach_step(float desired, float current)
        {
            float rate = 1;
            int dir = sign(desired, current);
            return current + dir * rate; 
        }

        void run_motor_controller()
        {
            RCLCPP_INFO(this->get_logger(), "Run motor controller");
            return;
            // positions 275 485 685 899
            const int positions[4] = {275, 485, 685, 899}; // magic numbers for center of each screw position for the camera
            static float pos_main = 0;
            static int pos_iter = 0;

            float desired_pos_main = (512 - positions[pos_iter])/3.4;
            
            float command_angle_main = get_approach_step(desired_pos_main, pos_main);
            commandMotor(1,command_angle_main);

        }


};



int main(int argc, char *argv[])
{

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<MotorControl>());

	rclcpp::shutdown();
	return 0;
}
