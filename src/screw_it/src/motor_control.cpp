#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


#include <chrono>

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

#define MOTOR_START         100 
#define CAMERA_START        270-10
#define CAMERA_END          900
#define MOUNT_DIFF          CAMERA_START - MOTOR_START;

#define STEPS_PER_DEG       (CAMERA_END - CAMERA_START) / 180.0

#define UPDATE_INTERVAL_US  50000
#define US_PER_SEC          1000000
#define STEPS_PER_SEC       US_PER_SEC / UPDATE_INTERVAL_US


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

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

			angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
                "/screw_angle",
                qos,
                std::bind(&MotorControl::onAngleMsg, this, std::placeholders::_1)
			);

            screw_type_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/CNN_screw_type",
                qos,
                std::bind(&MotorControl::ontypeMsg, this, std::placeholders::_1)
			);

            screw_type_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
				"/screw_type",
				qos
			);

            motor_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
				"/set_position",
				10
			);

            updater = this->create_wall_timer(
                std::chrono::microseconds(UPDATE_INTERVAL_US), 
                std::bind(&MotorControl::run_motor_controller, this)
            );
            
            this->declare_parameter("debug", "false");
            rclcpp::Parameter debug;
            this->get_parameter_or("debug", debug, rclcpp::Parameter("debug", "false"));
            std::string debugstr = debug.as_string();
            doDebug = debugstr == "true" ? true : false;

		}

	private:
		rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr motor_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr screw_type_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr screw_type_subscription_;

        rclcpp::TimerBase::SharedPtr updater;

        bool doDebug = true;

        float angle_ = 0.0;
        float screw_type_[4] = {0,0,0,0};
        bool new_angle_ = false; 
        bool new_type_ = false;

        void onAngleMsg(const std_msgs::msg::Float32::SharedPtr msg) {
            if (doDebug)
        	    RCLCPP_INFO(this->get_logger(), "Received angle from angle detector");
            angle_ = msg->data;
            new_angle_ = true;
        }

        void ontypeMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        	if (doDebug)
                RCLCPP_INFO(this->get_logger(), "Received screw type from CNN");
            for (int it = 0; it < 4; it++)
            {
                screw_type_[it] = msg->data[it];
            }
            new_type_ = true;
        }

        void commandMotor(int id, int motor_position, bool degrees = false)
        {
            dynamixel_sdk_custom_interfaces::msg::SetPosition msg;
            msg.id = id;
            if (degrees)
            {
                motor_position *= STEPS_PER_DEG;
                motor_position += 512;
            }
            msg.position = motor_position;
            motor_publisher_->publish(msg);
            return;
        }

        float get_approach_step(float desired, float current)
        {
            float deg_pr_sec = 30.0;
            float rate = deg_pr_sec/((float)STEPS_PER_SEC) * STEPS_PER_DEG;
            if ( abs(desired - current) <= rate )   
            {
                return desired;
            }
            int dir = sign(desired, current);
            return current + dir * rate; 
        }

        bool screw_angle_is_valid(int angle_count)
        {
            if (angle_count == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Could not identify screw angle");    
                return false;
            }
            return true;
        }

        bool screw_type_is_valid(float type_sum[4])
        {
            float test_sum = 0;
            for (int it = 0; it < 4; it++)
            {
                test_sum += type_sum[it];
            }
            if (test_sum == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Could not identify screw type");    
                return false;
            }
            return true;
        }

        void do_once()
        {
            // reset motors to middle
            RCLCPP_INFO(this->get_logger(), "First reset");
            commandMotor(0,512);
            rclcpp::sleep_for(std::chrono::nanoseconds(500000000)); // wait 0.5 seconds
            RCLCPP_INFO(this->get_logger(), "Second reset");
            commandMotor(1,CAMERA_START);
            rclcpp::sleep_for(std::chrono::nanoseconds(500000000)); // wait 0.5 seconds
        }

        void run_motor_controller()
        {
            static bool done_once = false;
            if (! done_once) {do_once(); done_once = true;}

            //RCLCPP_INFO(this->get_logger(), "Run motor controller");
            // magic numbers for center of each screw position for the camera
            // positions 275 485 685 899 (before tape)
            // positions 270 475 680 890 (with tape)
            
            const float motor_steps_per_slot = (CAMERA_END - CAMERA_START) / 3.0;
            static int pos_current = CAMERA_START;
            static int pos_iter = 0;
            static int i = 0;
            static int angle_count = 0;
            static float angle_sum = 0.0;
            static float type_sum[4] = {0, 0, 0, 0};
            static std::string type_names[4] = {"cross","flat","penta","square"};
            
            int desired_pos = CAMERA_START + pos_iter * motor_steps_per_slot;
            int command_pos = get_approach_step(desired_pos, pos_current);
            

            if (pos_current == desired_pos)
            {
                if (i == 0){
                    for (int it = 0; it < 4; it++)
                    {
                        type_sum[it] = 0;
                    }
                    new_type_ = false;
                }
                // wait for 1 second
                if (i == 1 * STEPS_PER_SEC)
                {
                    new_angle_ = false;
                    angle_count = 0;
                    angle_sum = 0.0;
                    if (doDebug)
                        RCLCPP_INFO(this->get_logger(), "Checking screw type");
                    if (screw_type_is_valid(type_sum))
                    {
                        
                        float max_val = 0;
                        int type = 0;
                        for (int it = 0; it < 4; it++)
                        {
                            if (type_sum[it] > max_val)
                            {
                                max_val = type_sum[it];
                                type = it;
                            }
                        }

                        RCLCPP_INFO(this->get_logger(), "Screw type: %s", type_names[type].c_str());

                        std_msgs::msg::Int16 msg;
                        msg.data = type;
                        screw_type_publisher_->publish(msg); // send message about screw type to angle detector
                    }

                }
                // wait an additional second
                else if (i == 2 * STEPS_PER_SEC)
                {
                    if (doDebug)
                        RCLCPP_INFO(this->get_logger(), "Checking screw angle");
                    
                    if (screw_angle_is_valid(angle_count))
                    {
                        float angle = -angle_sum/(float)angle_count;
                        RCLCPP_INFO(this->get_logger(), "Screw found at angle %f\n", angle);
                        commandMotor(0, angle, true);
                    }
                }
                else if (i == 35 * (STEPS_PER_SEC/10))
                {
                    i = 0;
                    pos_iter ++;
                    pos_iter %= 4;
                }
                else {
                    if (new_angle_){
                        if (doDebug)
                            RCLCPP_INFO(this->get_logger(), "Got angle %f", angle_);
                        if (angle_ != 360){
                            angle_sum += angle_;
                            angle_count++;
                        }
                        new_angle_ = false;
                    }
                    if (new_type_){
                        for (int it = 0; it < 4; it++)
                        {
                            type_sum[it] += screw_type_[it];
                        }
                        new_type_ = false;
                    }
                }
                i++;
            }
            else{   
                i = 0;
                if (doDebug)
                    RCLCPP_INFO(this->get_logger(), "position: %i", command_pos);
                commandMotor(1,command_pos);
                pos_current = command_pos;
            }
        }


};



int main(int argc, char *argv[])
{

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<MotorControl>());

	rclcpp::shutdown();
	return 0;
}
