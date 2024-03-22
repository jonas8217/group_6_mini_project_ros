#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>

// #include "AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
// #include "ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
// #include "infer_v1_0_axilite/src/xinfer.h"
#include "infer_v1_0_axilite/src/xinfer.c"
#include "infer_v1_0_axilite/src/xinfer_sinit.c"
#include "infer_v1_0_axilite/src/xinfer_linux.c"

#include "testImage.h"

#define XST_FAILURE         1L    //This is nice to have :)

#define IMAGE_WIDTH         60
#define IMAGE_HEIGHT        60
#define LENGTH_INPUT        IMAGE_WIDTH*IMAGE_HEIGHT*1
#define LENGTH_OUTPUT       4*4

XInfer xinference;

uint8_t *inp_buff;
int output;
int expected_type;

union float_uint {
    float float_val;
    uint32_t uint_val;
};

class CNNInterface : public rclcpp::Node
{
    public:
        CNNInterface() : Node("CNN_interface") {
            RCLCPP_INFO(this->get_logger(), "Initializing CNNInterface node");

            test((int*)test_image0, (int*)prediction0, 10000);
            
        }

    private:

        void test(int* data, int* predictions, int times) {
            // Load image
            for (int y = 0; y < IMAGE_HEIGHT; y++){
                for (int x = 0; x < IMAGE_WIDTH; x++){
                    inp_buff[y*IMAGE_WIDTH+x] = data[y*IMAGE_WIDTH+x];
                }
            }

            int type = 0;
            float value = 0;
            for (int i = 0; i < 4; i++)
            {
                if (predictions[i] > value) {
                    type = i;
                    value = predictions[i];
                }
            }
            expected_type = type;

            // Run tests
            uint total_time_run_avg = 0;
            uint ip_time_run_avg = 0;

            for (int i = 0; i < times; i++)
            {
                // Transfer image, run IP, transfer result
                uint total_time;
                uint ip_time;
                run_IP(total_time,ip_time);

                if (output != expected_type){
                    RCLCPP_INFO(this->get_logger(), "Wrong result %i != %i", output, expected_type);
                    break;
                }

                total_time_run_avg = (total_time_run_avg * i + total_time)/(i+1);
                ip_time_run_avg = (ip_time_run_avg * i + ip_time)/(i+1);
                
            }
            RCLCPP_INFO(this->get_logger(), "Tests (%d) done | Average total: %d, average IP: %d, average transfer time: %d", times, total_time_run_avg, ip_time_run_avg, total_time_run_avg-ip_time_run_avg);
            // rclcpp::shutdown(); // If we want it to automatically stop the node after running tests
        }


        void run_IP(uint &total_time,uint &ip_time){
            const auto start1{std::chrono::steady_clock::now()};

            XInfer_Write_in_r_Words(&xinference,0,(word_type*)inp_buff,LENGTH_INPUT/4);

            const auto start2{std::chrono::steady_clock::now()};
            XInfer_Start(&xinference);

            // Wait till its done and get the output
            while(!XInfer_IsDone(&xinference));

            const auto end2{std::chrono::steady_clock::now()};

            output = XInfer_Get_return(&xinference);
            const auto end1{std::chrono::steady_clock::now()};

            total_time = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1).count();
            ip_time = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count();
            // RCLCPP_INFO(this->get_logger(), "Ip time: %d, transfer to gather time: %d", ip_time, total_time);
        }
};

int main(int argc, char *argv[])
{
    // Assuming i dont need to allocate space in memory for my input
    inp_buff = (uint8_t *)malloc(LENGTH_INPUT);
    if (inp_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }

    int Status;
    Status = XInfer_Initialize(&xinference, "infer");
    
    if (Status != XST_SUCCESS) {
        printf("Infer initialization failed %d\r\n", Status);
        return XST_FAILURE;
    }

    // Dont understand assuming not relevant
    //setvbuf(stdout,NULL,_IONBF,BUFSIZ);

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CNNInterface>());

    rclcpp::shutdown();
    return 0;
}
