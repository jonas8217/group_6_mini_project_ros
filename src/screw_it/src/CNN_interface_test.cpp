#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>


#include "AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "infer_v1_0/src/xinfer.c"
#include "infer_v1_0/src/xinfer_sinit.c"
#include "infer_v1_0/src/xinfer_linux.c"

#include "testImage.h"


#define UIO_DMA_N           0

#define XST_FAILURE         1L    //This is nice to have :)

#define DEVICE_FILENAME     "/dev/reservedmemLKM"
#define IMAGE_WIDTH         60
#define IMAGE_HEIGHT        60
#define LENGTH_INPUT        IMAGE_WIDTH*IMAGE_HEIGHT*1
#define LENGTH_OUTPUT       4*4

#define P_START             0x70000000
#define TX_OFFSET           0
#define RX_OFFSET_BYTES     LENGTH_INPUT
#define RX_OFFSET_32        RX_OFFSET_BYTES/4 // This needs to be a whole number, otherwise input in ram is overwritten!

Reserved_Mem pmem;
AXIDMAController dma(UIO_DMA_N, 0x10000);
XInfer inferIP;


uint8_t *inp_buff;
float *out_buff;

union float_uint {
    float float_val;
    uint32_t uint_val;
};

class CNNInterface : public rclcpp::Node
{
    public:
        CNNInterface() : Node("CNN_interface") {
            RCLCPP_INFO(this->get_logger(), "Initializing CNNInterface node");

            test((int*)test_image0, (int*)prediction0, 1000);
            
        }

    private:

        void test(int* data, int* predictions, int times) {
            // Load image
            for (int y = 0; y < IMAGE_HEIGHT; y++){
                for (int x = 0; x < IMAGE_WIDTH; x++){
                    inp_buff[y*IMAGE_WIDTH+x] = data[y*IMAGE_WIDTH+x];
                }
            }

            // Run tests
            uint total_time_run_avg = 0;
            uint ip_time_run_avg = 0;
            int type = 0;
            float value = 0;
            for (int i = 0; i < 4; i++)
            {
                if (predictions[i] > value) {
                    type = i;
                    value = predictions[i];
                }
            }
            int expected_type = type;
            for (int i = 0; i < times; i++)
            {
                // Transfer image, run IP, transfer result
                uint total_time;
                uint ip_time;
                run_IP(total_time,ip_time);

                // Extract screw type
                std_msgs::msg::Float32MultiArray output_msg;
                outputResults(output_msg);
                int type = 0;
                float value = 0;
                for (int ii = 0; ii < 4; ii++)
                {
                    if (output_msg.data[ii] > value) {
                        type = ii;
                        value = output_msg.data[ii];
                    }
                }

                if (type != expected_type){
                    RCLCPP_INFO(this->get_logger(), "Wrong result %i != %i", type, expected_type);
                    break;
                }

                total_time_run_avg = (total_time_run_avg * i + total_time)/(i+1);
                ip_time_run_avg = (ip_time_run_avg * i + ip_time)/(i+1);
                
            }
            RCLCPP_INFO(this->get_logger(), "Tests (%d) done | Average total: %d, average IP: %d, average transfer time: %d", times, total_time_run_avg, ip_time_run_avg, total_time_run_avg-ip_time_run_avg);
        }

        void outputResults(std_msgs::msg::Float32MultiArray &output_msg) {
            std::vector<float> result = {0, 0, 0, 0};
            for (int i = 0; i < LENGTH_OUTPUT/4; i++)
                result[i] = out_buff[i];
            output_msg.data = result;
        }

        void run_IP(uint &total_time,uint &ip_time){
            const auto start1{std::chrono::steady_clock::now()};
            pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);

            dma.MM2SReset();
            dma.S2MMReset();

            dma.MM2SHalt();
            dma.S2MMHalt();

            dma.MM2SInterruptEnable();
            dma.S2MMInterruptEnable();

            dma.MM2SSetSourceAddress(P_START + TX_OFFSET);
            dma.S2MMSetDestinationAddress(P_START + RX_OFFSET_BYTES);

            const auto start2{std::chrono::steady_clock::now()};
            while(!XInfer_IsReady(&inferIP)) {}

            XInfer_Start(&inferIP);

            dma.MM2SStart();
            dma.S2MMStart();

            dma.MM2SSetLength(LENGTH_INPUT);
            dma.S2MMSetLength(LENGTH_OUTPUT);
            
            while (!dma.MM2SIsSynced()) {}
            while (!dma.S2MMIsSynced()) {}
            while(!XInfer_IsDone(&inferIP)) {}
            const auto end2{std::chrono::steady_clock::now()};

            pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);
            const auto end1{std::chrono::steady_clock::now()};

            total_time = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1).count();
            ip_time = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count();
            // RCLCPP_INFO(this->get_logger(), "Ip time: %d, transfer to gather time: %d", ip_time, total_time);
        }
};

int main(int argc, char *argv[])
{

    inp_buff = (uint8_t *)malloc(LENGTH_INPUT);
    if (inp_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }
    out_buff = (float *)malloc(LENGTH_OUTPUT);
    if (out_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }

    int Status;
    Status = XInfer_Initialize(&inferIP, "infer");
    
    if (Status != XST_SUCCESS) {
        printf("Infer initialization failed %d\r\n", Status);
        return XST_FAILURE;
    }

    setvbuf(stdout,NULL,_IONBF,BUFSIZ);

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CNNInterface>());

    rclcpp::shutdown();
    return 0;
}
