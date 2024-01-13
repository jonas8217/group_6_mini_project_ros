#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>


#include "AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "infer_v1_0/src/xinfer.c"
#include "infer_v1_0/src/xinfer_sinit.c"
#include "infer_v1_0/src/xinfer_linux.c"


#define UIO_INVERT_Nsjjk    0L

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

            RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

            camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "/image_raw",
                    qos,
                    std::bind(&CNNInterface::onImageMsg, this, std::placeholders::_1)
            );

            image_publisher_= this->create_publisher<sensor_msgs::msg::Image>(
                "/image_processed",
                qos
            );

            result_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "/CNN_screw_type",
                qos
            );

            out_img = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr result_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

        cv::Mat inp_img;
        cv::Mat out_img;

        float results[LENGTH_OUTPUT/4];
        
        void loadImage(const sensor_msgs::msg::Image::SharedPtr msg) {
            int rows = msg->height; // 120
            int cols = msg->width;  // 160
            int rows_start = rows/2 - IMAGE_HEIGHT/2;
            int cols_start = cols/2 - IMAGE_WIDTH/2;

            for (int y = 0; y < IMAGE_HEIGHT; y++){
                for (int x = 0; x < IMAGE_WIDTH; x++){
                    int idx = (rows_start + y)*cols*3+(cols_start + x)*3;
                    uint16_t grey = msg->data[idx+0] + msg->data[idx+1] + msg->data[idx+2];
                    inp_buff[y*IMAGE_WIDTH+x] = (uchar)(grey/3);
                    //out_img.at<uchar>(y,x) = (uchar)(grey/3);
                }
            }
        }

        void outputResults(std_msgs::msg::Float32MultiArray output_msg) {
            std::vector<float> result(4);
            for (int i = 0; i < LENGTH_OUTPUT; i++)
                result[i] = out_buff[i*4];
            output_msg.data = result;
        }

        void run_IP(){
            
            pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);

            dma.MM2SReset();
            dma.S2MMReset();

            dma.MM2SHalt();
            dma.S2MMHalt();

            dma.MM2SInterruptEnable();
            dma.S2MMInterruptEnable();

            dma.MM2SSetSourceAddress(P_START + TX_OFFSET);
            dma.S2MMSetDestinationAddress(P_START + RX_OFFSET_BYTES);

            while(!XInfer_IsReady(&inferIP)) {}

            XInfer_Start(&inferIP);

            dma.MM2SStart();
            dma.S2MMStart();

            dma.MM2SSetLength(LENGTH_INPUT);
            dma.S2MMSetLength(LENGTH_OUTPUT);
            
            while (!dma.MM2SIsSynced()) {}
            while (!dma.S2MMIsSynced()) {}
            while(!XInfer_IsDone(&inferIP)) {}

            pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);
        }

        void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Image received");
        
            RCLCPP_INFO(this->get_logger(), "Loading image to dram");
            loadImage(msg);
            RCLCPP_INFO(this->get_logger(), "Successfully loaded image");

            RCLCPP_INFO(this->get_logger(), "Running IP");
            run_IP();
            RCLCPP_INFO(this->get_logger(), "IP completed");

            //RCLCPP_INFO(this->get_logger(), "Loaded image from dram");
            //sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", out_img).toImageMsg();
            //RCLCPP_INFO(this->get_logger(), "Image message Created");

            //image_publisher_->publish(*processed_image_msg.get());
            //RCLCPP_INFO(this->get_logger(), "Image published");

            std_msgs::msg::Float32MultiArray output_msg;
            outputResults(output_msg);
            result_publisher_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), "CNN Result pusblished");
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
