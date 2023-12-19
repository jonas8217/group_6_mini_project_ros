#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int16.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>


#include "AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "Invert_v1_0/src/xinvert.c"
#include "Invert_v1_0/src/xinvert_sinit.c"
#include "Invert_v1_0/src/xinvert_linux.c"


#define UIO_INVERT_Nsjjk 0L

#define UIO_DMA_N 1

#define XST_FAILURE		1L	//This is nice to have :)

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define IMAGE_WIDTH		160
#define IMAGE_HEIGHT	120
#define LENGTH IMAGE_WIDTH*IMAGE_HEIGHT*4 //(160*120*4) // Number of bytes (rgb + grayscale)
#define LENGTH_INPUT 	LENGTH*3/4 // Number of bytes for input (3/4 because rgb)
#define LENGTH_OUTPUT	1

#define CROPPED_IMAGE_SIZE		60

#define P_START 0x70000000
#define TX_OFFSET 0
#define RX_OFFSET_BYTES LENGTH_INPUT
#define RX_OFFSET_32 RX_OFFSET_BYTES/4 // This needs to be a whole number, otherwise input in ram is overwritten!

//Reserved_Mem pmem;
//AXIDMAController dma(UIO_DMA_N, 0x10000);
//XInvert invertIP;


uint8_t *inp_buff;
uint8_t *out_buff;

class CNNInterface : public rclcpp::Node
{
	public:
		CNNInterface() : Node("CNN_interface") {
			RCLCPP_INFO(this->get_logger(), "Initializing CNNInterface node");

			RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_raw",
					10,
					std::bind(&CNNInterface::onImageMsg, this, std::placeholders::_1)
			);

            image_publisher_= this->create_publisher<sensor_msgs::msg::Image>(
				"/image_processed",
				10
			);

			result_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
				"/CNN_screw_type",
				10
			);

            out_img = cv::Mat(CROPPED_IMAGE_SIZE, CROPPED_IMAGE_SIZE, CV_8UC1);

            //init_IPs_and_setup();

		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr result_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

		cv::Mat inp_img;
        cv::Mat out_img;

        uchar results;


        // int init_IPs_and_setup(){
        //     printf("\r\n--- IPs Intialized --- \r\n");
        // }


		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");
            // std::string type = msg->encoding;
            // printf("type: %s\n", type.c_str());
			// cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			// inp_img = cv_ptr->image;
            // printf("type: %i\n", inp_img.type());
            
            
            //RCLCPP_INFO(this->get_logger(), "Loading image to dram");

            loadImage(msg);
            
            
            //return;

			// RCLCPP_INFO(this->get_logger(), "Successfully loaded image");

            // RCLCPP_INFO(this->get_logger(), "Running IP");

            // //run_Invert_IP();

            // RCLCPP_INFO(this->get_logger(), "IP completed");

            // outputResults();

            // RCLCPP_INFO(this->get_logger(), "Loaded image from dram");

			
            // RCLCPP_INFO(this->get_logger(), "image loaded tp msg");
            
            sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", out_img).toImageMsg();

			image_publisher_->publish(*processed_image_msg.get());
		}

        
        void loadImage(const sensor_msgs::msg::Image::SharedPtr msg) {
            int rows = IMAGE_HEIGHT;
            int cols = IMAGE_WIDTH;
            
            int rows_start = rows/2 - CROPPED_IMAGE_SIZE/2;
            int cols_start = cols/2 - CROPPED_IMAGE_SIZE/2;

            for (int y = 0; y < CROPPED_IMAGE_SIZE; y++){
                for (int x = 0; x < CROPPED_IMAGE_SIZE; x++){
                    int idx = (rows_start + y)*cols*3+(cols_start + x)*3;
                    //inp_buff[y*cols+x] = inp_img.at<cv::Vec3b>(rows_start + y, cols_start + x)[0];
                    uint16_t grey = msg->data[idx+0] + msg->data[idx+1] + msg->data[idx+2];
                    inp_buff[y*cols+x] = (uchar)(grey/3);
                    out_img.at<uchar>(y,x) = (uchar)(grey/3);
                }
            }
        }

        void outputResults() {
            results = out_buff[0];
        }

        // void run_Invert_IP(){
            
        //     pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);

        //     dma.MM2SReset();
        //     dma.S2MMReset();

        //     dma.MM2SHalt();
        //     dma.S2MMHalt();

        //     dma.MM2SInterruptEnable();
        //     dma.S2MMInterruptEnable();

        //     dma.MM2SSetSourceAddress(P_START + TX_OFFSET);
        //     dma.S2MMSetDestinationAddress(P_START + RX_OFFSET_BYTES);

        //     while(!XInvert_IsReady(&invertIP)) {}

        //     XInvert_Start(&invertIP);

        //     dma.MM2SStart();
        //     dma.S2MMStart();

        //     dma.MM2SSetLength(LENGTH_INPUT);
        //     dma.S2MMSetLength(LENGTH_OUTPUT);
            
        //     while (!dma.MM2SIsSynced()) {}
        //     while (!dma.S2MMIsSynced()) {}
        //     while(!XInvert_IsDone(&invertIP)) {}

        //     pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);
        // }


};

int main(int argc, char *argv[])
{

    inp_buff = (uint8_t *)malloc(LENGTH_INPUT);
    if (inp_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }
    out_buff = (uint8_t *)malloc(LENGTH_OUTPUT);
    if (out_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }

    /*
    int Status;
    Status = XInvert_Initialize(&invertIP, "Invert");

    if (Status != XST_SUCCESS) {
        printf("Invert initialization failed %d\r\n", Status);
        return XST_FAILURE;
    }
    */
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<CNNInterface>());

	rclcpp::shutdown();
	return 0;
}
