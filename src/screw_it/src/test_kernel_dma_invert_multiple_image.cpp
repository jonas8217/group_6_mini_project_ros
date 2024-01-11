#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>

#include <time.h>
#include <chrono>

#include <math.h>


#include "lib/AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "lib/ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "lib/Invert_v1_0/src/xinvert.c"
#include "lib/Invert_v1_0/src/xinvert_sinit.c"
#include "lib/Invert_v1_0/src/xinvert_linux.c"

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define IMAGE_WIDTH		800
#define IMAGE_HEIGHT	600
#define LENGTH IMAGE_WIDTH*IMAGE_HEIGHT*4 //(800*600*4) // Number of bytes (rgb + grayscale)
#define LENGTH_INPUT 	LENGTH*3/4 // Number of bytes for input (3/4 because rgb)
#define LENGTH_OUTPUT	LENGTH/4 // Number of bytes for output (1/4 because grayscale)
// #define LENGTH 0x007fffff // Length in bytes
#define P_START 0x70000000
#define TX_OFFSET 0
#define RX_OFFSET_BYTES LENGTH_INPUT
#define RX_OFFSET_32 RX_OFFSET_BYTES/4 // This needs to be a whole number, otherwise input in ram is overwritten!


#define UIO_DMA_N 1

#define XST_FAILURE		1L	//This is nice to have :)

#define inputVal		0xeb
#define outputVal		0x14
#define TRIES_N			1

#define R_Weight 0.299 //R constant
#define G_Weight 0.587 //G constant
#define B_Weight 0.114 //B constant

// clock_t t;
std::chrono::_V2::system_clock::time_point t1;
void start_timer()
{
	t1 = std::chrono::high_resolution_clock::now();
	// std::cout << "Start timer" << std::endl;
}
double stop_timer()
{
	auto t2 = std::chrono::high_resolution_clock::now();
	auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	// std::cout << "Duration: " << ms_double.count() << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;
	return ms_double.count();
}

void print_mem(void *virtual_address, int byte_count)
{
	char *data_ptr = (char *)virtual_address;

	for (int i = 0; i < byte_count; i++)
	{
		printf("%02X", data_ptr[i]);

		// print a space every 4 bytes (0 indexed)
		if (i % 4 == 3)
		{
			printf(" ");
		}
	}

	printf("\n");
}

void loadImage(uint32_t *inp_buff, uint8_t *test_buf) {
	uint32_t *Image = (uint32_t *)inp_buff;
	
	// Read test file
	std::string Test_Data;
	std::ifstream Test_File("lib/images/RGB_array.txt");

	// Image count
	volatile int Image_Count = 0;

	// Get test data
	while(std::getline(Test_File, Test_Data)){
		Image[Image_Count] = (uint32_t)std::stoul(Test_Data);
		test_buf[Image_Count*4] = (Image[Image_Count] & 0x000000ff);
		test_buf[Image_Count*4+1] = ((Image[Image_Count] & 0x0000ff00) >> 8);
		test_buf[Image_Count*4+2] = ((Image[Image_Count] & 0x00ff0000) >> 16);
		test_buf[Image_Count*4+3] = ((Image[Image_Count] & 0xff000000) >> 24);
		Image_Count++;
	}
	Test_File.close();
}

int main()
{
	int ret = 0;
	double total_t = 0;
	double tmp = 0;
	printf("\nHello World! - Running DMA transfer test application with specified memory.\n\n");

	Reserved_Mem pmem;
	AXIDMAController dma(UIO_DMA_N, 0x10000);
	
	printf("\nInitalizing invert ip\n");
	int Status;
	XInvert invertIP;
	Status = XInvert_Initialize(&invertIP, "Invert");
	
	if (Status != XST_SUCCESS) {
		printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}
	printf("\r\n--- Invert Intialized --- \r\n");

	bool FAIL = false;
	uint8_t finalResultBuff[IMAGE_WIDTH*IMAGE_HEIGHT];

	for (int try_n = 0; try_n < TRIES_N; try_n++)
	{
		uint32_t *inp_buff = (uint32_t *)malloc(LENGTH_INPUT);
		if (inp_buff == NULL)
		{
			printf("could not allocate user buffer\n");
			return -1;
		}

		uint8_t *test_buff = (uint8_t *)malloc(LENGTH_INPUT);
		pmem.gather(test_buff, TX_OFFSET, LENGTH_INPUT);

		loadImage(inp_buff, test_buff);

		printf("User memory reserved and filled\n");
		
		tmp = 0;
		start_timer();
		pmem.transfer(inp_buff, TX_OFFSET, LENGTH_INPUT);
		total_t += stop_timer();
		std::cout << "Data transfered to reserved memory: " << total_t << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]" << std::endl;

		start_timer();
		printf("Reset the DMA.\n");
		dma.MM2SReset();
		dma.S2MMReset();

		printf("Check MM2S status.\n");
		DMAStatus mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
		printf("Check S2MM status.\n");
		DMAStatus s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
		printf("\n");

		printf("Halt the DMA.\n");
		dma.MM2SHalt();
		dma.S2MMHalt();

		printf("Check MM2S status.\n");
		mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
		printf("Check S2MM status.\n");
		s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
		printf("\n");

		printf("Enable all interrupts.\n");
		dma.MM2SInterruptEnable();
		dma.S2MMInterruptEnable();

		printf("Check MM2S status.\n");
		mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
		printf("Check S2MM status.\n");
		s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
		printf("\n");

		printf("Writing source address of the data from MM2S in DDR...\n");
		dma.MM2SSetSourceAddress(P_START + TX_OFFSET);
		printf("Check MM2S status.\n");
		mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

		printf("Writing the destination address for the data from S2MM in DDR...\n");
		dma.S2MMSetDestinationAddress(P_START + RX_OFFSET_BYTES);
		printf("Check S2MM status.\n");
		s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
		printf("\n");

		while(!XInvert_IsReady(&invertIP)) {
			// wait
		}
		printf("\nIp ready\n");

		// Start IP
		XInvert_Start(&invertIP);

		dma.MM2SStart();
		printf("Run the MM2S channel.\n");
		printf("Check MM2S status.\n");
		mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

		printf("Run the S2MM channel.\n");
		dma.S2MMStart();
		printf("Check S2MM status.\n");
		s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
		printf("\n");

		printf("\nWriting MM2S transfer length of %i bytes...\n", LENGTH_INPUT);
		dma.MM2SSetLength(LENGTH_INPUT); //! WIll only work up to 2^23
		printf("Check MM2S status.\n");
		mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
		printf("Writing S2MM transfer length of %i bytes...\n",LENGTH_OUTPUT);
		dma.S2MMSetLength(LENGTH_OUTPUT);
		printf("Check S2MM status.\n");
		s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
		printf("\n");
		
		tmp = stop_timer();
		total_t+=tmp;
		std::cout << "\nDMA setup done, transfer begun: " << tmp << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]\n" << std::endl;

		start_timer();
		printf("...Waiting for MM2S synchronization...\n");

		while (!dma.MM2SIsSynced())
		{

		}

		tmp = stop_timer();
		total_t += tmp;
		std::cout << "\nData transfered to transfered by DMA: " << tmp << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]\n" << std::endl;

		printf("Check MM2S status.\n");
		mm2s_status = dma.MM2SGetStatus();
		printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
		printf("Waiting for S2MM sychronization...\n");
		while(!dma.S2MMIsSynced()) {
			//printf("Not synced yet...\n");
		}

		printf("Check S2MM status.\n");
		s2mm_status = dma.S2MMGetStatus();
		printf("S2MM status: %s\n", s2mm_status.to_string().c_str());

		// Wait for ip to finish
		printf("Wait for ip to finish");
		while(!XInvert_IsDone(&invertIP)) {
			// Wait
		}
		printf("\nIp done!\n");

		
		printf("\n\n");
		
		uint32_t *consist_buff = (uint32_t *)malloc(LENGTH_INPUT);
		pmem.gather(consist_buff, TX_OFFSET, LENGTH_INPUT);
		for (int i = 0; i < LENGTH_INPUT/4; i++) {
			if (consist_buff[i] != inp_buff[i]) {
				printf("\nFailure in out_buff: %i (curr: %d) = (old: %d) --addr: %x\n\r", i, consist_buff[i], inp_buff[i], &consist_buff[i]);
				FAIL = true;
				break;
			}
			if (i == LENGTH_INPUT/4-1) {
				printf("\n Input has correct value!\n");
			}
		}

		uint32_t *out_buff = (uint32_t *)malloc(LENGTH_OUTPUT);
		pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);
		
		uint8_t gray_scale[LENGTH_OUTPUT];
		for (int i = 0; i < LENGTH_OUTPUT/4; i++)
		{
			gray_scale[i*4] = (out_buff[i] & 0x000000ff);
			gray_scale[i*4+1] = ((out_buff[i] & 0x0000ff00) >> 8);
			gray_scale[i*4+2] = ((out_buff[i] & 0x00ff0000) >> 16);
			gray_scale[i*4+3] = ((out_buff[i] & 0xff000000) >> 24);
		}
		
		for (int i = 0; i < LENGTH_OUTPUT; i++) {
			double expected_val = (255 - (test_buff[i*3]*R_Weight + test_buff[(i*3) + 1]*G_Weight + test_buff[(i*3) + 2]*B_Weight));
			if (gray_scale[i] != (uint8_t)expected_val) {
				printf("%d %d %d",test_buff[i*3],test_buff[i*3+1],test_buff[i*3+2]);
				printf("\nFailure in gray_scale: %i %d != %f --addr: %x\n\r", i, gray_scale[i], expected_val, &gray_scale[i]);
				FAIL = true;
				break;
			}
			if (i == LENGTH_OUTPUT-1) {
				printf("\n Output has correct value!\n");
			}
		}

		printf("\nALL DONE!\n");

		std::cout << "\nTotal duration of transfer: " << total_t << "ms [" << (float)LENGTH_INPUT / 1000000. << "MB]" << std::endl;

		if (FAIL == true)
		{
			printf("\n Failed at try %i\n", try_n);
			break;
		}
		
		if (FAIL == false && try_n == (TRIES_N - 1)) {
			for (int i = 0; i < LENGTH_OUTPUT; i++)
			{
				finalResultBuff[i] = gray_scale[i];
			}
			
		}
	}

	if (FAIL == false)
	{
		printf("\nSuccess, all tries (%i) worked!\n", TRIES_N);
		// Write to file
		std::ofstream Result_File;
		Result_File.open("Result_File.txt");
		//char * temp = "";
		for(int i = 0; i < LENGTH_OUTPUT; i++){
			//sprintf(temp,"%d",finalResultBuff[i]);
			Result_File << (int)finalResultBuff[i] << "\n";
		}
		Result_File.close();
		printf("\nSaved result to file!\n", TRIES_N);
	}
	


	
	return ret;
}
