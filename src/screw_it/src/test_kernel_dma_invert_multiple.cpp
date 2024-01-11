#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <iostream>

#include <time.h>
#include <chrono>

#include <math.h>

#include "lib/AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "lib/ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"
#include "lib/Invert_v1_0/src/xinvert.c"
#include "lib/Invert_v1_0/src/xinvert_sinit.c"
#include "lib/Invert_v1_0/src/xinvert_linux.c"

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define LENGTH 800*600*4 //(800*600*4) // Number of bytes (rgb + grayscale)
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
#define TRIES_N			10

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

	for (int try_n = 0; try_n < TRIES_N; try_n++)
	{
		uint8_t *inp_buff = (uint8_t *)malloc(LENGTH_INPUT);
		if (inp_buff == NULL)
		{
			printf("could not allocate user buffer\n");
			return -1;
		}
		
		for (int i = 0; i < (LENGTH_INPUT); i++)
			inp_buff[i] = inputVal;

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
		
		uint8_t *in_buff = (uint8_t *)malloc(LENGTH_INPUT);
		pmem.gather(in_buff, TX_OFFSET, LENGTH_INPUT);
		for (int i = 0; i < LENGTH_INPUT; i++) {
			if (in_buff[i] != inputVal) {
				printf("\nFailure in out_buff: %i %x --addr: %x\n\r", i, in_buff[i], &in_buff[i]);
				FAIL = true;
				break;
			}
			if (i == LENGTH_INPUT-1) {
				printf("\n Input has correct value!\n");
			}
		}

		uint8_t *out_buff = (uint8_t *)malloc(LENGTH_OUTPUT);
		pmem.gather(out_buff, RX_OFFSET_32, LENGTH_OUTPUT);
		for (int i = 0; i < LENGTH_OUTPUT; i++) {
			if (out_buff[i] != outputVal) {
				printf("\nFailure in out_buff: %i %x --addr: %x\n\r", i, out_buff[i], &out_buff[i]);
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
		

	}

	if (FAIL == false)
	{
		printf("\nSuccess, all tries (%i) worked!\n", TRIES_N);
	}
	


	
	return ret;
}
