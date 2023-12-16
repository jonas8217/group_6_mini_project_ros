#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
// #include <stdlib.h>
#include <iostream>

#include <time.h>
#include <chrono>

#include <math.h>

#include "../lib/AXI-DMA-UIO-cpp-driver/include/axi_dma_controller.h"
#include "../lib/ReservedMemory-LKM-and-UserSpaceAPI/reserved_mem.hpp"

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define LENGTH 1800000
// #define LENGTH 0x007fffff // Length in bytes
#define P_START 0x70000000
#define P_OFFSET 0

//#define i_P_START 0
//#define i_LENGTH 1
//#define i_U_BUFFER_PTR_L 2
//#define i_U_BUFFER_PTR_H 3

#define UIO_DMA_N 0

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

	uint32_t *u_buff = (uint32_t *)malloc(LENGTH);
	if (u_buff == NULL)
	{
		printf("could not allocate user buffer\n");
		return -1;
	}

	for (int i = 0; i < LENGTH / sizeof(uint32_t); i++)
		u_buff[i] = i * 2;

	printf("User memory reserved and filled\n");
	
	tmp = 0;
	start_timer();
	// ret = write(reserved_mem_fd, write_info_LKM, sizeof(write_info_LKM));
	pmem.transfer(u_buff, P_OFFSET, LENGTH);
	total_t += stop_timer();
	std::cout << "Data transfered to reserved memory: " << total_t << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;

	start_timer();
	// printf("Reset the DMA.\n");
	dma.MM2SReset();
	// dma.S2MMReset();

	// printf("Check MM2S status.\n");
	DMAStatus mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	// printf("Check S2MM status.\n");
	// DMAStatus s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	// printf("\n");

	// printf("Halt the DMA.\n");
	dma.MM2SHalt();
	// dma.S2MMHalt();

	// printf("Check MM2S status.\n");
	// mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	// printf("Check S2MM status.\n");
	// s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	// printf("\n");

	// printf("Enable all interrupts.\n");
	dma.MM2SInterruptEnable();
	// dma.S2MMInterruptEnable();

	// printf("Check MM2S status.\n");
	// mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	// printf("Check S2MM status.\n");
	// s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	// printf("\n");

	// printf("Writing source address of the data from MM2S in DDR...\n");
	dma.MM2SSetSourceAddress(P_START + P_OFFSET);
	// printf("Check MM2S status.\n");
	// mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

	// printf("Writing the destination address for the data from S2MM in DDR...\n");
	// dma.S2MMSetDestinationAddress(P_START + P_OFFSET);
	// printf("Check S2MM status.\n");
	// s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	// printf("\n");

	dma.MM2SStart();
	// printf("Run the MM2S channel.\n");
	// printf("Check MM2S status.\n");
	// mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

	// printf("Run the S2MM channel.\n");
	// dma.S2MMStart();
	// printf("Check S2MM status.\n");
	// s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	// printf("\n");

	// printf("\nWriting MM2S transfer length of %i bytes...\n", LENGTH);
	dma.MM2SSetLength(LENGTH); //! WIll only work up to 2^23
	// printf("Check MM2S status.\n");
	// mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	// printf("Writing S2MM transfer length of 32 bytes...\n");
	// dma.S2MMSetLength(32*32*8);
	// printf("Check S2MM status.\n");
	// s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());
	// printf("\n");
	
	tmp = stop_timer();
	total_t+=tmp;
	std::cout << "\nDMA setup done, transfer begun: " << tmp << "ms [" << (float)LENGTH / 1000000. << "MB]\n" << std::endl;

	start_timer();
	printf("...Waiting for MM2S synchronization...\n");
	// bool first = true;
	while (!dma.MM2SIsSynced())
	{
		// if (first)
		// {
		// 	printf("Not synced yet...\n");
		// 	first = false;
		// }
	}

	tmp = stop_timer();
	total_t += tmp;
	std::cout << "\nData transfered to transfered by DMA: " << tmp << "ms [" << (float)LENGTH / 1000000. << "MB]\n" << std::endl;

	// printf("Check MM2S status.\n");
	// mm2s_status = dma.MM2SGetStatus();
	// printf("MM2S status: %s\n", mm2s_status.to_string().c_str());
	// printf("Waiting for S2MM sychronization...\n");
	// while(!dma.S2MMIsSynced()) {
	// 	printf("Not synced yet...\n");
	// }

	// printf("Check S2MM status.\n");
	//  s2mm_status = dma.S2MMGetStatus();
	// printf("S2MM status: %s\n", s2mm_status.to_string().c_str());

	// write_info_LKM[i_P_START] = 0; //! CHECK update read
	// write_info_LKM[i_LENGTH] = 100;
	// write_info_LKM[i_K_START] = 10;
	// ret = read(reserved_mem_fd, write_info_LKM, sizeof(write_info_LKM));
	// if (ret < 0)
	// {
	//     printf("read error!\n");
	//     ret = errno;
	//     goto out;
	// }

	// printf("Data in buffer after read\n");
	// for (int i = 0; i < 20; i++)
	// {
	//     printf("%i ", p[i]);
	// }
	printf("ALL DONE!\n");

	std::cout << "Total duration of transfer: " << total_t << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;
	return ret;
}
