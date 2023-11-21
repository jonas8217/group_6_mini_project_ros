#include "axi_dma_controller.h"

void print_mem(void *virtual_address, int byte_count)
{
	char *data_ptr = (char *)virtual_address;

	for(int i=0;i<byte_count;i++){
		printf("%02X", data_ptr[i]);

		// print a space every 4 bytes (0 indexed)
		if(i%4==3){
			printf(" ");
		}
	}

	printf("\n");
}

int main()
{
    printf("Hello World! - Running DMA transfer test application.\n");

	printf("Opening a character device file of the Arty's DDR memeory...\n");
	int ddr_memory = open("/dev/mem", O_RDWR | O_SYNC);

	printf("Memory map the MM2S source address register block.\n");
    unsigned int *virtual_src_addr  = (unsigned int *)mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x0e000000);

	printf("Memory map the S2MM destination address register block.\n");
    unsigned int *virtual_dst_addr = (unsigned int *)mmap(NULL, 65535, PROT_READ | PROT_WRITE, MAP_SHARED, ddr_memory, 0x0f000000);

	printf("Writing random data to source register block...\n");
	virtual_src_addr[0]= 0xEFBEADDE;
	virtual_src_addr[1]= 0x11223344;
	virtual_src_addr[2]= 0xABABABAB;
	virtual_src_addr[3]= 0xCDCDCDCD;
	virtual_src_addr[4]= 0x00001111;
	virtual_src_addr[5]= 0x22223333;
	virtual_src_addr[6]= 0x44445555;
	virtual_src_addr[7]= 0x66667777;

	printf("Clearing the destination register block...\n");
    memset(virtual_dst_addr, 0, 32);

    printf("Source memory block data:      ");
	print_mem(virtual_src_addr, 32);

    printf("Destination memory block data: ");
	print_mem(virtual_dst_addr, 32);


	printf("\n");

	AXIDMAController dma(0, 0x10000);

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
	dma.MM2SSetSourceAddress(0x0e000000);

	printf("Check MM2S status.\n");
	 mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

    printf("Writing the destination address for the data from S2MM in DDR...\n");
	dma.S2MMSetDestinationAddress(0x0f000000);

	printf("Check S2MM status.\n");
	 s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());






	printf("\n");





	printf("Run the MM2S channel.\n");
	dma.MM2SStart();

	printf("Check MM2S status.\n");
	 mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

	printf("Run the S2MM channel.\n");
	dma.S2MMStart();

	printf("Check S2MM status.\n");
	 s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());





	printf("\n");








    printf("Writing MM2S transfer length of 32 bytes...\n");
	dma.MM2SSetLength(32);

	printf("Check MM2S status.\n");
	 mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

    printf("Writing S2MM transfer length of 32 bytes...\n");
	dma.S2MMSetLength(32);

	printf("Check S2MM status.\n");
	s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());


	printf("\n");


    printf("Waiting for MM2S synchronization...\n");

	while(!dma.MM2SIsSynced()) {
		printf("Not synced yet...\n");
	}

	printf("Check MM2S status.\n");
	mm2s_status = dma.MM2SGetStatus();
	printf("MM2S status: %s\n", mm2s_status.to_string().c_str());

    printf("Waiting for S2MM sychronization...\n");

	while(!dma.S2MMIsSynced()) {
		printf("Not synced yet...\n");
	}

	printf("Check S2MM status.\n");
	 s2mm_status = dma.S2MMGetStatus();
	printf("S2MM status: %s\n", s2mm_status.to_string().c_str());


 


	printf("\n");




    printf("Destination memory block: ");
	print_mem(virtual_dst_addr, 32);

	printf("\n");

    return 0;
}