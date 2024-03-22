// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xinfer.h"

extern XInfer_Config XInfer_ConfigTable[];

XInfer_Config *XInfer_LookupConfig(u16 DeviceId) {
	XInfer_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XINFER_NUM_INSTANCES; Index++) {
		if (XInfer_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XInfer_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XInfer_Initialize(XInfer *InstancePtr, u16 DeviceId) {
	XInfer_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XInfer_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XInfer_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

