// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xinfer.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XInfer_CfgInitialize(XInfer *InstancePtr, XInfer_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XInfer_Start(XInfer *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL) & 0x80;
    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XInfer_IsDone(XInfer *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XInfer_IsIdle(XInfer *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XInfer_IsReady(XInfer *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XInfer_EnableAutoRestart(XInfer *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XInfer_DisableAutoRestart(XInfer *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_AP_CTRL, 0);
}

void XInfer_InterruptGlobalEnable(XInfer *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_GIE, 1);
}

void XInfer_InterruptGlobalDisable(XInfer *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_GIE, 0);
}

void XInfer_InterruptEnable(XInfer *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_IER);
    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_IER, Register | Mask);
}

void XInfer_InterruptDisable(XInfer *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_IER);
    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_IER, Register & (~Mask));
}

void XInfer_InterruptClear(XInfer *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInfer_WriteReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_ISR, Mask);
}

u32 XInfer_InterruptGetEnabled(XInfer *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_IER);
}

u32 XInfer_InterruptGetStatus(XInfer *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XInfer_ReadReg(InstancePtr->Control_BaseAddress, XINFER_CONTROL_ADDR_ISR);
}

