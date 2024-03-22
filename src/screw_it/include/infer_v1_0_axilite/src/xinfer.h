// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XINFER_H
#define XINFER_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xinfer_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Control_BaseAddress;
} XInfer_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XInfer;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XInfer_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XInfer_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XInfer_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XInfer_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XInfer_Initialize(XInfer *InstancePtr, u16 DeviceId);
XInfer_Config* XInfer_LookupConfig(u16 DeviceId);
int XInfer_CfgInitialize(XInfer *InstancePtr, XInfer_Config *ConfigPtr);
#else
int XInfer_Initialize(XInfer *InstancePtr, const char* InstanceName);
int XInfer_Release(XInfer *InstancePtr);
#endif

void XInfer_Start(XInfer *InstancePtr);
u32 XInfer_IsDone(XInfer *InstancePtr);
u32 XInfer_IsIdle(XInfer *InstancePtr);
u32 XInfer_IsReady(XInfer *InstancePtr);
void XInfer_EnableAutoRestart(XInfer *InstancePtr);
void XInfer_DisableAutoRestart(XInfer *InstancePtr);
u32 XInfer_Get_return(XInfer *InstancePtr);

u32 XInfer_Get_in_r_BaseAddress(XInfer *InstancePtr);
u32 XInfer_Get_in_r_HighAddress(XInfer *InstancePtr);
u32 XInfer_Get_in_r_TotalBytes(XInfer *InstancePtr);
u32 XInfer_Get_in_r_BitWidth(XInfer *InstancePtr);
u32 XInfer_Get_in_r_Depth(XInfer *InstancePtr);
u32 XInfer_Write_in_r_Words(XInfer *InstancePtr, int offset, word_type *data, int length);
u32 XInfer_Read_in_r_Words(XInfer *InstancePtr, int offset, word_type *data, int length);
u32 XInfer_Write_in_r_Bytes(XInfer *InstancePtr, int offset, char *data, int length);
u32 XInfer_Read_in_r_Bytes(XInfer *InstancePtr, int offset, char *data, int length);

void XInfer_InterruptGlobalEnable(XInfer *InstancePtr);
void XInfer_InterruptGlobalDisable(XInfer *InstancePtr);
void XInfer_InterruptEnable(XInfer *InstancePtr, u32 Mask);
void XInfer_InterruptDisable(XInfer *InstancePtr, u32 Mask);
void XInfer_InterruptClear(XInfer *InstancePtr, u32 Mask);
u32 XInfer_InterruptGetEnabled(XInfer *InstancePtr);
u32 XInfer_InterruptGetStatus(XInfer *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
