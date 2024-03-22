// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
// 0x0000 : Control signals
//          bit 0  - ap_start (Read/Write/COH)
//          bit 1  - ap_done (Read/COR)
//          bit 2  - ap_idle (Read)
//          bit 3  - ap_ready (Read)
//          bit 7  - auto_restart (Read/Write)
//          others - reserved
// 0x0004 : Global Interrupt Enable Register
//          bit 0  - Global Interrupt Enable (Read/Write)
//          others - reserved
// 0x0008 : IP Interrupt Enable Register (Read/Write)
//          bit 0  - enable ap_done interrupt (Read/Write)
//          bit 1  - enable ap_ready interrupt (Read/Write)
//          others - reserved
// 0x000c : IP Interrupt Status Register (Read/TOW)
//          bit 0  - ap_done (COR/TOW)
//          bit 1  - ap_ready (COR/TOW)
//          others - reserved
// 0x0010 : Data signal of ap_return
//          bit 31~0 - ap_return[31:0] (Read)
// 0x4000 ~
// 0x7fff : Memory 'in_r' (3600 * 32b)
//          Word n : bit [31:0] - in_r[n]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XINFER_CONTROL_ADDR_AP_CTRL   0x0000
#define XINFER_CONTROL_ADDR_GIE       0x0004
#define XINFER_CONTROL_ADDR_IER       0x0008
#define XINFER_CONTROL_ADDR_ISR       0x000c
#define XINFER_CONTROL_ADDR_AP_RETURN 0x0010
#define XINFER_CONTROL_BITS_AP_RETURN 32
#define XINFER_CONTROL_ADDR_IN_R_BASE 0x4000
#define XINFER_CONTROL_ADDR_IN_R_HIGH 0x7fff
#define XINFER_CONTROL_WIDTH_IN_R     32
#define XINFER_CONTROL_DEPTH_IN_R     3600

