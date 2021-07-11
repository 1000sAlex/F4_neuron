/**
******************************************************************************
* @file    network_data.c
* @author  AST Embedded Analytics Research Platform
* @date    Sun Jul 11 15:40:28 2021
* @brief   AI Tool Automatic Code Generator for Embedded NN computing
******************************************************************************
* @attention
*
* Copyright (c) 2021 STMicroelectronics.
* All rights reserved.
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/

#include "network_data.h"

ai_handle ai_network_data_weights_get(void)
{
  AI_ALIGNED(32)
  static const ai_u8 s_network_weights[1576] = {
    0x57, 0xea, 0x39, 0x3e, 0xb0, 0x41, 0xcf, 0xbe, 0xaa, 0x75,
    0x0b, 0xbf, 0xec, 0x79, 0x29, 0x3e, 0x1f, 0x47, 0x2a, 0xbf,
    0xb7, 0xaa, 0x19, 0xbe, 0xa4, 0xe9, 0x81, 0x3e, 0x33, 0x38,
    0x40, 0x3e, 0xc3, 0x0e, 0x95, 0x3e, 0xf3, 0x48, 0x38, 0x3f,
    0xe5, 0xd1, 0xdc, 0xbe, 0x7a, 0x62, 0x3a, 0x3f, 0xeb, 0x77,
    0x7d, 0x3e, 0x5d, 0x75, 0x53, 0x3d, 0xf6, 0x33, 0x9c, 0x3e,
    0x91, 0xec, 0x6c, 0xbf, 0x61, 0x79, 0x03, 0x3f, 0x2a, 0x3e,
    0xc5, 0xbe, 0xb0, 0x22, 0x39, 0xbe, 0xd1, 0xea, 0x4a, 0x3f,
    0x25, 0xe7, 0xf7, 0x3e, 0xe4, 0x69, 0xc9, 0x3e, 0xc7, 0xed,
    0xc3, 0x3e, 0xc7, 0x54, 0x4f, 0x3e, 0x7f, 0x39, 0xcd, 0xbd,
    0x69, 0x47, 0x66, 0xbf, 0xe8, 0x4e, 0x45, 0xbf, 0xcf, 0xc6,
    0x19, 0xbf, 0xe5, 0x15, 0x3e, 0xbe, 0x24, 0x06, 0x22, 0x3e,
    0x6b, 0x89, 0x5a, 0xbe, 0xbe, 0xa7, 0xd3, 0x3d, 0x46, 0xd6,
    0x90, 0x3e, 0xed, 0x4e, 0x94, 0x3f, 0x94, 0xa1, 0x5e, 0xbe,
    0x8d, 0xf8, 0x32, 0x3f, 0x95, 0x6e, 0x27, 0xbf, 0xf1, 0x52,
    0x90, 0xbd, 0xc3, 0x45, 0xbb, 0xbe, 0xb4, 0xb8, 0x21, 0x3c,
    0x4b, 0xf3, 0xc1, 0x3e, 0x79, 0xae, 0xe2, 0xbe, 0x57, 0x56,
    0x26, 0xbc, 0x58, 0xd5, 0x69, 0x3f, 0x4e, 0xb8, 0x8c, 0x3e,
    0xb5, 0xe3, 0x66, 0xbe, 0x06, 0x72, 0xf2, 0x3e, 0x69, 0xed,
    0x38, 0xbf, 0x3e, 0xdf, 0xc5, 0x3e, 0x0e, 0xb7, 0xbb, 0x3d,
    0x3d, 0xa9, 0xdc, 0xbc, 0x88, 0xec, 0xf4, 0x3d, 0x0c, 0xe8,
    0x03, 0xbf, 0x6b, 0x5e, 0x90, 0x3c, 0xa6, 0x74, 0x17, 0xbf,
    0x63, 0x73, 0x95, 0x3e, 0x92, 0x08, 0x63, 0x3d, 0x52, 0x71,
    0x0d, 0xbf, 0xc4, 0x76, 0xbb, 0xbe, 0xd0, 0x63, 0x90, 0xbc,
    0xc1, 0x12, 0xc1, 0xbe, 0xd7, 0x88, 0x13, 0xbf, 0xd3, 0x0a,
    0x9f, 0x3e, 0x38, 0x89, 0x18, 0x3f, 0xe8, 0x07, 0x2f, 0x3f,
    0x3b, 0x23, 0x6a, 0xbe, 0x5e, 0x70, 0x92, 0x3e, 0x11, 0x88,
    0x13, 0x3e, 0x54, 0x0a, 0x92, 0xbf, 0x7d, 0x11, 0x14, 0x3f,
    0x83, 0xd4, 0x11, 0x3f, 0x55, 0xf3, 0xbf, 0x3e, 0x9f, 0x07,
    0x24, 0x3d, 0x24, 0xd5, 0xd8, 0xbc, 0x10, 0x5a, 0x75, 0xbd,
    0x27, 0x15, 0xa7, 0xbe, 0x19, 0x94, 0x00, 0xbe, 0xc1, 0x21,
    0x24, 0x3e, 0x8a, 0xfd, 0x71, 0xbe, 0xbe, 0x99, 0xda, 0xbe,
    0x07, 0x03, 0x9b, 0x3e, 0x7d, 0x8e, 0x8d, 0x3f, 0x04, 0x18,
    0xc9, 0x3f, 0xb7, 0xb3, 0xcf, 0x3f, 0x83, 0xf9, 0x4c, 0x3f,
    0x18, 0xeb, 0xcf, 0x3e, 0xaa, 0x99, 0x75, 0x3f, 0x61, 0xb9,
    0x32, 0x3d, 0xfe, 0xbe, 0x9e, 0x3e, 0xe7, 0x6d, 0x84, 0x3f,
    0x73, 0x32, 0x98, 0x3f, 0x00, 0x0d, 0xde, 0x3e, 0x9d, 0x88,
    0x9a, 0x3e, 0x27, 0xb7, 0xb5, 0x3e, 0x1e, 0x36, 0xd7, 0x3e,
    0x01, 0x64, 0xc9, 0xbd, 0xf2, 0x64, 0x00, 0x3f, 0x10, 0xc1,
    0xab, 0x3f, 0x35, 0xf8, 0xcc, 0x3f, 0xc4, 0x52, 0xa2, 0x3f,
    0x0b, 0xa6, 0x58, 0x3f, 0xe7, 0x5d, 0x3a, 0x3d, 0xaa, 0x59,
    0x63, 0x3f, 0x08, 0x6d, 0x20, 0x3f, 0x74, 0x2b, 0xdc, 0xbe,
    0xf8, 0xa5, 0x5e, 0xbf, 0xbc, 0xb7, 0x17, 0xbf, 0x51, 0xdb,
    0x20, 0xbf, 0xac, 0xf9, 0x11, 0xbe, 0x0f, 0x4f, 0x90, 0x3e,
    0xdc, 0x8d, 0x52, 0xbd, 0x93, 0x64, 0x80, 0x3e, 0x38, 0xbe,
    0x9b, 0xbe, 0x3f, 0x16, 0x9a, 0xbf, 0x6a, 0x5d, 0x54, 0xbf,
    0x98, 0x0a, 0x08, 0x3f, 0x50, 0x9d, 0x54, 0xbd, 0x13, 0x47,
    0xd6, 0x3e, 0xa6, 0x42, 0xd7, 0xbe, 0x33, 0x2a, 0xe4, 0x3e,
    0x33, 0xf0, 0xb0, 0xbe, 0x68, 0xe8, 0xa7, 0xbf, 0xec, 0x9d,
    0x47, 0xbf, 0x25, 0x77, 0x42, 0xbf, 0x7d, 0x38, 0x63, 0x3d,
    0x38, 0x11, 0xb2, 0x3e, 0xda, 0xc5, 0x41, 0xbe, 0x87, 0x1b,
    0x55, 0x3e, 0xfa, 0x60, 0x05, 0x3f, 0x75, 0x82, 0x80, 0xbd,
    0xc4, 0xa8, 0xf2, 0x3e, 0x1f, 0xfb, 0x88, 0xbf, 0x61, 0x3d,
    0x41, 0x3f, 0x0e, 0xb2, 0xc0, 0xbd, 0xd7, 0x11, 0x61, 0x3f,
    0x3e, 0xb2, 0xbe, 0x3f, 0x48, 0xce, 0x0f, 0x3f, 0x88, 0x5e,
    0x09, 0xbe, 0xff, 0xa9, 0xe4, 0x3e, 0xd9, 0x1d, 0x6f, 0xbf,
    0x0d, 0x73, 0x53, 0x3f, 0x54, 0x04, 0x73, 0x3e, 0x21, 0x48,
    0x29, 0x3f, 0x59, 0x79, 0xaf, 0x3f, 0xe5, 0xc6, 0x9d, 0x3e,
    0xbb, 0x67, 0x55, 0x3d, 0x9e, 0xd5, 0x17, 0x3f, 0xd2, 0xdd,
    0x9f, 0xbf, 0x60, 0x70, 0x2a, 0x3f, 0x02, 0x59, 0x5d, 0xbe,
    0x7a, 0xbe, 0x3d, 0x3f, 0x4b, 0xe7, 0xe0, 0x3f, 0xe9, 0x12,
    0xf9, 0xbe, 0xd8, 0xa8, 0x97, 0xbe, 0xe4, 0x39, 0x91, 0xbf,
    0xac, 0xd6, 0xab, 0xbf, 0x24, 0x04, 0x90, 0xbc, 0x7e, 0x96,
    0xad, 0xbe, 0x93, 0x71, 0x05, 0xbe, 0x88, 0x49, 0x07, 0x3d,
    0xf7, 0x80, 0x04, 0xbf, 0x1e, 0x13, 0x49, 0x3c, 0x97, 0x8c,
    0xaa, 0xbf, 0x88, 0x29, 0x82, 0xbf, 0x1b, 0xda, 0x3b, 0xbe,
    0xbb, 0xa3, 0x46, 0xbe, 0xc3, 0x64, 0x86, 0xbd, 0x34, 0x78,
    0x8d, 0xbe, 0xa1, 0xa3, 0x1f, 0xbf, 0xb7, 0x8d, 0xb7, 0xbe,
    0xce, 0xea, 0x8e, 0xbf, 0xa6, 0x4c, 0x3c, 0xbf, 0x81, 0x8a,
    0x8a, 0xbe, 0x2f, 0x55, 0x92, 0xbe, 0x24, 0x6d, 0x99, 0xbd,
    0xca, 0x61, 0x2b, 0x3e, 0xa0, 0x13, 0x08, 0xbf, 0x88, 0x1e,
    0x62, 0x3f, 0x42, 0xf6, 0xc8, 0xbe, 0xaf, 0xc6, 0x41, 0x3e,
    0x22, 0xed, 0xcf, 0xbd, 0x60, 0xbd, 0x89, 0x3f, 0x39, 0xb7,
    0x61, 0x3f, 0xea, 0x1c, 0xa6, 0x3e, 0x8f, 0xbb, 0xb3, 0x3e,
    0xfe, 0xfc, 0xc7, 0x3f, 0x00, 0xcb, 0x76, 0x3f, 0x07, 0xbc,
    0x46, 0x3f, 0x12, 0x47, 0xb1, 0xbc, 0x0d, 0x7d, 0xb2, 0x3f,
    0x55, 0xaf, 0x23, 0x3f, 0x62, 0xa6, 0x09, 0x3f, 0xda, 0x1a,
    0xd2, 0xbd, 0x2c, 0xe0, 0xd0, 0x3f, 0xc8, 0x1b, 0xf7, 0x3e,
    0xb0, 0x26, 0xc2, 0x3e, 0x58, 0x0a, 0xd7, 0xbd, 0x2a, 0x94,
    0x85, 0x3f, 0x9a, 0xa0, 0xaf, 0xbd, 0x42, 0xdc, 0x01, 0x3f,
    0x9b, 0xad, 0x66, 0x3e, 0x41, 0xe8, 0x2c, 0x3f, 0x3d, 0xbc,
    0x29, 0x3e, 0x29, 0x8f, 0x49, 0x3e, 0x5f, 0x16, 0x92, 0x3e,
    0xf4, 0xc9, 0x56, 0x3f, 0x1f, 0x6f, 0xce, 0xbd, 0xa1, 0x87,
    0x28, 0x3e, 0x50, 0x47, 0x89, 0x3e, 0x42, 0x93, 0xf6, 0x3e,
    0xed, 0xfc, 0x3a, 0x3e, 0xb5, 0x09, 0x46, 0x3e, 0x78, 0x41,
    0x0d, 0xbe, 0xaf, 0xc6, 0x97, 0xbf, 0x52, 0x17, 0xa7, 0x3e,
    0x39, 0x4f, 0xd3, 0x3e, 0xe0, 0xd8, 0x95, 0x3e, 0xae, 0x48,
    0xbf, 0xbf, 0xe5, 0xb5, 0x13, 0x3f, 0x86, 0xa1, 0xab, 0xbd,
    0x8f, 0xa6, 0xbb, 0x3e, 0x76, 0x9b, 0xc9, 0xbf, 0xb8, 0xd1,
    0x20, 0x3e, 0x9f, 0x5a, 0x2d, 0xbf, 0x78, 0xcc, 0x38, 0x3f,
    0x6b, 0x29, 0xfa, 0xbf, 0x85, 0x2e, 0x85, 0x3e, 0x6c, 0xc6,
    0x22, 0xbf, 0xd3, 0x2a, 0x23, 0xbe, 0xaf, 0x0e, 0x1c, 0xbe,
    0xa9, 0x17, 0xf2, 0x3e, 0xc2, 0x64, 0x9e, 0xbf, 0xa4, 0xdd,
    0x53, 0x3e, 0x2c, 0x7e, 0x38, 0xbe, 0x2e, 0x6d, 0xd6, 0x3e,
    0x23, 0xf4, 0xb0, 0xbf, 0xfe, 0x3e, 0x03, 0xc0, 0x47, 0xb9,
    0x25, 0xbe, 0x09, 0xd1, 0xf5, 0x3e, 0x33, 0xf0, 0xa8, 0xbf,
    0x27, 0xfe, 0x1a, 0xc0, 0x1f, 0x77, 0xc2, 0xbb, 0xfd, 0xf4,
    0x00, 0x3f, 0x08, 0x25, 0xcb, 0xbf, 0x58, 0x55, 0x9d, 0x3e,
    0x30, 0x0c, 0x09, 0xc0, 0x2b, 0xef, 0x98, 0xbe, 0x40, 0x08,
    0x69, 0x3e, 0x7d, 0x36, 0xe8, 0x3d, 0xb0, 0x2e, 0x08, 0xc0,
    0x2c, 0x01, 0x95, 0x3e, 0x42, 0x2d, 0x35, 0x3f, 0xa2, 0xab,
    0x92, 0x3e, 0x15, 0xc0, 0xa6, 0xbf, 0x53, 0xb5, 0x61, 0xbe,
    0x94, 0xed, 0x59, 0x3e, 0x61, 0x1e, 0x12, 0xbf, 0x2b, 0xbc,
    0xd5, 0xbd, 0xfc, 0x93, 0x3d, 0xbe, 0xec, 0xc9, 0x26, 0x3f,
    0x91, 0x6b, 0x8d, 0xbe, 0x9f, 0x5b, 0xef, 0x3a, 0xa1, 0xd7,
    0xfd, 0xbd, 0xad, 0xeb, 0xf0, 0xbe, 0x77, 0x3d, 0x3f, 0xb8,
    0xe9, 0xe1, 0xc9, 0xbe, 0x47, 0x91, 0x05, 0xbd, 0x7a, 0x4e,
    0x55, 0xbe, 0xa2, 0xe9, 0xef, 0xbe, 0xc0, 0xa7, 0x23, 0x3c,
    0xda, 0xe0, 0x1c, 0xbe, 0x5c, 0x86, 0x66, 0xbe, 0xed, 0x93,
    0xe8, 0xbd, 0xc3, 0x34, 0x5b, 0xbe, 0x97, 0x93, 0xe6, 0xbe,
    0x1c, 0xc3, 0x9a, 0xbe, 0xb0, 0x39, 0x9d, 0x3f, 0x9d, 0x1b,
    0xd2, 0x3d, 0xd0, 0xf7, 0xb2, 0x3f, 0x23, 0xe6, 0x1d, 0xbf,
    0xc6, 0x97, 0x9c, 0x3f, 0xf1, 0x33, 0xe5, 0x3d, 0x95, 0x7c,
    0xa7, 0x3f, 0xf4, 0xb8, 0x19, 0xbf, 0x92, 0xb1, 0x96, 0x3f,
    0x5e, 0xd0, 0xf6, 0x3d, 0xbf, 0x9e, 0x70, 0x3f, 0xe9, 0xe6,
    0xff, 0xbe, 0x63, 0xe2, 0xd7, 0x3f, 0x4b, 0x0e, 0xd7, 0x3c,
    0x1f, 0xf8, 0x92, 0x3f, 0x83, 0x61, 0x3b, 0xbf, 0x14, 0x4b,
    0x86, 0x3f, 0x20, 0xd5, 0x2d, 0x3e, 0x89, 0x73, 0xde, 0x3f,
    0x42, 0x67, 0x52, 0xbf, 0xaa, 0x2c, 0x3a, 0x3f, 0xb9, 0x49,
    0x3e, 0x3e, 0xe7, 0x1f, 0xb6, 0x3f, 0xfd, 0x0f, 0x1d, 0xbf,
    0xe8, 0x55, 0x8c, 0x3f, 0xa5, 0x3c, 0x45, 0x3e, 0xbd, 0x95,
    0xb3, 0x3f, 0x07, 0xaf, 0x4c, 0xbf, 0x21, 0xdd, 0x2e, 0x3f,
    0xf2, 0x4f, 0x9b, 0x3d, 0x3f, 0xd8, 0xe5, 0x3f, 0xa9, 0x7a,
    0x73, 0xbf, 0x96, 0x12, 0x8c, 0x3c, 0x49, 0xa4, 0x03, 0x3f,
    0xc9, 0xfb, 0x4f, 0x3f, 0x49, 0xa9, 0xa9, 0x3f, 0x30, 0x34,
    0x8e, 0xbc, 0x25, 0xfd, 0x02, 0x3f, 0x2b, 0x26, 0xc2, 0x3e,
    0xa2, 0xb3, 0x9a, 0x3f, 0x91, 0xeb, 0x20, 0x3d, 0x6e, 0xab,
    0xf7, 0x3e, 0x19, 0x7b, 0x5d, 0x3f, 0x8c, 0xc9, 0xa5, 0x3f,
    0xa8, 0x5b, 0x92, 0xbd, 0x2e, 0xc5, 0x40, 0xc0, 0x6f, 0xe4,
    0x7e, 0x3f, 0x84, 0x85, 0xd3, 0x3f, 0xf4, 0x63, 0xa2, 0xbd,
    0xf6, 0x0b, 0x07, 0x3f, 0xd1, 0xae, 0xed, 0x3e, 0xb6, 0x34,
    0xde, 0x3f, 0x9e, 0xe9, 0xc2, 0xbd, 0x74, 0x8d, 0xed, 0x3e,
    0xc3, 0x8c, 0x85, 0x3f, 0xed, 0x77, 0x85, 0x3f, 0xca, 0x7a,
    0x0a, 0xbc, 0x1a, 0x97, 0xe5, 0x3e, 0xde, 0x3e, 0xf4, 0x3e,
    0xe3, 0xae, 0xb4, 0x3f, 0xe1, 0xea, 0x8a, 0x3d, 0xb3, 0x3c,
    0xab, 0x3f, 0x2f, 0x20, 0x42, 0x3f, 0x43, 0x8e, 0x7a, 0x3f,
    0xcc, 0x3d, 0x50, 0x3f, 0xfa, 0x81, 0x97, 0x3f, 0xd7, 0xde,
    0x38, 0xbf, 0xee, 0x3b, 0x08, 0x3f, 0x65, 0xa1, 0x99, 0x3e,
    0x4f, 0x55, 0x89, 0xbd, 0x88, 0xb1, 0xca, 0x3d, 0xa8, 0x51,
    0x85, 0x3d, 0xf1, 0xcc, 0x50, 0x3e, 0xc6, 0x1e, 0x43, 0x3f,
    0xb0, 0xee, 0xe3, 0x3e, 0xd5, 0x8d, 0xf8, 0x3e, 0x1a, 0x8f,
    0x15, 0xbf, 0x7e, 0x0c, 0x2f, 0xc0, 0x70, 0x9f, 0xbf, 0x3e,
    0x6b, 0x04, 0x3f, 0xbf, 0x49, 0x23, 0x1f, 0xc0, 0xfe, 0x85,
    0x24, 0xc0, 0x46, 0xa6, 0x0f, 0x3f, 0x3f, 0x5e, 0x81, 0x3f,
    0x53, 0xc6, 0x4e, 0x3e, 0xd1, 0x82, 0x9c, 0x3e, 0x53, 0xff,
    0xd2, 0xbf, 0x9b, 0x93, 0x79, 0xbf, 0x74, 0x5b, 0xe2, 0xbf,
    0x5d, 0x2d, 0x3b, 0xbe, 0xf8, 0x61, 0x34, 0x3f, 0x24, 0xe8,
    0x5c, 0x3e, 0x72, 0xe1, 0xfe, 0xbe, 0xda, 0x2e, 0x2d, 0x3e,
    0xe4, 0x9e, 0x75, 0xbf, 0xf1, 0x56, 0x2e, 0xbf, 0x63, 0x13,
    0x2a, 0x3f, 0x60, 0x9e, 0x45, 0x3f, 0xfb, 0xb4, 0x89, 0x3c,
    0x4f, 0xaf, 0x92, 0xbe, 0x35, 0x75, 0x28, 0x3f, 0xef, 0xbe,
    0x5d, 0x3e, 0xde, 0xef, 0xcd, 0x3e, 0x79, 0xe3, 0x8e, 0xbe,
    0xc3, 0xcd, 0xc4, 0xbf, 0xfb, 0x67, 0xa7, 0x3e, 0x1a, 0x4a,
    0x35, 0x3d, 0xfe, 0xac, 0x5a, 0x3f, 0xaa, 0xe0, 0xa5, 0x3e,
    0x6c, 0x50, 0x25, 0xbe, 0xce, 0xbc, 0x62, 0x3f, 0xb4,
    0x59, 0xd6, 0xbd, 0xf4, 0x08, 0xda, 0xbf, 0xeb, 0x25,
    0xa4, 0xbf, 0x70, 0x18, 0x37, 0x3f, 0xb9, 0xe2, 0xb0,
    0x3c, 0x67, 0xd8, 0x92, 0xbf, 0x3f, 0x12, 0x9d, 0x3e  };
  return AI_HANDLE_PTR(s_network_weights);
}
