#ifndef _WaveTables_H_
#define _WaveTables_H_

const uint8_t GDEP015OC1_LUTDefault_full[] =
{
  0x32,  // command
  0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GDEP015OC1_LUTDefault_part[] =
{
  0x32,  // command
  0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxGDE0213B1_LUTDefault_full[] =
{
  0x32,  // command
  0x22, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x01, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxGDE0213B1_LUTDefault_part[] =
{
  0x32,  // command
  0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxGDEH029A1_LUTDefault_full[] =
{
  0x32,  // command
  0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxGDEH029A1_LUTDefault_part[] =
{
  0x32,  // command
  0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxGDEW027W3_lut_20_vcomDC[] =
{
0x00  ,0x00 ,
0x00  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,    
0x00  ,0x32 ,0x32 ,0x00 ,0x00 ,0x02,    
0x00  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,    
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,    
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,    
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,    
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
};
//R21H
const uint8_t GxGDEW027W3_lut_21_ww[] =
{
0x50  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x60  ,0x32 ,0x32 ,0x00 ,0x00 ,0x02,
0xA0  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
};
//R22H  r
const uint8_t GxGDEW027W3_lut_22_bw[] =
{
0x50  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x60  ,0x32 ,0x32 ,0x00 ,0x00 ,0x02,
0xA0  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
};
//R23H  w
const uint8_t GxGDEW027W3_lut_23_wb[] =
{
0xA0  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x60  ,0x32 ,0x32 ,0x00 ,0x00 ,0x02,
0x50  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
};
//R24H  b
const uint8_t GxGDEW027W3_lut_24_bb[] =
{
0xA0  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x60  ,0x32 ,0x32 ,0x00 ,0x00 ,0x02,
0x50  ,0x0F ,0x0F ,0x00 ,0x00 ,0x05,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
};

const uint8_t GxGDEW0154Z04_lut_20_vcom0[] = {  0x0E  , 0x14 , 0x01 , 0x0A , 0x06 , 0x04 , 0x0A , 0x0A , 0x0F , 0x03 , 0x03 , 0x0C , 0x06 , 0x0A , 0x00 };
const uint8_t GxGDEW0154Z04_lut_21_w[] = {  0x0E  , 0x14 , 0x01 , 0x0A , 0x46 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x86 , 0x0A , 0x04 };
const uint8_t GxGDEW0154Z04_lut_22_b[] = {  0x0E  , 0x14 , 0x01 , 0x8A , 0x06 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x06 , 0x4A , 0x04 };
const uint8_t GxGDEW0154Z04_lut_23_g1[] = { 0x8E  , 0x94 , 0x01 , 0x8A , 0x06 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x06 , 0x0A , 0x04 };
const uint8_t GxGDEW0154Z04_lut_24_g2[] = { 0x8E  , 0x94 , 0x01 , 0x8A , 0x06 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x06 , 0x0A , 0x04 };
const uint8_t GxGDEW0154Z04_lut_25_vcom1[] = {  0x03  , 0x1D , 0x01 , 0x01 , 0x08 , 0x23 , 0x37 , 0x37 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };
const uint8_t GxGDEW0154Z04_lut_26_red0[] = { 0x83  , 0x5D , 0x01 , 0x81 , 0x48 , 0x23 , 0x77 , 0x77 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };
const uint8_t GxGDEW0154Z04_lut_27_red1[] = { 0x03  , 0x1D , 0x01 , 0x01 , 0x08 , 0x23 , 0x37 , 0x37 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };

const uint8_t GxGDEW027C44_lut_20_vcomDC[] =
{
  0x00  , 0x00,
  0x00  , 0x1A  , 0x1A  , 0x00  , 0x00  , 0x01,
  0x00  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x00  , 0x0E  , 0x01  , 0x0E  , 0x01  , 0x10,
  0x00  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x00  , 0x04  , 0x10  , 0x00  , 0x00  , 0x05,
  0x00  , 0x03  , 0x0E  , 0x00  , 0x00  , 0x0A,
  0x00  , 0x23  , 0x00  , 0x00  , 0x00  , 0x01
};
//R21H
const uint8_t GxGDEW027C44_lut_21[] = {
  0x90  , 0x1A  , 0x1A  , 0x00  , 0x00  , 0x01,
  0x40  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x84  , 0x0E  , 0x01  , 0x0E  , 0x01  , 0x10,
  0x80  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x00  , 0x04  , 0x10  , 0x00  , 0x00  , 0x05,
  0x00  , 0x03  , 0x0E  , 0x00  , 0x00  , 0x0A,
  0x00  , 0x23  , 0x00  , 0x00  , 0x00  , 0x01
};
//R22H  r
const uint8_t GxGDEW027C44_lut_22_red[] = {
  0xA0  , 0x1A  , 0x1A  , 0x00  , 0x00  , 0x01,
  0x00  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x84  , 0x0E  , 0x01  , 0x0E  , 0x01  , 0x10,
  0x90  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0xB0  , 0x04  , 0x10  , 0x00  , 0x00  , 0x05,
  0xB0  , 0x03  , 0x0E  , 0x00  , 0x00  , 0x0A,
  0xC0  , 0x23  , 0x00  , 0x00  , 0x00  , 0x01
};
//R23H  w
const uint8_t GxGDEW027C44_lut_23_white[] = {
  0x90  , 0x1A  , 0x1A  , 0x00  , 0x00  , 0x01,
  0x40  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x84  , 0x0E  , 0x01  , 0x0E  , 0x01  , 0x10,
  0x80  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x00  , 0x04  , 0x10  , 0x00  , 0x00  , 0x05,
  0x00  , 0x03  , 0x0E  , 0x00  , 0x00  , 0x0A,
  0x00  , 0x23  , 0x00  , 0x00  , 0x00  , 0x01
};
//R24H  b
const uint8_t GxGDEW027C44_lut_24_black[] = {
  0x90  , 0x1A  , 0x1A  , 0x00  , 0x00  , 0x01,
  0x20  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x84  , 0x0E  , 0x01  , 0x0E  , 0x01  , 0x10,
  0x10  , 0x0A  , 0x0A  , 0x00  , 0x00  , 0x08,
  0x00  , 0x04  , 0x10  , 0x00  , 0x00  , 0x05,
  0x00  , 0x03  , 0x0E  , 0x00  , 0x00  , 0x0A,
  0x00  , 0x23  , 0x00  , 0x00  , 0x00  , 0x01
};

const unsigned char GxGDEW042T2_lut_20_vcom0_full[] =
{
0x40, 0x17, 0x00, 0x00, 0x00, 0x02,        
0x00, 0x17, 0x17, 0x00, 0x00, 0x02,        
0x00, 0x0A, 0x01, 0x00, 0x00, 0x01,        
0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,        
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char GxGDEW042T2_lut_21_ww_full[] =
{
0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char GxGDEW042T2_lut_22_bw_full[] =
{
0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     
};

const unsigned char GxGDEW042T2_lut_23_wb_full[] =
{
0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,         
};

const unsigned char GxGDEW042T2_lut_24_bb_full[] =
{
0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
};

#define TP0A  2 // sustain phase for bb and ww, change phase for bw and wb
#define TP0B 45 // change phase for bw and wb

const unsigned char GxGDEW042T2_lut_20_vcom0_partial[] =
{
  0x00,
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char GxGDEW042T2_lut_21_ww_partial[] =
{
  0x80, // 10 00 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char GxGDEW042T2_lut_22_bw_partial[] =
{
  0xA0, // 10 10 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char GxGDEW042T2_lut_23_wb_partial[] =
{
  0x50, // 01 01 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char GxGDEW042T2_lut_24_bb_partial[] =
{
  0x40, // 01 00 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#endif

