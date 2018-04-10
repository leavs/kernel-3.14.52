#ifndef _LINUX_GOODIX_GT9XX_H
#define	_LINUX_GOODIX_GT9XX_H
//GT928_4052L 1280*800 DPT
uint8_t gt9xx_ts[] = {
    0x45,0x00,0x03,0x00,0x04,0x05,0x39,0x00,
    0x03,0x58,0x14,0x0F,0x78,0x64,0x03,0x03,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
    0x1B,0x1D,0x14,0x8C,0x2D,0x0E,0x06,0x08,
    0xC1,0x11,0x00,0x00,0x09,0x9C,0x03,0x3D,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x23,0xA0,0x00,0x00,0x00,
    0x00,0x00,0x00,0x05,0x0D,0x18,0xCD,0x0D,
    0x1F,0x5D,0x0F,0x26,0x25,0x10,0x31,0xED,
    0x10,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x1C,0x1A,0x18,0x16,0x14,0x12,0x10,0x0E,
	0x0C,0x0A,0x08,0x06,0x04,0x02,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x14,0x13,
	0x12,0x10,0x0F,0x0C,0x0A,0x08,0x06,0x04,
	0x02,0x00,0x16,0x18,0x1C,0x1D,0x1E,0x1F,
	0x20,0x21,0x22,0x24,0x26,0x28,0x29,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0x36,0x01
};

uint8_t gt911_group1[] = {
0x00,0x20,0x03,0x00,0x05,0x0A,0x05,0x00,
0x01,0x08,0x19,0x0B,0x50,0x3C,0x03,0x05,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
0x1A,0x1E,0x14,0x8F,0x2F,0xAA,0x1E,0x20,
0x31,0x0D,0x00,0x00,0x00,0x03,0x03,0x1D,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x19,0x46,0x94,0xC5,0x02,
0x07,0x00,0x00,0x04,0x8A,0x1B,0x00,0x74,
0x22,0x00,0x64,0x2A,0x00,0x58,0x33,0x00,
0x4E,0x3F,0x00,0x4E,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,
0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,
0x16,0x17,0x18,0x19,0xFF,0xFF,0xFF,0xFF,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
0x10,0x11,0x12,0x13,0x14,0x19,0x1B,0x1C,
0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,
0x26,0x27,0x28,0x29,0xFF,0xFF,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x08,0x01
};

uint8_t gt911_group2[] = {
	0x50,0x20,0x03,0x00,0x05,0x0A,0x35,0x00,
	0x01,0x0A,0x19,0x0B,0x5A,0x32,0x03,0x05,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
	0x1A,0x1E,0x14,0x8F,0x2F,0xAA,0x2E,0x30,
	0x46,0x10,0x00,0x00,0x00,0x9A,0x03,0x1D,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x24,0x48,0x94,0xC5,0x03,
	0x08,0x00,0x00,0x04,0x8E,0x26,0x00,0x83,
	0x2C,0x00,0x79,0x33,0x00,0x71,0x3A,0x00,
	0x6B,0x43,0x00,0x6B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,
	0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,
	0x16,0x17,0x18,0x19,0xFF,0xFF,0xFF,0xFF,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
	0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
	0x10,0x11,0x12,0x13,0x14,0x19,0x1B,0x1C,
	0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,
	0x26,0x27,0x28,0x29,0xFF,0xFF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x04,0x01
};
/*
uint8_t gt928_group2[] = {
	0x51,0x20,0x03,0x00,0x05,0x0A,0x34,0x00,
	0x02,0x0A,0x19,0x0C,0x64,0x3C,0x03,0x05,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
	0x1A,0x1D,0x14,0x8F,0x2F,0xAA,0x2E,0x30,
	0x46,0x10,0x00,0x00,0x00,0x9A,0x03,0x25,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,
	0x00,0x00,0x00,0x24,0x48,0x94,0xD5,0x03,
	0x08,0x00,0x00,0x04,0x8E,0x26,0x00,0x83,
	0x2C,0x00,0x79,0x33,0x00,0x72,0x3A,0x00,
	0x6B,0x43,0x00,0x6B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,
	0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,
	0x16,0x17,0x18,0x19,0xFF,0xFF,0xFF,0xFF,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
	0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
	0x10,0x11,0x12,0x13,0x14,0x19,0x1B,0x1C,
	0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,
	0x26,0x27,0x28,0x29,0xFF,0xFF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xA4,0x01
};

uint8_t gt911_group1[] = {
    0x45,0x00,0x03,0x00,0x04,0x05,0x39,0x00,
    0x03,0x58,0x14,0x0F,0x78,0x64,0x03,0x03,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
    0x1B,0x1D,0x14,0x8C,0x2D,0x0E,0x06,0x08,
    0xC1,0x11,0x00,0x00,0x09,0x9C,0x03,0x3D,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x23,0xA0,0x00,0x00,0x00,
    0x00,0x00,0x00,0x05,0x0D,0x18,0xCD,0x0D,
    0x1F,0x5D,0x0F,0x26,0x25,0x10,0x31,0xED,
    0x10,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x1C,0x1A,0x18,0x16,0x14,0x12,0x10,0x0E,
	0x0C,0x0A,0x08,0x06,0x04,0x02,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x14,0x13,
	0x12,0x10,0x0F,0x0C,0x0A,0x08,0x06,0x04,
	0x02,0x00,0x16,0x18,0x1C,0x1D,0x1E,0x1F,
	0x20,0x21,0x22,0x24,0x26,0x28,0x29,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0x36,0x01
};


uint8_t gt911_group2[] = {
	0x51,0x20,0x03,0x00,0x05,0x0A,0x34,0x00,
	0x02,0x0A,0x19,0x0C,0x64,0x3C,0x03,0x05,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
	0x1A,0x1D,0x14,0x8F,0x2F,0xAA,0x2E,0x30,
	0x46,0x10,0x00,0x00,0x00,0x9A,0x03,0x25,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,
	0x00,0x00,0x00,0x24,0x48,0x94,0xD5,0x03,
	0x08,0x00,0x00,0x04,0x8E,0x26,0x00,0x83,
	0x2C,0x00,0x79,0x33,0x00,0x72,0x3A,0x00,
	0x6B,0x43,0x00,0x6B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,
	0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,
	0x16,0x17,0x18,0x19,0xFF,0xFF,0xFF,0xFF,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
	0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
	0x10,0x11,0x12,0x13,0x14,0x19,0x1B,0x1C,
	0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,
	0x26,0x27,0x28,0x29,0xFF,0xFF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xA4,0x01
};


uint8_t gt911_group3[] = {
	0x55,0x00,0x04,0x00,0x03,0x0A,0x0D,0x00,
	0x01,0x0A,0x28,0x0F,0x50,0x3C,0x03,0x05,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x90,0x30,0xCC,0x2D,0x32,
	0xEE,0x06,0x00,0x00,0x02,0xBA,0x04,0x1D,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x1E,0x64,0x94,0xC5,0x02,
	0x07,0x00,0x00,0x04,0xA1,0x22,0x00,0x84,
	0x2B,0x00,0x6D,0x37,0x00,0x5B,0x46,0x00,
	0x4D,0x59,0x00,0x4D,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,
	0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,
	0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
	0x04,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0E,
	0x0F,0x10,0x11,0x12,0x13,0x14,0x19,0x1B,
	0x1C,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,
	0x25,0x26,0x27,0x28,0x29,0x2A,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x1C,0x01
	};

uint8_t gt911_group4[] = {
	0x52,0x00,0x04,0x58,0x02,0x05,0x31,0x00,
	0x03,0x9F,0x19,0x0F,0x50,0x3C,0x03,0x05,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x17,
	0x19,0x1C,0x14,0x8C,0x2E,0x0E,0x14,0x12,
	0x96,0x12,0x00,0x00,0x01,0x19,0x03,0x3E,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x32,0x96,0x94,0x85,0x02,
	0x08,0x00,0x00,0x87,0x13,0x12,0xF8,0x16,
	0x15,0xE3,0x1B,0x15,0x88,0x20,0x17,0x07,
	0x27,0x18,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,
	0x12,0x14,0x16,0x18,0x1A,0x1C,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x16,0x18,
	0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x24,
	0x26,0x28,0x29,0x2A,0x00,0x02,0x04,0x06,
	0x08,0x0A,0x0C,0x0F,0x10,0x12,0x13,0x14,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0x99,0x01
};*/
#endif