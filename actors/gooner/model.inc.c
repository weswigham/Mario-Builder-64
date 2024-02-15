Lights1 gooner_flamethrower_base_lights = gdSPDefLights1(
	0x7F, 0x7F, 0x7F,
	0xFF, 0xFF, 0xFF, 0x28, 0x28, 0x28);

Lights1 gooner_flamethrower_hole_lights = gdSPDefLights1(
	0x7F, 0x7F, 0x7F,
	0xFF, 0xFF, 0xFF, 0x28, 0x28, 0x28);

Gfx gooner_Flamethrower_Rock_ci8_aligner[] = {gsSPEndDisplayList()};
u8 gooner_Flamethrower_Rock_ci8[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
	0x06, 0x08, 0x09, 0x02, 0x01, 0x0a, 0x0b, 0x0a, 
	0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 
	0x14, 0x01, 0x15, 0x16, 0x08, 0x01, 0x0f, 0x17, 
	0x13, 0x0a, 0x18, 0x00, 0x19, 0x1a, 0x1b, 0x1c, 
	0x1d, 0x16, 0x03, 0x08, 0x08, 0x1e, 0x1f, 0x20, 
	0x21, 0x22, 0x23, 0x0f, 0x0f, 0x11, 0x23, 0x14, 
	0x00, 0x14, 0x0f, 0x24, 0x1e, 0x02, 0x02, 0x25, 
	0x20, 0x06, 0x18, 0x00, 0x26, 0x27, 0x28, 0x29, 
	0x14, 0x1c, 0x0b, 0x1e, 0x0a, 0x00, 0x2a, 0x21, 
	0x2b, 0x22, 0x21, 0x0b, 0x1b, 0x0f, 0x16, 0x13, 
	0x0a, 0x2c, 0x06, 0x2d, 0x1e, 0x2e, 0x06, 0x1b, 
	0x00, 0x0f, 0x18, 0x13, 0x2f, 0x30, 0x31, 0x05, 
	0x1b, 0x0b, 0x2a, 0x32, 0x33, 0x32, 0x10, 0x0b, 
	0x34, 0x35, 0x36, 0x37, 0x18, 0x38, 0x02, 0x13, 
	0x1d, 0x24, 0x13, 0x39, 0x1e, 0x0a, 0x00, 0x23, 
	0x00, 0x01, 0x0b, 0x3a, 0x3b, 0x1c, 0x1c, 0x26, 
	0x3c, 0x3c, 0x3d, 0x36, 0x3e, 0x1b, 0x3f, 0x01, 
	0x01, 0x18, 0x40, 0x41, 0x11, 0x42, 0x0a, 0x0a, 
	0x35, 0x01, 0x24, 0x16, 0x43, 0x14, 0x1b, 0x21, 
	0x0f, 0x01, 0x13, 0x37, 0x01, 0x1f, 0x0b, 0x18, 
	0x15, 0x44, 0x45, 0x1b, 0x0b, 0x08, 0x06, 0x46, 
	0x01, 0x0b, 0x1b, 0x3a, 0x47, 0x1b, 0x15, 0x0b, 
	0x1c, 0x39, 0x01, 0x06, 0x18, 0x14, 0x23, 0x17, 
	0x0f, 0x00, 0x48, 0x14, 0x02, 0x1e, 0x0a, 0x0b, 
	0x07, 0x0b, 0x32, 0x0c, 0x2d, 0x49, 0x14, 0x4a, 
	0x1e, 0x06, 0x14, 0x0b, 0x2f, 0x35, 0x18, 0x17, 
	0x08, 0x15, 0x0f, 0x1e, 0x14, 0x0b, 0x01, 0x00, 
	0x06, 0x4b, 0x1a, 0x0f, 0x24, 0x13, 0x4c, 0x06, 
	0x4d, 0x4e, 0x23, 0x18, 0x4f, 0x50, 0x00, 0x51, 
	0x14, 0x15, 0x14, 0x1f, 0x52, 0x48, 0x14, 0x23, 
	0x10, 0x53, 0x3e, 0x0d, 0x0f, 0x06, 0x54, 0x55, 
	0x56, 0x36, 0x0c, 0x14, 0x4c, 0x43, 0x13, 0x23, 
	0x09, 0x4e, 0x57, 0x18, 0x01, 0x16, 0x06, 0x00, 
	0x01, 0x06, 0x00, 0x14, 0x57, 0x58, 0x05, 0x2e, 
	0x59, 0x3e, 0x5a, 0x23, 0x23, 0x20, 0x5b, 0x06, 
	0x36, 0x0b, 0x1c, 0x13, 0x5c, 0x14, 0x0d, 0x23, 
	0x1c, 0x38, 0x10, 0x18, 0x1c, 0x50, 0x08, 0x34, 
	0x1e, 0x0b, 0x00, 0x13, 0x0a, 0x3e, 0x45, 0x48, 
	0x23, 0x2a, 0x3e, 0x20, 0x14, 0x1b, 0x4c, 0x56, 
	0x5d, 0x18, 0x5e, 0x0a, 0x5f, 0x06, 0x14, 0x12, 
	0x11, 0x03, 0x35, 0x18, 0x10, 0x60, 0x02, 0x17, 
	0x61, 0x14, 0x13, 0x10, 0x06, 0x14, 0x36, 0x62, 
	0x47, 0x59, 0x35, 0x05, 0x10, 0x26, 0x04, 0x63, 
	0x64, 0x0b, 0x2a, 0x01, 0x06, 0x01, 0x06, 0x35, 
	0x59, 0x65, 0x66, 0x09, 0x14, 0x14, 0x02, 0x0f, 
	0x0f, 0x17, 0x10, 0x39, 0x13, 0x00, 0x30, 0x2a, 
	0x1d, 0x01, 0x1f, 0x43, 0x0b, 0x1e, 0x18, 0x67, 
	0x10, 0x1b, 0x5a, 0x0d, 0x0a, 0x0b, 0x12, 0x59, 
	0x2a, 0x02, 0x33, 0x23, 0x01, 0x2d, 0x06, 0x06, 
	0x06, 0x17, 0x23, 0x35, 0x25, 0x37, 0x35, 0x0b, 
	0x38, 0x18, 0x1b, 0x1e, 0x61, 0x02, 0x02, 0x49, 
	0x06, 0x68, 0x0b, 0x0f, 0x00, 0x29, 0x35, 0x11, 
	0x00, 0x4c, 0x47, 0x47, 0x13, 0x06, 0x08, 0x16, 
	0x0d, 0x0d, 0x06, 0x10, 0x40, 0x21, 0x0d, 0x0b, 
	0x25, 0x13, 0x0c, 0x1e, 0x38, 0x2d, 0x08, 0x69, 
	0x02, 0x10, 0x6a, 0x20, 0x6b, 0x3e, 0x5a, 0x1b, 
	0x21, 0x21, 0x05, 0x68, 0x48, 0x05, 0x13, 0x01, 
	0x18, 0x2d, 0x6c, 0x4b, 0x6d, 0x00, 0x0f, 0x1d, 
	0x0b, 0x0f, 0x06, 0x0a, 0x6e, 0x6f, 0x01, 0x1c, 
	0x14, 0x1b, 0x35, 0x3e, 0x23, 0x3f, 0x70, 0x35, 
	0x71, 0x2c, 0x14, 0x0b, 0x1b, 0x56, 0x26, 0x1a, 
	0x35, 0x25, 0x35, 0x33, 0x72, 0x1e, 0x02, 0x35, 
	0x15, 0x01, 0x14, 0x14, 0x07, 0x06, 0x01, 0x0d, 
	0x23, 0x0b, 0x23, 0x41, 0x68, 0x05, 0x35, 0x73, 
	0x0b, 0x01, 0x74, 0x02, 0x46, 0x1e, 0x64, 0x70, 
	0x75, 0x68, 0x76, 0x21, 0x0a, 0x01, 0x06, 0x00, 
	0x0f, 0x14, 0x06, 0x06, 0x07, 0x24, 0x70, 0x17, 
	0x35, 0x6e, 0x00, 0x1a, 0x45, 0x29, 0x1b, 0x0b, 
	0x61, 0x14, 0x38, 0x77, 0x46, 0x14, 0x15, 0x2d, 
	0x14, 0x21, 0x78, 0x10, 0x0f, 0x77, 0x0d, 0x01, 
	0x1c, 0x0d, 0x15, 0x06, 0x15, 0x00, 0x20, 0x5a, 
	0x70, 0x1d, 0x1d, 0x79, 0x35, 0x0b, 0x18, 0x02, 
	0x7a, 0x13, 0x38, 0x02, 0x02, 0x1d, 0x14, 0x11, 
	0x06, 0x18, 0x1a, 0x18, 0x7b, 0x7c, 0x39, 0x01, 
	0x1c, 0x17, 0x00, 0x12, 0x06, 0x7d, 0x0d, 0x35, 
	0x0b, 0x6e, 0x32, 0x7e, 0x18, 0x18, 0x5c, 0x14, 
	0x16, 0x21, 0x02, 0x02, 0x61, 0x12, 0x23, 0x10, 
	0x14, 0x0b, 0x78, 0x18, 0x7f, 0x6e, 0x0d, 0x02, 
	0x10, 0x5a, 0x11, 0x1c, 0x0b, 0x80, 0x23, 0x2a, 
	0x0a, 0x3f, 0x81, 0x82, 0x13, 0x18, 0x83, 0x10, 
	0x15, 0x14, 0x0b, 0x1d, 0x61, 0x00, 0x70, 0x23, 
	0x06, 0x09, 0x79, 0x18, 0x23, 0x16, 0x1c, 0x02, 
	0x13, 0x39, 0x5f, 0x10, 0x00, 0x1d, 0x1c, 0x11, 
	0x5a, 0x21, 0x35, 0x56, 0x48, 0x1b, 0x0b, 0x0b, 
	0x7d, 0x5a, 0x1e, 0x0f, 0x0b, 0x01, 0x14, 0x0f, 
	0x0b, 0x84, 0x79, 0x1e, 0x17, 0x0d, 0x0d, 0x1d, 
	0x00, 0x17, 0x39, 0x14, 0x00, 0x0f, 0x0f, 0x3e, 
	0x10, 0x1b, 0x1f, 0x1e, 0x5e, 0x2b, 0x40, 0x5c, 
	0x1e, 0x18, 0x02, 0x35, 0x01, 0x7a, 0x06, 0x08, 
	0x25, 0x21, 0x85, 0x2e, 0x1d, 0x1c, 0x16, 0x01, 
	0x14, 0x17, 0x1d, 0x16, 0x1e, 0x0a, 0x0f, 0x86, 
	0x24, 0x1e, 0x1f, 0x1f, 0x7a, 0x0b, 0x1b, 0x40, 
	0x3e, 0x00, 0x02, 0x6e, 0x18, 0x87, 0x4e, 0x00, 
	0x88, 0x6a, 0x85, 0x23, 0x09, 0x17, 0x16, 0x15, 
	0x01, 0x0f, 0x0f, 0x0f, 0x01, 0x16, 0x10, 0x23, 
	0x89, 0x01, 0x00, 0x2e, 0x1f, 0x1e, 0x18, 0x0b, 
	0x6a, 0x05, 0x04, 0x21, 0x23, 0x40, 0x77, 0x3e, 
	0x21, 0x37, 0x04, 0x8a, 0x00, 0x0a, 0x01, 0x0a, 
	0x0f, 0x0f, 0x0a, 0x0f, 0x20, 0x21, 0x8b, 0x00, 
	0x01, 0x06, 0x8c, 0x2d, 0x01, 0x01, 0x7a, 0x5c, 
	0x1b, 0x1a, 0x5c, 0x43, 0x4a, 0x1f, 0x25, 0x06, 
	0x14, 0x11, 0x8d, 0x31, 0x3e, 0x13, 0x8a, 0x6a, 
	0x8e, 0x37, 0x8f, 0x78, 0x26, 0x30, 0x30, 0x0c, 
	0x23, 0x4f, 0x10, 0x89, 0x73, 0x13, 0x0b, 0x13, 
	0x1b, 0x90, 0x18, 0x0b, 0x20, 0x06, 0x01, 0x01, 
	0x01, 0x06, 0x5a, 0x6d, 0x47, 0x32, 0x59, 0x59, 
	0x0a, 0x1e, 0x18, 0x18, 0x18, 0x0b, 0x3e, 0x13, 
	0x14, 0x35, 0x06, 0x89, 0x00, 0x2e, 0x43, 0x0a, 
	0x21, 0x68, 0x18, 0x14, 0x70, 0x14, 0x14, 0x1b, 
	0x01, 0x01, 0x00, 0x32, 0x21, 0x12, 0x01, 0x80, 
	0x02, 0x01, 0x80, 0x49, 0x4d, 0x13, 0x0b, 0x59, 
	0x91, 0x11, 0x14, 0x18, 0x06, 0x1e, 0x1e, 0x2b, 
	0x2f, 0x14, 0x06, 0x25, 0x01, 0x00, 0x14, 0x0a, 
	0x01, 0x18, 0x25, 0x3d, 0x23, 0x1b, 0x35, 0x49, 
	0x12, 0x14, 0x12, 0x17, 0x49, 0x0c, 0x83, 0x06, 
	0x01, 0x2a, 0x35, 0x0a, 0x14, 0x18, 0x10, 0x21, 
	0x13, 0x7d, 0x14, 0x14, 0x00, 0x13, 0x0d, 0x20, 
	0x0a, 0x18, 0x06, 0x6a, 0x34, 0x56, 0x5f, 0x06, 
	0x13, 0x06, 0x06, 0x1e, 0x02, 0x4c, 0x43, 0x1f, 
	0x02, 0x2e, 0x5e, 0x2a, 0x06, 0x14, 0x92, 0x23, 
	0x1e, 0x35, 0x00, 0x0f, 0x13, 0x14, 0x14, 0x14, 
	0x02, 0x18, 0x40, 0x01, 0x5b, 0x93, 0x74, 0x50, 
	0x12, 0x01, 0x0f, 0x1e, 0x0d, 0x15, 0x08, 0x0b, 
	0x39, 0x60, 0x06, 0x3c, 0x21, 0x8e, 0x0b, 0x4a, 
	0x02, 0x0f, 0x14, 0x01, 0x1b, 0x14, 0x14, 0x00, 
	0x1e, 0x18, 0x30, 0x0b, 0x94, 0x95, 0x12, 0x13, 
	0x1d, 0x24, 0x14, 0x1d, 0x15, 0x0f, 0x08, 0x0f, 
	
};

Gfx gooner_Flamethrower_Rock_pal_rgba16_aligner[] = {gsSPEndDisplayList()};
u8 gooner_Flamethrower_Rock_pal_rgba16[] = {
	0xb5, 0xed, 0xce, 0xb3, 0xd6, 0xf5, 0xad, 0x69, 
	0x4a, 0xd3, 0x84, 0xa1, 0xc6, 0x71, 0xc6, 0x2d, 
	0xd6, 0xb3, 0xef, 0x7b, 0xce, 0x73, 0xd6, 0xb5, 
	0xbe, 0x31, 0xbd, 0xed, 0x94, 0x9f, 0xce, 0x71, 
	0x9d, 0x27, 0x9d, 0x25, 0xa5, 0xa9, 0xad, 0xab, 
	0xbe, 0x2f, 0xc6, 0x2f, 0xbe, 0x2d, 0xad, 0xa9, 
	0xe7, 0x39, 0x3a, 0x4f, 0x74, 0x1d, 0xc6, 0x31, 
	0xb5, 0xab, 0xb5, 0xeb, 0xde, 0xf7, 0xce, 0xb5, 
	0xb5, 0xad, 0x8c, 0xa3, 0x5b, 0x15, 0xa5, 0x69, 
	0xc6, 0x6f, 0xbd, 0xef, 0x7c, 0x5f, 0x42, 0x91, 
	0x8c, 0xa5, 0xb5, 0xef, 0x94, 0xa3, 0x74, 0x1f, 
	0xa5, 0xa7, 0xce, 0xb1, 0xc6, 0x73, 0x63, 0x59, 
	0x7c, 0x1d, 0x52, 0xd3, 0x84, 0x5f, 0x5b, 0x55, 
	0xb6, 0x67, 0x94, 0xe5, 0x63, 0x99, 0x73, 0xdb, 
	0xc6, 0x6d, 0xa5, 0x67, 0x84, 0x9d, 0x74, 0x17, 
	0x5b, 0x97, 0x42, 0x8d, 0x94, 0xa5, 0xa5, 0x6b, 
	0x84, 0x61, 0x63, 0x55, 0xad, 0xe7, 0xdf, 0x39, 
	0x95, 0x25, 0x53, 0x15, 0xd6, 0xf3, 0x5b, 0x57, 
	0x7c, 0x1f, 0xbe, 0x6f, 0xdf, 0x37, 0x8c, 0x9f, 
	0x9d, 0x67, 0xce, 0x2f, 0xf7, 0x7d, 0xc6, 0xb1, 
	0xb6, 0x2d, 0xd7, 0x37, 0x6b, 0x55, 0xc5, 0xef, 
	0xe7, 0x7b, 0xbe, 0x6d, 0x95, 0x27, 0xa5, 0x25, 
	0x63, 0x57, 0x8c, 0xa1, 0xa5, 0x27, 0xc6, 0xb3, 
	0xa5, 0xab, 0x6b, 0xdd, 0x94, 0xa1, 0xad, 0xeb, 
	0xce, 0xf3, 0xde, 0xf5, 0x4a, 0xcf, 0x42, 0x8f, 
	0x9d, 0x65, 0xdf, 0x33, 0x7c, 0x5d, 0xa5, 0xa5, 
	0x73, 0xdd, 0xb5, 0xa9, 0x6b, 0x9b, 0xa5, 0x29, 
	0xb5, 0xe9, 0x63, 0x97, 0xbd, 0xeb, 0xce, 0x6f, 
	0xad, 0x6b, 0x7c, 0x1b, 0x9d, 0x29, 0xb6, 0x2f, 
	0xb6, 0x2b, 0x73, 0xd9, 0x52, 0xcf, 0xb6, 0x27, 
	0x6b, 0xdb, 0x52, 0xd5, 0xd6, 0xf7, 0xa5, 0x65, 
	0xad, 0x65, 0x9c, 0xe5, 0x5b, 0x17, 0xad, 0xe9, 
	0x9c, 0xe3, 0x4a, 0x53, 0x7c, 0x5b, 0xbe, 0x71, 
	0xf7, 0xbd, 0x42, 0x51, 0x8c, 0x5f, 0xc6, 0xad, 
	0x84, 0x21, 0xc6, 0xaf, 0x8c, 0x9d, 0x53, 0x17, 
	0x9d, 0x69, 0x6b, 0x59, 0x84, 0x5d, 0x7c, 0x19, 
	0x74, 0x1b, 0x7b, 0xd7, 0x73, 0xdf, 0xa5, 0xe7, 
	0xad, 0xef, 0x94, 0xe3, 
};

Gfx gooner_Flamethrower_Hole_ci8_aligner[] = {gsSPEndDisplayList()};
u8 gooner_Flamethrower_Hole_ci8[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 
	0x06, 0x07, 0x08, 0x09, 0x0a, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x06, 0x10, 
	0x11, 0x07, 0x12, 0x13, 0x0d, 0x14, 0x15, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
	0x08, 0x13, 0x16, 0x0f, 0x17, 0x0d, 0x18, 0x06, 
	0x05, 0x19, 0x0d, 0x1a, 0x0e, 0x0d, 0x1b, 0x1c, 
	0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x0c, 
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x0d, 
	0x0d, 0x25, 0x10, 0x26, 0x27, 0x28, 0x0e, 0x29, 
	0x2a, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x2c, 0x2d, 
	0x2e, 0x26, 0x26, 0x1f, 0x13, 0x10, 0x2f, 0x11, 
	0x30, 0x31, 0x2a, 0x32, 0x12, 0x1a, 0x0f, 0x2e, 
	0x1b, 0x25, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1d, 0x0c, 0x0d, 0x32, 
	0x2e, 0x34, 0x1b, 0x04, 0x35, 0x24, 0x25, 0x36, 
	0x36, 0x02, 0x10, 0x10, 0x08, 0x32, 0x2e, 0x12, 
	0x2e, 0x1b, 0x37, 0x15, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x38, 0x2a, 0x05, 0x39, 0x3a, 
	0x3b, 0x37, 0x25, 0x30, 0x31, 0x3c, 0x3d, 0x31, 
	0x31, 0x19, 0x18, 0x3d, 0x10, 0x03, 0x3e, 0x3f, 
	0x12, 0x0d, 0x37, 0x08, 0x0a, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x02, 0x37, 0x2d, 0x06, 0x40, 
	0x41, 0x42, 0x18, 0x36, 0x43, 0x36, 0x44, 0x44, 
	0x44, 0x44, 0x45, 0x46, 0x31, 0x11, 0x35, 0x41, 
	0x13, 0x1e, 0x2d, 0x0c, 0x24, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0x24, 0x0c, 0x32, 0x2a, 0x1c, 
	0x41, 0x47, 0x46, 0x36, 0x44, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x44, 0x46, 0x31, 0x49, 0x35, 
	0x06, 0x39, 0x13, 0x2c, 0x08, 0x4a, 0x00, 0x00, 
	0x00, 0x00, 0x3d, 0x03, 0x0c, 0x06, 0x05, 0x03, 
	0x11, 0x18, 0x36, 0x44, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x4b, 0x46, 0x3d, 0x24, 
	0x08, 0x2d, 0x0d, 0x05, 0x4c, 0x49, 0x00, 0x00, 
	0x00, 0x00, 0x42, 0x03, 0x06, 0x39, 0x06, 0x35, 
	0x19, 0x3c, 0x44, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x44, 0x4d, 0x49, 
	0x03, 0x14, 0x0d, 0x05, 0x04, 0x24, 0x4e, 0x00, 
	0x00, 0x01, 0x49, 0x2a, 0x05, 0x2d, 0x04, 0x35, 
	0x30, 0x46, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x36, 0x19, 
	0x35, 0x08, 0x32, 0x05, 0x04, 0x35, 0x0a, 0x00, 
	0x00, 0x3c, 0x25, 0x3d, 0x47, 0x4d, 0x02, 0x24, 
	0x19, 0x4f, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x44, 0x30, 
	0x35, 0x02, 0x46, 0x47, 0x30, 0x35, 0x31, 0x00, 
	0x00, 0x4d, 0x18, 0x4d, 0x04, 0x08, 0x3c, 0x2f, 
	0x3d, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x3d, 
	0x30, 0x47, 0x08, 0x07, 0x31, 0x02, 0x2f, 0x00, 
	0x00, 0x3c, 0x30, 0x02, 0x0d, 0x34, 0x24, 0x3d, 
	0x11, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x19, 
	0x3d, 0x49, 0x3f, 0x06, 0x11, 0x11, 0x2f, 0x00, 
	0x00, 0x4d, 0x49, 0x4d, 0x3d, 0x30, 0x31, 0x35, 
	0x02, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x49, 
	0x11, 0x31, 0x02, 0x31, 0x43, 0x35, 0x4d, 0x00, 
	0x00, 0x43, 0x49, 0x11, 0x36, 0x36, 0x02, 0x03, 
	0x10, 0x50, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x44, 0x35, 
	0x08, 0x25, 0x36, 0x44, 0x25, 0x24, 0x43, 0x00, 
	0x00, 0x36, 0x19, 0x41, 0x23, 0x35, 0x2c, 0x14, 
	0x03, 0x3c, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x3c, 0x1c, 
	0x14, 0x08, 0x03, 0x41, 0x03, 0x30, 0x45, 0x00, 
	0x00, 0x51, 0x31, 0x49, 0x35, 0x06, 0x2d, 0x2d, 
	0x0c, 0x52, 0x44, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x35, 0x37, 
	0x32, 0x0d, 0x05, 0x03, 0x25, 0x2f, 0x4e, 0x00, 
	0x00, 0x00, 0x43, 0x30, 0x24, 0x08, 0x32, 0x13, 
	0x06, 0x0c, 0x49, 0x48, 0x48, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x48, 0x49, 0x2c, 0x2d, 
	0x13, 0x13, 0x07, 0x35, 0x30, 0x47, 0x00, 0x00, 
	0x00, 0x00, 0x4f, 0x31, 0x49, 0x24, 0x06, 0x1e, 
	0x0d, 0x06, 0x06, 0x52, 0x44, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x48, 0x24, 0x39, 0x13, 0x1e, 
	0x1e, 0x32, 0x03, 0x49, 0x18, 0x4f, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x36, 0x53, 0x10, 0x03, 0x0d, 
	0x2e, 0x0d, 0x0d, 0x0d, 0x2d, 0x10, 0x47, 0x44, 
	0x48, 0x36, 0x24, 0x2c, 0x0d, 0x13, 0x0e, 0x0e, 
	0x0d, 0x08, 0x24, 0x3d, 0x43, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x4b, 0x3c, 0x30, 0x25, 0x23, 
	0x54, 0x3f, 0x2e, 0x0f, 0x1f, 0x12, 0x0e, 0x55, 
	0x55, 0x0e, 0x12, 0x12, 0x56, 0x1f, 0x57, 0x0d, 
	0x07, 0x41, 0x30, 0x3c, 0x4b, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x44, 0x3c, 0x30, 0x10, 
	0x2a, 0x0d, 0x58, 0x59, 0x17, 0x5a, 0x07, 0x49, 
	0x49, 0x05, 0x5b, 0x5c, 0x58, 0x58, 0x13, 0x04, 
	0x41, 0x30, 0x3c, 0x44, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x47, 0x53, 
	0x02, 0x35, 0x05, 0x0e, 0x5d, 0x3f, 0x4d, 0x07, 
	0x0c, 0x3c, 0x34, 0x28, 0x12, 0x05, 0x03, 0x10, 
	0x3d, 0x47, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x46, 
	0x18, 0x53, 0x49, 0x37, 0x06, 0x02, 0x49, 0x3f, 
	0x32, 0x49, 0x3d, 0x06, 0x5e, 0x24, 0x02, 0x31, 
	0x46, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4b, 
	0x45, 0x43, 0x18, 0x3d, 0x30, 0x30, 0x18, 0x30, 
	0x2f, 0x4d, 0x24, 0x02, 0x30, 0x31, 0x43, 0x4f, 
	0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x48, 0x4b, 0x44, 0x36, 0x36, 0x2f, 0x02, 0x36, 
	0x45, 0x30, 0x02, 0x36, 0x45, 0x4f, 0x44, 0x48, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x48, 0x48, 0x4b, 0x44, 0x31, 0x18, 
	0x4d, 0x2f, 0x48, 0x44, 0x50, 0x48, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x48, 0x48, 
	0x48, 0x48, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	
};

Gfx gooner_Flamethrower_Hole_pal_rgba16_aligner[] = {gsSPEndDisplayList()};
u8 gooner_Flamethrower_Hole_pal_rgba16[] = {
	0x10, 0x84, 0x21, 0x08, 0x42, 0x13, 0x5b, 0x1b, 
	0x63, 0x9f, 0x74, 0x23, 0x73, 0xe1, 0x6b, 0x9f, 
	0x63, 0x5d, 0x42, 0x54, 0x21, 0x4a, 0x31, 0xce, 
	0x6b, 0xa1, 0x84, 0x65, 0x94, 0xe9, 0x9d, 0x2d, 
	0x4a, 0x95, 0x42, 0x53, 0x94, 0xeb, 0x84, 0xa5, 
	0x6b, 0xe1, 0x31, 0x8e, 0x94, 0xa9, 0xbe, 0x75, 
	0x31, 0x8d, 0x3a, 0x13, 0xad, 0xaf, 0x73, 0x9f, 
	0x5a, 0xd9, 0x29, 0x8c, 0x8c, 0xa7, 0x95, 0x2b, 
	0xad, 0xf3, 0xbe, 0x77, 0xce, 0xfb, 0x52, 0xd7, 
	0x4a, 0x97, 0x4a, 0x55, 0x9d, 0x2b, 0xb5, 0xf3, 
	0xad, 0xf1, 0x73, 0xdf, 0x5b, 0x19, 0x3a, 0x10, 
	0x73, 0xe3, 0x7c, 0x25, 0x94, 0xa7, 0x31, 0xcf, 
	0x3a, 0x11, 0x31, 0x8f, 0x7c, 0x23, 0x39, 0xd0, 
	0x84, 0x23, 0x52, 0xd9, 0x21, 0x09, 0x63, 0x1b, 
	0x18, 0xc8, 0x7c, 0x65, 0x84, 0x21, 0x73, 0x5d, 
	0x29, 0x4d, 0x39, 0xd1, 0x6b, 0x5d, 0x8c, 0xa5, 
	0x62, 0xd9, 0x52, 0x97, 0x39, 0xcf, 0x21, 0x4b, 
	0x18, 0xc7, 0x19, 0x09, 0x21, 0x0b, 0x29, 0x4b, 
	0x10, 0x85, 0x42, 0x55, 0x29, 0x4c, 0x10, 0xc7, 
	0x63, 0x5f, 0x29, 0x8d, 0x10, 0xc6, 0x18, 0xc9, 
	0x10, 0x87, 0x10, 0x86, 0x53, 0x19, 0x31, 0xd1, 
	0x7c, 0x21, 0x8c, 0xa9, 0x95, 0x2d, 0x9d, 0x6d, 
	0xa5, 0x6f, 0xb6, 0x33, 0xb6, 0x35, 0xa5, 0xb1, 
	0xa5, 0x6d, 0xbe, 0x35, 0x5b, 0x1d, 
};

Vtx gooner_gooner_mesh_layer_1_vtx_0[40] = {
	{{ {-77, -43, 101}, 0, {38, 496}, {138, 46, 0, 255} }},
	{{ {-30, 80, 12}, 0, {343, -16}, {138, 46, 0, 255} }},
	{{ {-30, 80, -51}, 0, {649, -16}, {138, 46, 0, 255} }},
	{{ {-77, -43, -106}, 0, {954, 496}, {138, 46, 0, 255} }},
	{{ {-77, -43, -106}, 0, {138, 496}, {0, 52, 140, 255} }},
	{{ {-30, 80, -51}, 0, {334, -16}, {0, 52, 140, 255} }},
	{{ {30, 80, -51}, 0, {658, -16}, {0, 52, 140, 255} }},
	{{ {77, -43, -106}, 0, {854, 496}, {0, 52, 140, 255} }},
	{{ {77, -43, -106}, 0, {954, 496}, {118, 46, 0, 255} }},
	{{ {30, 80, -51}, 0, {649, -16}, {118, 46, 0, 255} }},
	{{ {30, 80, 12}, 0, {343, -16}, {118, 46, 0, 255} }},
	{{ {77, -43, 101}, 0, {38, 496}, {118, 46, 0, 255} }},
	{{ {77, -43, 101}, 0, {854, 496}, {0, 75, 103, 255} }},
	{{ {30, 80, 12}, 0, {658, -16}, {0, 75, 103, 255} }},
	{{ {-30, 80, 12}, 0, {334, -16}, {0, 75, 103, 255} }},
	{{ {-77, -43, 101}, 0, {138, 496}, {0, 75, 103, 255} }},
	{{ {-38, -128, -49}, 0, {334, 343}, {0, 129, 0, 255} }},
	{{ {38, -128, -49}, 0, {658, 343}, {0, 129, 0, 255} }},
	{{ {38, -128, 12}, 0, {658, 649}, {0, 129, 0, 255} }},
	{{ {-38, -128, 12}, 0, {334, 649}, {0, 129, 0, 255} }},
	{{ {30, 80, -51}, 0, {658, 343}, {0, 127, 0, 255} }},
	{{ {-30, 80, -51}, 0, {334, 343}, {0, 127, 0, 255} }},
	{{ {-30, 80, 12}, 0, {334, 649}, {0, 127, 0, 255} }},
	{{ {30, 80, 12}, 0, {658, 649}, {0, 127, 0, 255} }},
	{{ {38, -128, 12}, 0, {658, 1008}, {0, 164, 88, 255} }},
	{{ {77, -43, 101}, 0, {854, 496}, {0, 164, 88, 255} }},
	{{ {-77, -43, 101}, 0, {138, 496}, {0, 164, 88, 255} }},
	{{ {-38, -128, 12}, 0, {334, 1008}, {0, 164, 88, 255} }},
	{{ {38, -128, -49}, 0, {649, 1008}, {116, 203, 0, 255} }},
	{{ {77, -43, -106}, 0, {954, 496}, {116, 203, 0, 255} }},
	{{ {77, -43, 101}, 0, {38, 496}, {116, 203, 0, 255} }},
	{{ {38, -128, 12}, 0, {343, 1008}, {116, 203, 0, 255} }},
	{{ {-38, -128, -49}, 0, {334, 1008}, {0, 186, 150, 255} }},
	{{ {-77, -43, -106}, 0, {138, 496}, {0, 186, 150, 255} }},
	{{ {77, -43, -106}, 0, {854, 496}, {0, 186, 150, 255} }},
	{{ {38, -128, -49}, 0, {658, 1008}, {0, 186, 150, 255} }},
	{{ {-38, -128, 12}, 0, {343, 1008}, {140, 203, 0, 255} }},
	{{ {-77, -43, 101}, 0, {38, 496}, {140, 203, 0, 255} }},
	{{ {-77, -43, -106}, 0, {954, 496}, {140, 203, 0, 255} }},
	{{ {-38, -128, -49}, 0, {649, 1008}, {140, 203, 0, 255} }},
};

Gfx gooner_gooner_mesh_layer_1_tri_0[] = {
	gsSPVertex(gooner_gooner_mesh_layer_1_vtx_0 + 0, 32, 0),
	gsSP2Triangles(0, 1, 2, 0, 0, 2, 3, 0),
	gsSP2Triangles(4, 5, 6, 0, 4, 6, 7, 0),
	gsSP2Triangles(8, 9, 10, 0, 8, 10, 11, 0),
	gsSP2Triangles(12, 13, 14, 0, 12, 14, 15, 0),
	gsSP2Triangles(16, 17, 18, 0, 16, 18, 19, 0),
	gsSP2Triangles(20, 21, 22, 0, 20, 22, 23, 0),
	gsSP2Triangles(24, 25, 26, 0, 24, 26, 27, 0),
	gsSP2Triangles(28, 29, 30, 0, 28, 30, 31, 0),
	gsSPVertex(gooner_gooner_mesh_layer_1_vtx_0 + 32, 8, 0),
	gsSP2Triangles(0, 1, 2, 0, 0, 2, 3, 0),
	gsSP2Triangles(4, 5, 6, 0, 4, 6, 7, 0),
	gsSPEndDisplayList(),
};

Vtx gooner_gooner_mesh_layer_4_vtx_0[16] = {
	{{ {-15, -18, 91}, 0, {265, 1247}, {0, 76, 102, 255} }},
	{{ {-37, 7, 72}, 0, {-108, 746}, {0, 76, 102, 255} }},
	{{ {-44, -18, 91}, 0, {-215, 1247}, {0, 76, 102, 255} }},
	{{ {-12, 7, 72}, 0, {301, 746}, {0, 76, 102, 255} }},
	{{ {15, -18, 91}, 0, {745, 1247}, {0, 76, 102, 255} }},
	{{ {12, 7, 72}, 0, {709, 746}, {0, 76, 102, 255} }},
	{{ {44, -18, 91}, 0, {1225, 1247}, {0, 76, 102, 255} }},
	{{ {37, 7, 72}, 0, {1118, 746}, {0, 76, 102, 255} }},
	{{ {10, 31, 54}, 0, {673, 246}, {0, 76, 102, 255} }},
	{{ {31, 31, 54}, 0, {1010, 246}, {0, 76, 102, 255} }},
	{{ {8, 56, 36}, 0, {638, -255}, {0, 76, 102, 255} }},
	{{ {24, 56, 36}, 0, {903, -255}, {0, 76, 102, 255} }},
	{{ {-8, 56, 36}, 0, {372, -255}, {0, 76, 102, 255} }},
	{{ {-10, 31, 54}, 0, {337, 246}, {0, 76, 102, 255} }},
	{{ {-24, 56, 36}, 0, {107, -255}, {0, 76, 102, 255} }},
	{{ {-31, 31, 54}, 0, {0, 246}, {0, 76, 102, 255} }},
};

Gfx gooner_gooner_mesh_layer_4_tri_0[] = {
	gsSPVertex(gooner_gooner_mesh_layer_4_vtx_0 + 0, 16, 0),
	gsSP2Triangles(0, 1, 2, 0, 0, 3, 1, 0),
	gsSP2Triangles(4, 3, 0, 0, 4, 5, 3, 0),
	gsSP2Triangles(6, 5, 4, 0, 6, 7, 5, 0),
	gsSP2Triangles(7, 8, 5, 0, 7, 9, 8, 0),
	gsSP2Triangles(9, 10, 8, 0, 9, 11, 10, 0),
	gsSP2Triangles(8, 10, 12, 0, 8, 12, 13, 0),
	gsSP2Triangles(13, 12, 14, 0, 13, 14, 15, 0),
	gsSP2Triangles(3, 13, 15, 0, 3, 15, 1, 0),
	gsSP2Triangles(5, 13, 3, 0, 5, 8, 13, 0),
	gsSPEndDisplayList(),
};


Gfx mat_gooner_flamethrower_base[] = {
	gsDPPipeSync(),
	gsDPSetCombineLERP(TEXEL0, 0, SHADE, 0, 0, 0, 0, ENVIRONMENT, TEXEL0, 0, SHADE, 0, 0, 0, 0, ENVIRONMENT),
	gsDPSetTextureLUT(G_TT_RGBA16),
	gsSPTexture(65535, 65535, 0, 0, 1),
	gsSPSetLights1(gooner_flamethrower_base_lights),
	gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, gooner_Flamethrower_Rock_pal_rgba16),
	gsDPSetTile(0, 0, 0, 256, 5, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadTLUTCmd(5, 149),
	gsDPSetTextureImage(G_IM_FMT_CI, G_IM_SIZ_8b_LOAD_BLOCK, 1, gooner_Flamethrower_Rock_ci8),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b_LOAD_BLOCK, 0, 0, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadBlock(7, 0, 0, 511, 512),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 5, 0, G_TX_WRAP | G_TX_NOMIRROR, 5, 0),
	gsDPSetTileSize(0, 0, 0, 124, 124),
	gsSPEndDisplayList(),
};

Gfx mat_revert_gooner_flamethrower_base[] = {
	gsDPPipeSync(),
	gsDPSetTextureLUT(G_TT_NONE),
	gsSPEndDisplayList(),
};

Gfx mat_gooner_flamethrower_hole[] = {
	gsDPPipeSync(),
	gsDPSetCombineLERP(TEXEL0, 0, SHADE, 0, TEXEL0, 0, ENVIRONMENT, 0, TEXEL0, 0, SHADE, 0, TEXEL0, 0, ENVIRONMENT, 0),
	gsSPGeometryMode(G_CULL_BACK, 0),
	gsDPSetTextureLUT(G_TT_RGBA16),
	gsSPTexture(65535, 65535, 0, 0, 1),
	gsSPSetLights1(gooner_flamethrower_hole_lights),
	gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, gooner_Flamethrower_Hole_pal_rgba16),
	gsDPSetTile(0, 0, 0, 256, 5, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadTLUTCmd(5, 94),
	gsDPSetTextureImage(G_IM_FMT_CI, G_IM_SIZ_8b_LOAD_BLOCK, 1, gooner_Flamethrower_Hole_ci8),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b_LOAD_BLOCK, 0, 0, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadBlock(7, 0, 0, 511, 512),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 0, 0, G_TX_CLAMP | G_TX_NOMIRROR, 5, 0, G_TX_CLAMP | G_TX_NOMIRROR, 5, 0),
	gsDPSetTileSize(0, 0, 0, 124, 124),
	gsSPEndDisplayList(),
};

Gfx mat_revert_gooner_flamethrower_hole[] = {
	gsDPPipeSync(),
	gsSPGeometryMode(0, G_CULL_BACK),
	gsDPSetTextureLUT(G_TT_NONE),
	gsSPEndDisplayList(),
};

Gfx gooner_gooner_mesh_layer_1[] = {
	gsSPDisplayList(mat_gooner_flamethrower_base),
	gsSPDisplayList(gooner_gooner_mesh_layer_1_tri_0),
	gsSPDisplayList(mat_revert_gooner_flamethrower_base),
	gsSPEndDisplayList(),
};

Gfx gooner_gooner_mesh_layer_4[] = {
	gsSPDisplayList(mat_gooner_flamethrower_hole),
	gsSPDisplayList(gooner_gooner_mesh_layer_4_tri_0),
	gsSPDisplayList(mat_revert_gooner_flamethrower_hole),
	gsSPEndDisplayList(),
};

Gfx gooner_material_revert_render_settings[] = {
	gsDPPipeSync(),
	gsSPSetGeometryMode(G_LIGHTING),
	gsSPClearGeometryMode(G_TEXTURE_GEN),
	gsDPSetCombineLERP(0, 0, 0, SHADE, 0, 0, 0, ENVIRONMENT, 0, 0, 0, SHADE, 0, 0, 0, ENVIRONMENT),
	gsSPTexture(65535, 65535, 0, 0, 0),
	gsDPSetEnvColor(255, 255, 255, 255),
	gsDPSetAlphaCompare(G_AC_NONE),
	gsSPEndDisplayList(),
};

