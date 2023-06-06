Lights1 secret_books_books_lights = gdSPDefLights1(
	0x7F, 0x7F, 0x7F,
	0xFE, 0xFE, 0xFE, 0x28, 0x28, 0x28);

Lights1 secret_books_bookwall_lights = gdSPDefLights1(
	0x7F, 0x7F, 0x7F,
	0xFE, 0xFE, 0xFE, 0x28, 0x28, 0x28);

Gfx secret_books_Books_ci8_aligner[] = {gsSPEndDisplayList()};
u8 secret_books_Books_ci8[] = {
	0x0, 0x1, 0x0, 0x2, 0x2, 0x2, 0x2, 0x3, 0x2, 
	0x3, 0x3, 0x3, 0x3, 0x2, 0x2, 0x3, 0x4, 0x5, 
	0x5, 0x5, 0x5, 0x5, 0x5, 0x6, 0x3, 0x2, 0x2, 
	0x3, 0x3, 0x3, 0x3, 0x1, 0x0, 0x1, 0x0, 0x2, 
	0x2, 0x2, 0x3, 0x2, 0x2, 0x3, 0x3, 0x3, 0x3, 
	0x2, 0x2, 0x2, 0x4, 0x5, 0x5, 0x5, 0x5, 0x5, 
	0x4, 0x6, 0x3, 0x2, 0x2, 0x3, 0x3, 0x3, 0x2, 
	0x1, 0x0, 0x1, 0x0, 0x2, 0x2, 0x2, 0x2, 0x2, 
	0x3, 0x3, 0x3, 0x3, 0x2, 0x2, 0x2, 0x2, 0x4, 
	0x4, 0x4, 0x5, 0x5, 0x5, 0x5, 0x4, 0x3, 0x3, 
	0x2, 0x2, 0x3, 0x3, 0x2, 0x1, 0x0, 0x7, 0x7, 
	0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x7, 0x8, 0x7, 0x8, 0x8, 0x8, 0x7, 
	0x7, 0x7, 0x7, 0x8, 0x8, 0x8, 0x8, 0x7, 0x7, 
	0x7, 0x1, 0x6, 0x9, 0x9, 0x9, 0xA, 0xA, 0xB, 
	0xB, 0xC, 0xC, 0xC, 0xC, 0xD, 0xD, 0xD, 0xD, 
	0xE, 0xE, 0xE, 0xE, 0xF, 0xF, 0xF, 0xF, 0xD, 
	0xD, 0xD, 0xD, 0xC, 0xC, 0xC, 0x10, 0x6, 0x9, 
	0x11, 0x12, 0x13, 0xA, 0x14, 0x14, 0x15, 0xC, 0x16, 
	0x17, 0xD, 0xD, 0x18, 0x19, 0x1A, 0xE, 0x1B, 0xE, 
	0xF, 0xF, 0x1C, 0x1D, 0xD, 0xD, 0x1B, 0x1E, 0x1F, 
	0xC, 0xC, 0x10, 0x6, 0x9, 0x11, 0x12, 0x13, 0xA, 
	0x14, 0xB, 0x15, 0xC, 0x16, 0x17, 0xD, 0xD, 0x18, 
	0x19, 0x1A, 0xE, 0x1B, 0x1E, 0xF, 0xF, 0x1C, 0x20, 
	0xD, 0xD, 0x1B, 0x1E, 0x21, 0xC, 0xC, 0x10, 0x6, 
	0x9, 0x11, 0x12, 0x13, 0xA, 0x14, 0x14, 0x15, 0xC, 
	0x16, 0x17, 0xD, 0xD, 0x18, 0x22, 0x1A, 0xE, 0x1B, 
	0x1B, 0xF, 0xF, 0x1C, 0x20, 0xD, 0xD, 0x1B, 0x1E, 
	0x21, 0xC, 0xC, 0x10, 0x6, 0x23, 0x24, 0x13, 0x25, 
	0x25, 0x26, 0x27, 0x28, 0x28, 0x29, 0x2A, 0x2B, 0x2B, 
	0x2C, 0x2D, 0x1A, 0xE, 0x1B, 0x1B, 0x2E, 0x2E, 0x2F, 
	0x2F, 0x2A, 0x28, 0x29, 0x2A, 0x30, 0x31, 0x31, 0x32, 
	0x6, 0x23, 0x13, 0x13, 0x25, 0x25, 0x25, 0x25, 0x28, 
	0x28, 0x2A, 0x2A, 0x2B, 0x2B, 0x2D, 0x2D, 0x1A, 0xE, 
	0xE, 0xE, 0x2E, 0x2E, 0x33, 0x2E, 0x28, 0x28, 0x2A, 
	0x2A, 0x31, 0x31, 0x31, 0x32, 0x6, 0x6, 0x6, 0x6, 
	0x2, 0x2, 0x2, 0x2, 0x34, 0x34, 0x34, 0x34, 0x6, 
	0x6, 0x6, 0x6, 0x35, 0x35, 0x35, 0x35, 0x3, 0x3, 
	0x3, 0x3, 0x34, 0x34, 0x34, 0x34, 0x0, 0x0, 0x0, 
	0x32, 0x6, 0x6, 0x36, 0x36, 0x2, 0x2, 0x2, 0x2, 
	0x34, 0x34, 0x34, 0x34, 0x6, 0x6, 0x6, 0x6, 0x37, 
	0x35, 0x35, 0x35, 0x3, 0x3, 0x3, 0x3, 0x34, 0x34, 
	0x34, 0x34, 0x0, 0x38, 0x10, 0x32, 0x6, 0x2B, 0x2B, 
	0x2B, 0xE, 0xE, 0xE, 0xE, 0xA, 0xA, 0xA, 0xA, 
	0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x28, 
	0x28, 0x28, 0x28, 0x3A, 0x3A, 0x3B, 0x3A, 0xC, 0xC, 
	0xC, 0x3C, 0x6, 0x2B, 0x2B, 0x2B, 0xE, 0xE, 0xE, 
	0xE, 0xA, 0xA, 0x3D, 0x3D, 0x39, 0x39, 0x3B, 0x3B, 
	0x39, 0x39, 0x3E, 0x3E, 0x28, 0x28, 0x28, 0x28, 0x3A, 
	0x3A, 0x3B, 0x3A, 0xC, 0xC, 0xC, 0x10, 0x6, 0x2B, 
	0x2C, 0x2D, 0x1A, 0xE, 0x1B, 0x1E, 0x3D, 0xA, 0x3F, 
	0x40, 0x3B, 0x39, 0x41, 0x3E, 0x3E, 0x39, 0x42, 0x3E, 
	0x28, 0x28, 0x29, 0x43, 0x3A, 0x3A, 0x41, 0x44, 0x1F, 
	0xC, 0xC, 0x10, 0x6, 0x2B, 0x2C, 0x2C, 0x1A, 0xE, 
	0x1B, 0x1E, 0x3D, 0xA, 0x40, 0x40, 0x3B, 0x39, 0x41, 
	0x3E, 0x3E, 0x39, 0x42, 0x45, 0x28, 0x28, 0x29, 0x2A, 
	0x3B, 0x3A, 0x41, 0x44, 0x1F, 0xC, 0xC, 0x10, 0x6, 
	0x2B, 0x2C, 0x2D, 0x46, 0x47, 0x48, 0x48, 0x21, 0xA, 
	0x49, 0x49, 0x21, 0xA, 0x49, 0x21, 0x17, 0xC, 0x18, 
	0x22, 0xA, 0xA, 0x49, 0x21, 0x2A, 0x28, 0x29, 0x2A, 
	0x23, 0x23, 0x23, 0x4A, 0x6, 0x2B, 0x2C, 0x2D, 0x46, 
	0x47, 0x48, 0x48, 0x21, 0xA, 0x49, 0x21, 0x21, 0xA, 
	0x49, 0x49, 0x17, 0xC, 0x18, 0x18, 0x49, 0xA, 0x49, 
	0x21, 0x28, 0x28, 0x29, 0x2A, 0x23, 0x23, 0x23, 0x4A, 
	0x6, 0x2B, 0x2D, 0x2D, 0x47, 0x47, 0x46, 0x46, 0x21, 
	0xA, 0x21, 0x21, 0x21, 0xA, 0x49, 0x49, 0xC, 0xC, 
	0x18, 0x18, 0x49, 0xA, 0x21, 0x21, 0x28, 0x28, 0x2A, 
	0x2A, 0x23, 0x23, 0x23, 0x4A, 0x6, 0x6, 0x6, 0x6, 
	0x37, 0x37, 0x37, 0x37, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 
	0x4B, 0x4B, 0x4B, 0x4C, 0x4C, 0x4C, 0x4C, 0x4B, 0x4B, 
	0x4B, 0x4B, 0x3, 0x3, 0x3, 0x3, 0x6, 0x6, 0x6, 
	0x4A, 0x6, 0x6, 0x6, 0x6, 0x1, 0x1, 0x1, 0x1, 
	0x4D, 0x4D, 0x4D, 0x4D, 0x6, 0x6, 0x6, 0x6, 0x3, 
	0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x6, 0x6, 
	0x6, 0x6, 0x4D, 0x4D, 0x4D, 0x32, 0x6, 0x23, 0x23, 
	0x23, 0x4E, 0x4E, 0x4E, 0x4E, 0xC, 0xC, 0xC, 0xC, 
	0x2B, 0x2B, 0x2B, 0x2B, 0x25, 0x25, 0x25, 0x25, 0x2E, 
	0x2E, 0x2E, 0x2E, 0x9, 0x9, 0x9, 0x9, 0x31, 0x31, 
	0x31, 0x32, 0x6, 0x23, 0x23, 0x23, 0x4E, 0x4E, 0x4E, 
	0x4F, 0x1F, 0x1F, 0xC, 0x1F, 0x2B, 0x2B, 0x2B, 0x2D, 
	0x25, 0x25, 0x25, 0x29, 0x2E, 0x2E, 0x2E, 0x2E, 0x9, 
	0x9, 0x9, 0x12, 0x50, 0x31, 0x31, 0x32, 0x6, 0x23, 
	0x23, 0x14, 0x51, 0x52, 0x4E, 0x4F, 0x1F, 0x1F, 0xC, 
	0x1F, 0x24, 0x2B, 0x2B, 0x2C, 0x27, 0x27, 0x25, 0x29, 
	0x53, 0x2E, 0x2E, 0x54, 0x55, 0x9, 0x9, 0x11, 0x56, 
	0x50, 0x31, 0x4A, 0x3, 0x57, 0x57, 0x58, 0x1A, 0x1A, 
	0xE, 0x1C, 0x59, 0x5A, 0x5A, 0x5B, 0x48, 0x48, 0x46, 
	0x5C, 0x22, 0x19, 0xD, 0x18, 0x20, 0xF, 0xF, 0x1C, 
	0x53, 0x2E, 0x2E, 0x43, 0x5D, 0x5E, 0x47, 0x5F, 0x3, 
	0x57, 0x57, 0x58, 0x20, 0x1A, 0xE, 0x1C, 0x59, 0x5A, 
	0x5A, 0x5B, 0x48, 0x48, 0x46, 0x5C, 0x19, 0x19, 0xD, 
	0x18, 0x1D, 0x1D, 0xF, 0x1C, 0x53, 0x2E, 0x2E, 0x43, 
	0x5D, 0x5E, 0x47, 0x5F, 0x3, 0x57, 0x57, 0x58, 0x1A, 
	0x1A, 0xE, 0x1C, 0x59, 0x5A, 0x5A, 0x5B, 0x48, 0x48, 
	0x60, 0x5C, 0x22, 0x61, 0x62, 0x18, 0x1C, 0x1D, 0xF, 
	0x1C, 0x54, 0x2E, 0x2E, 0x43, 0x63, 0x5E, 0x47, 0x5F, 
	0x3, 0x57, 0x57, 0x64, 0x1C, 0x1A, 0xE, 0x1C, 0x5B, 
	0x65, 0x5A, 0x65, 0x48, 0x60, 0x60, 0x66, 0x67, 0x62, 
	0x62, 0x18, 0x1C, 0xF, 0xF, 0x1C, 0x54, 0x2E, 0x2E, 
	0x2E, 0x5D, 0x5E, 0x47, 0x5F, 0x2, 0x4D, 0x4D, 0x4D, 
	0x4D, 0x4D, 0x4D, 0x4D, 0x34, 0x34, 0x34, 0x34, 0x2, 
	0x2, 0x2, 0x2, 0x0, 0x35, 0x35, 0x35, 0x35, 0x35, 
	0x35, 0x0, 0x0, 0x68, 0x0, 0x0, 0x0, 0x0, 0x2, 
	0x1, 0x2, 0x4D, 0x38, 0x38, 0x37, 0x37, 0x37, 0x37, 
	0x34, 0x34, 0x69, 0x69, 0x2, 0x2, 0x2, 0x2, 0x6A, 
	0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6, 0x6, 
	0x6, 0x6, 0x2, 0x2, 0x1, 0x1, 0x2, 0x31, 0x31, 
	0x31, 0x6B, 0x6B, 0x6B, 0x6B, 0x28, 0x28, 0x28, 0x28, 
	0xC, 0xC, 0x1F, 0xC, 0x6C, 0x6C, 0x6D, 0x6E, 0x6F, 
	0x6F, 0x6F, 0x6F, 0x23, 0x23, 0x23, 0x23, 0x70, 0x70, 
	0x70, 0x1, 0x2, 0x31, 0x56, 0x56, 0x6B, 0x6B, 0x50, 
	0x50, 0x28, 0x28, 0x2A, 0x2A, 0xC, 0xC, 0x1F, 0x1F, 
	0x6F, 0x6F, 0x6E, 0x6E, 0x6E, 0x6F, 0x6E, 0x6F, 0x23, 
	0x23, 0x23, 0x23, 0x70, 0x71, 0x70, 0x1, 0x6, 0x23, 
	0x24, 0x13, 0x56, 0x31, 0x72, 0x56, 0xF, 0xF, 0x1C, 
	0x1D, 0x40, 0x31, 0x40, 0x40, 0x13, 0xA, 0x14, 0xB, 
	0x15, 0xC, 0x16, 0x17, 0x9, 0x9, 0x11, 0x12, 0x50, 
	0x56, 0x31, 0x32, 0x6, 0x23, 0x24, 0x13, 0x56, 0x31, 
	0x73, 0x56, 0xF, 0xF, 0x1C, 0x20, 0x40, 0x31, 0x40, 
	0x40, 0x13, 0xA, 0x14, 0x14, 0x15, 0xC, 0x16, 0xC, 
	0x9, 0x9, 0x11, 0x12, 0x50, 0x56, 0x31, 0x32, 0x6, 
	0x23, 0x24, 0x13, 0x56, 0x31, 0x73, 0x56, 0xF, 0xF, 
	0x1C, 0x1C, 0x40, 0x31, 0x40, 0x40, 0x13, 0xA, 0x14, 
	0x14, 0x15, 0xC, 0x16, 0x17, 0x9, 0x9, 0x11, 0x12, 
	0x56, 0x56, 0x31, 0x32, 0x6, 0x23, 0x24, 0x13, 0x56, 
	0x31, 0x73, 0x73, 0xF, 0xF, 0x1C, 0x1C, 0x40, 0x31, 
	0x74, 0x40, 0x13, 0xA, 0x14, 0x14, 0x15, 0xC, 0x16, 
	0x16, 0x9, 0x9, 0x11, 0x12, 0x56, 0x31, 0x31, 0x32, 
	0x4B, 0x70, 0x75, 0x75, 0x70, 0x70, 0x71, 0x71, 0x28, 
	0x28, 0x29, 0x29, 0x30, 0x31, 0x72, 0x72, 0x21, 0xA, 
	0x49, 0x49, 0x21, 0xA, 0x21, 0x21, 0x76, 0x76, 0x72, 
	0x72, 0x31, 0x31, 0x31, 0x32, 0x77, 0x4B, 0x4B, 0x37, 
	0x2, 0x2, 0x0, 0x0, 0x3, 0x3, 0x3, 0x3, 0x0, 
	0x0, 0x0, 0x0, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 
	0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x0, 0x38, 0x38, 
	0x32, 0x77, 0x4B, 0x37, 0x37, 0x2, 0x1, 0x2, 0x2, 
	0x3, 0x3, 0x3, 0x3, 0x38, 0x38, 0x38, 0x0, 0x4B, 
	0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 
	0x4B, 0x4B, 0x0, 0x38, 0x38, 0x32, 0x77, 0x70, 0x70, 
	0x70, 0x70, 0x70, 0x70, 0x70, 0x28, 0x28, 0x28, 0x28, 
	0x31, 0x31, 0x31, 0x31, 0xA, 0xA, 0xA, 0x21, 0xA, 
	0xA, 0xA, 0x21, 0x76, 0x76, 0x70, 0x76, 0x31, 0x31, 
	0x31, 0x32, 0x0, 0x57, 0x57, 0x57, 0x78, 0x47, 0x47, 
	0x47, 0x79, 0x47, 0x47, 0x79, 0x79, 0x47, 0x47, 0x79, 
	0x78, 0x47, 0x47, 0x78, 0x7A, 0x7A, 0x7A, 0x7A, 0x21, 
	0xA, 0xA, 0x21, 0x5E, 0x47, 0x47, 0x10, 0x0, 0x57, 
	0x57, 0x58, 0x7B, 0x78, 0x47, 0x54, 0x7C, 0x79, 0x47, 
	0x54, 0x54, 0x79, 0x47, 0x54, 0x7B, 0x78, 0x47, 0x54, 
	0x7D, 0x7D, 0x7A, 0x7E, 0x21, 0x21, 0xA, 0x7E, 0x5D, 
	0x5E, 0x47, 0x10, 0x0, 0x57, 0x57, 0x58, 0x78, 0x78, 
	0x47, 0x54, 0x54, 0x7C, 0x47, 0x54, 0x54, 0x7C, 0x47, 
	0x54, 0x7B, 0x7B, 0x47, 0x54, 0x7D, 0x7D, 0x7A, 0x7E, 
	0x21, 0x21, 0xA, 0x7E, 0x5E, 0x5E, 0x5D, 0x10, 0x0, 
	0x57, 0x57, 0x58, 0x78, 0x78, 0x47, 0x54, 0x54, 0x7C, 
	0x47, 0x54, 0x54, 0x79, 0x47, 0x54, 0x7B, 0x7B, 0x47, 
	0x54, 0x7D, 0x7D, 0x7A, 0x7E, 0x21, 0x21, 0xA, 0x7E, 
	0x5D, 0x5E, 0x5D, 0x10, 0x6, 0x2B, 0x2B, 0x2C, 0x27, 
	0x25, 0x25, 0x26, 0x49, 0x49, 0xA, 0x49, 0x7D, 0x7D, 
	0x7A, 0x7F, 0x80, 0x80, 0x81, 0x49, 0x2F, 0x2F, 0x2E, 
	0x82, 0x29, 0x2A, 0x28, 0x43, 0x50, 0x56, 0x31, 0x83, 
	0x6, 0x2B, 0x2B, 0x2D, 0x25, 0x25, 0x25, 0x25, 0x49, 
	0x21, 0xA, 0x49, 0x7D, 0x7A, 0x7A, 0x7F, 0x49, 0x81, 
	0x81, 0x49, 0x2F, 0x2E, 0x2E, 0x33, 0x29, 0x28, 0x28, 
	0x29, 0x56, 0x50, 0x31, 0x83, 0x6, 0x6, 0x6, 0x6, 
	0x4D, 0x4D, 0x4D, 0x4D, 0x4B, 0x4B, 0x4B, 0x4B, 0x2, 
	0x2, 0x2, 0x2, 0x4B, 0x4B, 0x4B, 0x4B, 0x6A, 0x6A, 
	0x6A, 0x6A, 0x34, 0x34, 0x34, 0x34, 0x4D, 0x4D, 0x4D, 
	0x83, 0x6, 0x6, 0x6, 0x6, 0x4D, 0x4D, 0x4D, 0x4D, 
	0x4B, 0x4B, 0x4B, 0x4B, 0x2, 0x2, 0x2, 0x2, 0x4B, 
	0x4B, 0x4B, 0x4B, 0x6A, 0x6A, 0x6A, 0x6A, 0x34, 0x34, 
	0x34, 0x34, 0x4D, 0x4D, 0x4D, 0x83, 0x0, 0x31, 0x31, 
	0x31, 0x47, 0x47, 0x47, 0x79, 0x57, 0x57, 0x57, 0x57, 
	0x3A, 0x3A, 0x3A, 0x3B, 0x28, 0x28, 0x28, 0x28, 0x13, 
	0x23, 0x23, 0x13, 0xE, 0xE, 0xE, 0xE, 0x7A, 0x7A, 
	0x7A, 0x32, 0x38, 0x31, 0x31, 0x30, 0x47, 0x47, 0x47, 
	0x7C, 0x84, 0x84, 0x57, 0x84, 0x3B, 0x3A, 0x3A, 0x3B, 
	0x28, 0x28, 0x28, 0x28, 0x13, 0x13, 0x23, 0x13, 0x1A, 
	0x1A, 0xE, 0x1A, 0x7A, 0x7A, 0x7A, 0x32, 0x0, 0x31, 
	0x31, 0x30, 0x79, 0x47, 0x47, 0x54, 0x84, 0x84, 0x57, 
	0x84, 0x44, 0x3B, 0x3A, 0x41, 0x2C, 0x28, 0x28, 0x24, 
	0x13, 0x13, 0x23, 0x14, 0x1A, 0x1A, 0xE, 0x1C, 0x7D, 
	0x7A, 0x7A, 0x32, 0x0, 0x31, 0x31, 0x72, 0x79, 0x79, 
	0x47, 0x54, 0x58, 0x84, 0x57, 0x85, 0x44, 0x3B, 0x3A, 
	0x41, 0x2C, 0x28, 0x28, 0x24, 0x24, 0x13, 0x23, 0x14, 
	0x20, 0x1A, 0xE, 0x1C, 0x86, 0x7D, 0x7A, 0x32, 0x0, 
	0x70, 0x70, 0x7E, 0x21, 0x21, 0xA, 0x7F, 0x80, 0x80, 
	0x81, 0x49, 0x2A, 0x2A, 0x28, 0x29, 0x80, 0x81, 0x81, 
	0x87, 0x75, 0x75, 0x70, 0x34, 0x2C, 0x2D, 0x2B, 0x2C, 
	0x2F, 0x2E, 0x2E, 0x88, 0x0, 0x70, 0x70, 0x7E, 0x21, 
	0x21, 0xA, 0x7F, 0x80, 0x80, 0x81, 0x49, 0x2A, 0x2A, 
	0x28, 0x29, 0x80, 0x81, 0x81, 0x87, 0x87, 0x75, 0x70, 
	0x34, 0x2D, 0x2D, 0x2B, 0x2C, 0x2F, 0x2E, 0x2E, 0x88, 
	0x0, 0x70, 0x70, 0x71, 0x7E, 0x21, 0xA, 0x7F, 0x49, 
	0x81, 0x81, 0x49, 0x2A, 0x28, 0x28, 0x29, 0x80, 0x81, 
	0x81, 0x49, 0x34, 0x70, 0x70, 0x34, 0x2D, 0x2B, 0x2B, 
	0x24, 0x2F, 0x2E, 0x2E, 0x88, 0x0, 0x2, 0x2, 0x2, 
	0x2, 0x2, 0x2, 0x2, 0x4B, 0x4B, 0x4B, 0x4B, 0x3, 
	0x3, 0x3, 0x3, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 
	0x4B, 0x37, 0x6, 0x6, 0x6, 0x6, 0x0, 0x0, 0x0, 
	0x88, 0x4D, 0x1, 0x88, 0x88, 0x89, 0x1, 0x1, 0x37, 
	0x2, 0x4D, 0x4D, 0x2, 0x1, 0x1, 0x1, 0x1, 0x4D, 
	0x0, 0x0, 0x6, 0x3, 0x3, 0x4D, 0x4D, 0x2, 0x38, 
	0x38, 0x0, 0x6, 0x0, 0x0, 0x38, 0x4D, 0x1, 0x88, 
	0x88, 0x89, 0x1, 0x89, 0x1, 0x38, 0x4D, 0x3, 0x4D, 
	0x1, 0x1, 0x88, 0x1, 0x4D, 0x0, 0x0, 0x6, 0x3, 
	0x3, 0x4D, 0x4D, 0x2, 0x2, 0x2, 0x0, 0x4D, 0x6, 
	0x0, 0x4D, 0x4D, 0x4D, 0x88, 0x88, 0x83, 0x37, 0x89, 
	0x89, 0x38, 0x2, 0x4D, 0x2, 0x88, 0x88, 0x88, 0x88, 
	0x38, 0x0, 0x6, 0x6, 0x4D, 0x4D, 0x2, 0x4D, 0x38, 
	0x38, 0x2, 0x0, 0x38, 0x38, 0x0, 0x38, 0x4D, 0x1, 
	0x88, 0x88, 0x89, 0x60, 0x89, 0x60, 0x38, 0x4D, 0x3, 
	0x4D, 0x1, 0x1, 0x88, 0x1, 0x4D, 0x0, 0x0, 0x6, 
	0x3, 0x3, 0x4D, 0x4D, 0x2, 0x2, 0x2, 0x6A, 0x4D, 
	0x6, 0x0, 0x4D, 0x4D, 0x1, 0x88, 0x88, 0x89, 0x60, 
	0x60, 0x37, 0x37, 0x4D, 0x4D, 0x37, 0x1, 0x1, 0x1, 
	0x1, 0x4D, 0x0, 0x0, 0x6, 0x3, 0x3, 0x4D, 0x4D, 
	0x2, 0x38, 0x38, 0x6A, 0x6, 0x0, 0x0, 0x38, 0x0, 
	0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x4B, 0x4B, 
	0x4B, 0x4B, 0x34, 0x34, 0x34, 0x34, 0x4B, 0x4B, 0x4B, 
	0x4B, 0x4B, 0x4B, 0x4B, 0x37, 0x6, 0x6, 0x6, 0x6, 
	0x6A, 0x6A, 0x6A, 0x88, 0x0, 0x2, 0x2, 0x2, 0x2, 
	0x2, 0x2, 0x2, 0x4B, 0x4B, 0x4B, 0x4B, 0x3, 0x3, 
	0x3, 0x3, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 
	0x37, 0x6, 0x6, 0x6, 0x6, 0x0, 0x0, 0x0, 0x88, 
	0x4D, 0x1, 0x88, 0x88, 0x89, 0x1, 0x1, 0x37, 0x2, 
	0x4D, 0x4D, 0x2, 0x1, 0x1, 0x1, 0x1, 0x4D, 0x0, 
	0x0, 0x6, 0x3, 0x3, 0x4D, 0x4D, 0x2, 0x38, 0x38, 
	0x0, 0x6, 0x0, 0x0, 0x38, 
};

Gfx secret_books_Books_ci8_pal_rgba16_aligner[] = {gsSPEndDisplayList()};
u8 secret_books_Books_ci8_pal_rgba16[] = {
	0x52, 0xEB, 0x31, 0xDD, 0x3A, 0x23, 0x4A, 0xAB, 0x52, 
	0xED, 0x6B, 0xAD, 0x42, 0x67, 0x8, 0x89, 0x10, 0x89, 
	0x21, 0x93, 0x10, 0xD3, 0x13, 0x8D, 0x19, 0x11, 0x29, 
	0x11, 0x28, 0xD9, 0x28, 0xD5, 0x21, 0x55, 0x5B, 0x2F, 
	0x22, 0x25, 0x19, 0x65, 0x45, 0x1D, 0x13, 0xCF, 0x82, 
	0x9D, 0x79, 0xD, 0x4D, 0x1F, 0xD, 0x11, 0xB, 0x4D, 
	0x6A, 0x65, 0x6C, 0x1B, 0x53, 0x8F, 0xB3, 0xEF, 0x38, 
	0xD7, 0xA5, 0xE5, 0x41, 0x1D, 0x76, 0x67, 0x10, 0xCF, 
	0x63, 0x2F, 0x19, 0xDB, 0x3D, 0x9B, 0x1D, 0x11, 0x11, 
	0xD3, 0x72, 0xDF, 0x59, 0x51, 0x30, 0xD1, 0x55, 0x9D, 
	0x1C, 0x53, 0x29, 0x8F, 0x5B, 0xD1, 0x31, 0xA1, 0x18, 
	0xD7, 0x21, 0x57, 0x39, 0xE5, 0x4A, 0xA9, 0x73, 0xEF, 
	0x21, 0x11, 0x3A, 0x21, 0x31, 0xDF, 0x11, 0x13, 0x20, 
	0xD3, 0x39, 0x91, 0x10, 0xCD, 0x31, 0x17, 0x22, 0xCD, 
	0x41, 0x5D, 0x72, 0x21, 0x72, 0xD9, 0x45, 0x5B, 0x6B, 
	0x71, 0x89, 0xD, 0xBE, 0xF5, 0x31, 0x9D, 0x19, 0x15, 
	0x7A, 0x57, 0x6B, 0x1D, 0x21, 0x13, 0x5B, 0x2D, 0x8C, 
	0xB3, 0x42, 0x65, 0x8, 0xCF, 0x4A, 0x65, 0x29, 0x5F, 
	0x5E, 0x65, 0x13, 0x4D, 0xA5, 0xE7, 0x6C, 0x17, 0x5C, 
	0x51, 0x31, 0xA3, 0x19, 0x51, 0x4D, 0x5F, 0x81, 0x4F, 
	0x39, 0x53, 0x72, 0x9D, 0x45, 0x9D, 0x1A, 0x29, 0x19, 
	0xE3, 0x21, 0x15, 0x31, 0x9B, 0xD, 0xF, 0x29, 0xF, 
	0x22, 0xF3, 0x5C, 0x63, 0x52, 0x1D, 0x33, 0xDF, 0x4C, 
	0xDF, 0x63, 0x6D, 0x42, 0xA9, 0x4A, 0xEB, 0x20, 0xDB, 
	0x18, 0x91, 0x12, 0x11, 0x12, 0x13, 0x18, 0xD3, 0x8, 
	0xD3, 0x42, 0x29, 0x4A, 0x69, 0x5A, 0xED, 0x73, 0xB1, 
	0x29, 0xA1, 0x19, 0x19, 0x5B, 0x6D, 0x13, 0xD, 0x32, 
	0x15, 0x21, 0x51, 0x14, 0x4F, 0x42, 0xD5, 0x4B, 0x4F, 
	0x5A, 0xAD, 0x4D, 0x21, 0x14, 0xCF, 0x9, 0x51, 0x6B, 
	0xB3, 0x29, 0x57, 0x22, 0x97, 0x33, 0x9B, 0x9D, 0xA5, 
	0x7B, 0xF3, 0x29, 0x99, 0x29, 0x9B, 
};

Gfx secret_books_Bookwall_ci8_aligner[] = {gsSPEndDisplayList()};
u8 secret_books_Bookwall_ci8[] = {
	0x0, 0x1, 0x2, 0x1, 0x3, 0x1, 0x1, 0x4, 0x1, 
	0x5, 0x5, 0x5, 0x5, 0x1, 0x1, 0x6, 0x0, 0x7, 
	0x7, 0x7, 0x7, 0x7, 0x7, 0x8, 0x5, 0x9, 0x3, 
	0xA, 0x1, 0xB, 0x1, 0xB, 0x0, 0x6, 0x2, 0x6, 
	0x3, 0x3, 0x8, 0x1, 0xC, 0x6, 0x5, 0xD, 0x4, 
	0x1, 0x1, 0x3, 0x0, 0x7, 0x7, 0x7, 0x7, 0x7, 
	0xE, 0x6, 0x4, 0x8, 0xF, 0x1, 0xB, 0xB, 0xB, 
	0xB, 0x0, 0x0, 0x2, 0x5, 0x10, 0x8, 0xA, 0xA, 
	0x9, 0x4, 0x5, 0x4, 0xC, 0x1, 0x1, 0x3, 0x0, 
	0x11, 0x12, 0x7, 0x7, 0x7, 0x11, 0x0, 0x6, 0x9, 
	0x13, 0x1, 0xB, 0xB, 0xB, 0xB, 0x0, 0x3, 0x6, 
	0x8, 0x6, 0x3, 0x3, 0x8, 0x3, 0x4, 0x5, 0x4, 
	0x8, 0x1, 0x1, 0x8, 0x0, 0x11, 0x7, 0x7, 0x7, 
	0x7, 0x11, 0x6, 0x6, 0x8, 0xA, 0x1, 0xF, 0xB, 
	0xB, 0xB, 0x2, 0x6, 0x6, 0x6, 0x6, 0x8, 0x8, 
	0x3, 0x8, 0x4, 0x5, 0x4, 0x8, 0xA, 0x1, 0x3, 
	0x0, 0x11, 0x7, 0x7, 0x7, 0x7, 0x14, 0x6, 0x6, 
	0x8, 0x1, 0xF, 0x15, 0x16, 0x17, 0x17, 0x0, 0x2, 
	0x6, 0x6, 0x6, 0x8, 0x3, 0x3, 0x8, 0x9, 0x5, 
	0x4, 0x3, 0xA, 0x1, 0x3, 0x18, 0x11, 0x7, 0x7, 
	0x7, 0x7, 0x14, 0x0, 0x6, 0x8, 0x1, 0x19, 0x1A, 
	0x1B, 0x17, 0x17, 0x0, 0x3, 0x6, 0x10, 0x6, 0x8, 
	0x3, 0x8, 0x8, 0x4, 0x5, 0x4, 0x8, 0xA, 0x1, 
	0x10, 0x18, 0x11, 0x7, 0x7, 0x7, 0x7, 0x14, 0x2, 
	0x6, 0x8, 0xA, 0x1, 0x1C, 0x16, 0x17, 0x17, 0x2, 
	0x6, 0x6, 0x6, 0x6, 0x8, 0x8, 0x3, 0x8, 0x4, 
	0x4, 0x4, 0x8, 0x1, 0x1, 0x3, 0x18, 0x11, 0x7, 
	0x7, 0x7, 0x7, 0xE, 0x2, 0x4, 0x8, 0x1, 0x19, 
	0x1D, 0x17, 0x17, 0x17, 0x6, 0x5, 0x6, 0x6, 0x6, 
	0x8, 0x3, 0xC, 0x8, 0x4, 0x4, 0x9, 0xC, 0x1, 
	0x1, 0x3, 0x1E, 0x14, 0x7, 0x7, 0x7, 0x11, 0x0, 
	0x5, 0x4, 0x8, 0x13, 0x1C, 0x1F, 0x1B, 0x1B, 0x1B, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x3, 0x3, 0x8, 
	0x8, 0x9, 0x9, 0x8, 0x3, 0x3, 0x3, 0x3, 0x14, 
	0x7, 0x7, 0x11, 0x0, 0x0, 0x5, 0x4, 0x8, 0x19, 
	0x1D, 0x20, 0x1B, 0x1B, 0x1B, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x9, 0x8, 
	0x8, 0x3, 0x3, 0x21, 0x22, 0x23, 0x7, 0x11, 0x0, 
	0x5, 0x5, 0x4, 0x8, 0x1, 0x1C, 0x24, 0x1B, 0x1B, 
	0x1B, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x9, 0x8, 0x8, 0x3, 0x3, 0x3, 
	0x14, 0x23, 0x23, 0x11, 0x0, 0x5, 0x6, 0x4, 0x8, 
	0xF, 0x1D, 0x1F, 0x16, 0x1B, 0x1B, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x3, 0x21, 0x14, 0x25, 0x7, 0x11, 
	0x2, 0x5, 0x6, 0x4, 0x8, 0x1, 0x1C, 0x24, 0x1B, 
	0x1B, 0x1B, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x3, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x3, 
	0x10, 0x14, 0x25, 0x7, 0x11, 0x0, 0x5, 0x4, 0x9, 
	0x3, 0xF, 0x1D, 0x1F, 0x1B, 0x1B, 0x1B, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x10, 0x8, 0x8, 0x8, 0x8, 0x14, 0x7, 0x7, 
	0x14, 0x2, 0x4, 0x8, 0x3, 0x1, 0x1C, 0x1D, 0x1D, 
	0x1B, 0x26, 0x27, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x21, 0x10, 
	0x8, 0x21, 0x21, 0x14, 0x14, 0x0, 0x21, 0x3, 0x1, 
	0xF, 0x1C, 0x20, 0x17, 0x28, 0x17, 0x17, 0x17, 0x8, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x21, 0x18, 0x0, 0x0, 0x18, 0x18, 0x0, 
	0x18, 0x21, 0x29, 0x1C, 0x15, 0x1D, 0x1B, 0x26, 0x2A, 
	0x2A, 0x27, 0x2B, 0x2B, 0x8, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x6, 0x14, 
	0x7, 0x7, 0x11, 0x0, 0x0, 0x6, 0x8, 0x1, 0xB, 
	0x20, 0x1B, 0x17, 0x28, 0x28, 0x2A, 0x2A, 0x2B, 0x2A, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x3, 0x3, 0x3, 
	0x3, 0x8, 0x21, 0x18, 0x14, 0x7, 0x7, 0x7, 0x11, 
	0xE, 0x18, 0x21, 0x29, 0x2C, 0xB, 0x1D, 0x1F, 0x1B, 
	0x17, 0x26, 0x28, 0x28, 0x28, 0x8, 0x8, 0x8, 0x8, 
	0x8, 0x3, 0xA, 0x1, 0x2D, 0x8, 0x21, 0x1E, 0x0, 
	0x7, 0x7, 0x25, 0x7, 0x7, 0x7, 0x14, 0x18, 0x21, 
	0x3, 0x1, 0xF, 0x1C, 0x1D, 0x1D, 0x1D, 0x26, 0x28, 
	0x28, 0x8, 0x8, 0x8, 0x8, 0x8, 0x3, 0xF, 0xF, 
	0x3, 0x21, 0x1E, 0x0, 0x14, 0x11, 0x7, 0x7, 0x23, 
	0x2E, 0x2E, 0x23, 0x2F, 0x0, 0x1E, 0x21, 0x8, 0x1, 
	0x1, 0xF, 0x1C, 0x17, 0x17, 0x17, 0x8, 0x8, 0x8, 
	0x8, 0x8, 0x3, 0xF, 0x1, 0x6, 0x0, 0x0, 0x0, 
	0xE, 0x14, 0x11, 0x7, 0x2E, 0x30, 0x30, 0x31, 0x7, 
	0x14, 0x0, 0x6, 0x4, 0x8, 0xC, 0x1, 0xB, 0x17, 
	0x26, 0x17, 0x8, 0x3, 0x8, 0x8, 0x8, 0x3, 0xF, 
	0x1, 0x21, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x11, 
	0x23, 0x2E, 0x2E, 0x23, 0x12, 0xE, 0x0, 0x6, 0x4, 
	0x8, 0x3, 0x1, 0x1F, 0x17, 0x17, 0x1B, 0x3, 0x3, 
	0x8, 0x8, 0x8, 0x3, 0xF, 0xF, 0x3, 0x21, 0x21, 
	0x6, 0x6, 0x6, 0x1E, 0x0, 0x7, 0x23, 0x23, 0x7, 
	0x11, 0x0, 0x0, 0x6, 0x9, 0x8, 0x3, 0xF, 0x16, 
	0x17, 0x17, 0x17, 0x8, 0x8, 0x8, 0x3, 0x3, 0xA, 
	0xF, 0xF, 0x21, 0x6, 0x6, 0x6, 0x6, 0x6, 0x6, 
	0x18, 0x7, 0x7, 0x7, 0x12, 0x14, 0x0, 0x2, 0x6, 
	0x8, 0x8, 0xC, 0xF, 0x1F, 0x17, 0x17, 0x17, 0x3, 
	0x1, 0x3, 0x3, 0x3, 0xA, 0xF, 0xF, 0x3, 0x3, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x21, 0x0, 0x14, 0x11, 
	0x11, 0xE, 0x0, 0x0, 0x6, 0x8, 0x3, 0x1, 0x1C, 
	0x1D, 0x20, 0x16, 0x26, 0x8, 0x1, 0x1, 0x1, 0x1, 
	0x1, 0x1, 0x1, 0x1, 0xA, 0x8, 0x8, 0x3, 0x1, 
	0x1, 0x3, 0x6, 0x0, 0xE, 0x14, 0x14, 0x0, 0x0, 
	0x6, 0x10, 0x3, 0xF, 0x1C, 0x15, 0x1D, 0x20, 0x17, 
	0x5, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x8, 
	0x8, 0x8, 0x8, 0x3, 0x1, 0x1, 0x3, 0x6, 0x0, 
	0x11, 0x11, 0x14, 0x14, 0x0, 0x2, 0x6, 0x8, 0x3, 
	0xF, 0xB, 0x20, 0x1B, 0x26, 0x1, 0x3, 0x1, 0x1, 
	0x1, 0xA, 0xA, 0x3, 0x8, 0x9, 0x9, 0x8, 0x8, 
	0x1, 0x1, 0x3, 0x21, 0x0, 0x14, 0x11, 0x14, 0x14, 
	0x0, 0x0, 0x6, 0x4, 0x8, 0xF, 0xB, 0x20, 0x1D, 
	0xB, 0x3, 0x8, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 
	0x8, 0x8, 0x8, 0x8, 0x8, 0x1, 0x1, 0x3, 0x21, 
	0x0, 0x14, 0x11, 0x11, 0x14, 0x0, 0x0, 0x6, 0x10, 
	0x3, 0xF, 0x1D, 0x1D, 0x1D, 0xB, 0x1, 0x1, 0x3, 
	0x3, 0x3, 0xA, 0xA, 0x1, 0x2D, 0x3, 0x8, 0x8, 
	0x3, 0x1, 0x1, 0x3, 0x6, 0x0, 0x11, 0x11, 0x11, 
	0x14, 0xE, 0x0, 0x6, 0x10, 0x1, 0x1C, 0xB, 0x24, 
	0x32, 0xB, 0x1, 0x1, 0x1, 0x1, 0x1, 0xA, 0x1, 
	0x1, 0x2D, 0x3, 0x8, 0x8, 0x2D, 0x1, 0x1, 0x3, 
	0x6, 0xE, 0x11, 0x11, 0x11, 0x11, 0x14, 0x0, 0x6, 
	0x3, 0xF, 0x1C, 0xB, 0x24, 0x24, 0xB, 0x3, 0x1, 
	0x1, 0x1, 0x3, 0xA, 0xA, 0x3, 0x8, 0x8, 0x9, 
	0x8, 0x2D, 0x1, 0x1, 0x3, 0x6, 0x14, 0x11, 0x11, 
	0x11, 0x11, 0xE, 0x2, 0x6, 0x8, 0x3, 0x13, 0x24, 
	0x20, 0x1D, 0x16, 0x8, 0x3, 0x3, 0x3, 0x8, 0x3, 
	0x3, 0x8, 0x8, 0x8, 0x9, 0x8, 0x2D, 0x1, 0x1, 
	0x3, 0x6, 0xE, 0x11, 0x11, 0x11, 0x11, 0x0, 0x0, 
	0x6, 0x8, 0x3, 0x13, 0x1D, 0x1F, 0x20, 0x1B, 0x8, 
	0x21, 0x1E, 0x18, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 
	0x8, 0x8, 0xA, 0x1, 0x1, 0x3, 0x6, 0xE, 0x11, 
	0x11, 0x11, 0x11, 0x14, 0x1E, 0x6, 0x8, 0xF, 0xB, 
	0x20, 0x1B, 0x1B, 0x1B, 0x8, 0x0, 0x11, 0x11, 0x6, 
	0x21, 0x3, 0x3, 0x3, 0x3, 0x8, 0x8, 0x3, 0x2D, 
	0x3, 0x8, 0x6, 0x0, 0xE, 0x11, 0x11, 0x11, 0x0, 
	0x0, 0x10, 0xF, 0xB, 0x20, 0x1B, 0x1B, 0x1B, 0x1B, 
	0x11, 0x11, 0x11, 0x11, 0x11, 0xE, 0x21, 0x3, 0x4, 
	0x6, 0x9, 0x9, 0x8, 0x8, 0x8, 0x8, 0x10, 0x6, 
	0x0, 0xE, 0x11, 0x11, 0xE, 0x0, 0x29, 0x1C, 0x1D, 
	0x16, 0x1B, 0x1B, 0x17, 0x1B, 0x11, 0x33, 0x11, 0x11, 
	0x11, 0x0, 0x0, 0x8, 0x6, 0x6, 0x4, 0x4, 0x9, 
	0x9, 0x8, 0x8, 0x8, 0x21, 0x0, 0xE, 0x11, 0x11, 
	0x11, 0x0, 0x29, 0xB, 0x20, 0x16, 0x1B, 0x1B, 0x1B, 
	0x1B, 0x12, 0x11, 0x0, 0x11, 0x14, 0xE, 0x3, 0x6, 
	0x6, 0x6, 0x4, 0x4, 0x9, 0x9, 0x8, 0x8, 0x8, 
	0x6, 0x18, 0xE, 0x11, 0x11, 0x11, 0x0, 0x34, 0xB, 
	0x1F, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x12, 0x11, 0x11, 
	0x11, 0x14, 0x18, 0x21, 0x6, 0x6, 0x6, 0x4, 0x4, 
	0x9, 0x8, 0x8, 0x8, 0x8, 0x21, 0x18, 0xE, 0x11, 
	0x11, 0xE, 0x18, 0x29, 0xB, 0x1F, 0x1B, 0x1B, 0x17, 
	0x17, 0x17, 0x0, 0x11, 0x11, 0x11, 0x0, 0x18, 0x10, 
	0x6, 0x6, 0x6, 0x4, 0x4, 0x9, 0x9, 0x3, 0x3, 
	0x8, 0x6, 0x0, 0xE, 0x11, 0x11, 0x0, 0x35, 0x2C, 
	0x24, 0x16, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x0, 0x0, 
	0x11, 0x11, 0x14, 0x21, 0x6, 0x6, 0x6, 0x6, 0x6, 
	0x4, 0x4, 0x9, 0x8, 0x8, 0x10, 0x6, 0x0, 0x11, 
	0x11, 0x11, 0x0, 0x29, 0xB, 0x16, 0x1B, 0x1B, 0x1B, 
	0x1B, 0x1B, 0x1B, 0x0, 0x0, 0x11, 0x11, 0x14, 0x18, 
	0x6, 0x4, 0x6, 0x6, 0x6, 0x4, 0x4, 0x9, 0x9, 
	0x8, 0x8, 0x6, 0x0, 0x11, 0x14, 0x0, 0x21, 0x29, 
	0x1D, 0x16, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x17, 0x0, 
	0x5, 0x2, 0x14, 0x0, 0x21, 0x8, 0x6, 0x6, 0x6, 
	0x6, 0x6, 0x4, 0x4, 0x9, 0x8, 0x10, 0x6, 0x0, 
	0x11, 0x0, 0x1E, 0x29, 0xB, 0x20, 0x1F, 0x20, 0x1D, 
	0x1B, 0x1B, 0x1B, 0x1B, 0x8, 0x6, 0x6, 0x18, 0x1E, 
	0x3, 0x3, 0x8, 0x6, 0x2, 0x2, 0x6, 0x6, 0x6, 
	0x8, 0x8, 0x8, 0x6, 0x0, 0xE, 0x0, 0x21, 0xF, 
	0x24, 0x20, 0x1D, 0xB, 0xB, 0x1F, 0x1B, 0x1B, 0x1B, 
	0x8, 0x1, 0x3, 0x4, 0x8, 0x3, 0x3, 0x10, 0x1E, 
	0x0, 0x0, 0x0, 0x1E, 0x21, 0x8, 0x3, 0x3, 0x6, 
	0x0, 0x0, 0x2, 0x21, 0x29, 0x1C, 0x15, 0x1C, 0x1C, 
	0xB, 0x1F, 0x1B, 0x1B, 0x1B, 0x8, 0x6, 0x0, 0x10, 
	0x8, 0x8, 0x8, 0x21, 0x0, 0x0, 0x0, 0x18, 0x1E, 
	0x21, 0x29, 0xF, 0xF, 0x3, 0x6, 0x2, 0x6, 0x4, 
	0x3, 0x1, 0xF, 0xF, 0x1C, 0x1D, 0x16, 0x1B, 0x1B, 
	0x1B, 0x8, 0x6, 0x5, 0x2, 0x6, 0x21, 0x6, 0x18, 
	0x0, 0x0, 0x21, 0x21, 0x3, 0x29, 0x2C, 0xB, 0x1C, 
	0x36, 0x6, 0x6, 0x6, 0x8, 0x8, 0x3, 0xA, 0x1, 
	0x1C, 0x24, 0x1D, 0x1B, 0x1B, 0x16, 0x0, 0x0, 0x6, 
	0x5, 0x6, 0x10, 0x6, 0x18, 0x0, 0x1E, 0x8, 0x3, 
	0x1, 0x1C, 0xB, 0xB, 0x1C, 0x29, 0x6, 0x6, 0x4, 
	0x4, 0x8, 0x3, 0x3, 0x1, 0xF, 0x1C, 0x24, 0x1B, 
	0x1B, 0x1B, 0x1C, 0xF, 0x3, 0x6, 0x6, 0x3, 0x3, 
	0x21, 0x21, 0x6, 0x21, 0x3, 0x1, 0xF, 0x1C, 0x1C, 
	0xF, 0x3, 0x6, 0x5, 0x6, 0x4, 0x8, 0x3, 0x3, 
	0x1, 0x1, 0x1C, 0x1F, 0x1B, 0x1B, 0x1B, 0x0, 0x0, 
	0x0, 0x10, 0x8, 0x3, 0x29, 0x3, 0x21, 0x6, 0x6, 
	0x21, 0x3, 0x1, 0xF, 0xF, 0xF, 0x3, 0x6, 0x6, 
	0x6, 0x6, 0x8, 0x3, 0x1, 0x1, 0x13, 0x1D, 0x16, 
	0x1B, 0x1B, 0x1B, 0x0, 0x0, 0x0, 0x0, 0x2, 0x8, 
	0x21, 0x21, 0x6, 0x6, 0x21, 0x8, 0x3, 0xF, 0x1C, 
	0x1C, 0xF, 0x3, 0x6, 0x6, 0x6, 0x4, 0x8, 0x3, 
	0xA, 0x1, 0x1C, 0xB, 0x24, 0x16, 0x1B, 0x16, 0x0, 
	0x0, 0x0, 0x0, 0x0, 0x8, 0x6, 0x18, 0x18, 0x6, 
	0x8, 0x3, 0x1, 0x1C, 0xB, 0xB, 0x1C, 0x29, 0x6, 
	0x6, 0x6, 0x4, 0x8, 0x3, 0x3, 0x1, 0x1, 0xF, 
	0x3, 0x1C, 0x1C, 0x37, 0x0, 0x0, 0x6, 0x0, 0x6, 
	0x3, 0x3, 0x34, 0x21, 0x21, 0x21, 0x3, 0x1, 0xF, 
	0x1C, 0x1C, 0xF, 0x3, 0x6, 0x2, 0x6, 0x6, 0x8, 
	0x3, 0x3, 0xA, 0x1, 0x8, 0x6, 0x8, 0x8, 0x6, 
	0x0, 0x5, 0x8, 0x0, 0x6, 0xF, 0xB, 0x2C, 0x1, 
	0x3, 0x21, 0x8, 0x3, 0x1, 0xF, 0xF, 0x1, 0x10, 
	0x1E, 0x0, 0x1E, 0x6, 0x10, 0x3, 0x2D, 0x3, 0x3, 
	0x10, 0x8, 0x1, 0x1C, 0x0, 0x0, 0x6, 0x2C, 0xF, 
	0x1, 0x1C, 0x1D, 0x1D, 0x1C, 0x1, 0x3, 0x3, 0x1, 
	0xF, 0x13, 0x1C, 0xF, 0x3, 0x1E, 0x2, 0x6, 0x6, 
	0x10, 0x3, 0x3, 0x3, 0x3, 0x3, 0x6, 0x3, 0x8, 
	0x0, 0x3, 0x3, 0x1C, 0xB, 0xB, 0x1C, 0xB, 0x15, 
	0x13, 0x1, 0x3, 0x3, 0x36, 0xF, 0x38, 0xF, 0xF, 
	0x8, 0x0, 0x0, 0x6, 0x6, 0x6, 0x8, 0x3, 0x1, 
	0x1, 0x1, 0x6, 0x8, 0x6, 0x3, 0x3, 0x1, 0x1, 
	0xF, 0x1C, 0x1C, 0x1C, 0xB, 0xF, 0x3, 0x21, 0x21, 
	0x3, 0x3, 0x3, 0x3, 0x8, 0x18, 0x0, 0x0, 0x0, 
	0x1E, 0x6, 0x21, 0x1, 0x1, 0xA, 0x8, 0x8, 0x8, 
	0x6, 0x3, 0x3, 0x3, 0x1C, 0x15, 0x16, 0xF, 0x1D, 
	0x1D, 0x1C, 0x1, 0x3, 0x1, 0xF, 0x1, 0xA, 0x3, 
	0x8, 0x0, 0x0, 0x0, 0x6, 0x6, 0x10, 0x3, 0x1, 
	0xF, 0x3, 0x10, 0x1, 0xF, 0x0, 0x1C, 0x3, 0xB, 
	0x20, 0x20, 0x1D, 0x24, 0x1D, 0x24, 0x1C, 0x1, 0x9, 
	0x1, 0xB, 0xB, 0x1D, 0xB, 0x3, 0x0, 0x0, 0x4, 
	0x6, 0x4, 0x3, 0x1, 0x1, 0x1, 0x36, 0x4, 0x3, 
	0x8, 0x0, 0x3, 0x3, 0xB, 0x20, 0x20, 0x1D, 0x24, 
	0x24, 0xF, 0xF, 0x3, 0x3, 0xF, 0xB, 0xB, 0xB, 
	0xB, 0x3, 0x0, 0x0, 0x4, 0x5, 0x6, 0x3, 0x3, 
	0x1, 0xF, 0x1, 0x6, 0x8, 0x0, 0x0, 0x1C, 0x0, 
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x11, 0x11, 
	0x11, 0x11, 0x6, 0x6, 0x6, 0x6, 0x11, 0x11, 0x11, 
	0x11, 0x11, 0x11, 0x0, 0x1, 0x3, 0x3, 0x8, 0x8, 
	0x5, 0x2, 0x5, 0x20, 0x0, 0x1, 0x1, 0x1, 0x1, 
	0x1, 0x1, 0x1, 0x11, 0x11, 0x11, 0x11, 0x5, 0x5, 
	0x5, 0x5, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 
	0xF, 0x8, 0x8, 0x8, 0x8, 0x2, 0x2, 0x0, 0x20, 
	0x3, 0xB, 0x20, 0x20, 0x1D, 0xB, 0xB, 0xF, 0x1, 
	0x3, 0x3, 0x1, 0xB, 0xB, 0xB, 0xB, 0x3, 0x0, 
	0x0, 0x8, 0x5, 0x5, 0x3, 0x3, 0x1, 0x1C, 0x37, 
	0x2, 0x8, 0x0, 0x0, 0x1C, 
};

Gfx secret_books_Bookwall_ci8_pal_rgba16_aligner[] = {gsSPEndDisplayList()};
u8 secret_books_Bookwall_ci8_pal_rgba16[] = {
	0x52, 0xEB, 0x3A, 0x23, 0x4A, 0xEB, 0x42, 0x65, 0x42, 
	0xA9, 0x4A, 0xAB, 0x4A, 0xA9, 0x63, 0x6D, 0x42, 0x67, 
	0x42, 0x69, 0x3A, 0x25, 0x31, 0xDD, 0x3A, 0x65, 0x42, 
	0xAB, 0x53, 0x2B, 0x3A, 0x21, 0x42, 0xA7, 0x5B, 0x2D, 
	0x5B, 0x6D, 0x31, 0xE1, 0x5B, 0x2B, 0x31, 0x9D, 0x29, 
	0x57, 0x21, 0x55, 0x52, 0xE9, 0x32, 0x21, 0x29, 0x9D, 
	0x21, 0x57, 0x31, 0xDF, 0x29, 0x9B, 0x4A, 0xE9, 0x29, 
	0x59, 0x29, 0x99, 0x4A, 0xA7, 0x63, 0x6B, 0x6B, 0xAD, 
	0x31, 0x9B, 0x6B, 0x6D, 0x21, 0x15, 0x18, 0xCF, 0x21, 
	0x13, 0x42, 0x63, 0x19, 0x13, 0x19, 0x11, 0x3A, 0x1F, 
	0x42, 0x25, 0x73, 0xEF, 0x5B, 0x6B, 0x7C, 0x2F, 0x6B, 
	0xAF, 0x31, 0xDB, 0x53, 0x2D, 0x4A, 0xA5, 0x42, 0xA5, 
	0x42, 0x23, 0x39, 0xDF, 0x39, 0xE1, 
};

Vtx secret_books_Secret_Book_mesh_vtx_cull[8] = {
	{{{-283, -200, -366},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{-283, -200, 0},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{-283, 0, 0},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{-283, 0, -366},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{83, -200, -366},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{83, -200, 0},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{83, 0, 0},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
	{{{83, 0, -366},0, {-16, -16},{0x0, 0x0, 0x0, 0xFF}}},
};

Vtx secret_books_Secret_Book_mesh_vtx_0[4] = {
	{{{83, -200, -83},0, {2032, 2032},{0x5A, 0x0, 0xA6, 0xFF}}},
	{{{-200, -200, -366},0, {-1040, 2032},{0x5A, 0x0, 0xA6, 0xFF}}},
	{{{-200, 0, -366},0, {-1040, -16},{0x5A, 0x0, 0xA6, 0xFF}}},
	{{{83, 0, -83},0, {2032, -16},{0x5A, 0x0, 0xA6, 0xFF}}},
};

Gfx secret_books_Secret_Book_mesh_tri_0[] = {
	gsSPVertex(secret_books_Secret_Book_mesh_vtx_0 + 0, 4, 0),
	gsSP2Triangles(0, 1, 2, 0, 0, 2, 3, 0),
	gsSPEndDisplayList(),
};Vtx secret_books_Secret_Book_mesh_vtx_1[12] = {
	{{{0, -200, 0},0, {923, 2032},{0x5A, 0x0, 0x5A, 0xFF}}},
	{{{83, -200, -83},0, {129, 2032},{0x5A, 0x0, 0x5A, 0xFF}}},
	{{{83, 0, -83},0, {129, 1098},{0x5A, 0x0, 0x5A, 0xFF}}},
	{{{0, 0, 0},0, {923, 1098},{0x5A, 0x0, 0x5A, 0xFF}}},
	{{{-283, 0, -283},0, {923, -16},{0x0, 0x7F, 0x0, 0xFF}}},
	{{{0, 0, 0},0, {923, 1098},{0x0, 0x7F, 0x0, 0xFF}}},
	{{{83, 0, -83},0, {129, 1098},{0x0, 0x7F, 0x0, 0xFF}}},
	{{{-200, 0, -366},0, {129, -16},{0x0, 0x7F, 0x0, 0xFF}}},
	{{{-283, -200, -283},0, {923, 2032},{0xA6, 0x0, 0xA6, 0xFF}}},
	{{{-283, 0, -283},0, {923, 1098},{0xA6, 0x0, 0xA6, 0xFF}}},
	{{{-200, 0, -366},0, {129, 1098},{0xA6, 0x0, 0xA6, 0xFF}}},
	{{{-200, -200, -366},0, {129, 2032},{0xA6, 0x0, 0xA6, 0xFF}}},
};

Gfx secret_books_Secret_Book_mesh_tri_1[] = {
	gsSPVertex(secret_books_Secret_Book_mesh_vtx_1 + 0, 12, 0),
	gsSP2Triangles(0, 1, 2, 0, 0, 2, 3, 0),
	gsSP2Triangles(4, 5, 6, 0, 4, 6, 7, 0),
	gsSP2Triangles(8, 9, 10, 0, 8, 10, 11, 0),
	gsSPEndDisplayList(),
};


Gfx mat_secret_books_books[] = {
	gsDPPipeSync(),
	gsDPSetCombineLERP(TEXEL0, 0, SHADE, 0, 0, 0, 0, ENVIRONMENT, TEXEL0, 0, SHADE, 0, 0, 0, 0, ENVIRONMENT),
	gsSPTexture(65535, 65535, 0, 0, 1),
	gsDPSetTextureLUT(G_TT_RGBA16),
	gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, secret_books_Books_ci8_pal_rgba16),
	gsDPTileSync(),
	gsDPSetTile(0, 0, 0, 256, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadSync(),
	gsDPLoadTLUTCmd(7, 137),
	gsDPPipeSync(),
	gsDPTileSync(),
	gsDPSetTextureImage(G_IM_FMT_CI, G_IM_SIZ_8b, 32, secret_books_Books_ci8),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 6, 0, G_TX_WRAP | G_TX_NOMIRROR, 5, 0),
	gsDPLoadSync(),
	gsDPLoadTile(7, 0, 0, 124, 252),
	gsDPPipeSync(),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 6, 0, G_TX_WRAP | G_TX_NOMIRROR, 5, 0),
	gsDPSetTileSize(0, 0, 0, 124, 252),
	gsSPSetLights1(secret_books_books_lights),
	gsSPEndDisplayList(),
};

Gfx mat_revert_secret_books_books[] = {
	gsDPPipeSync(),
	gsDPSetTextureLUT(G_TT_NONE),
	gsSPEndDisplayList(),
};


Gfx mat_secret_books_bookwall[] = {
	gsDPPipeSync(),
	gsDPSetCombineLERP(TEXEL0, 0, SHADE, 0, 0, 0, 0, ENVIRONMENT, TEXEL0, 0, SHADE, 0, 0, 0, 0, ENVIRONMENT),
	gsSPTexture(65535, 65535, 0, 0, 1),
	gsDPSetTextureLUT(G_TT_RGBA16),
	gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, secret_books_Bookwall_ci8_pal_rgba16),
	gsDPTileSync(),
	gsDPSetTile(0, 0, 0, 256, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadSync(),
	gsDPLoadTLUTCmd(7, 56),
	gsDPPipeSync(),
	gsDPTileSync(),
	gsDPSetTextureImage(G_IM_FMT_CI, G_IM_SIZ_8b, 32, secret_books_Bookwall_ci8),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 6, 0, G_TX_WRAP | G_TX_NOMIRROR, 5, 0),
	gsDPLoadSync(),
	gsDPLoadTile(7, 0, 0, 124, 252),
	gsDPPipeSync(),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 6, 0, G_TX_WRAP | G_TX_NOMIRROR, 5, 0),
	gsDPSetTileSize(0, 0, 0, 124, 252),
	gsSPSetLights1(secret_books_bookwall_lights),
	gsSPEndDisplayList(),
};

Gfx mat_revert_secret_books_bookwall[] = {
	gsDPPipeSync(),
	gsDPSetTextureLUT(G_TT_NONE),
	gsSPEndDisplayList(),
};


Gfx secret_books_Secret_Book_mesh[] = {
	gsSPClearGeometryMode(G_LIGHTING),
	gsSPVertex(secret_books_Secret_Book_mesh_vtx_cull + 0, 8, 0),
	gsSPSetGeometryMode(G_LIGHTING),
	gsSPCullDisplayList(0, 7),
	gsSPDisplayList(mat_secret_books_books),
	gsSPDisplayList(secret_books_Secret_Book_mesh_tri_0),
	gsSPDisplayList(mat_revert_secret_books_books),
	gsSPDisplayList(mat_secret_books_bookwall),
	gsSPDisplayList(secret_books_Secret_Book_mesh_tri_1),
	gsSPDisplayList(mat_revert_secret_books_bookwall),
	gsSPEndDisplayList(),
};



Gfx secret_books_material_revert_render_settings[] = {
	gsDPPipeSync(),
	gsSPSetGeometryMode(G_LIGHTING),
	gsSPClearGeometryMode(G_TEXTURE_GEN),
	gsDPSetCombineLERP(0, 0, 0, SHADE, 0, 0, 0, ENVIRONMENT, 0, 0, 0, SHADE, 0, 0, 0, ENVIRONMENT),
	gsSPTexture(65535, 65535, 0, 0, 0),
	gsDPSetEnvColor(255, 255, 255, 255),
	gsDPSetAlphaCompare(G_AC_NONE),
	gsSPEndDisplayList(),
};