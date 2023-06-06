Lights1 wheelo_tire_lights = gdSPDefLights1(
	0x7F, 0x7F, 0x7F,
	0xFE, 0xFE, 0xFE, 0x28, 0x28, 0x28);

Gfx wheelo_Wheel_ci8_aligner[] = {gsSPEndDisplayList()};
u8 wheelo_Wheel_ci8[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x02, 
	0x02, 0x03, 0x02, 0x04, 0x05, 0x05, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x05, 0x04, 0x03, 0x06, 0x03, 0x07, 0x08, 0x09, 
	0x09, 0x08, 0x07, 0x03, 0x06, 0x03, 0x02, 0x05, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x02, 
	0x06, 0x07, 0x08, 0x03, 0x03, 0x06, 0x02, 0x02, 
	0x02, 0x02, 0x06, 0x03, 0x03, 0x07, 0x07, 0x02, 
	0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 
	0x08, 0x03, 0x06, 0x02, 0x04, 0x0a, 0x0a, 0x0b, 
	0x0b, 0x0b, 0x0a, 0x04, 0x02, 0x02, 0x03, 0x07, 
	0x03, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0c, 0x06, 0x03, 0x03, 
	0x06, 0x02, 0x0a, 0x0d, 0x0b, 0x0e, 0x02, 0x02, 
	0x02, 0x02, 0x0e, 0x02, 0x0d, 0x0a, 0x02, 0x06, 
	0x03, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0x06, 0x07, 0x03, 0x06, 
	0x0b, 0x0d, 0x10, 0x0d, 0x0a, 0x0b, 0x0b, 0x0b, 
	0x0b, 0x0b, 0x04, 0x0b, 0x0d, 0x10, 0x0d, 0x04, 
	0x02, 0x03, 0x07, 0x02, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0x02, 0x07, 0x03, 0x06, 0x0a, 
	0x0d, 0x0d, 0x0a, 0x0b, 0x06, 0x03, 0x08, 0x09, 
	0x09, 0x08, 0x03, 0x06, 0x0b, 0x0a, 0x0d, 0x0d, 
	0x0a, 0x02, 0x03, 0x07, 0x02, 0x05, 0x00, 0x00, 
	0x00, 0x00, 0x02, 0x07, 0x03, 0x06, 0x0a, 0x11, 
	0x11, 0x0a, 0x06, 0x08, 0x12, 0x13, 0x14, 0x15, 
	0x16, 0x17, 0x13, 0x18, 0x08, 0x0e, 0x0a, 0x11, 
	0x0d, 0x0a, 0x02, 0x03, 0x03, 0x04, 0x00, 0x00, 
	0x00, 0x19, 0x07, 0x07, 0x0e, 0x04, 0x0a, 0x0a, 
	0x04, 0x03, 0x1a, 0x14, 0x1b, 0x1c, 0x1d, 0x0a, 
	0x0a, 0x1e, 0x1f, 0x20, 0x17, 0x1a, 0x03, 0x02, 
	0x0a, 0x11, 0x04, 0x06, 0x07, 0x03, 0x21, 0x00, 
	0x00, 0x03, 0x03, 0x03, 0x02, 0x0a, 0x02, 0x0b, 
	0x03, 0x22, 0x23, 0x24, 0x0d, 0x10, 0x0d, 0x25, 
	0x09, 0x0d, 0x10, 0x10, 0x26, 0x27, 0x1f, 0x03, 
	0x04, 0x0a, 0x11, 0x02, 0x03, 0x03, 0x03, 0x00, 
	0x28, 0x06, 0x07, 0x0e, 0x0a, 0x02, 0x02, 0x03, 
	0x18, 0x15, 0x1c, 0x29, 0x09, 0x0a, 0x11, 0x0d, 
	0x0d, 0x11, 0x0a, 0x03, 0x0e, 0x2a, 0x23, 0x2b, 
	0x03, 0x0a, 0x11, 0x0a, 0x06, 0x03, 0x06, 0x0f, 
	0x05, 0x03, 0x03, 0x02, 0x0a, 0x02, 0x02, 0x0a, 
	0x23, 0x2c, 0x1e, 0x0a, 0x0d, 0x2d, 0x22, 0x1c, 
	0x1c, 0x22, 0x10, 0x11, 0x0d, 0x1e, 0x24, 0x23, 
	0x02, 0x02, 0x0a, 0x11, 0x02, 0x03, 0x03, 0x2e, 
	0x03, 0x07, 0x0e, 0x04, 0x04, 0x04, 0x03, 0x2f, 
	0x16, 0x30, 0x1d, 0x11, 0x31, 0x32, 0x2b, 0x31, 
	0x31, 0x33, 0x32, 0x34, 0x11, 0x1d, 0x35, 0x16, 
	0x36, 0x03, 0x0b, 0x0a, 0x04, 0x0e, 0x03, 0x02, 
	0x07, 0x07, 0x0e, 0x0a, 0x0b, 0x02, 0x09, 0x23, 
	0x2a, 0x03, 0x02, 0x37, 0x38, 0x37, 0x30, 0x30, 
	0x2d, 0x1e, 0x2b, 0x1c, 0x37, 0x02, 0x35, 0x2a, 
	0x23, 0x09, 0x02, 0x0a, 0x0a, 0x0e, 0x03, 0x07, 
	0x0e, 0x03, 0x03, 0x0a, 0x03, 0x06, 0x35, 0x39, 
	0x34, 0x02, 0x2a, 0x14, 0x26, 0x30, 0x30, 0x3a, 
	0x0d, 0x1e, 0x1d, 0x26, 0x27, 0x38, 0x35, 0x1d, 
	0x39, 0x3a, 0x0e, 0x0a, 0x0a, 0x0e, 0x03, 0x0e, 
	0x03, 0x03, 0x0e, 0x0a, 0x02, 0x0e, 0x3b, 0x39, 
	0x03, 0x16, 0x39, 0x3c, 0x36, 0x0b, 0x0d, 0x30, 
	0x35, 0x0b, 0x0b, 0x3c, 0x14, 0x3d, 0x3e, 0x02, 
	0x3f, 0x12, 0x03, 0x0a, 0x0a, 0x0e, 0x03, 0x0e, 
	0x08, 0x03, 0x0e, 0x0a, 0x04, 0x03, 0x1f, 0x40, 
	0x03, 0x40, 0x41, 0x15, 0x42, 0x02, 0x03, 0x1d, 
	0x1d, 0x08, 0x02, 0x23, 0x23, 0x40, 0x43, 0x02, 
	0x39, 0x1f, 0x03, 0x0b, 0x0a, 0x0e, 0x03, 0x07, 
	0x03, 0x03, 0x03, 0x0a, 0x06, 0x03, 0x12, 0x41, 
	0x30, 0x1a, 0x3e, 0x14, 0x17, 0x0a, 0x02, 0x30, 
	0x10, 0x02, 0x1a, 0x1b, 0x15, 0x44, 0x45, 0x1e, 
	0x40, 0x2b, 0x03, 0x0b, 0x0a, 0x03, 0x03, 0x03, 
	0x0e, 0x03, 0x03, 0x0a, 0x02, 0x0e, 0x0a, 0x43, 
	0x38, 0x3a, 0x24, 0x46, 0x27, 0x3e, 0x0a, 0x02, 
	0x06, 0x2d, 0x23, 0x23, 0x39, 0x22, 0x37, 0x1c, 
	0x43, 0x02, 0x03, 0x0a, 0x0b, 0x03, 0x03, 0x0e, 
	0x03, 0x03, 0x03, 0x02, 0x02, 0x0e, 0x09, 0x47, 
	0x16, 0x45, 0x40, 0x40, 0x23, 0x24, 0x46, 0x17, 
	0x48, 0x23, 0x13, 0x49, 0x3d, 0x3d, 0x1c, 0x16, 
	0x23, 0x09, 0x06, 0x0a, 0x02, 0x03, 0x03, 0x03, 
	0x4a, 0x03, 0x03, 0x02, 0x0b, 0x02, 0x07, 0x45, 
	0x43, 0x2f, 0x16, 0x20, 0x3e, 0x46, 0x15, 0x3c, 
	0x1b, 0x4b, 0x3d, 0x17, 0x17, 0x16, 0x4c, 0x4d, 
	0x22, 0x07, 0x02, 0x0b, 0x02, 0x03, 0x03, 0x4a, 
	0x4e, 0x03, 0x03, 0x03, 0x02, 0x09, 0x09, 0x09, 
	0x47, 0x4f, 0x36, 0x2a, 0x11, 0x46, 0x40, 0x46, 
	0x39, 0x3d, 0x17, 0x04, 0x24, 0x36, 0x50, 0x47, 
	0x09, 0x03, 0x51, 0x03, 0x03, 0x03, 0x03, 0x52, 
	0x00, 0x03, 0x03, 0x03, 0x02, 0x09, 0x09, 0x07, 
	0x3a, 0x4f, 0x43, 0x53, 0x42, 0x39, 0x46, 0x0d, 
	0x1a, 0x46, 0x39, 0x17, 0x2a, 0x4d, 0x4f, 0x10, 
	0x03, 0x09, 0x25, 0x03, 0x03, 0x03, 0x0e, 0x00, 
	0x00, 0x0f, 0x03, 0x03, 0x03, 0x06, 0x51, 0x09, 
	0x08, 0x35, 0x4f, 0x54, 0x39, 0x15, 0x15, 0x17, 
	0x17, 0x15, 0x15, 0x3f, 0x54, 0x4f, 0x35, 0x08, 
	0x08, 0x51, 0x03, 0x03, 0x03, 0x03, 0x0f, 0x00, 
	0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x08, 
	0x09, 0x07, 0x06, 0x20, 0x4d, 0x55, 0x56, 0x57, 
	0x4d, 0x58, 0x56, 0x4d, 0x59, 0x03, 0x07, 0x08, 
	0x09, 0x08, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0x03, 0x03, 0x03, 0x03, 0x03, 
	0x09, 0x09, 0x08, 0x07, 0x06, 0x45, 0x48, 0x23, 
	0x23, 0x17, 0x1c, 0x0e, 0x08, 0x07, 0x09, 0x09, 
	0x09, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0x03, 0x03, 0x03, 0x03, 
	0x03, 0x09, 0x09, 0x09, 0x09, 0x03, 0x08, 0x07, 
	0x07, 0x08, 0x03, 0x08, 0x09, 0x09, 0x09, 0x07, 
	0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0f, 0x03, 0x03, 0x03, 
	0x03, 0x0e, 0x07, 0x09, 0x51, 0x51, 0x09, 0x09, 
	0x08, 0x09, 0x09, 0x51, 0x09, 0x08, 0x03, 0x03, 
	0x03, 0x03, 0x03, 0x5a, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 
	0x03, 0x03, 0x03, 0x03, 0x03, 0x08, 0x09, 0x09, 
	0x08, 0x09, 0x09, 0x03, 0x03, 0x03, 0x03, 0x03, 
	0x03, 0x03, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
	0x07, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 
	0x03, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x05, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x0c, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x4a, 0x0f, 0x03, 0x07, 0x03, 0x03, 
	0x03, 0x03, 0x03, 0x03, 0x0f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	
};

Gfx wheelo_Wheel_ci8_pal_rgba16_aligner[] = {gsSPEndDisplayList()};
u8 wheelo_Wheel_ci8_pal_rgba16[] = {
	0x31, 0xce, 0x3a, 0x10, 0x42, 0x11, 0x39, 0xcf, 
	0x42, 0x13, 0x42, 0x10, 0x3a, 0x11, 0x31, 0xcf, 
	0x31, 0x8f, 0x31, 0x8d, 0x4a, 0x53, 0x42, 0x53, 
	0x39, 0xd0, 0x4a, 0x95, 0x39, 0xd1, 0x39, 0xce, 
	0x52, 0x95, 0x4a, 0x55, 0x7b, 0xdf, 0x9d, 0x29, 
	0xad, 0xaf, 0xb5, 0xf1, 0xb5, 0xef, 0xad, 0xad, 
	0x73, 0xdf, 0x42, 0x52, 0x6b, 0x5b, 0xa5, 0x6d, 
	0x84, 0x63, 0x63, 0x5b, 0x5b, 0x19, 0x7c, 0x21, 
	0xa5, 0x6b, 0x4a, 0x52, 0x84, 0x21, 0xbe, 0x31, 
	0x94, 0xe7, 0x29, 0x8d, 0x94, 0xe9, 0xbe, 0x33, 
	0x29, 0x4c, 0x29, 0x4b, 0x8c, 0xa5, 0x73, 0x9d, 
	0x8c, 0xa7, 0x5a, 0xd9, 0x42, 0x12, 0x9c, 0xe7, 
	0x52, 0xd7, 0x6b, 0x9d, 0x8c, 0x65, 0x73, 0x9f, 
	0x63, 0x1b, 0x5a, 0xd7, 0x94, 0xa7, 0x6b, 0x5d, 
	0x84, 0x23, 0xce, 0xb5, 0x52, 0x97, 0x7b, 0xe1, 
	0x9d, 0x2b, 0xc6, 0x75, 0xb5, 0xaf, 0xce, 0x75, 
	0xce, 0xb7, 0xce, 0xf7, 0xad, 0x6d, 0xd6, 0xf7, 
	0xad, 0x6b, 0x8c, 0x63, 0xc6, 0x73, 0xc6, 0x31, 
	0xb5, 0xad, 0xbe, 0x73, 0x31, 0x8c, 0xbd, 0xf1, 
	0x9c, 0xe9, 0xde, 0xf7, 0x4a, 0x54, 0xd6, 0xb5, 
	0xd6, 0xb7, 0x29, 0x4d, 0x4a, 0x94, 0x94, 0xa5, 
	0xe7, 0x7b, 0xef, 0x7b, 0xe7, 0x39, 0xde, 0xf9, 
	0xdf, 0x39, 0xa5, 0x29, 0x31, 0x8e, 
};

Vtx wheelo_Cube_002_mesh_layer_4_vtx_cull[8] = {
	{{{-278, -118, -118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{-278, -118, 118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{-278, 118, 118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{-278, 118, -118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{278, -118, -118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{278, -118, 118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{278, 118, 118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
	{{{278, 118, -118},0, {-16, -16},{0x0, 0x0, 0x0, 0x0}}},
};

Vtx wheelo_Cube_002_mesh_layer_4_vtx_0[8] = {
	{{{278, 118, 118},0, {-16, 1008},{0x7F, 0x0, 0x0, 0xFF}}},
	{{{278, -118, 118},0, {1008, 1008},{0x7F, 0x0, 0x0, 0xFF}}},
	{{{278, -118, -118},0, {1008, -16},{0x7F, 0x0, 0x0, 0xFF}}},
	{{{278, 118, -118},0, {-16, -16},{0x7F, 0x0, 0x0, 0xFF}}},
	{{{-278, 118, 118},0, {-16, 1008},{0x81, 0x0, 0x0, 0xFF}}},
	{{{-278, 118, -118},0, {-16, -16},{0x81, 0x0, 0x0, 0xFF}}},
	{{{-278, -118, -118},0, {1008, -16},{0x81, 0x0, 0x0, 0xFF}}},
	{{{-278, -118, 118},0, {1008, 1008},{0x81, 0x0, 0x0, 0xFF}}},
};

Gfx wheelo_Cube_002_mesh_layer_4_tri_0[] = {
	gsSPVertex(wheelo_Cube_002_mesh_layer_4_vtx_0 + 0, 8, 0),
	gsSP2Triangles(0, 1, 2, 0, 0, 2, 3, 0),
	gsSP2Triangles(4, 5, 6, 0, 4, 6, 7, 0),
	gsSPEndDisplayList(),
};


Gfx mat_wheelo_tire[] = {
	gsDPPipeSync(),
	gsDPSetCombineLERP(TEXEL0, 0, SHADE, 0, TEXEL0, 0, ENVIRONMENT, 0, TEXEL0, 0, SHADE, 0, TEXEL0, 0, ENVIRONMENT, 0),
	gsSPClearGeometryMode(G_CULL_BACK),
	gsSPTexture(65535, 65535, 0, 0, 1),
	gsDPSetTextureLUT(G_TT_RGBA16),
	gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, wheelo_Wheel_ci8_pal_rgba16),
	gsDPTileSync(),
	gsDPSetTile(0, 0, 0, 256, 7, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0, G_TX_WRAP | G_TX_NOMIRROR, 0, 0),
	gsDPLoadSync(),
	gsDPLoadTLUTCmd(7, 90),
	gsDPPipeSync(),
	gsDPTileSync(),
	gsDPSetTextureImage(G_IM_FMT_CI, G_IM_SIZ_8b, 32, wheelo_Wheel_ci8),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 7, 0, G_TX_CLAMP | G_TX_NOMIRROR, 5, 0, G_TX_CLAMP | G_TX_NOMIRROR, 5, 0),
	gsDPLoadSync(),
	gsDPLoadTile(7, 0, 0, 124, 124),
	gsDPPipeSync(),
	gsDPSetTile(G_IM_FMT_CI, G_IM_SIZ_8b, 4, 0, 0, 0, G_TX_CLAMP | G_TX_NOMIRROR, 5, 0, G_TX_CLAMP | G_TX_NOMIRROR, 5, 0),
	gsDPSetTileSize(0, 0, 0, 124, 124),
	gsSPSetLights1(wheelo_tire_lights),
	gsSPEndDisplayList(),
};

Gfx mat_revert_wheelo_tire[] = {
	gsDPPipeSync(),
	gsSPSetGeometryMode(G_CULL_BACK),
	gsDPSetTextureLUT(G_TT_NONE),
	gsSPEndDisplayList(),
};

Gfx wheelo_Cube_002_mesh_layer_4[] = {
	gsSPClearGeometryMode(G_LIGHTING),
	gsSPVertex(wheelo_Cube_002_mesh_layer_4_vtx_cull + 0, 8, 0),
	gsSPSetGeometryMode(G_LIGHTING),
	gsSPCullDisplayList(0, 7),
	gsSPDisplayList(mat_wheelo_tire),
	gsSPDisplayList(wheelo_Cube_002_mesh_layer_4_tri_0),
	gsSPDisplayList(mat_revert_wheelo_tire),
	gsSPEndDisplayList(),
};

Gfx wheelo_material_revert_render_settings[] = {
	gsDPPipeSync(),
	gsSPSetGeometryMode(G_LIGHTING),
	gsSPClearGeometryMode(G_TEXTURE_GEN),
	gsDPSetCombineLERP(0, 0, 0, SHADE, 0, 0, 0, ENVIRONMENT, 0, 0, 0, SHADE, 0, 0, 0, ENVIRONMENT),
	gsSPTexture(65535, 65535, 0, 0, 0),
	gsDPSetEnvColor(255, 255, 255, 255),
	gsDPSetAlphaCompare(G_AC_NONE),
	gsSPEndDisplayList(),
};
