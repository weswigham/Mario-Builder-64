f32 cmm_camera_zoom_table[][2] = {
    {900.0f,50.0f},
    {900.0f,400.0f},
    {1500.0f,800.0f},
    {2000.0f,900.0f},
    {2100.0f,1500.0f},
};

// lookup table for rotating blocks
u8 cmm_rotated_dirs[4][6] = {
    {CMM_DIRECTION_UP, CMM_DIRECTION_DOWN, CMM_DIRECTION_POS_X, CMM_DIRECTION_NEG_X, CMM_DIRECTION_POS_Z, CMM_DIRECTION_NEG_Z},
    {CMM_DIRECTION_UP, CMM_DIRECTION_DOWN, CMM_DIRECTION_NEG_Z, CMM_DIRECTION_POS_Z, CMM_DIRECTION_POS_X, CMM_DIRECTION_NEG_X},
    {CMM_DIRECTION_UP, CMM_DIRECTION_DOWN, CMM_DIRECTION_NEG_X, CMM_DIRECTION_POS_X, CMM_DIRECTION_NEG_Z, CMM_DIRECTION_POS_Z},
    {CMM_DIRECTION_UP, CMM_DIRECTION_DOWN, CMM_DIRECTION_POS_Z, CMM_DIRECTION_NEG_Z, CMM_DIRECTION_NEG_X, CMM_DIRECTION_POS_X},
};

#define ROTATE_DIRECTION(dir, rot) (cmm_rotated_dirs[rot][dir])

struct cmm_terrain_quad cmm_terrain_fullblock_quads[] = {
    {{{1, 1, 1},  {1, 1, -1},  {-1, 1, 1},  {-1, 1, -1}},  1, CMM_DIRECTION_UP, CMM_FACESHAPE_FULL, CMM_GROWTH_FULL, NULL}, // TOP
    {{{1, -1, 1}, {-1, -1, 1}, {1, -1, -1}, {-1, -1, -1}}, 1, CMM_DIRECTION_DOWN, CMM_FACESHAPE_FULL, 0, NULL}, // BOTTOM
    {{{1, 1, 1},  {1, -1, 1},  {1, 1, -1},  {1, -1, -1}},  0, CMM_DIRECTION_POS_X, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // LEFT
    {{{-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}}, 0, CMM_DIRECTION_NEG_X, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // RIGHT
    {{{1, 1, 1},  {-1, 1, 1},  {1, -1, 1},  {-1, -1, 1}},  2, CMM_DIRECTION_POS_Z, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // FRONT
    {{{1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}}, 2, CMM_DIRECTION_NEG_Z, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // BACK
};
struct cmm_terrain_block cmm_terrain_fullblock = {
    6,
    0,
    &cmm_terrain_fullblock_quads,
    NULL
};

s8 slope_decal_uvs1[3][2] = {{-1, -1}, {-1, 1}, {1, 1}};
s8 slope_decal_uvs2[3][2] = {{-1, 1}, {-1, -1}, {1, 1}};

struct cmm_terrain_quad cmm_terrain_slope_quads[] = {
    {{{1, -1, 1},  {1, 1, -1},  {-1, -1, 1},  {-1, 1, -1}},  1, CMM_NO_CULLING, CMM_FACESHAPE_EMPTY, CMM_GROWTH_FULL, NULL}, // TOP
    {{{1, -1, 1}, {-1, -1, 1}, {1, -1, -1}, {-1, -1, -1}}, 1, CMM_DIRECTION_DOWN, CMM_FACESHAPE_FULL, 0, NULL}, // BOTTOM
    {{{1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}}, 2, CMM_DIRECTION_NEG_Z, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // BACK
};

struct cmm_terrain_tri cmm_terrain_slope_tris[] = {
    {{{1, -1, -1}, {1, 1, -1}, {1, -1, 1}},  0, CMM_DIRECTION_POS_X, CMM_FACESHAPE_TRI_1, CMM_GROWTH_SLOPE_SIDE_L, &slope_decal_uvs1}, // LEFT
    {{{-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}}, 0, CMM_DIRECTION_NEG_X, CMM_FACESHAPE_TRI_2, CMM_GROWTH_SLOPE_SIDE_R, &slope_decal_uvs2}, // RIGHT
};

struct cmm_terrain_block cmm_terrain_slope = {
    3,
    2,
    &cmm_terrain_slope_quads,
    &cmm_terrain_slope_tris,
};

struct cmm_terrain_quad cmm_terrain_corner_quads[] = {
    {{{1, -1, 1}, {-1, -1, 1}, {1, -1, -1}, {-1, -1, -1}}, 1, CMM_DIRECTION_DOWN, CMM_FACESHAPE_FULL, 0, NULL}, // BOTTOM
};

struct cmm_terrain_tri cmm_terrain_corner_tris[] = {
    {{{-1, -1, 1}, {1, -1, 1}, {-1, 1, -1}},  1, CMM_NO_CULLING, CMM_FACESHAPE_FULL, CMM_GROWTH_FULL, NULL}, // TOP-FRONT
    {{{-1, 1, -1}, {1, -1, 1}, {1, -1, -1}},  1, CMM_NO_CULLING, CMM_FACESHAPE_FULL, CMM_GROWTH_FULL, NULL}, // TOP-LEFT 
    {{{-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}}, 0, CMM_DIRECTION_NEG_X, CMM_FACESHAPE_TRI_2, CMM_GROWTH_SLOPE_SIDE_R, &slope_decal_uvs2}, // RIGHT
    {{{-1, -1, -1}, {-1, 1, -1}, {1, -1, -1}},  2, CMM_DIRECTION_NEG_Z, CMM_FACESHAPE_TRI_1, CMM_GROWTH_SLOPE_SIDE_L, &slope_decal_uvs1}, // BACK
};

struct cmm_terrain_block cmm_terrain_corner = {
    1,
    4,
    &cmm_terrain_corner_quads,
    &cmm_terrain_corner_tris,
};

struct cmm_terrain_quad cmm_terrain_icorner_quads[] = {
    {{{1, -1, 1}, {-1, -1, 1}, {1, -1, -1}, {-1, -1, -1}}, 1, CMM_DIRECTION_DOWN, CMM_FACESHAPE_FULL, 0, NULL}, // BOTTOM
    {{{1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}}, 2, CMM_DIRECTION_NEG_Z, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // BACK
    {{{-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}}, 0, CMM_DIRECTION_NEG_X, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // RIGHT
};

struct cmm_terrain_tri cmm_terrain_icorner_tris[] = {
    {{{-1, 1, 1}, {1, -1, 1}, {-1, 1, -1}},  1, CMM_NO_CULLING, CMM_FACESHAPE_FULL, CMM_GROWTH_FULL, NULL}, // TOP-FRONT
    {{{-1, 1, -1}, {1, -1, 1}, {1, 1, -1}},  1, CMM_NO_CULLING, CMM_FACESHAPE_FULL, CMM_GROWTH_FULL, NULL}, // TOP-LEFT
    {{{1, -1, -1}, {1, 1, -1}, {1, -1, 1}},  0, CMM_DIRECTION_POS_X, CMM_FACESHAPE_TRI_1, CMM_GROWTH_SLOPE_SIDE_L, &slope_decal_uvs1}, // LEFT
    {{{-1, 1, 1}, {-1, -1, 1}, {1, -1, 1}},  2, CMM_DIRECTION_POS_Z, CMM_FACESHAPE_TRI_2, CMM_GROWTH_SLOPE_SIDE_R, &slope_decal_uvs2}, // FRONT
};

struct cmm_terrain_block cmm_terrain_icorner = {
    3,
    4,
    &cmm_terrain_icorner_quads,
    &cmm_terrain_icorner_tris,
};

struct cmm_terrain_quad cmm_terrain_dslope_quads[] = {
    {{{1, 1, 1},  {1, 1, -1},  {-1, 1, 1},  {-1, 1, -1}},  1, CMM_DIRECTION_UP, CMM_FACESHAPE_FULL, CMM_GROWTH_FULL, NULL}, // TOP
    {{{1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}}, 2, CMM_DIRECTION_NEG_Z, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // BACK
    {{{1, 1, 1}, {-1, 1, 1}, {1, -1, -1}, {-1, -1, -1}}, 2, CMM_NO_CULLING, CMM_FACESHAPE_EMPTY, CMM_GROWTH_UNDERSLOPE, NULL}, // BOTTOM
};

struct cmm_terrain_tri cmm_terrain_dslope_tris[] = {
    {{{1, -1, -1}, {1, 1, -1}, {1, 1, 1}},  0, CMM_DIRECTION_POS_X, CMM_FACESHAPE_DOWNTRI_1, CMM_GROWTH_NORMAL_SIDE, NULL}, // LEFT
    {{{-1, 1, -1}, {-1, -1, -1}, {-1, 1, 1}}, 0, CMM_DIRECTION_NEG_X, CMM_FACESHAPE_DOWNTRI_2, CMM_GROWTH_NORMAL_SIDE, NULL}, // RIGHT
};

struct cmm_terrain_block cmm_terrain_dslope = {
    3,
    2,
    &cmm_terrain_dslope_quads,
    &cmm_terrain_dslope_tris,
};

struct cmm_terrain_quad cmm_terrain_sslope_quads[] = {
    {{{1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}}, 2, CMM_DIRECTION_NEG_Z, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // BACK
    {{{-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}}, 0, CMM_DIRECTION_NEG_X, CMM_FACESHAPE_FULL, CMM_GROWTH_NORMAL_SIDE, NULL}, // RIGHT
    {{{1, 1, -1},  {-1, 1, 1},  {1, -1, -1},  {-1, -1, 1}},  2, CMM_NO_CULLING, CMM_FACESHAPE_EMPTY, CMM_GROWTH_DIAGONAL_SIDE, NULL}, // DIAGONAL
};

struct cmm_terrain_tri cmm_terrain_sslope_tris[] = {
    {{{-1, 1, 1}, {1, 1, -1}, {-1, 1, -1}}, 1, CMM_DIRECTION_UP, CMM_FACESHAPE_TOPTRI, CMM_GROWTH_FULL, NULL}, // TOP
    {{{1, -1, -1}, {-1, -1, 1}, {-1, -1, -1}}, 1, CMM_DIRECTION_DOWN, CMM_FACESHAPE_TOPTRI, 0, NULL}, // BOTTOM
};

struct cmm_terrain_block cmm_terrain_sslope = {
    3,
    2,
    &cmm_terrain_sslope_quads,
    &cmm_terrain_sslope_tris,
};


// All of this is for the system where slopes can place a decal on the face below them.
// Very hardcoded, so best not to touch any of these numbers
s8 slope_decal_below_uvsquad_l[4][2] = {{-1, -1}, {1, 1}, {-1, -3}, {1, -1}};
s8 slope_decal_below_uvsquad_r[4][2] = {{1, 1}, {-1, -1}, {1, -1}, {-1, -3}};
s8 slope_decal_below_uvstri_1[3][2] = {{1, 1}, {-1, -1}, {1, -1}};
s8 slope_decal_below_uvstri_2[3][2] = {{1, 1}, {1, -1}, {-1, -1}};

struct cmm_terrain_quad cmm_terrain_slopebelowdecal_quad = {
    {{1, 1, 1},  {-1, 1, 1},  {1, -1, 1},  {-1, -1, 1}},  2, CMM_DIRECTION_POS_Z, CMM_FACESHAPE_FULL, 0, NULL
};
struct cmm_terrain_tri cmm_terrain_slopebelowdecal_downtri1 = {
    {{1, 1, 1},  {-1, 1, 1},  {1, -1, 1}},  2, CMM_DIRECTION_POS_Z, CMM_FACESHAPE_DOWNTRI_1, 0, &slope_decal_below_uvstri_1
};
struct cmm_terrain_tri cmm_terrain_slopebelowdecal_downtri2 = {
    {{-1, 1, 1},  {-1, -1, 1}, {1, 1, 1}},  2, CMM_DIRECTION_POS_Z, CMM_FACESHAPE_DOWNTRI_2, 0, &slope_decal_below_uvstri_2
};

struct cmm_tile_type_struct cmm_tile_types[] = {
    //Model GFX           //F3D Material            //Collision Mesh             //Grass   //Transparent
    {NULL, &cmm_terrain_fullblock, NULL   , FALSE},//TILE_TYPE_BLOCK
    {NULL, &cmm_terrain_slope,     NULL  , FALSE},//TILE_TYPE_SLOPE
    {NULL, &cmm_terrain_corner,    NULL , FALSE},//TILE_TYPE_CORNER
    {NULL, &cmm_terrain_icorner,  NULL, FALSE},//TILE_TYPE_ICORNER
    {NULL, &cmm_terrain_fullblock,                   NULL                       , FALSE},//TILE_TYPE_TROLL
    {&cull_cull_mesh, NULL,        NULL                       , FALSE},//TILE_TYPE_CULL
    {NULL, &cmm_terrain_dslope,    NULL           , FALSE},//TILE_TYPE_DSLOPE
    {&makerwater_mw_mesh, NULL,    makerwater_collision       , TRUE },//TILE_TYPE_WATER
    {&makerfence_mfv_mesh, NULL,   makerfence_collision       , TRUE },//TILE_TYPE_FENCE
    {NULL, &cmm_terrain_sslope,    NULL           , FALSE },//TILE_TYPE_SSLOPE
};

struct cmm_object_type_struct cmm_object_types[] = {
    //BEHAVIOR                 //Y-OFF      //MODEL ID                 //BILLB //TRAJ //SCALE  //ANIMATION PTR              //MX BPRM   //DISP.FUNC
    {bhvStar,                  TILE_SIZE/2, MODEL_STAR                 ,FALSE  ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvGoomba,                0,           MODEL_GOOMBA               ,FALSE  ,FALSE ,1.5f   ,goomba_seg8_anims_0801DA4C   , 0       , NULL       },
    {bhvYellowCoin,            0,           MODEL_YELLOW_COIN          ,TRUE   ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvGreenCoin,             0,           0xEF                       ,TRUE   ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvRedCoin,               0,           MODEL_RED_COIN             ,TRUE   ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvHiddenBlueCoin,        0,           MODEL_BLUE_COIN            ,TRUE   ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvBlueCoinSwitch,        0,           MODEL_BLUE_COIN_SWITCH     ,FALSE  ,FALSE ,3.0f   ,NULL                         , 0       , NULL       },
    {bhvHiddenRedCoinStar,     TILE_SIZE/2, MODEL_TRANSPARENT_STAR     ,FALSE  ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvNoteblock,             TILE_SIZE/2, MODEL_NOTEBLOCK            ,FALSE  ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvPodoboo,               TILE_SIZE/2, MODEL_PODOBOO              ,FALSE  ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvRex,                   0,           0xE1                       ,FALSE  ,FALSE ,1.5f   ,Rex_anims                    , 0       , NULL       },
    {bhvSmallBully,            0,           MODEL_BULLY                ,FALSE  ,FALSE ,1.0f   ,bully_seg5_anims_0500470C    , 0       , NULL       },
    {bhvBobomb,                0,           MODEL_BLACK_BOBOMB         ,FALSE  ,FALSE ,1.0f   ,bobomb_seg8_anims_0802396C   , 0       , NULL       },
    {bhvTree,                  0,           MODEL_MAKER_TREE_1         ,TRUE   ,FALSE ,1.0f   ,NULL                         , 4       , &df_tree   },
    {bhvExclamationBox,        TILE_SIZE/2, MODEL_EXCLAMATION_BOX      ,FALSE  ,FALSE ,2.0f   ,NULL                         , 8       , &df_exbox  },
    {bhvChuckya,               0,           MODEL_CHUCKYA              ,FALSE  ,FALSE ,2.0f   ,chuckya_seg8_anims_0800C070  , 0       , NULL       },
    {bhvSpawn,                 TILE_SIZE/2, MODEL_SPAWN                ,FALSE  ,FALSE ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvPhantasm,              0,           MODEL_MARIO                ,FALSE  ,FALSE ,1.0f   ,&evil_mario_anims[2]         , 0       , NULL       },
    {bhvWarpPipe,              0,           MODEL_MAKER_PIPE           ,FALSE  ,FALSE ,1.0f   ,NULL                         , 1       , NULL       },
    {bhvBadge,                 TILE_SIZE/2, MODEL_BADGE                ,TRUE   ,FALSE ,5.0f   ,NULL                         , 23      , &df_badge  },
    {bhvBoss,                  0,           MODEL_KINGBOMB_MAKER       ,FALSE  ,FALSE ,1.0f   ,king_bobomb_seg5_anims_0500FE30, 2     , &df_boss   },
    {bhvPlatformOnTrack,       TILE_SIZE/2, MODEL_CHECKERBOARD_PLATFORM,FALSE  ,TRUE  ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvBobBowlingBallSpawner, TILE_SIZE/2, MODEL_BOWLING_BALL         ,TRUE   ,TRUE  ,1.0f   ,NULL                         , 0       , NULL       },
    {bhvKoopa,                 0,           MODEL_KOOPA_WITH_SHELL     ,FALSE  ,TRUE  ,3.0f   ,koopa_seg6_anims_06011364    , 0       , NULL       },
};

u8 cmm_star_objects[] = {
    OBJECT_TYPE_STAR,
    OBJECT_TYPE_RCS,
    OBJECT_TYPE_BOSS,
    OBJECT_TYPE_KTQ,
};

//skybox table
u8 *cmm_skybox_table[] = {
    _water_skybox_yay0SegmentRomStart,
    _water_skybox_yay0SegmentRomEnd,
    _cloud_floor_skybox_yay0SegmentRomStart,
    _cloud_floor_skybox_yay0SegmentRomEnd,
    _bitfs_skybox_yay0SegmentRomStart,
    _bitfs_skybox_yay0SegmentRomEnd,
    _bidw_skybox_yay0SegmentRomStart,
    _bidw_skybox_yay0SegmentRomEnd,
    _bbh_skybox_yay0SegmentRomStart,
    _bbh_skybox_yay0SegmentRomEnd,
    _ccm_skybox_yay0SegmentRomStart,
    _ccm_skybox_yay0SegmentRomEnd,
    _ssl_skybox_yay0SegmentRomStart,
    _ssl_skybox_yay0SegmentRomEnd,
    _bits_skybox_yay0SegmentRomStart,
    _bits_skybox_yay0SegmentRomEnd,
};

//envfx strings
u8 txt_envfx_1[] = {TXT_ENVFX_1};u8 txt_envfx_2[] = {TXT_ENVFX_2};u8 txt_envfx_3[] = {TXT_ENVFX_3};
u8 txt_envfx_4[] = {TXT_ENVFX_4};u8 txt_envfx_5[] = {TXT_ENVFX_5};u8 txt_envfx_6[] = {TXT_ENVFX_6};
u8 *cmm_envfx_string_table[] = {
    &txt_envfx_1,
    &txt_envfx_2,
    &txt_envfx_3,
    &txt_envfx_4,
    &txt_envfx_5,
    &txt_envfx_6,
};

//envfx table
u8 cmm_envfx_table[] = {
    ENVFX_MODE_NONE,         // no effects
    ENVFX_SNOW_NORMAL,       // CCM, SL
    ENVFX_SNOW_BLIZZARD,     // unused
    ENVFX_FLOWERS,           // unused
    ENVFX_LAVA_BUBBLES,      // LLL, BitFS, Bowser 2
    ENVFX_RAIN
};

//background strings
u8 txt_bg_1[] = {TXT_BG1}; u8 txt_bg_2[] = {TXT_BG2}; u8 txt_bg_3[] = {TXT_BG3}; u8 txt_bg_4[] = {TXT_BG4};
u8 txt_bg_5[] = {TXT_BG5}; u8 txt_bg_6[] = {TXT_BG6}; u8 txt_bg_7[] = {TXT_BG7}; u8 txt_bg_8[] = {TXT_BG8};
u8 *cmm_bg_string_table[] = {
    txt_bg_1,
    txt_bg_2,
    txt_bg_3,
    txt_bg_4,
    txt_bg_5,
    txt_bg_6,
    txt_bg_7,
    txt_bg_8,
};

//bottom plane strings
u8 txt_plane_1[] = {TXT_PLANE_1};u8 txt_plane_2[] = {TXT_PLANE_2};u8 txt_plane_3[] = {TXT_PLANE_3};
u8 *cmm_plane_string_table[] = {
    txt_plane_1,
    txt_plane_2,
    txt_plane_3,
};

//behparam2 strings
u8 txt_bp_box1[] = {TXT_BP_BOX_1}; u8 txt_bp_box2[] = {TXT_BP_BOX_2}; u8 txt_bp_box3[] = {TXT_BP_BOX_3}; u8 txt_bp_box4[] = {TXT_BP_BOX_4};
u8 txt_bp_box5[] = {TXT_BP_BOX_5}; u8 txt_bp_box6[] = {TXT_BP_BOX_6}; u8 txt_bp_box7[] = {TXT_BP_BOX_7}; u8 txt_bp_box8[] = {TXT_BP_BOX_8};
u8 *txt_bp_box[] = {
    txt_bp_box1,
    txt_bp_box2,
    txt_bp_box3,
    txt_bp_box4,
    txt_bp_box5,
    txt_bp_box6,
    txt_bp_box7,
    txt_bp_box8
};
u8 txt_bp_tree1[] = {TXT_BP_TREE_1}; u8 txt_bp_tree2[] = {TXT_BP_TREE_2};
u8 txt_bp_tree3[] = {TXT_BP_TREE_3}; u8 txt_bp_tree4[] = {TXT_BP_TREE_4};
u8 *txt_bp_tree[] = {
    txt_bp_tree1,
    txt_bp_tree2,
    txt_bp_tree3,
    txt_bp_tree4
};

u8 txt_bp_pipe1[] = {TXT_BP_PIPE_1};u8 txt_bp_pipe2[] = {TXT_BP_PIPE_2};
u8 *txt_pipe[] = {
    txt_bp_pipe1,
    txt_bp_pipe2,
    txt_bp_pipe1,
};

u8 txt_bp_boss1[] = {TXT_BP_BOSS_1};u8 txt_bp_boss2[] = {TXT_BP_BOSS_2};u8 txt_bp_boss3[] = {TXT_BP_BOSS_3};u8 txt_bp_boss4[] = {TXT_BP_BOSS_4};
u8 *txt_bp_boss[] = {
    txt_bp_boss1,
    txt_bp_boss2,
    txt_bp_boss3,
    txt_bp_boss4,
};

//theme strings
u8 txt_theme_1[] = {TXT_THEME_GENERIC};u8 txt_theme_2[] = {TXT_THEME_SAND};u8 txt_theme_3[] = {TXT_THEME_RHR};
u8 txt_theme_4[] = {TXT_THEME_CAVE}; u8 txt_theme_5[] = {TXT_THEME_CASTLE};u8 txt_theme_6[] = {TXT_THEME_VIRTUAPLEX};
u8 txt_theme_7[] = {TXT_THEME_RETRO};

u8 txt_mat_grass[] = {TXT_MAT_GRASS};u8 txt_mat_bricks[] = {TXT_MAT_BRICK};u8 txt_mat_stone[] = {TXT_MAT_STONE};u8 txt_mat_wood[] = {TXT_MAT_WOOD};u8 txt_mat_tiles[] = {TXT_MAT_TILES};u8 txt_mat_lava[] = {TXT_MAT_LAVA};
u8 txt_mat_sand[] = {TXT_MAT_SAND};u8 txt_mat_snow[] = {TXT_MAT_SNOW};u8 txt_mat_quicksand[] = {TXT_MAT_QUICKSAND};u8 txt_mat_roof[] = {TXT_MAT_ROOF};
u8 txt_mat_tiles2[] = {TXT_MAT_TILES2};u8 txt_mat_block[] = {TXT_MAT_BLOCK};
u8 txt_mat_basalt[] = {TXT_MAT_BASALT};u8 txt_mat_pillar[] = {TXT_MAT_PILLAR};u8 txt_mat_mesh[] = {TXT_MAT_MESH};
u8 txt_mat_mazewall[] = {TXT_MAT_MAZEWALL};u8 txt_mat_mazefloor[] = {TXT_MAT_MAZEFLOOR};u8 txt_mat_light[] = {TXT_MAT_LIGHT};u8 txt_mat_lakegrass[] = {TXT_MAT_LAKEGRASS};
u8 txt_mat_tileswood[] = {TXT_MAT_TILESWOOD};u8 txt_mat_tilesbricks[] = {TXT_MAT_TILESBRICKS};u8 txt_mat_tilesstone[] = {TXT_MAT_TILESSTONE};u8 txt_mat_carpet[] = {TXT_MAT_CARPET};u8 txt_mat_castlewall[] = {TXT_MAT_CASTLEWALL};u8 txt_mat_basementwall[] = {TXT_MAT_BASEMENTWALL};
u8 txt_mat_bluetiles[] = {TXT_MAT_BLUETILES};u8 txt_mat_rustyblock[] = {TXT_MAT_RUSTYBLOCK};u8 txt_mat_screen[] = {TXT_MAT_SCREEN};u8 txt_mat_hazardstripes[] = {TXT_MAT_HAZARDSTRIPES};u8 txt_mat_snowblock[] = {TXT_MAT_SNOWBLOCK};u8 txt_mat_void[] = {TXT_MAT_VOID};

u8 txt_mat_ground[] = {TXT_MAT_GROUND};u8 txt_mat_rblock[] = {TXT_MAT_RBLOCK};u8 txt_mat_treeplat[] = {TXT_MAT_TREEPLAT};u8 txt_mat_blueground[] = {TXT_MAT_BLUEGROUND};u8 txt_mat_bluebricks[] = {TXT_MAT_BLUEBRICKS};u8 txt_mat_blueblock[] = {TXT_MAT_BLUEBLOCK};u8 txt_mat_whitebrick[] = {TXT_MAT_WHITEBRICK};u8 txt_mat_underwaterground[] = {TXT_MAT_UNDERWATERGROUND};

u8 *cmm_theme_string_table[NUM_THEMES] = {
    &txt_theme_1,
    &txt_theme_2,
    &txt_theme_3,
    &txt_theme_4,
    &txt_theme_5,
    &txt_theme_6,
    &txt_theme_7,
};

enum cmm_materials {
    // Generic
    CMM_MAT_DIRT,
    CMM_MAT_GRASS,
    CMM_MAT_BRICKS,
    CMM_MAT_STONE,
    CMM_MAT_COBBLESTONE,
    CMM_MAT_TILES,
    CMM_MAT_TILESBRICKS,
    CMM_MAT_ROOF,
    CMM_MAT_WOOD,
    CMM_MAT_SANDDIRT,
    CMM_MAT_SAND,
    CMM_MAT_SNOWDIRT,
    CMM_MAT_SNOW,
    CMM_MAT_LAVA,
    CMM_MAT_QUICKSAND,
    // Desert
    CMM_MAT_DESERT_BRICKS,
    CMM_MAT_DESERT_STONE,
    CMM_MAT_DESERT_TILES,
    CMM_MAT_DESERT_BLOCK,
    CMM_MAT_DESERT_TILES2,
    // Red Hot Reservoir
    CMM_MAT_RHR_OBSIDIAN,
    CMM_MAT_RHR_STONE,
    CMM_MAT_RHR_BRICK,
    CMM_MAT_RHR_BASALT,
    CMM_MAT_RHR_TILES,
    CMM_MAT_RHR_BLOCK,
    CMM_MAT_RHR_WOOD,
    CMM_MAT_RHR_PILLAR,
    CMM_MAT_RHR_MESH,
    // Hazy Maze Cave
    CMM_MAT_HMC_GRASS,
    CMM_MAT_HMC_DIRT,
    CMM_MAT_HMC_BRICK,
    CMM_MAT_HMC_STONE,
    CMM_MAT_HMC_MAZEWALL,
    CMM_MAT_HMC_MAZEFLOOR,
    CMM_MAT_HMC_LIGHT,
    CMM_MAT_HMC_LAKEGRASS,
    CMM_MAT_HMC_MESH,
    // Castle
    CMM_MAT_C_TILES,
    CMM_MAT_C_WOOD,
    CMM_MAT_C_BRICK,
    CMM_MAT_C_STONETOP,
    CMM_MAT_C_STONESIDE,
    CMM_MAT_C_CARPET,
    CMM_MAT_C_ROOF,
    CMM_MAT_C_WALL,
    CMM_MAT_C_PILLAR,
    CMM_MAT_C_BASEMENTWALL,
    // Virtuaplex
    CMM_MAT_VP_BLOCK,
    CMM_MAT_VP_TILES,
    CMM_MAT_VP_BLUETILES,
    CMM_MAT_VP_RUSTYBLOCK,
    CMM_MAT_VP_SCREEN,
    CMM_MAT_VP_CAUTION,
    CMM_MAT_VP_SNOWBLOCK,
    CMM_MAT_VP_VOID,
    // Retro
    CMM_MAT_RETRO_GROUND,
    CMM_MAT_RETRO_BRICKS,
    CMM_MAT_RETRO_TREETOP,
    CMM_MAT_RETRO_TREEPLAT,
    CMM_MAT_RETRO_BLOCK,
    CMM_MAT_RETRO_BLUEGROUND,
    CMM_MAT_RETRO_BLUEBRICKS,
    CMM_MAT_RETRO_BLUEBLOCK,
    CMM_MAT_RETRO_WHITEBRICK,
    CMM_MAT_RETRO_LAVA,
    CMM_MAT_RETRO_UNDERWATERGROUND,
};

struct cmm_material cmm_mat_table[] = {
    // Generic
    {&mat_maker_MakerDirt,            0}, // CMM_MAT_DIRT
    {&mat_maker_MakerGrass,           0}, // CMM_MAT_GRASS
    {&mat_maker_MakerBricks,          0}, // CMM_MAT_BRICKS
    {&mat_maker_MakerStoneSide,       0}, // CMM_MAT_STONE
    {&mat_maker_MakerStone,           0}, // CMM_MAT_COBBLESTONE
    {&mat_maker_MakerTiles,           0}, // CMM_MAT_TILES
    {&mat_maker_MakerTileBricks,      0}, // CMM_MAT_TILESBRICKS
    {&mat_maker_MakerRoof,            0}, // CMM_MAT_ROOF
    {&mat_maker_MakerWood,            0}, // CMM_MAT_WOOD
    {&mat_maker_MakerSandDirt,        0}, // CMM_MAT_SANDDIRT
    {&mat_maker_MakerSand,            0}, // CMM_MAT_SAND
    {&mat_maker_MakerSnowDirt,        0}, // CMM_MAT_SNOWDIRT
    {&mat_maker_MakerSnow,            0}, // CMM_MAT_SNOW
    {&mat_maker_MakerLava,            0}, // CMM_MAT_LAVA
    {&mat_maker_MakerQuicksand,       0}, // CMM_MAT_QUICKSAND
    // Desert
    {&mat_maker_MakerDBrick,          0}, // CMM_MAT_DESERT_BRICKS
    {&mat_maker_MakerDStone,          0}, // CMM_MAT_DESERT_STONE
    {&mat_maker_MakerDTiles,          0}, // CMM_MAT_DESERT_TILES
    {&mat_maker_MakerDStoneBlock,     0}, // CMM_MAT_DESERT_BLOCK
    {&mat_maker_MakerDTiles2,         0}, // CMM_MAT_DESERT_TILES2
    // Red Hot Reservoir
    {&mat_maker_MakerRHRObsidian,     0}, // CMM_MAT_RHR_OBSIDIAN
    {&mat_maker_MakerRHRStone,        0}, // CMM_MAT_RHR_STONE
    {&mat_maker_MakerRHRBrick,        0}, // CMM_MAT_RHR_BRICK
    {&mat_maker_MakerRHRBasalt,       0}, // CMM_MAT_RHR_BASALT
    {&mat_maker_MakerRHRTiles,        0}, // CMM_MAT_RHR_TILES
    {&mat_maker_MakerRHRPattern,      0}, // CMM_MAT_RHR_BLOCK
    {&mat_maker_MakerRHRWood,         0}, // CMM_MAT_RHR_WOOD
    {&mat_maker_MakerRHRPillar,       0}, // CMM_MAT_RHR_PILLAR
    {&mat_maker_MakerRHRGrate_layer1, 1}, // CMM_MAT_RHR_MESH
    // Hazy Maze Cave
    {&mat_maker_MakerHGrass,          0}, // CMM_MAT_HMC_GRASS
    {&mat_maker_MakerHDirt,           0}, // CMM_MAT_HMC_DIRT
    {&mat_maker_MakerHBrick,          0}, // CMM_MAT_HMC_BRICK
    {&mat_maker_MakerHStone,          0}, // CMM_MAT_HMC_STONE
    {&mat_maker_MakerHMazewall,       0}, // CMM_MAT_HMC_MAZEWALL
    {&mat_maker_MakerHMazefloor,      0}, // CMM_MAT_HMC_MAZEFLOOR
    {&mat_maker_MakerHLight,          0}, // CMM_MAT_HMC_LIGHT
    {&mat_maker_MakerHLakewall,       0}, // CMM_MAT_HMC_LAKEGRASS
    {&mat_maker_MakerHFence_layer1,   1}, // CMM_MAT_HMC_MESH
    // Castle
    {&mat_maker_MakerCTile,           0}, // CMM_MAT_C_TILES
    {&mat_maker_MakerCWood,           0}, // CMM_MAT_C_WOOD
    {&mat_maker_MakerCBrick,          0}, // CMM_MAT_C_BRICK
    {&mat_maker_MakerCStone,          0}, // CMM_MAT_C_STONETOP
    {&mat_maker_MakerCStoneSide,      0}, // CMM_MAT_C_STONESIDE
    {&mat_maker_MakerCCarpet,         0}, // CMM_MAT_C_CARPET
    {&mat_maker_MakerCRoof,           0}, // CMM_MAT_C_ROOF
    {&mat_maker_MakerCWall,           0}, // CMM_MAT_C_WALL
    {&mat_maker_MakerCPillar,         0}, // CMM_MAT_C_PILLAR
    {&mat_maker_MakerCBasementWall,   0}, // CMM_MAT_C_BASEMENTWALL
    // Virtuaplex
    {&mat_maker_MakerVBlock,          0}, // CMM_MAT_VP_BLOCK
    {&mat_maker_MakerVTiles,          0}, // CMM_MAT_VP_TILES
    {&mat_maker_MakerVBlueTiles,      0}, // CMM_MAT_VP_BLUETILES
    {&mat_maker_MakerVRustyBlock,     0}, // CMM_MAT_VP_RUSTYBLOCK
    {&mat_maker_MakerScreen,          0}, // CMM_MAT_VP_SCREEN
    {&mat_maker_MakerCaution,         0}, // CMM_MAT_VP_CAUTION
    {&mat_maker_MakerVBlock,          0}, // CMM_MAT_VP_SNOWBLOCK
    {&mat_maker_MakerVoid,            0}, // CMM_MAT_VP_VOID
    // Retro
    {&mat_maker_MakerRetroGround,     0}, // CMM_MAT_RETRO_GROUND
    {&mat_maker_MakerRetroBrick,      0}, // CMM_MAT_RETRO_BRICKS
    {&mat_maker_MakerRetroTreeTop,    0}, // CMM_MAT_RETRO_TREETOP
    {&mat_maker_MakerRetroTree,       0}, // CMM_MAT_RETRO_TREEPLAT
    {&mat_maker_MakerRetroBlock,      0}, // CMM_MAT_RETRO_BLOCK
    {&mat_maker_MakerRetroBGround,    0}, // CMM_MAT_RETRO_BLUEGROUND
    {&mat_maker_MakerRetroBBrick,     0}, // CMM_MAT_RETRO_BLUEBRICKS
    {&mat_maker_MakerRetroBBlock,     0}, // CMM_MAT_RETRO_BLUEBLOCK
    {&mat_maker_MakerRetroWBrick,     0}, // CMM_MAT_RETRO_WHITEBRICK
    {&mat_maker_MakerRetroLava,       0}, // CMM_MAT_RETRO_LAVA
    {&mat_maker_MakerRetroUnderwater, 0}, // CMM_MAT_RETRO_UNDERWATERGROUND
};

enum cmm_topmaterials {
    CMM_TOPMAT_NONE,
    CMM_TOPMAT_GRASS,
    CMM_TOPMAT_STONE,
    CMM_TOPMAT_TILES,
    CMM_TOPMAT_SAND,
    CMM_TOPMAT_SNOW,
    CMM_TOPMAT_DESERT_TILES2,
    CMM_TOPMAT_RHR_OBSIDIAN,
    CMM_TOPMAT_RHR_TILES,
    CMM_TOPMAT_HMC_GRASS,
    CMM_TOPMAT_C_TILES,
    CMM_TOPMAT_C_STONE,
    CMM_TOPMAT_C_CARPET,
    CMM_TOPMAT_VP_BLUETILES,
    CMM_TOPMAT_RETRO_TREETOP,
};

struct cmm_topmaterial cmm_topmat_table[] = {
    {CMM_MAT_GRASS,         &mat_maker_MakerGrassSide_layer1},     // CMM_TOPMAT_GRASS
    {CMM_MAT_STONE,         NULL},                                 // CMM_TOPMAT_STONE
    {CMM_MAT_TILES,         NULL},                                 // CMM_TOPMAT_TILES
    {CMM_MAT_SAND,          &mat_maker_MakerSandSide_layer1},      // CMM_TOPMAT_SAND
    {CMM_MAT_SNOW,          &mat_maker_MakerSnowSide_layer1},      // CMM_TOPMAT_SNOW
    {CMM_MAT_DESERT_TILES2, NULL},                                 // CMM_TOPMAT_DESERT_TILES2
    {CMM_MAT_RHR_OBSIDIAN,  NULL},                                 // CMM_TOPMAT_RHR_OBSIDIAN
    {CMM_MAT_RHR_TILES,     NULL},                                 // CMM_TOPMAT_RHR_TILES
    {CMM_MAT_HMC_GRASS,     &mat_maker_MakerHGrassSide_layer1},    // CMM_TOPMAT_HMC_GRASS
    {CMM_MAT_C_TILES,       NULL},                                 // CMM_TOPMAT_C_TILES
    {CMM_MAT_C_STONETOP,    NULL},                                 // CMM_TOPMAT_C_STONE
    {CMM_MAT_C_CARPET,      NULL},                                 // CMM_TOPMAT_C_CARPET
    {CMM_MAT_VP_BLUETILES,  NULL},                                 // CMM_TOPMAT_VP_BLUETILES
    {CMM_MAT_RETRO_TREETOP, &mat_maker_MakerRetroTreeSide_layer1}, // CMM_TOPMAT_RETRO_TREETOP
};

struct cmm_theme cmm_theme_table[NUM_THEMES] = {
    // GENERIC
    {
        {
            {CMM_MAT_DIRT,        CMM_TOPMAT_GRASS, &txt_mat_grass},
            {CMM_MAT_BRICKS,      0,                &txt_mat_bricks},
            {CMM_MAT_COBBLESTONE, CMM_TOPMAT_STONE, &txt_mat_stone},
            {CMM_MAT_TILESBRICKS, CMM_TOPMAT_TILES, &txt_mat_tiles},
            {CMM_MAT_ROOF,        0,                &txt_mat_roof},
            {CMM_MAT_WOOD,        0,                &txt_mat_wood},
            {CMM_MAT_SANDDIRT,    CMM_TOPMAT_SAND,  &txt_mat_sand},
            {CMM_MAT_SNOWDIRT,    CMM_TOPMAT_SNOW,  &txt_mat_snow},
            {CMM_MAT_LAVA,        0,                &txt_mat_lava},
            {CMM_MAT_QUICKSAND,   0,                &txt_mat_quicksand},
        },
    },
    // DESERT
    {
        {
            {CMM_MAT_SANDDIRT,      CMM_TOPMAT_SAND,          &txt_mat_sand},
            {CMM_MAT_DESERT_BRICKS, 0,                        &txt_mat_bricks},
            {CMM_MAT_DESERT_STONE,  0,                        &txt_mat_stone},
            {CMM_MAT_DESERT_TILES,  0,                        &txt_mat_tiles},
            {CMM_MAT_DESERT_BLOCK,  0,                        &txt_mat_block},
            {CMM_MAT_WOOD,          0,                        &txt_mat_wood},
            {CMM_MAT_DESERT_BRICKS, CMM_TOPMAT_DESERT_TILES2, &txt_mat_tiles2},
            {CMM_MAT_DIRT,          CMM_TOPMAT_GRASS,         &txt_mat_grass},
            {CMM_MAT_LAVA,          0,                        &txt_mat_lava},
            {CMM_MAT_QUICKSAND,     0,                        &txt_mat_quicksand},
        },
    },
    // LAVA
    {
        {
            {CMM_MAT_RHR_STONE,  CMM_TOPMAT_RHR_OBSIDIAN, &txt_mat_stone},
            {CMM_MAT_RHR_BRICK,  CMM_TOPMAT_RHR_OBSIDIAN, &txt_mat_bricks},
            {CMM_MAT_RHR_BASALT, 0,                       &txt_mat_basalt},
            {CMM_MAT_RHR_TILES,  0,                       &txt_mat_tiles},
            {CMM_MAT_RHR_BLOCK,  0,                       &txt_mat_block},
            {CMM_MAT_RHR_WOOD,   0,                       &txt_mat_wood},
            {CMM_MAT_RHR_PILLAR, CMM_TOPMAT_RHR_TILES,    &txt_mat_pillar},
            {CMM_MAT_RHR_MESH,   0,                       &txt_mat_mesh},
            {CMM_MAT_LAVA,       0,                       &txt_mat_lava},
            {CMM_MAT_QUICKSAND,  0,                       &txt_mat_quicksand},
        },
    },
    // CAVE
    {
        {
            {CMM_MAT_HMC_DIRT,      CMM_TOPMAT_HMC_GRASS, &txt_mat_grass},
            {CMM_MAT_HMC_BRICK,     0,                    &txt_mat_bricks},
            {CMM_MAT_HMC_STONE,     0,                    &txt_mat_stone},
            {CMM_MAT_HMC_MAZEWALL,  0,                    &txt_mat_mazewall},
            {CMM_MAT_HMC_MAZEFLOOR, 0,                    &txt_mat_mazefloor},
            {CMM_MAT_HMC_LIGHT,     0,                    &txt_mat_light},
            {CMM_MAT_HMC_LAKEGRASS, CMM_TOPMAT_HMC_GRASS, &txt_mat_lakegrass},
            {CMM_MAT_HMC_MESH,      0,                    &txt_mat_mesh},
            {CMM_MAT_LAVA,          0,                    &txt_mat_lava},
            {CMM_MAT_QUICKSAND,     0,                    &txt_mat_quicksand},
        },
    },
    // CASTLE
    {
        {
            {CMM_MAT_C_WOOD,         CMM_TOPMAT_C_TILES,  &txt_mat_tileswood},
            {CMM_MAT_C_BRICK,        CMM_TOPMAT_C_TILES,  &txt_mat_tilesbricks},
            {CMM_MAT_C_STONESIDE,    CMM_TOPMAT_C_STONE,  &txt_mat_tilesstone},
            {CMM_MAT_C_WOOD,         CMM_TOPMAT_C_CARPET, &txt_mat_carpet},
            {CMM_MAT_C_ROOF,         0,                   &txt_mat_roof},
            {CMM_MAT_C_WALL,         0,                   &txt_mat_castlewall},
            {CMM_MAT_C_PILLAR,       CMM_TOPMAT_C_STONE,  &txt_mat_pillar},
            {CMM_MAT_C_BASEMENTWALL, 0,                   &txt_mat_basementwall},
            {CMM_MAT_LAVA,           0,                   &txt_mat_lava},
            {CMM_MAT_QUICKSAND,      0,                   &txt_mat_quicksand},
        },
    },
    // VIRTUAPLEX
    {
        {
            {CMM_MAT_VP_BLOCK,      0,                       &txt_mat_block},
            {CMM_MAT_VP_TILES,      0,                       &txt_mat_tiles},
            {CMM_MAT_DIRT,          CMM_TOPMAT_GRASS,        &txt_mat_grass},
            {CMM_MAT_VP_TILES,      CMM_TOPMAT_VP_BLUETILES, &txt_mat_bluetiles},
            {CMM_MAT_VP_RUSTYBLOCK, 0,                       &txt_mat_rustyblock},
            {CMM_MAT_VP_SCREEN,     0,                       &txt_mat_screen},
            {CMM_MAT_VP_CAUTION,    0,                       &txt_mat_hazardstripes},
            {CMM_MAT_VP_SNOWBLOCK,  CMM_TOPMAT_SNOW,         &txt_mat_snowblock},
            {CMM_MAT_LAVA,          0,                       &txt_mat_lava},
            {CMM_MAT_VP_VOID,       0,                       &txt_mat_void},
        },
    },
    // RETRO
    {
        {
            {CMM_MAT_RETRO_GROUND,           0,                        &txt_mat_ground},
            {CMM_MAT_RETRO_BRICKS,           0,                        &txt_mat_bricks},
            {CMM_MAT_RETRO_TREEPLAT,         CMM_TOPMAT_RETRO_TREETOP, &txt_mat_treeplat},
            {CMM_MAT_RETRO_BLOCK,            0,                        &txt_mat_rblock},
            {CMM_MAT_RETRO_BLUEGROUND,       0,                        &txt_mat_blueground},
            {CMM_MAT_RETRO_BLUEBRICKS,       0,                        &txt_mat_bluebricks},
            {CMM_MAT_RETRO_BLUEBLOCK,        0,                        &txt_mat_blueblock},
            {CMM_MAT_RETRO_WHITEBRICK,       0,                        &txt_mat_whitebrick},
            {CMM_MAT_RETRO_LAVA,             0,                        &txt_mat_lava},
            {CMM_MAT_RETRO_UNDERWATERGROUND, 0,                        &txt_mat_underwaterground},
        },
    },
};

// Returns full tile definition (struct cmm_tilemat_def)
#define TILE_MATDEF(matid) (cmm_theme_table[cmm_lopt_theme].mats[matid])
// Returns main material (struct cmm_material)
#define MATERIAL(matid) (cmm_mat_table[TILE_MATDEF(matid).mat])

// Returns TRUE if given material has a unique top texture
#define HAS_TOPMAT(matid) (TILE_MATDEF(matid).topmat != 0)
// Returns top material's topmat struct (struct cmm_topmaterial)
#define TOPMAT_DEF(matid) (cmm_topmat_table[TILE_MATDEF(matid).topmat - 1])
// Returns top material (struct cmm_material)
#define TOPMAT(matid) (cmm_mat_table[TOPMAT_DEF(matid).mat])
// Returns side decal texture (Gfx *)
#define SIDETEX(matid) (TOPMAT_DEF(matid).decaltex)

u8 cmm_ui_bar[9] = {
    CMM_BUTTON_TERRAIN,
    CMM_BUTTON_FENCE,
    CMM_BUTTON_SLOPE,
    CMM_BUTTON_COIN,
    CMM_BUTTON_GOOMBA,
    CMM_BUTTON_STAR,
    CMM_BUTTON_PLAY,
    CMM_BUTTON_SETTINGS,
    CMM_BUTTON_SAVE,
};

u8 cmm_toolbox[45];
//Different toolboxes for different game styles
u8 cmm_toolbox_btcm[45] = {
    /*Tiles    */ CMM_BUTTON_TERRAIN, CMM_BUTTON_FENCE, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK,
    /*Tiles 2  */ CMM_BUTTON_SLOPE,CMM_BUTTON_CORNER,CMM_BUTTON_ICORNER,CMM_BUTTON_DSLOPE,CMM_BUTTON_SSLOPE,CMM_BUTTON_CULL, CMM_BUTTON_TROLL, CMM_BUTTON_WATER, CMM_BUTTON_BLANK,
    /*Items    */ CMM_BUTTON_STAR, CMM_BUTTON_RCS, CMM_BUTTON_COIN,CMM_BUTTON_GCOIN,CMM_BUTTON_RCOIN,CMM_BUTTON_BCOIN,CMM_BUTTON_BCS,CMM_BUTTON_BADGE,CMM_BUTTON_BLANK,
    /*Enemies  */ CMM_BUTTON_GOOMBA,CMM_BUTTON_REX,CMM_BUTTON_PODOBOO,CMM_BUTTON_BULLY,CMM_BUTTON_BOMB,CMM_BUTTON_CHUCKYA,CMM_BUTTON_PHANTASM,CMM_BUTTON_BLANK,CMM_BUTTON_BLANK,
    /*Obstacles*/ CMM_BUTTON_NOTEBLOCK,CMM_BUTTON_TREE,CMM_BUTTON_EXCLA,CMM_BUTTON_PIPE,CMM_BUTTON_MPLAT,CMM_BUTTON_SPAWN, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK,
};
u8 cmm_toolbox_vanilla[45] = {
    /*Tiles    */ CMM_BUTTON_TERRAIN, CMM_BUTTON_FENCE, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK,
    /*Tiles 2  */ CMM_BUTTON_SLOPE,CMM_BUTTON_CORNER,CMM_BUTTON_ICORNER,CMM_BUTTON_DSLOPE, CMM_BUTTON_SSLOPE, CMM_BUTTON_CULL, CMM_BUTTON_TROLL, CMM_BUTTON_WATER, CMM_BUTTON_BLANK,
    /*Items    */ CMM_BUTTON_STAR, CMM_BUTTON_RCS, CMM_BUTTON_COIN,CMM_BUTTON_RCOIN,CMM_BUTTON_BCOIN,CMM_BUTTON_BCS,CMM_BUTTON_KTQ,CMM_BUTTON_BLANK,CMM_BUTTON_BLANK,
    /*Enemies  */ CMM_BUTTON_GOOMBA,CMM_BUTTON_BULLY,CMM_BUTTON_BOMB,CMM_BUTTON_CHUCKYA,CMM_BUTTON_BBALL,CMM_BUTTON_BOSS,CMM_BUTTON_BLANK,CMM_BUTTON_BLANK,CMM_BUTTON_BLANK,
    /*Obstacles*/ CMM_BUTTON_TREE,CMM_BUTTON_PIPE,CMM_BUTTON_MPLAT,CMM_BUTTON_SPAWN,CMM_BUTTON_BLANK,CMM_BUTTON_BLANK,CMM_BUTTON_BLANK, CMM_BUTTON_BLANK, CMM_BUTTON_BLANK,
};

//button strings. is there a better way to do this?
u8 txt_btn_1[] = {TXT_BTN_1};u8 txt_btn_2[] = {TXT_BTN_2};u8 txt_btn_3[] = {TXT_BTN_3};u8 txt_btn_4[] = {TXT_BTN_4};u8 txt_btn_5[] = {TXT_BTN_5};
u8 txt_btn_6[] = {TXT_BTN_6};u8 txt_btn_7[] = {TXT_BTN_7};u8 txt_btn_8[] = {TXT_BTN_8};u8 txt_btn_9[] = {TXT_BTN_9};u8 txt_btn_10[] = {TXT_BTN_10};
u8 txt_btn_11[] = {TXT_BTN_11};u8 txt_btn_12[] = {TXT_BTN_12};u8 txt_btn_13[] = {TXT_BTN_13};u8 txt_btn_14[] = {TXT_BTN_14};u8 txt_btn_15[] = {TXT_BTN_15};
u8 txt_btn_16[] = {TXT_BTN_16};u8 txt_btn_17[] = {TXT_BTN_17};u8 txt_btn_18[] = {TXT_BTN_18};u8 txt_btn_19[] = {TXT_BTN_19};u8 txt_btn_20[] = {TXT_BTN_20};
u8 txt_btn_21[] = {TXT_BTN_21};u8 txt_btn_22[] = {TXT_BTN_22};u8 txt_btn_23[] = {TXT_BTN_23};u8 txt_btn_24[] = {TXT_BTN_24};u8 txt_btn_25[] = {TXT_BTN_25};
u8 txt_btn_26[] = {TXT_BTN_26};u8 txt_btn_27[] = {TXT_BTN_27};u8 txt_btn_28[] = {TXT_BTN_28};u8 txt_btn_29[] = {TXT_BTN_29};u8 txt_btn_30[] = {TXT_BTN_30};
u8 txt_btn_31[] = {TXT_BTN_31};u8 txt_btn_32[] = {TXT_BTN_32};u8 txt_btn_33[] = {TXT_BTN_33};u8 txt_btn_34[] = {TXT_BTN_34};u8 txt_btn_35[] = {TXT_BTN_35};
u8 txt_btn_36[] = {TXT_BTN_36};u8 txt_btn_37[] = {TXT_BTN_37};u8 txt_btn_38[] = {TXT_BTN_38};u8 txt_btn_39[] = {TXT_BTN_39};u8 txt_btn_40[] = {TXT_BTN_40};
u8 txt_btn_41[] = {TXT_BTN_41};u8 txt_btn_42[] = {TXT_BTN_42};u8 txt_btn_43[] = {TXT_BTN_43};

struct cmm_ui_button_type cmm_ui_buttons[] = {
    //button texture      //TILE/OBJ ID       //PLACE MODE //TXT POINTER   //PARAM STR
    {&mat_b_btn_save     , 0                  ,0           , &txt_btn_1    , NULL         }, //CMM_BUTTON_SAVE
    {&mat_b_btn_settings , 0                  ,0           , &txt_btn_2    , NULL         }, //CMM_BUTTON_SETTINGS
    {&mat_b_btn_check    , 0                  ,0           , &txt_btn_3    , NULL         }, //CMM_BUTTON_PLAY
    {&mat_b_btn_grass    , TILE_TYPE_BLOCK    ,CMM_PM_TILE , &txt_btn_4    , NULL         }, //CMM_BUTTON_GRASS
    {&mat_b_btn_slope    , TILE_TYPE_SLOPE    ,CMM_PM_TILE , &txt_btn_7    , NULL         }, //CMM_BUTTON_SLOPE
    {&mat_b_btn_troll    , TILE_TYPE_TROLL    ,CMM_PM_TILE , &txt_btn_8    , NULL         }, //CMM_BUTTON_TROLL
    {&mat_b_btn_star     , OBJECT_TYPE_STAR   ,CMM_PM_OBJ  , &txt_btn_9    , NULL         }, //CMM_BUTTON_STAR
    {&mat_b_btn_goomba   , OBJECT_TYPE_GOOMBA ,CMM_PM_OBJ  , &txt_btn_10   , NULL         }, //CMM_BUTTON_GOOMBA
    {&mat_b_btn_coin     , OBJECT_TYPE_COIN   ,CMM_PM_OBJ  , &txt_btn_11   , NULL         }, //CMM_BUTTON_COIN
    {&mat_b_btn_blank    , TILE_TYPE_BLOCK    ,CMM_PM_TILE , &txt_btn_12   , NULL         }, //CMM_BUTTON_BLANK
    {&mat_b_btn_greencoin, OBJECT_TYPE_GCOIN  ,CMM_PM_OBJ  , &txt_btn_15   , NULL         }, //CMM_BUTTON_GCOIN
    {&mat_b_btn_corner   , TILE_TYPE_CORNER   ,CMM_PM_TILE , &txt_btn_16   , NULL         }, //CMM_BUTTON_CORNER
    {&mat_b_btn_icorner  , TILE_TYPE_ICORNER  ,CMM_PM_TILE , &txt_btn_17   , NULL         }, //CMM_BUTTON_ICORNER
    {&mat_b_btn_redcoin  , OBJECT_TYPE_RCOIN  ,CMM_PM_OBJ  , &txt_btn_18   , NULL         }, //CMM_BUTTON_RCOIN
    {&mat_b_btn_bluecoin , OBJECT_TYPE_BCOIN  ,CMM_PM_OBJ  , &txt_btn_19   , NULL         }, //CMM_BUTTON_BCOIN
    {&mat_b_btn_bcs      , OBJECT_TYPE_BCS    ,CMM_PM_OBJ  , &txt_btn_20   , NULL         }, //CMM_BUTTON_BCS
    {&mat_b_btn_rcs      , OBJECT_TYPE_RCS    ,CMM_PM_OBJ  , &txt_btn_21   , NULL         }, //CMM_BUTTON_RCS
    {&mat_b_btn_noteblock, OBJECT_TYPE_NOTE   ,CMM_PM_OBJ  , &txt_btn_22   , NULL         }, //CMM_BUTTON_NOTEBLOCK
    {&mat_b_btn_cull     , TILE_TYPE_CULL     ,CMM_PM_TILE , &txt_btn_23   , NULL         }, //CMM_BUTTON_CULL
    {&mat_b_btn_podoboo  , OBJECT_TYPE_PODOB  ,CMM_PM_OBJ  , &txt_btn_25   , NULL         }, //CMM_BUTTON_PODOBOO
    {&mat_b_btn_rex      , OBJECT_TYPE_REX    ,CMM_PM_OBJ  , &txt_btn_26   , NULL         }, //CMM_BUTTON_REX
    {&mat_b_btn_bully    , OBJECT_TYPE_BULLY  ,CMM_PM_OBJ  , &txt_btn_27   , NULL         }, //CMM_BUTTON_BULLY
    {&mat_b_btn_bobomb   , OBJECT_TYPE_BOMB   ,CMM_PM_OBJ  , &txt_btn_28   , NULL         }, //CMM_BUTTON_BOMB
    {&mat_b_btn_tree     , OBJECT_TYPE_TREE   ,CMM_PM_OBJ  , &txt_btn_29   , txt_bp_tree  }, //CMM_BUTTON_TREE
    {&mat_b_btn_excla    , OBJECT_TYPE_EXCLA  ,CMM_PM_OBJ  , &txt_btn_30   , txt_bp_box   }, //CMM_BUTTON_EXCLA
    {&mat_b_btn_downslope, TILE_TYPE_DSLOPE   ,CMM_PM_TILE , &txt_btn_31   , NULL         }, //CMM_BUTTON_DSLOPE
    {&mat_b_btn_chuckya  , OBJECT_TYPE_CHUCKYA,CMM_PM_OBJ  , &txt_btn_32   , NULL         }, //CMM_BUTTON_CHUCKYA
    {&mat_b_btn_spawn    , OBJECT_TYPE_SPAWN  ,CMM_PM_OBJ  , &txt_btn_33   , NULL         }, //CMM_BUTTON_SPAWN
    {&mat_b_btn_phantasm , OBJECT_TYPE_PHNTSM ,CMM_PM_OBJ  , &txt_btn_34   , NULL         }, //CMM_BUTTON_PHANTASM
    {&mat_b_btn_pipe     , OBJECT_TYPE_PIPE   ,CMM_PM_OBJ  , &txt_btn_35   , txt_pipe     }, //CMM_BUTTON_PIPE
    {&mat_b_btn_badge    , OBJECT_TYPE_BADGE  ,CMM_PM_OBJ  , &txt_btn_36   , badgenames   }, //CMM_BUTTON_BADGE
    {&mat_b_btn_water    , TILE_TYPE_WATER    ,CMM_PM_TILE , &txt_btn_37   , NULL         }, //CMM_BUTTON_WATER
    {&mat_b_btn_fence    , TILE_TYPE_FENCE    ,CMM_PM_TILE , &txt_btn_38   , NULL         }, //CMM_BUTTON_FENCE
    {&mat_b_btn_boss     , OBJECT_TYPE_BOSS   ,CMM_PM_OBJ  , &txt_btn_39   , txt_bp_boss  }, //CMM_BUTTON_BOSS
    {&mat_b_btn_checker  , OBJECT_TYPE_MPLAT  ,CMM_PM_OBJ  , &txt_btn_40   , NULL         }, //CMM_BUTTON_MPLAT
    {&mat_b_btn_bball    , OBJECT_TYPE_BBALL  ,CMM_PM_OBJ  , &txt_btn_41   , NULL         }, //CMM_BUTTON_BBALL
    {&mat_b_btn_kuppa    , OBJECT_TYPE_KTQ    ,CMM_PM_OBJ  , &txt_btn_42   , NULL         }, //CMM_BUTTON_KTQ
    {&mat_b_btn_sideslope, TILE_TYPE_SSLOPE   ,CMM_PM_TILE , &txt_btn_43   , NULL         }, //CMM_BUTTON_SSLOPE
};

u8 txt_ls_costume[] = {TXT_LS_COSTUME};
u8 txt_ls_music[] = {TXT_LS_MUSIC};
u8 txt_ls_envfx[] = {TXT_LS_ENVFX};
u8 txt_ls_theme[] = {TXT_LS_THEME};
u8 txt_ls_bg[] = {TXT_LS_BG};
u8 txt_ls_plane[] = {TXT_LS_PLANE};

struct cmm_settings_button cmm_settings_buttons[] = {
    {&txt_ls_costume, &cmm_lopt_costume, &costume_text, 15},
    {&txt_ls_music, &cmm_lopt_seq, &musicmenu_titles, 29},
    {&txt_ls_envfx, &cmm_lopt_envfx, &cmm_envfx_string_table, 6},
    {&txt_ls_theme, &cmm_lopt_theme, &cmm_theme_string_table, NUM_THEMES},
    {&txt_ls_bg, &cmm_lopt_bg, &cmm_bg_string_table, 8},
    {&txt_ls_plane, &cmm_lopt_plane, &cmm_plane_string_table, 3},
};

#define SETTINGS_SIZE sizeof(cmm_settings_buttons)/sizeof(cmm_settings_buttons[0])