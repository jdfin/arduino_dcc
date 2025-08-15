#pragma once

namespace DccCv
{

const int address = 1;
const int acceleration = 3;
const int deceleration = 4;
const int mfg_id = 8;
const int address_hi = 17;
const int address_lo = 18;
const int config = 29;

const int index_hi = 31;
const int index_lo = 32;

const int master_volume = 63; // Loksound 5

const int ext_config_2 = 124;

// Indexed: cv31=16, cv32=1
const int prime_vol = 259;
const int horn_vol = 275;
const int bell_vol = 283;
const int clank_vol = 291;
const int squeal_vol = 435;

};

// SP2265
//
// CV  CV
// 31  32    CV  Def  Description               Slot
// --  --   ---  ---  ------------------------  ----  --------
//            1    3  Address (short)
//            3   20  Acceleration
//            4   20  Deceleration
//           17  192  Address hi (1100_0000)
//           18  128  Address lo (1000_0000)
//                    (default long address 128)
//           29   14  Config (0000_1110)
//           31   16  Index hi
//           32    0  Index lo
//           63  128  Master volume                   verified
// 16   1   259  192  Prime volume                1   verified
// 16   1   275  128  Horn volume                 3   verified
// 16   1   283   60  Bell volume                 4   verified
// 16   1   291   40  Coupler clank volume        5   verified
// 16   1   435   50  Flange squeal              23   verified
