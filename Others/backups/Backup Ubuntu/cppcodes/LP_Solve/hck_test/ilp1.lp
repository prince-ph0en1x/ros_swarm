
/* PLEASE DO NOT EDIT THIS FILE */
/* Auto-generated by check_mapp.cpp for lp_solve */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~ Hopcroft Karp Algorithm ILP formulation ~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

min: mksp;

5562 x00 + 8865 x01 + 8112 x02 + 8144 x03 + 2037 x04 + 5819 x05 < mksp;

6276 x10 + 8031 x11 + 4640 x12 + 5746 x13 + 9632 x14 + 5582 x15 < mksp;

6541 x20 + 6326 x21 + 4897 x22 + 2777 x23 + 5447 x24 + 6824 x25 < mksp;

3947 x30 + 4309 x31 + 557 x32 + 8 x33 + 6024 x34 + 7669 x35 < mksp;

9317 x40 + 699 x41 + 2765 x42 + 4633 x43 + 843 x44 + 6573 x45 < mksp;

6281 x50 + 6406 x51 + 5440 x52 + 4394 x53 + 4551 x54 + 7477 x55 < mksp;

x00 + x10 + x20 + x30 + x40 + x50 = 1;

x01 + x11 + x21 + x31 + x41 + x51 = 1;

x02 + x12 + x22 + x32 + x42 + x52 = 1;

x03 + x13 + x23 + x33 + x43 + x53 = 1;

x04 + x14 + x24 + x34 + x44 + x54 = 1;

x05 + x15 + x25 + x35 + x45 + x55 = 1;

x00 + x01 + x02 + x03 + x04 + x05 = 1;

x10 + x11 + x12 + x13 + x14 + x15 = 1;

x20 + x21 + x22 + x23 + x24 + x25 = 1;

x30 + x31 + x32 + x33 + x34 + x35 = 1;

x40 + x41 + x42 + x43 + x44 + x45 = 1;

x50 + x51 + x52 + x53 + x54 + x55 = 1;

bin x00, x01, x02, x03, x04, x05;
bin x10, x11, x12, x13, x14, x15;
bin x20, x21, x22, x23, x24, x25;
bin x30, x31, x32, x33, x34, x35;
bin x40, x41, x42, x43, x44, x45;
bin x50, x51, x52, x53, x54, x55;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
