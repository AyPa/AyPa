
// digits 0123456789ABCDEF 4x8
static uint8_t Dig[] PROGMEM ={
	0x7F, 0x41, 0x7F,  // 0
	0x41, 0x7F, 0x40,   // 1
        0x79,0x49,0x4F, //2
        0x49,0x49,0x7F, //3
        0x0F,0x08,0x7F, //4
        0x4F,0x49,0x79, //5
        0x7F,0x49,0x79, //6
        0x01,0x01,0x7F, //7
        0x7F,0x49,0x7F, //8
        0x4F,0x49,0x7F, //9
        0x7F,0x09,0x7F, //A
        0x7F,0x49,0x36, //B
        0x7F,0x41,0x41, //C
        0x7F,0x41,0x3E, //D
        0x7F,0x49,0x49, //E
        0x7F,0x09,0x09 //F
};

// 6x8
static uint8_t Rus[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00,// (space) 0x20
	0x00, 0x00, 0x5F, 0x00, 0x00,// !	0x21
	0x00, 0x07, 0x00, 0x07, 0x00,// "	0x22
	0x14, 0x7F, 0x14, 0x7F, 0x14,// #	0x23
	0x24, 0x2A, 0x7F, 0x2A, 0x12,// $	0x24
	0x23, 0x13, 0x08, 0x64, 0x62,// %	0x25
	0x36, 0x49, 0x55, 0x22, 0x50,// &	0x26
	0x00, 0x05, 0x03, 0x00, 0x00,// '	0x27
	0x00, 0x1C, 0x22, 0x41, 0x00,// (	0x28
	0x00, 0x41, 0x22, 0x1C, 0x00,// )	0x29
	0x08, 0x2A, 0x1C, 0x2A, 0x08,// *	0x2A
	0x08, 0x08, 0x3E, 0x08, 0x08,// +	0x2B
	0x00, 0x50, 0x30, 0x00, 0x00,// ,	0x2C
	0x08, 0x08, 0x08, 0x08, 0x08,// -	0x2D
	0x00, 0x60, 0x60, 0x00, 0x00,// .	0x2E
	0x20, 0x10, 0x08, 0x04, 0x02,// /	0x2F
	0x3E, 0x51, 0x49, 0x45, 0x3E,// 	0x30
	0x00, 0x42, 0x7F, 0x40, 0x00,// 1	0x31
	0x42, 0x61, 0x51, 0x49, 0x46,// 2	0x32
	0x21, 0x41, 0x45, 0x4B, 0x31,// 3	0x33
	0x18, 0x14, 0x12, 0x7F, 0x10,// 4	0x34
	0x27, 0x45, 0x45, 0x45, 0x39,// 5	0x35
	0x3C, 0x4A, 0x49, 0x49, 0x30,// 6	0x36
	0x01, 0x71, 0x09, 0x05, 0x03,// 7	0x37
	0x36, 0x49, 0x49, 0x49, 0x36,// 8	0x38
	0x06, 0x49, 0x49, 0x29, 0x1E,// 9	0x39
	0x00, 0x36, 0x36, 0x00, 0x00,// :	0x3A
	0x00, 0x56, 0x36, 0x00, 0x00,// ;	0x3B
	0x00, 0x08, 0x14, 0x22, 0x41,// <	0x3C
	0x14, 0x14, 0x14, 0x14, 0x14,// =	0x3D
	0x41, 0x22, 0x14, 0x08, 0x00,// >	0x3E
	0x02, 0x01, 0x51, 0x09, 0x06,// ?	0x3F
	0x32, 0x49, 0x79, 0x41, 0x3E,// @	0x40
	0x7E, 0x11, 0x11, 0x11, 0x7E,// A	0x41
	0x7F, 0x49, 0x49, 0x49, 0x36,// B	0x42
	0x3E, 0x41, 0x41, 0x41, 0x22,// C	0x43
	0x7F, 0x41, 0x41, 0x22, 0x1C,// D	0x44
	0x7F, 0x49, 0x49, 0x49, 0x41,// E	0x45
	0x7F, 0x09, 0x09, 0x01, 0x01,// F	0x46
	0x3E, 0x41, 0x41, 0x51, 0x32,// G	0x47
	0x7F, 0x08, 0x08, 0x08, 0x7F,// H	0x48
	0x00, 0x41, 0x7F, 0x41, 0x00,// I	0x49
	0x20, 0x40, 0x41, 0x3F, 0x01,// J	0x4A
	0x7F, 0x08, 0x14, 0x22, 0x41,// K	0x4B
	0x7F, 0x40, 0x40, 0x40, 0x40,// L	0x4C
	0x7F, 0x02, 0x04, 0x02, 0x7F,// M	0x4D
	0x7F, 0x04, 0x08, 0x10, 0x7F,// N	0x4E
	0x3E, 0x41, 0x41, 0x41, 0x3E,// O	0x4F
	0x7F, 0x09, 0x09, 0x09, 0x06,// P	0x50
	0x3E, 0x41, 0x51, 0x21, 0x5E,// Q	0x51
	0x7F, 0x09, 0x19, 0x29, 0x46,// R	0x52
	0x46, 0x49, 0x49, 0x49, 0x31,// S	0x53
	0x01, 0x01, 0x7F, 0x01, 0x01,// T	0x54
	0x3F, 0x40, 0x40, 0x40, 0x3F,// U	0x55
	0x1F, 0x20, 0x40, 0x20, 0x1F,// V	0x56
	0x7F, 0x20, 0x18, 0x20, 0x7F,// W	0x57
	0x63, 0x14, 0x08, 0x14, 0x63,// X	0x58
	0x03, 0x04, 0x78, 0x04, 0x03,// Y	0x59
	0x61, 0x51, 0x49, 0x45, 0x43,// Z	0x5A
	0x00, 0x00, 0x7F, 0x41, 0x41,// [	0x5B
	0x02, 0x04, 0x08, 0x10, 0x20,// "\"	0x5C
	0x41, 0x41, 0x7F, 0x00, 0x00,// ]	0x5D
	0x04, 0x02, 0x01, 0x02, 0x04,// ^	0x5E
	0x40, 0x40, 0x40, 0x40, 0x40,// _	0x5F
	0x00, 0x01, 0x02, 0x04, 0x00,// `	0x60
	0x20, 0x54, 0x54, 0x54, 0x78,// a	0x61
	0x7F, 0x48, 0x44, 0x44, 0x38,// b	0x62
	0x38, 0x44, 0x44, 0x44, 0x20,// c	0x63
	0x38, 0x44, 0x44, 0x48, 0x7F,// d	0x64
	0x38, 0x54, 0x54, 0x54, 0x18,// e	0x65
	0x08, 0x7E, 0x09, 0x01, 0x02,// f	0x66
	0x08, 0x14, 0x54, 0x54, 0x3C,// g	0x67
	0x7F, 0x08, 0x04, 0x04, 0x78,// h	0x68
	0x00, 0x44, 0x7D, 0x40, 0x00,// i	0x69
	0x20, 0x40, 0x44, 0x3D, 0x00,// j	0x6A
	0x00, 0x7F, 0x10, 0x28, 0x44,// k	0x6B
	0x00, 0x41, 0x7F, 0x40, 0x00,// l	0x6C
	0x7C, 0x04, 0x18, 0x04, 0x78,// m	0x6D
	0x7C, 0x08, 0x04, 0x04, 0x78,// n	0x6E
	0x38, 0x44, 0x44, 0x44, 0x38,// o	0x6F
	0x7C, 0x14, 0x14, 0x14, 0x08,// p	0x70
	0x08, 0x14, 0x14, 0x18, 0x7C,// q	0x71
	0x7C, 0x08, 0x04, 0x04, 0x08,// r	0x72
	0x48, 0x54, 0x54, 0x54, 0x20,// s	0x73
	0x04, 0x3F, 0x44, 0x40, 0x20,// t	0x74
	0x3C, 0x40, 0x40, 0x20, 0x7C,// u	0x75
	0x1C, 0x20, 0x40, 0x20, 0x1C,// v	0x76
	0x3C, 0x40, 0x30, 0x40, 0x3C,// w	0x77
	0x44, 0x28, 0x10, 0x28, 0x44,// x	0x78
	0x0C, 0x50, 0x50, 0x50, 0x3C,// y	0x79
	0x44, 0x64, 0x54, 0x4C, 0x44,// z	0x7A
	0x00, 0x08, 0x36, 0x41, 0x00,// {	0x7B
	0x00, 0x00, 0x7F, 0x00, 0x00,// |	0x7C
	0x00, 0x41, 0x36, 0x08, 0x00,// }	0x7D
	0x08, 0x08, 0x2A, 0x1C, 0x08,// ->	0x7E
	0x08, 0x1C, 0x2A, 0x08, 0x08, // <-	0x7F

// Cirillic blocks 

	0x7C, 0x14, 0x14, 0x14, 0x08,//p 	0x80
	0x38, 0x44, 0x44, 0x44, 0x20,//c	0x81
	0x04, 0x04, 0x7c, 0x04, 0x04,//ò	0x82
	0x0C, 0x50, 0x50, 0x50, 0x3C,//ó	0x83
	0x30, 0x48, 0xfc, 0x48, 0x30,//ô	0x84
	0x44, 0x28, 0x10, 0x28, 0x44,//x	0x85
	0x7c, 0x40, 0x40, 0x40, 0xfc,//ö	0x86
	0x0c, 0x10, 0x10, 0x10, 0x7c,//÷	0x87
	0x7c, 0x40, 0x7c, 0x40, 0x7c,//ø	0x88
	0x7c, 0x40, 0x7c, 0x40, 0xfc,//ù	0x89
	0x04, 0x7c, 0x50, 0x50, 0x20,//ú	0x8A
	0x7c, 0x50, 0x50, 0x20, 0x7c,//û	0x8B
	0x7c, 0x50, 0x50, 0x20, 0x00,//ü	0x8C
	0x28, 0x44, 0x54, 0x54, 0x38,//ý	0x8D
	0x7c, 0x10, 0x38, 0x44, 0x38,//þ	0x8E
	0x08, 0x54, 0x34, 0x14, 0x7c,//ÿ	0x8F

// -------------------------------------------------	

	0x7e, 0x11, 0x11, 0x11, 0x7e,//A	0x90
	0x7f, 0x49, 0x49, 0x49, 0x33,//Á	0x91
	0x7f, 0x49, 0x49, 0x49, 0x36,//Â	0x92
	0x7f, 0x01, 0x01, 0x01, 0x03,//Ã	0x93
	0xe0, 0x51, 0x4f, 0x41, 0xff,//Ä	0x94
	0x7f, 0x49, 0x49, 0x49, 0x41,//E	0x95
	0x77, 0x08, 0x7f, 0x08, 0x77,//Æ	0x96
	0x41, 0x49, 0x49, 0x49, 0x36,//Ç	0x97
	0x7f, 0x10, 0x08, 0x04, 0x7f,//È	0x98
	0x7c, 0x21, 0x12, 0x09, 0x7c,//É	0x99
	0x7f, 0x08, 0x14, 0x22, 0x41,//K	0x9A
	0x20, 0x41, 0x3f, 0x01, 0x7f,//Ë	0x9B
	0x7f, 0x02, 0x0c, 0x02, 0x7f,//M	0x9C
	0x7f, 0x08, 0x08, 0x08, 0x7f,//H	0x9D
	0x3e, 0x41, 0x41, 0x41, 0x3e,//O	0x9E
	0x7f, 0x01, 0x01, 0x01, 0x7f,//Ï	0x9F
	0x7f, 0x09, 0x09, 0x09, 0x06,//P	0xA0
	0x3e, 0x41, 0x41, 0x41, 0x22,//C	0xA1
	0x01, 0x01, 0x7f, 0x01, 0x01,//T	0xA2
	0x47, 0x28, 0x10, 0x08, 0x07,//Ó	0xA3
	0x1c, 0x22, 0x7f, 0x22, 0x1c,//Ô	0xA4
	0x63, 0x14, 0x08, 0x14, 0x63,//X	0xA5
	0x7f, 0x40, 0x40, 0x40, 0xff,//Ö	0xA6
	0x07, 0x08, 0x08, 0x08, 0x7f,//×	0xA7
	0x7f, 0x40, 0x7f, 0x40, 0x7f,//Ø	0xA8
	0x7f, 0x40, 0x7f, 0x40, 0xff,//Ù	0xA9
	0x01, 0x7f, 0x48, 0x48, 0x30,//Ú	0xAA
	0x7f, 0x48, 0x30, 0x00, 0x7f,//Û	0xAB
	0x00, 0x7f, 0x48, 0x48, 0x30,//Ý	0xAC
	0x22, 0x41, 0x49, 0x49, 0x3e,//Ü	0xAD
	0x7f, 0x08, 0x3e, 0x41, 0x3e,//Þ	0xAE
	0x46, 0x29, 0x19, 0x09, 0x7f,//ß	0xAF

// ìàëåíüêèå áóêâû 

 	0x20, 0x54, 0x54, 0x54, 0x78,//a	0xB0
	0x3c, 0x4a, 0x4a, 0x49, 0x31,//á	0xB1
	0x7c, 0x54, 0x54, 0x28, 0x00,//â	0xB2
	0x7c, 0x04, 0x04, 0x04, 0x0c,//ã	0xB3
	0xe0, 0x54, 0x4c, 0x44, 0xfc,//ä	0xB4
	0x38, 0x54, 0x54, 0x54, 0x18,//e	0xB5
	0x6c, 0x10, 0x7c, 0x10, 0x6c,//æ	0xB6
	0x44, 0x44, 0x54, 0x54, 0x28,//ç	0xB7
	0x7c, 0x20, 0x10, 0x08, 0x7c,//è	0xB8
	0x7c, 0x41, 0x22, 0x11, 0x7c,//é	0xB9
	0x7c, 0x10, 0x28, 0x44, 0x00,//ê	0xBA
	0x20, 0x44, 0x3c, 0x04, 0x7c,//ë	0xBB
	0x7c, 0x08, 0x10, 0x08, 0x7c,//ì	0xBC
	0x7c, 0x10, 0x10, 0x10, 0x7c,//í	0xBD
	0x38, 0x44, 0x44, 0x44, 0x38,//o	0xBE
	0x7c, 0x04, 0x04, 0x04, 0x7c //ï	0xBF
};
