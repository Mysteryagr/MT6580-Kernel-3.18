#ifdef IAP_PORTION
const u8 huaruichuan_fw[]=
{

#include "S5030_Hua_Printing_Ver_C40f.i"

};
const u8 huaruichuan_fw2[]=
{
#include "S5030_Hua_Photo_Ver_C415.i"
};
const u8 dj_fw[]=
{
#include "Tinno3702_DJN_V4515.i"
};
struct vendor_map
{
	int vendor_id;
	char vendor_name[30];
	uint8_t* fw_array;
};
const struct vendor_map g_vendor_map[]=
{
	{0x18F4,"HUARUIC",huaruichuan_fw},
	{0x5030,"HUARUIC",huaruichuan_fw},
	{0x18F5,"HUARUIC2",huaruichuan_fw2},	
	{0x2AE7,"DIJING",dj_fw} //0x2AE7
};

#endif/*IAP_PORTION*/
