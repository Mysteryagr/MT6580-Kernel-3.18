#ifdef IAP_PORTION
const u8 huaruichuan_fw[]=
{
//#include "v2800_Hua_photo_Ver_V2800_V4502.i"
#include "v2800_Hua_photo_Ver_V2800_VC508.i"
//#include "v2800_Hua_photo_Ver_V2800_VC507.i"
};
const u8 huaruichuan_fw2[]=
{

//#include "S5030_Hua_Photo_Ver_C403.i"
//#include "S5030_Hua_Photo_Ver_C415.i"
};
const u8 dj_fw[]=
{

//#include "S5030_DJ_Photo_Ver_C404.i"
//#include "S5030_DJ_Photo_Ver_C414.i"
};
struct vendor_map
{
	int vendor_id;
	char vendor_name[30];
	uint8_t* fw_array;
};
const struct vendor_map g_vendor_map[]=
{
	//{0x18F4,"HUARUIC",huaruichuan_fw},
	//{0x5030,"HUARUIC",huaruichuan_fw},
	//{0x18F5,"HUARUIC2",huaruichuan_fw2},	
	//{0x2AE5,"DIJING",dj_fw},
             {0x2AE8,"HUARUIC",huaruichuan_fw}
};

#endif/*IAP_PORTION*/
