#ifdef IAP_PORTION
const u8 huaruichuan_fw[]=
{
//#include "S4050_hrc_v1208.i"
#include "HUARUIC_EKTF2527_V2500_1206_V1.0_Tpfactory_RES.i"
};
const u8 yijian_fw[]=
{ 
//#include "S4050_hrc_v1208.i"
#include "YJ_TinnoV2500_2AE9_V1203_20151127.i"
};
struct vendor_map
{
	int vendor_id;
	char vendor_name[30];
	uint8_t* fw_array;
};
const struct vendor_map g_vendor_map[]=
{
	{0x2ae0,"HUARUIC",huaruichuan_fw},
	{0x2500,"YIJIAN",yijian_fw},
	{0x2aeb,"HUARUIC",huaruichuan_fw},
	{0x2ae9,"YIJIAN",yijian_fw}

};
#endif/*IAP_PORTION*/
