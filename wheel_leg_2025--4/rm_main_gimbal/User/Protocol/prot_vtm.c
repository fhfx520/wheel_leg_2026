#include "prot_vtm.h"
#include "crc.h"
#include "string.h"
vtm_t vtm;

/*
 * @brief     图传遥控数据接收函数
 * @param[in] data: 数据指针
 */
void vtm_get_data(uint8_t *data)
{
	//帧头校验
	if(data[0] != 0xA9 || data[1] != 0x53)
		return;
	//帧尾CRC16整包校验
//	if(!crc16_verify_checksum(data,VTM_DATA_LEN))
//		return;
	//数据拷贝
	memcpy(&vtm.vtm_data,data+2,VTM_DATA_LEN-4);
	//标记在线
	vtm.online = 1;
}

uint8_t vtm_check_offline(void)
{
    if (vtm.online == 0) {
        return 1;
    } else {
        vtm.online = 0;
        return 0;
    }
}
