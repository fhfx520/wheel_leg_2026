#include "ICM42688_driver.h"
#include "stdio.h"
#include "bsp_dwt.h"
#include "ICM42688_Middleware.h"
#include "ICM42688_reg.h"
#include "bsp_imu.h"
#include "data_processing.h"

uint8_t reg_val;
uint8_t init_flag;
IMU_Data_t IMU_Data;
   
int16_t temp;


uint32_t GetRaw_DWT_Count;
extern GyroFilter gf;

float accSensitivity ;// 加速计灵敏度
float gyroSensitivity ;// 角速度计灵敏度
float accel_scale;
float gyro_scale;

static void ICM42688_write_single_reg(uint8_t reg, uint8_t data);
static void ICM42688_read_single_reg(uint8_t reg, uint8_t *return_data);
static void ICM42688_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);



#define ICM42688DelayMs(ms) HAL_Delay(ms);

#define ICM_SPI_CS_LOW() HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_RESET)

#define ICM_SPI_CS_HIGH() HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET)

#define ICM42688_WRITE_SINGLE_REG(reg, data)      \
    {                                             \
        ICM_SPI_CS_LOW();                         \
        ICM42688_write_single_reg((reg), (data)); \
        ICM_SPI_CS_HIGH();                        \
    }

#define ICM42688_READ_SINGLE_REG(reg,data)      \
    {                                            \
        ICM_SPI_CS_LOW();                        \
        ICM42688_read_single_reg(reg,&data);      \
        ICM_SPI_CS_HIGH();                       \
    }

#define ICM42688_READ_MULI_REG(reg, data, len) \
    {                                          \
    ICM_SPI_CS_LOW();                          \
    ICM42688_read_write_byte((reg) | 0x80);    \
    ICM42688_read_muli_reg(reg, data , len);    \
    ICM_SPI_CS_HIGH();                          \
    }

int16_t ICM42688_init(void)
{
//    uint8_t reg_val = 0;
    /* 读取 who am i 寄存器 */
    ICM42688_READ_SINGLE_REG(ICM42688_WHO_AM_I, reg_val);
    ICM42688_READ_MULI_REG(ICM42688_WHO_AM_I,&reg_val,1);
    ICM42688_READ_SINGLE_REG(ICM42688_WHO_AM_I, reg_val);
    // printf("reg_val:%d\n",reg_val);
    ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0);    // 设置bank 0区域寄存器
    ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x01); // 软复位传感器
    //DWT_Delay(0.01);
    ICM42688DelayMs(100);

    if (reg_val == ICM42688_ID)
    {

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 1);    // 设置bank 1区域寄存器
        ICM42688_WRITE_SINGLE_REG(ICM42688_INTF_CONFIG4, 0x02); // 设置为4线SPI通信

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0);   // 设置bank 0区域寄存器
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG, 0x40); // Stream-to-FIFO Mode(page63)

        ICM42688_READ_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_SOURCE0, 0x00);
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG2, 0x00); // watermark
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG3, 0x02); // watermark
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);
        ICM42688_WRITE_SINGLE_REG(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_CONFIG, 0x36);

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);
        reg_val |= (1 << 2); // FIFO_THS_INT1_ENABLE
        ICM42688_WRITE_SINGLE_REG(ICM42688_INT_SOURCE0, reg_val);

        bsp_Icm42688GetAres(AFS_16G);
        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_ACCEL_CONFIG0, reg_val); // page74
        reg_val |= (AFS_16G << 5);                                  // 量程 ±16g
        reg_val |= (AODR_1000Hz);                                    // 输出速率 1000HZ
        ICM42688_WRITE_SINGLE_REG(ICM42688_ACCEL_CONFIG0, reg_val);

        bsp_Icm42688GetGres(GFS_2000DPS);
        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_GYRO_CONFIG0, reg_val); // page73
        reg_val |= (GFS_2000DPS << 5);                            // 量程 ±2000dps
        reg_val |= (AODR_1000Hz);                                 // 输出速率 1000HZ
        ICM42688_WRITE_SINGLE_REG(ICM42688_GYRO_CONFIG0, reg_val);
				
									
				/*****抗混叠滤波器@536Hz*****/
					
					/*GYRO抗混叠滤波器配置*/
					/*指定Bank1*/
					ICM42688_WRITE_SINGLE_REG(0x76,0x01);
					/*GYRO抗混叠滤波器配置*/
					ICM42688_WRITE_SINGLE_REG(0x0B,0xA0);//开启抗混叠和陷波滤波器
					ICM42688_WRITE_SINGLE_REG(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
					ICM42688_WRITE_SINGLE_REG(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
					ICM42688_WRITE_SINGLE_REG(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
					
					/*ACCEL抗混叠滤波器配置*/
					/*指定Bank2*/
				ICM42688_WRITE_SINGLE_REG(0x76,0x02);
					/*ACCEL抗混叠滤波器配置*/
				ICM42688_WRITE_SINGLE_REG(0x03,0x18);//开启滤波器 ACCEL_AFF_DELT 12 (default 24)
				ICM42688_WRITE_SINGLE_REG(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
				ICM42688_WRITE_SINGLE_REG(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

				/*****自定义滤波器1号@111Hz*****/

					/*指定Bank0*/
				ICM42688_WRITE_SINGLE_REG(0x76,0x00);
					/*滤波器顺序*/
				ICM42688_WRITE_SINGLE_REG(0x51,0x12);//GYRO滤波器1st
				ICM42688_WRITE_SINGLE_REG(0x53,0x05);//ACCEL滤波器1st
					/*滤波器设置*/
				ICM42688_WRITE_SINGLE_REG(0x52,0x33);//111Hz 03
					
	

        ICM42688_WRITE_SINGLE_REG(ICM42688_REG_BANK_SEL, 0x00);
        ICM42688_READ_SINGLE_REG(ICM42688_PWR_MGMT0, reg_val); // 读取PWR—MGMT0当前寄存器的值(page72)
        reg_val &= ~(1 << 5);                                  // 使能温度测量
        reg_val |= ((3) << 2);                                 // 设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);                                        // 设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        ICM42688_WRITE_SINGLE_REG(ICM42688_PWR_MGMT0, reg_val);
        
        ICM42688DelayMs(1); // 操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作
        
        init_flag = 1;
        return 1;
    }
		else
			
    return 0;
}
float dt_raw;
/*******************************************************************************
* 名    称： bsp_IcmGetRawData
* 功    能： 读取Icm42688加速度陀螺仪数据
* 入口参数： 六轴
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62,63
*******************************************************************************/
void bsp_IcmGetRawData(IMU_Data_t *ICM42688)
{

   
	
		uint8_t buffer1[6] = {0};
		uint8_t buffer2[6] = {0}; 
    ICM42688_READ_MULI_REG(ICM42688_ACCEL_DATA_X1, buffer1, 6);
    ICM42688_READ_MULI_REG(ICM42688_GYRO_DATA_X1, buffer2, 6);
    
    ICM42688->Accel_raw[0] =   (int16_t)((buffer1[0] << 8)  | buffer1[1]);
    ICM42688->Accel_raw[1]  = (int16_t)((buffer1[2] << 8)  | buffer1[3]);
    ICM42688->Accel_raw[2]  = (int16_t)((buffer1[4] << 8)  | buffer1[5]);		
   
    ICM42688->Gyro_raw[0]   = (int16_t)((buffer2[0] << 8)  | buffer2[1]);
    ICM42688->Gyro_raw[1]  =  (int16_t)((buffer2[2] << 8)  | buffer2[3]);
    ICM42688->Gyro_raw[2]  =  (int16_t)((buffer2[4] << 8)  | buffer2[5]);
		
		if(califlag)
		{
			
		ICM42688->Gyro_raw[0] = ICM42688->Gyro_raw[0] - IMU_Data.GyroOffset[0];
		ICM42688->Gyro_raw[1] = ICM42688->Gyro_raw[1] - IMU_Data.GyroOffset[1];
		ICM42688->Gyro_raw[2] = ICM42688->Gyro_raw[2] - IMU_Data.GyroOffset[2];

		}

}

/**
 * @brief 将原始数据转换为物理量
 * @param raw 原始数据
 * @param data 转换后的物理量数据
 * @param accel_range 加速度计量程
 * @param gyro_range 陀螺仪量程
 */

void ICM42688P_ConvertToPhysical(IMU_Data_t *ICM42688) {
    // 加速度转换（单位：m/s²）
		
		accel_scale = 9.81f / accSensitivity;
    ICM42688->Accel[0] = ICM42688->Accel_raw[0] * accel_scale;
    ICM42688->Accel[1]=  ICM42688->Accel_raw[1] * accel_scale;
    ICM42688->Accel[2] = ICM42688->Accel_raw[2] * accel_scale;

    // 陀螺仪转换（单位：rad/s）
     gyro_scale = (1.0f / gyroSensitivity) * (PI / 180.0f);
	  ICM42688->Gyro[0] = ICM42688->Gyro_raw[0] * gyro_scale;
    ICM42688->Gyro[1] = ICM42688->Gyro_raw[1] * gyro_scale;
    ICM42688->Gyro[2] = ICM42688->Gyro_raw[2] * gyro_scale;


}

/*******************************************************************************
* 名    称： bsp_IcmGetTemperature
* 功    能： 读取Icm42688 内部传感器温度
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    ICM42688_READ_MULI_REG(ICM42688_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

static void ICM42688_write_single_reg(uint8_t reg, uint8_t data)
{
    ICM42688_read_write_byte(reg);
    ICM42688_read_write_byte(data);
}

/*******************************************************************************
 * 名    称： ICM42688_read_single_reg
 * 功    能： 读取单个寄存器的值
 * 入口参数： reg: 寄存器地址
 * 出口参数： 当前寄存器地址的值
 * 作　　者： zgh
 * 创建日期： 2024-10
 * 修    改：
 * 修改日期：
 * 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
 *******************************************************************************/
static void ICM42688_read_single_reg(uint8_t reg, uint8_t *return_data)
{
     uint8_t regval = 0xff;
	 ICM42688_read_write_byte(reg | 0x80);//表示读取，最高位为读写位
     *return_data = ICM42688_read_write_byte(regval);
}

/*******************************************************************************
 * 名    称： ICM42688_read_single_regs
 * 功    能： 连续读取多个寄存器的值
 * 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
 * 出口参数： 无
 * 作　　者： zgh
 * 创建日期： 2024-10
 * 修    改：
 * 修改日期：
 * 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
 *******************************************************************************/
static void ICM42688_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;
//    ICM42688_read_write_byte(reg | 0x80);
    for (i = 0; i < len; i++)
    {

        *buf = ICM42688_read_write_byte(*buf);
        buf++;
    }
}

float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch (Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 16384.0f;
        break;
    case AFS_4G:
        accSensitivity = 8192.0f;
        break;
    case AFS_8G:
        accSensitivity = 4096.0f;
        break;
    case AFS_16G:
        accSensitivity = 2048.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch (Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 131.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 131.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 65.5f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 32.8f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 16.4f;
	
        break;
    }
    return gyroSensitivity;
}
