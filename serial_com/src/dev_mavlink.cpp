#include "dev_mavlink.h"

#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

_Debug debug;
RX rx_;

void _MAVLINK::Data_Receive_Anl(u8 *data_buf, u8 num)
{
    u8 sum = 0;

    for (u8 i = 0; i < (num - 1); i++)
	sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1)))
	return;
    if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAA))
	return;
    /////////////////////////////////////////////////////////////////////////////////////
    if (*(data_buf + 2) == 0X01) //传感器
    {
	rx_.attitude[x] = (float)(((s16)(*(data_buf + 4) << 8) | *(data_buf + 5)) / 100.f);
	rx_.attitude[y] = (float)(((s16)(*(data_buf + 6) << 8) | *(data_buf + 7)) / 100.f);
	rx_.attitude[z] = (float)(((s16)(*(data_buf + 8) << 8) | *(data_buf + 9)) / 100.f);

	std::cout << "姿态x轴" << rx_.attitude[x] << std::endl;
	std::cout << "姿态y轴" << rx_.attitude[y] << std::endl;
	std::cout << "姿态z轴" << rx_.attitude[z] << std::endl;
	std::cout << "" << std::endl;
    }
    ////传感器
    if (*(data_buf + 2) == 0X02)
    {
	rx_.acc[x] = (s16)(*(data_buf + 4) << 8) | *(data_buf + 5);
	rx_.acc[y] = (s16)(*(data_buf + 6) << 8) | *(data_buf + 7);
	rx_.acc[z] = (s16)(*(data_buf + 8) << 8) | *(data_buf + 9);

	rx_.gyr[x] = (s16)(*(data_buf + 10) << 8) | *(data_buf + 11);
	rx_.gyr[y] = (s16)(*(data_buf + 12) << 8) | *(data_buf + 13);
	rx_.gyr[z] = (s16)(*(data_buf + 14) << 8) | *(data_buf + 15);

	rx_.mag[x] = (s16)(*(data_buf + 16) << 8) | *(data_buf + 17);
	rx_.mag[y] = (s16)(*(data_buf + 18) << 18) | *(data_buf + 19);
	rx_.mag[z] = (s16)(*(data_buf + 20) << 20) | *(data_buf + 21);

	//	 std::cout<<"加速度x轴"<< rx_.acc[x]<<std::endl;
	//	 std::cout<<"加速度y轴"<< rx_.acc[y]<<std::endl;
	//	 std::cout<<"加速度z轴"<< rx_.acc[z]<<std::endl;
	//    std::cout<<""<< std::endl;
	//	 std::cout<<"陀螺仪x轴"<< rx_.gyr[x]<<std::endl;
	//	 std::cout<<"陀螺仪y轴"<< rx_.gyr[y]<<std::endl;
	//	 std::cout<<"陀螺仪z轴"<< rx_.gyr[z]<<std::endl;
	//     std::cout<<""<< std::endl;
	//    std::cout<<"磁罗盘x轴"<< rx_.mag[x]<<std::endl;
	//	 std::cout<<"磁罗盘y轴"<< rx_.mag[y]<<std::endl;
	//	 std::cout<<"磁罗盘z轴"<< rx_.mag[z]<<std::endl;

	std::cout << "" << std::endl;
    }

    if (*(data_buf + 2) == 0X03)
    {
	//		rc.rawData[THROTTLE] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
	//		rc.rawData[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
	//		rc.rawData[ROLL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
	//		rc.rawData[PITCH] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
	//		rc.rawData[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
	//		rc.rawData[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
	//		rc.rawData[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
	//		rc.rawData[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
    }

    if (*(data_buf + 2) == 0X10) //PID1
    {
	Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0X11) //PID2
    {

	Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0X12) //PID3
    {

	Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0X13) //PID4
    {

	Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0X14) //PID5
    {

	Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0X15) //PID6
    {
	Send_Check(*(data_buf + 2), sum);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    if (*(data_buf + 2) == 0x18)
    {
    }
}

void _MAVLINK::Check_Event(void)
{

#ifdef ANO_DT_USE_NRF24l01
    nrf.Check_Event();
#endif
}

void _MAVLINK::Data_Exchange(void)
{
    static u8 cnt = 0;
    static u8 senser_cnt = 10;
    static u8 user_cnt = 10;
    static u8 status_cnt = 15;
    static u8 rcdata_cnt = 20;
    static u8 motopwm_cnt = 20;
    static u8 power_cnt = 50;
    static u8 senser2_cnt = 50;

    if ((cnt % senser_cnt) == (senser_cnt - 1))
	f.send_senser = 1;
    if ((cnt % senser2_cnt) == (senser2_cnt - 1))
	f.send_senser2 = 1;

    if ((cnt % user_cnt) == (user_cnt - 1))
	f.send_user = 1;

    if ((cnt % status_cnt) == (status_cnt - 1))
	f.send_status = 1;

    if ((cnt % rcdata_cnt) == (rcdata_cnt - 1))
	f.send_rcdata = 1;

    if ((cnt % motopwm_cnt) == (motopwm_cnt - 1))
	f.send_motopwm = 1;

    if ((cnt % power_cnt) == (power_cnt - 1))
	f.send_power = 1;

    cnt++;
    /////////////////////////////////////////////////////////////////////////////////////
    if (f.send_version)
    {
	f.send_version = 0;
	Send_Version(1, 300, 110, 400, 0);
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_status)
    {
	f.send_status = 0;
	//Send_Status(imu.angle.x,imu.angle.y,imu.angle.z,0,0,2);
    }
    //		else if(f.send_speed)
    //	{
    //		f.send_speed = 0;
    //		//ANO_DT_Send_Speed(0,0,wz_speed);
    //	}
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_user)
    {
	f.send_user = 0;
	//Send_User();
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_senser)
    {
	f.send_senser = 0;
	//	Send_Senser(imu.Acc_lpf.x,imu.Acc_lpf.y,imu.Acc_lpf.z,\
			//					Mpu6050.gyr.x,	Mpu6050.gyr.y,	Mpu6050.gyr.z,\
			//					Ak8975.mag.x,Ak8975.mag.y,Ak8975.mag.z);
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_senser2)
    {
	f.send_senser2 = 0;
	Send_Senser2(0, 0);
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_rcdata)
    {
	f.send_rcdata = 0;
	//Send_RCData(Rc.g_CH[thr]+500,Rc.g_CH[yaw]+500,Rc.g_CH[rol]+500,Rc.g_CH[pit]+500,
	//		debug.aux1,debug.aux2,debug.aux3,
	//		debug.aux4,debug.aux5,Rc.g_ThrVAL);
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_motopwm)
    {
	f.send_motopwm = 0;
	uint16_t Moto_PWM[4];
	//		motor.getPWM(Moto_PWM);
	//for(u8 i=0;i<4;i++)
	//Moto_PWM[i] -= 1000;

	//Send_MotoPWM(adc.ad_value[0],adc.ad_value[1],adc.ad_value[2],adc.ad_value[3],adc.ad_value[4],adc.ad_value[5],0,0);
	//Send_MotoPWM(adc.ad_fil_value[0],adc.ad_fil_value[0],adc.ad_fil_value[2],adc.ad_fil_value[3],adc.ad_fil_value[4],adc.ad_fil_value[5],0,0);
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_power)
    {
	f.send_power = 0;
	Send_Power(123, 456);
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_pid1)
    {
	f.send_pid1 = 0;
    }
    /////////////////////////////////////////////////////////////////////////////////////
    else if (f.send_pid2)
    {
	f.send_pid2 = 0;
    }
    else if (f.send_pid5)
    {
	f.send_pid5 = 0;
    }

    else if (f.send_pid6)
    {
	f.send_pid6 = 0;
    }

    /////////////////////////////////////////////////////////////////////////////////////
}

void _MAVLINK::Send_Version(u8 hardware_type, u16 hardware_ver, u16 software_ver, u16 protocol_ver, u16 bootloader_ver)
{
    u8 _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x00;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = hardware_type;
    data_to_send[_cnt++] = BYTE1(hardware_ver);
    data_to_send[_cnt++] = BYTE0(hardware_ver);
    data_to_send[_cnt++] = BYTE1(software_ver);
    data_to_send[_cnt++] = BYTE0(software_ver);
    data_to_send[_cnt++] = BYTE1(protocol_ver);
    data_to_send[_cnt++] = BYTE0(protocol_ver);
    data_to_send[_cnt++] = BYTE1(bootloader_ver);
    data_to_send[_cnt++] = BYTE0(bootloader_ver);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}

void _MAVLINK::Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
    u8 _cnt = 0;
    s16 _temp;
    s32 _temp2 = alt;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    _temp = (int)(angle_rol * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_pit * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}

void _MAVLINK::Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z)
{
    u8 _cnt = 0;
    s16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;

    _temp = a_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}
void _MAVLINK::Send_Senser2(s32 alt_bar, u16 alt_csb)
{
    u8 _cnt = 0;
    s32 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x07;
    data_to_send[_cnt++] = 0;

    _temp = alt_bar;
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = alt_csb;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}

void _MAVLINK::Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6)
{
    u8 _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(thr);
    data_to_send[_cnt++] = BYTE0(thr);
    data_to_send[_cnt++] = BYTE1(yaw);
    data_to_send[_cnt++] = BYTE0(yaw);
    data_to_send[_cnt++] = BYTE1(rol);
    data_to_send[_cnt++] = BYTE0(rol);
    data_to_send[_cnt++] = BYTE1(pit);
    data_to_send[_cnt++] = BYTE0(pit);
    data_to_send[_cnt++] = BYTE1(aux1);
    data_to_send[_cnt++] = BYTE0(aux1);
    data_to_send[_cnt++] = BYTE1(aux2);
    data_to_send[_cnt++] = BYTE0(aux2);
    data_to_send[_cnt++] = BYTE1(aux3);
    data_to_send[_cnt++] = BYTE0(aux3);
    data_to_send[_cnt++] = BYTE1(aux4);
    data_to_send[_cnt++] = BYTE0(aux4);
    data_to_send[_cnt++] = BYTE1(aux5);
    data_to_send[_cnt++] = BYTE0(aux5);
    data_to_send[_cnt++] = BYTE1(aux6);
    data_to_send[_cnt++] = BYTE0(aux6);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}
void _MAVLINK::Send_Power(u16 votage, u16 current)
{
    u8 _cnt = 0;
    u16 temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x05;
    data_to_send[_cnt++] = 0;

    temp = votage;
    data_to_send[_cnt++] = BYTE1(temp);
    data_to_send[_cnt++] = BYTE0(temp);
    temp = current;
    data_to_send[_cnt++] = BYTE1(temp);
    data_to_send[_cnt++] = BYTE0(temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}
void _MAVLINK::Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8)
{
    u8 _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x06;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE1(m_1);
    data_to_send[_cnt++] = BYTE0(m_1);
    data_to_send[_cnt++] = BYTE1(m_2);
    data_to_send[_cnt++] = BYTE0(m_2);
    data_to_send[_cnt++] = BYTE1(m_3);
    data_to_send[_cnt++] = BYTE0(m_3);
    data_to_send[_cnt++] = BYTE1(m_4);
    data_to_send[_cnt++] = BYTE0(m_4);
    data_to_send[_cnt++] = BYTE1(m_5);
    data_to_send[_cnt++] = BYTE0(m_5);
    data_to_send[_cnt++] = BYTE1(m_6);
    data_to_send[_cnt++] = BYTE0(m_6);
    data_to_send[_cnt++] = BYTE1(m_7);
    data_to_send[_cnt++] = BYTE0(m_7);
    data_to_send[_cnt++] = BYTE1(m_8);
    data_to_send[_cnt++] = BYTE0(m_8);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}

void _MAVLINK::Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
    u8 _cnt = 0;
    s16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x10 + group - 1;
    data_to_send[_cnt++] = 0;

    _temp = p1_p * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p1_i * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p1_d * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p2_p * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p2_i * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p2_d * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p3_p * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p3_i * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = p3_d * 1000;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    u8 sum = 0;
    for (u8 i = 0; i < _cnt; i++)
	sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    Send_Data(data_to_send, _cnt);
}
void _MAVLINK::Send_Check(u8 head, u8 check_sum)
{
    data_to_send[0] = 0xAA;
    data_to_send[1] = 0xAA;
    data_to_send[2] = 0xEF;
    data_to_send[3] = 2;
    data_to_send[4] = head;
    data_to_send[5] = check_sum;

    u8 sum = 0;
    for (u8 i = 0; i < 6; i++)
	sum += data_to_send[i];
    data_to_send[6] = sum;

    Send_Data(data_to_send, 8);
}

void _MAVLINK::Send_Data(u8 *dataToSend, u8 length)
{

    //Usart.Uart2_Put_Buf(data_to_send,length);
}

void _MAVLINK::read_data(u8 com_data)
{
    static u8 RxState = 0;
    static u8 RxBuffer[50];
    static u8 _data_len = 0, _data_cnt = 0;
    //std::cout<<"进入这儿"<<std::endl;
    if (RxState == 0 && com_data == 0xAA)
    {
	//std::cout<<"第一振正确"<<std::endl;
	RxState = 1;
	RxBuffer[0] = com_data;
    }
    else if (RxState == 1 && com_data == 0xAA)
    {
	//std::cout<<"第二振正确"<<std::endl;
	RxState = 2;
	RxBuffer[1] = com_data;
    }
    else if (RxState == 2 && com_data > 0 && com_data < 0XF1)
    {
	RxState = 3;
	RxBuffer[2] = com_data;
    }
    else if (RxState == 3 && com_data < 50)
    {
	RxState = 4;
	RxBuffer[3] = com_data;
	_data_len = com_data;
	_data_cnt = 0;
    }
    else if (RxState == 4 && _data_len > 0)
    {
	_data_len--;
	RxBuffer[4 + _data_cnt++] = com_data;
	if (_data_len == 0)
	    RxState = 5;
    }
    else if (RxState == 5)
    {
	RxState = 0;
	RxBuffer[4 + _data_cnt] = com_data;
	Data_Receive_Anl(RxBuffer, _data_cnt + 5);
    }
    else
	RxState = 0;
}
