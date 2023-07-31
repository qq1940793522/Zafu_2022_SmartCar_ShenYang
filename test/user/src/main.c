/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file             main
* @company          成都逐飞科技有限公司
* @author           逐飞科技(QQ3184284598)
* @version          查看doc内version文件 版本说明
* @Software         IAR 8.3 or MDK 5.24
* @Target core      MM32F3277
* @Taobao           https://seekfree.taobao.com/
* @date             2021-11-11
********************************************************************************************************************/

#include "zf_common_headfile.h"

//函数声明区域

void beep_init(void);
void beep_ms(unsigned int i);
void wireless_control(void);

// **************************** 代码区域 ****************************
#define LED1                    H2												//核心板上的蓝色LED

//注意：按键默认高电平，当检测到低电平时按键为按下状态
#define KEY1                    G2												//主板右侧的按键G2	示意图：	|BEEP|
#define KEY2                    G3												//主板右侧的按键G3				
#define KEY3                    G4												//主板右侧的按键G4			|G4|	|G5|
#define KEY4                    G5												//主板右侧的按键G5			|G2|	|G3|
		
#define BEEP					D12												//主板右侧的蜂鸣器			

#define MOTOR_PWM               TIM5_PWM_CH2_A1								//电机的PWM输出为A1
#define MOTOR_DIR				A0											//电机的方向控制为A0

#define SERVO_PWM				TIM2_PWM_CH1_A15							//舵机的PWM控制为A15

#define FLASH_SECTION_INDEX       FLASH_SECTION_127								// 存储数据用的扇区 倒数第一个扇区
#define FLASH_PAGE_INDEX          FLASH_PAGE_3									// 存储数据用的页码 倒数第一个页码

//*******************************************************************

static float now_longitude = 114.514;											//定义实时经度
static float now_latitude = 191.810;											//定义实时纬度

static float now_azimuth = 114.514;												//定义实时位置与目标位置之间的方位角
static float now_distance = 191.810;											//定义实时位置与目标位置之间的距离

static float now_distance_wan = 191.810;										//定义实时位置与圆心之间的距离

static float now_direction = 114.514;											//定义实时航向角

static float first_circle_longitude;											//第一个弯道圆心的经度
static float first_circle_latitude;												//第一个弯道圆心的纬度
static float second_circle_longitude;											//第二个弯道圆心的经度
static float second_circle_latitude;											//第二个弯道圆心的纬度

static float latitude[4] = {0,0,0,0};											//用于记录4个点的纬度位置
static float longitude[4] = {0,0,0,0};											//用于记录4个点的经度位置

static float begin_latitude = 0;												//用于记录起点纬度（起点即终点）
static float begin_longitude = 0;												//用于记录起点经度（起点即终点）
static float final_distance = 0;												//定义实时位置与终点的距离
		
static float static_azimuth[4] = {0,0,0,0};										//用于记录4个点的之间的基准方位角，顺序为点4到点1的方位角，点1到点2的方位角。。。。点3到点4的方位角。
static float target_azimuth[4] = {0,0,0,0};										//用于记录两个弯道圆心与目标点的方位角
static float active_azimuth = 114.514;											//用于存放实时位置到圆心之间的方位角
	
static unsigned int target_point = 0;											//当前目标点

unsigned int motor_en = 0;														//电机使能	高电平有效

static float Kp_distance = 4;													//直道误差 Kp 参数				占空比100：5，50；4，80					占空比80：4,65;4,70
static float Kd_distance = 65;													//直道误差 Kd 参数
static float Kp_curve = 4;														//弯道误差 Kp 参数
static float Kd_curve = 70;														//弯道误差 Kd 参数

static float now_error_distance_zhi = 0;										//直道误差距离
static float pre_error_distance_zhi = 0;										//上一次的直道误差距离

static float now_error_distance_wan = 0;										//弯道误差距离
static float pre_error_distance_wan = 0;										//上一次的弯道误差距离

static int16 PWM_DUTY = 5000;													//电机 PWM 默认占空比

static float servo_angle_dir = 750;												//直道舵机偏转角度

static float servo_angle_wan = 750;												//弯道舵机偏转角度

static float R1=0;																//第 1 个圆的半径
static float R2=0;																//第 2 个圆的半径

static unsigned int G2_State = 0;												//定义按键G2的状态 0为短按 1为长按
static unsigned int G3_State = 0;												//定义按键G3的状态 0为短按 1为长按
static unsigned int G4_State = 0;												//定义按键G4的状态 0为短按 1为长按
static unsigned int G5_State = 0;												//定义按键G5的状态 0为短按 1为长按

static unsigned int T0 = 0;														//用于定时输出车辆信息

static unsigned int end_flag = 0;												//定义结束标志位

static float Ref_longitude = -0.000100;											//经度修正参数
static float Ref_latitude = 0.000000;												//纬度修正参数
static float point_Ref_longitude = -0.000100;											//经度修正参数
static float point_Ref_latitude = 0.000000;												//纬度修正参数


static unsigned int Ref_flag = 0;												//定义修正标志位
	
uint8 wireless_data[64];														// 存储串口接收的数据

//***********************************************************************************************************************************************************************

int main(void)
{	
// MM32系统时钟初始化
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	printf("\n\n\nClock&Debug OK!\n");
    
// 蜂鸣器初始化
	beep_init();
	printf("Beep OK!\n");
	beep_ms(500);																// 初始化 蜂鸣器 响一秒 代表蜂鸣器初始化完成
	system_delay_ms(1000);

// flash初始化
	if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
		printf("Flash Ready!\n");
	beep_ms(100);																// 代表 FLASH 初始化完成
	system_delay_ms(1000);
	
// 核心板LED初始化
	gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
	printf("LED OK!\n");
	beep_ms(100);																// 代表 LED 初始化完成
	system_delay_ms(1000);
	
// 主板按键初始化
    key_init(10);																// 初始化按键 扫描周期为 10ms
	printf("KEY OK!\n");
	beep_ms(100);																// 代表 KEY 初始化完成
	system_delay_ms(1000);
	
// 电机信号初始化
	gpio_init(MOTOR_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);                         // 初始化 MOTOR_DIR 输入 默认低电平 推挽输出模式	
	pwm_init(MOTOR_PWM,17000,0);												// 初始化 PWM_A1 通道 频率 17KHz 初始占空比 0%
	printf("MOTOR OK!\n");
	beep_ms(100);																// 代表 MOTOR 初始化完成
	system_delay_ms(1000);
	
// 舵机信号初始化
	pwm_init(SERVO_PWM,50,750);													// 初始化 SERVO 频率 50Hz 初始占空比 7.5 %
	printf("SERVO OK!\n");
	beep_ms(100);																// 代表 SERVO 初始化完成
	system_delay_ms(1000);

	
//// ICM20602初始化	
//	icm20602_init();															// 初始化 ICM20602
//	printf("ICM20602 OK!\n");
//	beep_ms(100);																// 代表 ICM20602 初始化完成
//	system_delay_ms(1000);
	
// GPS-TAU1201初始化
	gps_init();																	// 初始化 GPS-TAU1201
	printf("GPS OK!\n");  
	beep_ms(100);																// 代表 GPS-TAU1201 初始化完成
	system_delay_ms(1000);
		
// IPS200初始化
	ips200_init(IPS200_TYPE_PARALLEL8);											// 初始化 IPS200
	printf("IPS OK!\n");
	beep_ms(100);																// 代表 IPS200 初始化完成
	system_delay_ms(1000);
	
// 完成所有初始化
	printf("ALL OK!\n");
	beep_ms(500);																// 代表 所有硬件 初始化完成
	gpio_set_level(LED1,0);	
																																									 		
// 屏幕固定显示																																							
	ips200_show_string(0, 16*0, "N_Loc:");									//实时位置																				
	ips200_show_string(0, 16*1, "N_Azi:");									//实时方位角
	ips200_show_string(120, 16*1, "N_Dir:");								//实时航向角
	ips200_show_string(0, 16*2, "N_Dis:");									//实时位置距离目标点的实时距离	
	ips200_show_string(120,16*2,"Sin:");									//sin
	ips200_show_string(0, 16*3, "E_Dis:");									//误差距离
	ips200_show_string(120,16*3,"PWM:");									//舵机PWM
	ips200_show_string(0,16*4,"T_Azi:");									//切换方位角
	ips200_show_string(120,16*4,"A_Azi:");									//实时位置到圆心的方位角
	ips200_show_string(0, 16*5, "1.");										//目标位置 1
	ips200_show_string(0, 16*6, "2.");										//目标位置 2
	ips200_show_string(0, 16*7, "3.");										//目标位置 3
	ips200_show_string(0, 16*8, "4.");										//目标位置 4
	ips200_show_string(0, 16*9, "Cir:");									//实时位置到圆心的距离
	ips200_show_string(0, 16*10, "R1");										//第 1 个圆的半径
	ips200_show_string(0, 16*11, "R2");										//第 2 个圆的半径
	ips200_show_string(0, 16*12, "E_Cir:");									//到圆心的误差
	ips200_show_string(0, 16*13, "Ref:");									//基准偏置
	ips200_show_string(0, 16*14, "Kp_distance:");									//直道kp
	ips200_show_string(0, 16*15, "Kd_distance:");									//直道kd
	ips200_show_string(0, 16*16, "Kp_curve:");									//弯道kp
	ips200_show_string(0, 16*17, "Kd_curve:");									//弯道kd
	ips200_show_string(0, 16*19, "Tar:");									//当前目标点
	ips200_show_string(130, 16*19, "CJR & SyhenXX");						//代码作者
	
//*************************************************************************************************************************************************************************************
     
 while(1)
{
		
	debug_read_ring_buffer(wireless_data);		//读取串口接受到的数据
	wireless_control();							//处理串口接受到的数据
		
	//按键功能
	//G2	短按 	先计算4个点的基准方位角、2个圆的半径，然后蜂鸣器响3秒，最后启动电机
	//G2	长按
	//G3	短按 	切换到下一个目标点
	//G3	长按
	//G4	短按	读取flash
	//G4	长按	将修正经纬度
	//G5	短按 	将现在的位置写到当前目标点的数组中,并存入flash
	//G5	长按	
	
	key_scanner();
	while(key_get_state(KEY_1) == KEY_SHORT_PRESS)
	{
		while(key_get_state(KEY_1) != KEY_RELEASE)
		{
			system_delay_ms(1);
			G2_State++;
			key_scanner();
		}
		if(G2_State < 1000)
		{
			// G2短按代码位置
			
			// 计算圆心位置
			first_circle_longitude = (longitude[0]+longitude[1])/2;				//计算第一个圆心的经度
			first_circle_latitude = (latitude[0]+latitude[1])/2;				//计算第一个圆心的纬度
			second_circle_longitude = (longitude[2]+longitude[3])/2;			//计算第二个圆心的经度
			second_circle_latitude = (latitude[2]+latitude[3])/2;				//计算第二个圆心的纬度
			
			// 计算切换方位角
			target_azimuth[0] = get_two_points_azimuth(latitude[0],longitude[0],first_circle_latitude,first_circle_longitude);					//第 1 个切换方位角		第 1 个点到第 1 个圆心的方位角
			target_azimuth[1] = get_two_points_azimuth(latitude[1],longitude[1],first_circle_latitude,first_circle_longitude);					//第 2 个切换方位角		第 2 个点到第 1 个圆心的方位角
			target_azimuth[2] = get_two_points_azimuth(latitude[2],longitude[2],second_circle_latitude,second_circle_longitude);				//第 3 个切换方位角		第 3 个点到第 2 个圆心的方位角
			target_azimuth[3] = get_two_points_azimuth(latitude[3],longitude[3],second_circle_latitude,second_circle_longitude);				//第 4 个切换方位角		第 4 个点到第 2 个圆心的方位角
			
			// 计算圆心到各个点的方位角
			static_azimuth[0] = get_two_points_azimuth(latitude[3],longitude[3],latitude[0],longitude[0]);										//第 1 个静态方位角		第 4 个点到第 1 个点的方位角
			static_azimuth[1] = get_two_points_azimuth(latitude[0],longitude[0],latitude[1],longitude[1]);										//第 2 个静态方位角		第 1 个点到第 2 个点的方位角
			static_azimuth[2] = get_two_points_azimuth(latitude[1],longitude[1],latitude[2],longitude[2]);										//第 3 个静态方位角		第 2 个点到第 3 个点的方位角
			static_azimuth[3] = get_two_points_azimuth(latitude[2],longitude[2],latitude[3],longitude[3]);										//第 4 个静态方位角		第 3 个点到第 4 个点的方位角
			
			// 计算圆的半径R1、R2
			R1 = get_two_points_distance(latitude[0],longitude[0],latitude[1],longitude[1])/2;					//第 1 个圆的半径
			R2 = get_two_points_distance(latitude[2],longitude[2],latitude[3],longitude[3])/2;					//第 2 个圆的半径
			
			//记录起点位置
			begin_latitude = now_latitude;
			begin_longitude = now_longitude;
			
			end_flag = 0;

			beep_ms(3000);
			pwm_set_duty(MOTOR_PWM,PWM_DUTY);
			if(motor_en == 0)
				motor_en = 1;
			else
				motor_en = 0;
			gpio_set_level(MOTOR_DIR,motor_en);
			
			// G2短按代码位置
			printf("G2_SHORT_PRESS\n");
			beep_ms(50);
			G2_State = 0;
			break;
		}
		else if(G2_State >= 1000)
		{
			// G2长按代码位置
		
			// G2长按代码位置
			printf("G2_LONG_PRESS\n");
			beep_ms(50);
			G2_State = 0;
			break;
		}
	}
	
	while(key_get_state(KEY_2) == KEY_SHORT_PRESS)
	{
		while(key_get_state(KEY_2) != KEY_RELEASE)
		{
			system_delay_ms(1);
			G3_State++;
			key_scanner();
		}
		if(G3_State < 1000)
		{
			// G3短按代码位置
			
			target_point++;
			printf("now_point:%d,now_longitude:%lf,now_latitude:%lf\n",target_point,longitude[target_point],latitude[target_point]);//打印坐标点
			if(target_point >= 4)
				target_point = 0;
			
			// G3短按代码位置
			printf("G3_SHORT_PRESS\n");
			beep_ms(50);
			G3_State = 0;
			break;
		}
		else if(G3_State >= 1000)
		{
			// G3长按代码位置
				
			
				
			// G3长按代码位置
			printf("G3_LONG_PRESS\n");
			beep_ms(50);
			G3_State = 0;
			break;
		}
	}
	
	while(key_get_state(KEY_3) == KEY_SHORT_PRESS)
	{
		while(key_get_state(KEY_3) != KEY_RELEASE)
		{
			system_delay_ms(1);
			G4_State++;
			key_scanner();
		}
		if(G4_State < 1000)
		{
			// G4短按代码位置
			
			flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区

			for(int i=0;i<4;i++)
			{
				latitude[i] = flash_union_buffer[i].float_type;
				longitude[i] = flash_union_buffer[i+4].float_type;
			}
			
			// G4短按代码位置
			printf("G4_SHORT_PRESS\n");
			beep_ms(50);
			G4_State = 0;
			break;
		}
		else if(G4_State >= 1000)
		{
			// G4长按代码位置

			for(int t = 0; t < 4;t++)
			{
				longitude[t] = longitude[t] + Ref_longitude;
				latitude[t] = latitude[t] + Ref_latitude;
			}
				
			// G4长按代码位置
			printf("G4_LONG_PRESS\n");
			beep_ms(50);
			G4_State = 0;
			break;
		}
	}
	
	while(key_get_state(KEY_4) == KEY_SHORT_PRESS)
	{
		while(key_get_state(KEY_4) != KEY_RELEASE)
		{
			system_delay_ms(1);
			G5_State++;
			key_scanner();
		}
		if(G5_State < 1000)
		{
			// G5短按代码位置
			
			latitude[target_point] = now_latitude;
			longitude[target_point] = now_longitude;
			printf("now_point:%d,longitude:%lf,latitude:%lf\n",target_point,longitude[target_point],latitude[target_point]);	//打印坐标点
			
			flash_union_buffer[target_point].float_type  = latitude[target_point];                              
			flash_union_buffer[target_point+4].float_type  = longitude[target_point];
			
			flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区已记录的数据
			
			// G5短按代码位置
			printf("G5_SHORT_PRESS\n");
			beep_ms(50);
			G5_State = 0;
			break;
		}
		else if(G5_State >= 1000)
		{
			// G5长按代码位置
		
			// G5长按代码位置
			printf("G5_LONG_PRESS\n");
			beep_ms(50);
			G5_State = 0;
			break;
		}
	}
		
//************************************************************************************************************************************************************************************************		
		
        //gps数据接收与解析都是通过串口中断调用gps_uart_callback函数进行实现的
        //数据解析完毕之后gps_tau1201_flag标志位会置 1
        if(gps_tau1201_flag)
        {
            gps_tau1201_flag = 0;																	//gps_tau1201_flag标志位置 0
			
            gps_data_parse();           															//开始解析数据
			
			T0++;																					//T0 + 1
			
			if(gps_tau1201.state)																	//如果GPS数据有效	1为有效 0为无效
			{
				// 上一次的数据
				pre_error_distance_zhi = now_error_distance_zhi;							//将之前的直道误差距离放到上一次直道误差距离中
				pre_error_distance_wan = now_error_distance_wan;							//将之前的弯道误差距离放到上一次弯道误差距离中

				// 实时经纬度
				now_longitude = gps_tau1201.longitude;																																						//将GPS数据放到now_longitude中
				now_latitude = gps_tau1201.latitude;																																						//将GPS数据放到now_latitude中
				ips200_show_float(50, 16*0, now_longitude, 4, 6);																																			//屏幕显示 now_longitude
				ips200_show_float(140, 16*0, now_latitude, 4, 6);																																			//屏幕显示 now_latitude								
				
				// 实时方位角
				now_azimuth = get_two_points_azimuth(now_latitude,now_longitude,latitude[target_point],longitude[target_point]);			//计算实时位置到目标点的方位角
				ips200_show_float(50,16*1,now_azimuth,3,3);																																					//屏幕显示实时位置到目标点的方位角
				
				// 实时航向角
				now_direction = gps_tau1201.direction;																																						//将GPS数据放到now_direction中
				ips200_show_float(170,16*1,now_direction,3,3);																																				//屏幕显示实时航向角
				
				// 实时距离
				now_distance = get_two_points_distance(now_latitude,now_longitude,latitude[target_point],longitude[target_point]);			//计算实时位置与目标位置之间的距离
				ips200_show_float(50,16*2,now_distance,3,3);																																				//屏幕显示实时位置与目标位置之间的距离
	
				// 目标点位置
				ips200_show_float(30,16*5,longitude[0], 4, 6);ips200_show_float(120, 16*5, latitude[0], 4, 6);				//显示第 1 个点的经纬度
				ips200_show_float(30,16*6,longitude[1], 4, 6);ips200_show_float(120, 16*6, latitude[1], 4, 6);				//显示第 2 个点的经纬度
				ips200_show_float(30,16*7,longitude[2], 4, 6);ips200_show_float(120, 16*7, latitude[2], 4, 6);				//显示第 3 个点的经纬度
				ips200_show_float(30,16*8,longitude[3], 4, 6);ips200_show_float(120, 16*8, latitude[3], 4, 6);				//显示第 4 个点的经纬度

				ips200_show_float(50,16*10,R1,4,6);										//显示圆1半径
				ips200_show_float(50,16*11,R2,4,6);										//显示圆2半径

				//直道与弯道条件判断

				if(target_point <= 1)		//在前 2 个点使用第一个圆心													
				{
					active_azimuth = get_two_points_azimuth(now_latitude,now_longitude,first_circle_latitude,first_circle_longitude);		//计算实时位置到第一个圆心的方位角					
				}
				
				if(target_point > 1 && target_point <=3)			//在后 2 个点使用第二个圆心	
				{
					active_azimuth = get_two_points_azimuth(now_latitude,now_longitude,second_circle_latitude,second_circle_longitude);	//计算实时位置到第二个圆心的方位角					
				}
				
				//直道处理
				
				if(target_point == 0||target_point == 2)					//目标点是0或者2时，正在走直道,用直道的PID																																														
				{
					// 直道误差距离
				now_error_distance_zhi = now_distance * sin((now_azimuth-static_azimuth[target_point])/180*PI);									//计算误差距离
				ips200_show_float(50,16*3,now_error_distance_zhi,3,3);									 																				//屏幕显示计算误差距离
				ips200_show_float(150,16*2,sin((now_azimuth-static_azimuth[target_point])),3,3);														//屏幕显示sin的数值
					
					// PD控制直道舵机转向
					servo_angle_dir =750 + Kp_distance*now_error_distance_zhi + Kd_distance*(now_error_distance_zhi-pre_error_distance_zhi); 				//根据PID公式计算舵机偏转量
					if(servo_angle_dir >= 900)									
						pwm_set_duty(SERVO_PWM,900);										//如果偏转量超量程就按最大量程来
					else if(servo_angle_dir <= 600)
						pwm_set_duty(SERVO_PWM,600);
					else
						pwm_set_duty(SERVO_PWM,(unsigned int)servo_angle_dir);				//没有超量程就按计算的来
					
					ips200_show_float(150,16*3,servo_angle_dir,3,3);						//显示直道舵机角度
					ips200_show_string(50, 16*9, "zhi_dao");								//实时位置到圆心的距离  显示在直道
					
				}	
				
			
				//弯道误差距离=所在圆半径-当前位置与圆心距离
				if(target_point == 1)				//	在第 1 个弯道时
				{
					now_distance_wan = get_two_points_distance(now_latitude,now_longitude,first_circle_latitude,first_circle_longitude);		//当前位置与圆心 1 距离
					now_error_distance_wan = R1-now_distance_wan;
					servo_angle_wan =750 + Kp_curve*now_error_distance_wan + Kd_curve*(now_error_distance_wan-pre_error_distance_wan); 				//根据PID公式计算舵机偏转量
					if(servo_angle_wan >= 900)									
						pwm_set_duty(SERVO_PWM,900);							//如果偏转量超量程就按最大量程来
					else if(servo_angle_wan <= 600)
						pwm_set_duty(SERVO_PWM,600);
					else
						pwm_set_duty(SERVO_PWM,(unsigned int)servo_angle_wan);
					ips200_show_float(150,16*3,servo_angle_wan,3,3);			//显示弯道舵机角度
					ips200_show_float(50,16*9,now_distance_wan,3,3);			//显示实时位置到圆心的距离
					ips200_show_float(50,16*12,now_error_distance_wan,3,3);
				}
				if(target_point == 3)				//	在第 2 个弯道时
				{
					now_distance_wan = get_two_points_distance(now_latitude,now_longitude,second_circle_latitude,second_circle_longitude);		//当前位置与圆心 2 距离
					now_error_distance_wan = R2-now_distance_wan;
					servo_angle_wan =750 + Kp_curve*now_error_distance_wan + Kd_curve*(now_error_distance_wan-pre_error_distance_wan); 				//根据PID公式计算舵机偏转量
					if(servo_angle_wan >= 900)									
						pwm_set_duty(SERVO_PWM,900);							//如果偏转量超量程就按最大量程来
					else if(servo_angle_wan <= 600)
						pwm_set_duty(SERVO_PWM,600);
					else
						pwm_set_duty(SERVO_PWM,(unsigned int)servo_angle_wan);	//没有超量程就按计算的来				
				
					ips200_show_float(150,16*3,servo_angle_wan,3,3);			//显示弯道舵机角度
					ips200_show_float(50,16*9,now_distance_wan,3,3);			//显示实时位置到圆心的距离
					ips200_show_float(50,16*12,now_error_distance_wan,3,3);
				}
					
				if((active_azimuth-target_azimuth[target_point]) < 2 && (target_azimuth[target_point] - active_azimuth) < 2)
				{
					target_point++;
					printf("now_target_point:%d",target_point);
					beep_ms(10);
				}
				ips200_show_float(50,16*4,target_azimuth[target_point],3,3);											//显示切换方位角
				ips200_show_float(170,16*4,active_azimuth,3,3);															//显示实时位置到圆心的方位角
				
				// 实时目标
				ips200_show_int(30,16*19,target_point+1,2);																//屏幕显示目标点

				//显示速度
				ips200_show_float(60,16*19,gps_tau1201.speed,3,3);														//显示速度
					
				//切换目标点   
				if(target_point >= 4)
				{
					target_point = 0;
					end_flag = 1;					//当目标点重置为0判断为即将到达终点
				}
				
				//停车代码
				if(end_flag)					
				{
					final_distance = get_two_points_distance(now_latitude,now_longitude,begin_latitude,begin_longitude);	//计算实时位置与终点距离
					if(final_distance < 3)																																									//若实时位置与终点距离小于3，停车（需改进）
					{
						motor_en = 0;
						gpio_set_level(MOTOR_DIR,0);
					}
				}
					
				//基准偏置参数显示
				ips200_show_float(50,16*13,Ref_longitude,2,6);
				ips200_show_float(150,16*13,Ref_latitude,2,6);
				
				//直道、弯道参数显示
				ips200_show_float(100,16*14,Kp_distance,2,1);
				ips200_show_float(100,16*15,Kd_distance,2,1);
				ips200_show_float(100,16*16,Kp_curve,2,1);
				ips200_show_float(100,16*17,Kd_curve,2,1);
			}
		}
    }
}




// 蜂鸣器初始化代码 使用前务必声明
void beep_init(void)
{
	gpio_init(BEEP,GPO,GPIO_LOW,GPO_PUSH_PULL);									// 初始化 BEEP 输出 默认低电平 推挽输出模式
}

void beep_ms(unsigned int i)
{
	gpio_set_level(BEEP,1);
	system_delay_ms(i);
	gpio_set_level(BEEP,0);
}

//************************************************************************************************************************************************

void wireless_control(void)							//无线串口控制指令
{
	unsigned int i;
	if(wireless_data[0]=='z' && wireless_data[1]=='a' && wireless_data[2]=='f' && wireless_data[3]=='u' && wireless_data[4]=='_')
	{
		//停车
		if(wireless_data[5]=='s'&&wireless_data[6]=='t'&&wireless_data[7]=='o'&&wireless_data[8]=='p')	//发送 stop 电机制动
		{	
			gpio_set_level(MOTOR_DIR,0);
			motor_en = 0;
			pwm_set_duty(MOTOR_PWM,PWM_DUTY);
			printf("CJR&SY MUST SUCCEED!\n");
			beep_ms(50);
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}
		}
		//启动
		if(wireless_data[5]=='r'&&wireless_data[6]=='u'&&wireless_data[7]=='n')	//发送 run  先计算4个点的基准方位角、2个圆的半径，然后蜂鸣器响3秒，最后启动电机（相当于短按G2）
		{
			first_circle_longitude = (longitude[0]+longitude[1])/2;				//计算第一个圆心的经度
			first_circle_latitude = (latitude[0]+latitude[1])/2;				//计算第一个圆心的纬度
			second_circle_longitude = (longitude[2]+longitude[3])/2;			//计算第二个圆心的经度
			second_circle_latitude = (latitude[2]+latitude[3])/2;				//计算第二个圆心的纬度
			
			// 计算切换方位角
			target_azimuth[0] = get_two_points_azimuth(latitude[0],longitude[0],first_circle_latitude,first_circle_longitude);					//第 1 个切换方位角		第 1 个点到第 1 个圆心的方位角
			target_azimuth[1] = get_two_points_azimuth(latitude[1],longitude[1],first_circle_latitude,first_circle_longitude);					//第 2 个切换方位角		第 2 个点到第 1 个圆心的方位角
			target_azimuth[2] = get_two_points_azimuth(latitude[2],longitude[2],second_circle_latitude,second_circle_longitude);				//第 3 个切换方位角		第 3 个点到第 2 个圆心的方位角
			target_azimuth[3] = get_two_points_azimuth(latitude[3],longitude[3],second_circle_latitude,second_circle_longitude);				//第 4 个切换方位角		第 4 个点到第 2 个圆心的方位角
			
			// 计算圆心到各个点的方位角
			static_azimuth[0] = get_two_points_azimuth(latitude[3],longitude[3],latitude[0],longitude[0]);										//第 1 个静态方位角		第 4 个点到第 1 个点的方位角
			static_azimuth[1] = get_two_points_azimuth(latitude[0],longitude[0],latitude[1],longitude[1]);										//第 2 个静态方位角		第 1 个点到第 2 个点的方位角
			static_azimuth[2] = get_two_points_azimuth(latitude[1],longitude[1],latitude[2],longitude[2]);										//第 3 个静态方位角		第 2 个点到第 3 个点的方位角
			static_azimuth[3] = get_two_points_azimuth(latitude[2],longitude[2],latitude[3],longitude[3]);										//第 4 个静态方位角		第 3 个点到第 4 个点的方位角
			
			// 计算圆的半径R1、R2
			R1 = get_two_points_distance(latitude[0],longitude[0],latitude[1],longitude[1])/2;					//第 1 个圆的半径
			R2 = get_two_points_distance(latitude[2],longitude[2],latitude[3],longitude[3])/2;					//第 2 个圆的半径
			
			//记录起点位置
			begin_latitude = now_latitude;
			begin_longitude = now_longitude;

			beep_ms(3000);
			motor_en = 1;
			gpio_set_level(MOTOR_DIR,1);
			pwm_set_duty(MOTOR_PWM,PWM_DUTY);
			
			end_flag = 0;
			
			printf("Ready go!\n");
			beep_ms(50);
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}
		}
		//修改占空比
		if(wireless_data[5]=='d')						//	d更改占空比首字母
		{
			if(wireless_data[6]=='5' && wireless_data[7]=='0')			//	d50表示更改占空比为50
			{
				PWM_DUTY = 5000;
				pwm_set_duty(MOTOR_PWM,PWM_DUTY);
				printf("\nDUTY 50 ok!\n");
				beep_ms(50);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
			}
						
			if(wireless_data[6]=='6'&&wireless_data[7]=='0')			//	d60表示更改占空比为60
			{
				PWM_DUTY = 6000;
				pwm_set_duty(MOTOR_PWM,PWM_DUTY);
				printf("\nDUTY 60 ok!\n");
				beep_ms(50);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
			}
			
			if(wireless_data[6]=='7'&&wireless_data[7]=='0')			//	d70表示更改占空比为70
			{
				PWM_DUTY = 7000;
				pwm_set_duty(MOTOR_PWM,PWM_DUTY);
				printf("\nDUTY 70 ok!\n");
				beep_ms(50);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
			}
						
			if(wireless_data[6]=='8'&&wireless_data[7]=='0')	//	d80表示更改占空比为80
			{
				PWM_DUTY = 8000;
				pwm_set_duty(MOTOR_PWM,PWM_DUTY);
				printf("\nDUTY 80 ok!\n");
				beep_ms(50);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
			}
			
			if(wireless_data[6]=='9'&&wireless_data[7]=='0')		//	d90表示更改占空比为90
			{
				PWM_DUTY = 9000;
				pwm_set_duty(MOTOR_PWM,PWM_DUTY);
				printf("\nDUTY 90 ok!\n");
				beep_ms(50);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
			}
						
			if(wireless_data[6]=='1'&&wireless_data[7]=='0'&&wireless_data[8]=='0')		//	d100表示更改占空比为100
			{
				PWM_DUTY = 10000;
				pwm_set_duty(MOTOR_PWM,PWM_DUTY);
				printf("\nDUTY 100 ok!\n");
				beep_ms(50);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
			}
		}
		if(wireless_data[5] == 'g' && wireless_data[7] == 's')
		{
			switch(wireless_data[6])
			{
				case 50 :
					//G2短按代码
				
					// 计算圆心位置
					first_circle_longitude = (longitude[0]+longitude[1])/2;				//计算第一个圆心的经度
					first_circle_latitude = (latitude[0]+latitude[1])/2;				//计算第一个圆心的纬度
					second_circle_longitude = (longitude[2]+longitude[3])/2;			//计算第二个圆心的经度
					second_circle_latitude = (latitude[2]+latitude[3])/2;				//计算第二个圆心的纬度
					
					// 计算切换方位角
					target_azimuth[0] = get_two_points_azimuth(latitude[0],longitude[0],first_circle_latitude,first_circle_longitude);					//第 1 个切换方位角		第 1 个点到第 1 个圆心的方位角
					target_azimuth[1] = get_two_points_azimuth(latitude[1],longitude[1],first_circle_latitude,first_circle_longitude);					//第 2 个切换方位角		第 2 个点到第 1 个圆心的方位角
					target_azimuth[2] = get_two_points_azimuth(latitude[2],longitude[2],second_circle_latitude,second_circle_longitude);				//第 3 个切换方位角		第 3 个点到第 2 个圆心的方位角
					target_azimuth[3] = get_two_points_azimuth(latitude[3],longitude[3],second_circle_latitude,second_circle_longitude);				//第 4 个切换方位角		第 4 个点到第 2 个圆心的方位角
					
					// 计算圆心到各个点的方位角
					static_azimuth[0] = get_two_points_azimuth(latitude[3],longitude[3],latitude[0],longitude[0]);										//第 1 个静态方位角		第 4 个点到第 1 个点的方位角
					static_azimuth[1] = get_two_points_azimuth(latitude[0],longitude[0],latitude[1],longitude[1]);										//第 2 个静态方位角		第 1 个点到第 2 个点的方位角
					static_azimuth[2] = get_two_points_azimuth(latitude[1],longitude[1],latitude[2],longitude[2]);										//第 3 个静态方位角		第 2 个点到第 3 个点的方位角
					static_azimuth[3] = get_two_points_azimuth(latitude[2],longitude[2],latitude[3],longitude[3]);										//第 4 个静态方位角		第 3 个点到第 4 个点的方位角
					
					// 计算圆的半径R1、R2
					R1 = get_two_points_distance(latitude[0],longitude[0],latitude[1],longitude[1])/2;					//第 1 个圆的半径
					R2 = get_two_points_distance(latitude[2],longitude[2],latitude[3],longitude[3])/2;					//第 2 个圆的半径
					
					//记录起点位置
					begin_latitude = now_latitude;
					begin_longitude = now_longitude;
					
					end_flag = 0;

					beep_ms(3000);
					pwm_set_duty(MOTOR_PWM,PWM_DUTY);
					if(motor_en == 0)
						motor_en = 1;
					else
						motor_en = 0;
					gpio_set_level(MOTOR_DIR,motor_en);
				
					//G2短按代码
					printf("\nG2S ok!\n");
					break;
				case 51 :
					//G3短按代码
				
					target_point++;
					printf("now_point:%d,now_longitude:%lf,now_latitude:%lf\n",target_point,longitude[target_point],latitude[target_point]);//打印坐标点
					if(target_point >= 4)
						target_point = 0;
					
					//G3短按代码
					printf("\nG3S ok!\n");
					break;
				case 52 :
					//G4短按代码
				
					flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区

					for(int i=0;i<4;i++)
					{
						latitude[i] = flash_union_buffer[i].float_type;
						longitude[i] = flash_union_buffer[i+4].float_type;
					}
					
					//G4短按代码
					printf("\nG4S ok!\n");
					break;
				case 53 :
					//G5短按代码
				
					latitude[target_point] = now_latitude;
					longitude[target_point] = now_longitude;
					printf("now_point:%d,longitude:%lf,latitude:%lf\n",target_point,longitude[target_point],latitude[target_point]);	//打印坐标点
					
					flash_union_buffer[target_point].float_type  = latitude[target_point];                              
					flash_union_buffer[target_point+4].float_type  = longitude[target_point];
					
					flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区已记录的数据
				
					//G5短按代码
					printf("\nG5S ok!\n");
					break;
			}
			beep_ms(50);
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}	
		}
		if(wireless_data[5] == 'g' && wireless_data[7] == 'l')
		{
			switch(wireless_data[6])
			{
				case 50 :
					//G2长按代码
				
					//G2长按代码
					printf("\nG2L ok!\n");
					break;
				case 51 :
					//G3长按代码
				
					//G3长按代码
					printf("\nG3L ok!\n");
					break;
				case 52 :
					//G4长按代码
				
					for(int t = 0; t < 4;t++)
					{
						longitude[t] = longitude[t] + Ref_longitude;
						latitude[t] = latitude[t] + Ref_latitude;
					}
				
					//G4长按代码
					printf("\nG4L ok!\n");
					break;
				case 53 :
					//G5长按代码
				
					//G5长按代码
					printf("\nG5L ok!\n");
					break;
			}
			beep_ms(50);
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}	
		}
		//修改经度基准
		if(wireless_data[5] == 'r' && wireless_data[6] == 'j')
		{
			unsigned int Ref_longitude_temp = 0;
			for(int temp = 8;temp < 15;temp++)
			{
				if(48 <= wireless_data[temp] && wireless_data[temp] <= 57)
				{
					Ref_longitude_temp = Ref_longitude_temp * 10;
					Ref_longitude_temp = Ref_longitude_temp + (wireless_data[temp] - 48);
				}
				if(wireless_data[temp] == ';')
					break;
			}
			if(wireless_data[7] == '-')
				Ref_longitude = -(Ref_longitude_temp * 0.000001);
			if(wireless_data[7] == '+')
				Ref_longitude = Ref_longitude_temp * 0.000001;
			printf("\nRef_longitude:%f\n",Ref_longitude);
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}
			beep_ms(50);
		}
		
//修改点位的经度基准
		if(wireless_data[5] == 'p')
		{
			unsigned int point_Ref_longitude = 0;
			unsigned int point_temp = 0;
			if(48 <= wireless_data[6] && wireless_data[6] <= 57)
				point_temp = wireless_data[6] - 49;
			if(wireless_data[7] == 'r' && wireless_data[8] == 'j')
			{
				for(int temp = 10;temp < 17;temp++)
				{
					if(48 <= wireless_data[temp] && wireless_data[temp] <= 57)
					{
						point_Ref_longitude = point_Ref_longitude * 10;
						point_Ref_longitude = point_Ref_longitude + (wireless_data[temp] - 48);
					}
					if(wireless_data[temp] == ';')
						break;
				}
				if(wireless_data[9] == '-')
					longitude[point_temp] = longitude[point_temp] - (point_Ref_longitude * 0.000001);
				if(wireless_data[9] == '+')
					longitude[point_temp] = longitude[point_temp] + point_Ref_longitude * 0.000001;
				printf("\nNow_longitude_%d:%f\n",point_temp,longitude[point_temp]);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
				beep_ms(50);
			}
		}
		
		//修改点位的纬度基准
		if(wireless_data[5] == 'p')
		{
			unsigned int point_Ref_latitude = 0;
			unsigned int point_temp = 0;
			if(48 <= wireless_data[6] && wireless_data[6] <= 57)
				point_temp = wireless_data[6] - 49;
			if(wireless_data[7] == 'r' && wireless_data[8] == 'w')
			{
				for(int temp = 10;temp < 17;temp++)
				{
					if(48 <= wireless_data[temp] && wireless_data[temp] <= 57)
					{
						point_Ref_latitude = point_Ref_latitude * 10;
						point_Ref_latitude = point_Ref_latitude + (wireless_data[temp] - 48);
					}
					if(wireless_data[temp] == ';')
						break;
				}
				if(wireless_data[9] == '-')
					latitude[point_temp] = latitude[point_temp] - (point_Ref_latitude * 0.000001);
				if(wireless_data[9] == '+')
					latitude[point_temp] = latitude[point_temp] + point_Ref_latitude * 0.000001;
				printf("\nNow_latitude_%d:%f\n",point_temp,latitude[point_temp]);
				for(i = 0;i < 64;i++)					//接收完一次指令清空数据
				{
					wireless_data[i]=0;
				}
				beep_ms(50);
			}
		}

		
		//修改纬度基准
		if(wireless_data[5] == 'r' && wireless_data[6] == 'w')
		{
			unsigned int Ref_latitude_temp = 0;
			for(int temp = 8;temp < 15;temp++)
			{
				if(48 <= wireless_data[temp] && wireless_data[temp] <= 57)
				{
					Ref_latitude_temp = Ref_latitude_temp * 10;
					Ref_latitude_temp = Ref_latitude_temp + (wireless_data[temp] - 48);
				}
				if(wireless_data[temp] == ';')
					break;
			}
			if(wireless_data[7] == '-')
				Ref_latitude = -(Ref_latitude_temp * 0.000001);
			if(wireless_data[7] == '+')
				Ref_latitude = Ref_latitude_temp * 0.000001;
			printf("\nRef_latitude:%f\n",Ref_latitude);
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}
			beep_ms(50);
		}
		//修改直道或弯道的kp
		if(wireless_data[5] == 'k' && wireless_data[6] == 'p')
		{
			unsigned int Kp_temp = 0;
			for(int temp = 8;temp < 15;temp++)
			{
				if(48 <= wireless_data[temp] && wireless_data[temp] <= 57)
				{
					Kp_temp = Kp_temp * 10;
					Kp_temp = Kp_temp + (wireless_data[temp] - 48);
				}
				if(wireless_data[temp] == ';')
					break;
				
			}
			if(wireless_data[7] == 'd')
				{
					Kp_distance = Kp_temp;
					printf("\nKp_distance:%f\n",Kp_distance);
				}
				else if(wireless_data[7] == 'c')
				{
					Kp_curve = Kp_temp;
					printf("\nKp_curve:%f\n",Kp_curve);
				}
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}
			beep_ms(50);
		}
		//修改直道或弯道的kd
		if(wireless_data[5] == 'k' && wireless_data[6] == 'd')
		{
			unsigned int Kd_temp = 0;
			for(int temp = 8;temp < 15;temp++)
			{
				if(48 <= wireless_data[temp] && wireless_data[temp] <= 57)
				{
					Kd_temp = Kd_temp * 10;
					Kd_temp = Kd_temp + (wireless_data[temp] - 48);
				}
				if(wireless_data[temp] == ';')
					break;

			}
							if(wireless_data[7] == 'd')
				{
					Kd_distance = Kd_temp;
					printf("\nKd_distance:%f\n",Kd_distance);
				}
				else if(wireless_data[7] == 'c')
				{
					Kd_curve = Kd_temp;
					printf("\nKd_curve:%f\n",Kd_curve);
				}
			for(i = 0;i < 64;i++)					//接收完一次指令清空数据
			{
				wireless_data[i]=0;
			}
			beep_ms(50);
		}
	}
}
