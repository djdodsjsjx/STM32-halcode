#include "Scheduler.h"
#include "tim.h"
#include "filter.h"
#include "show.h"
uint32_t test_dT_1000hz[3],test_rT[6];

static void Loop_1000Hz(void)	//1msִ��һ��
{
	test_dT_1000hz[0] = test_dT_1000hz[1];
	test_rT[3] = test_dT_1000hz[1] = GetSysTime_us ();
	test_dT_1000hz[2] = (uint32_t)(test_dT_1000hz[1] - test_dT_1000hz[0]);
//////////////////////////////////////////////////////////////////////	

	

//////////////////////////////////////////////////////////////////////	
	test_rT[4]= GetSysTime_us ();
	test_rT[5] = (uint32_t)(test_rT[4] - test_rT[3]) ;	
}

static void Loop_500Hz(void)	//2msִ��һ��
{	
//	lanya(USART1_com_data); 

	// DOG_CONTROL();    //6ms+1.9ms��̬����
}

static void Loop_200Hz(void)	//5msִ��һ��
{
	
//	 AHRS_Get_Angle();         //mpu9250

	Filter_Get_Angle();   //4.7ms

	// LED_demo();
}

static void Loop_100Hz(void)	//10msִ��һ��
{
			test_rT[0]= GetSysTime_us ();
//////////////////////////////////////////////////////////////////////	
	
	
//////////////////////////////////////////////////////////////////////		
			test_rT[1]= GetSysTime_us ();
			test_rT[2] = (uint32_t)(test_rT[1] - test_rT[0]) ;	
//				printf("%d \n",test_rT[2]);
}

static void Loop_50Hz(void)	//20msִ��һ��
{	
	
}

static void Loop_20Hz(void)	//50msִ��һ��
{
	
//	printf(" opmv.lt.sta:%d ,opmv.lt.angle:%d,pmv.lt.deviation:%d ,opmv.lt.p_flag:%d ,opmv.lt.pos_x:%d,opmv.lt.pos_y: %d,opmv.lt.dT_ms: %d\n",opmv.lt.sta,opmv.lt.angle,opmv.lt.deviation,opmv.lt.p_flag,opmv.lt.pos_x,opmv.lt.pos_y,opmv.lt.dT_ms);

}

static void Loop_10Hz(void)	//100msִ��һ��
{

	
}

static void Loop_2Hz(void)	//500msִ��һ��
{
	oled_show();   //40msһ������
//	//========   ����   =======
//	 demo();
	// printf("fdsf\n");
}

//ϵͳ�������ã�������ִͬ��Ƶ�ʵġ��̡߳�
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},
	{Loop_500Hz , 500,  0, 0},
	{Loop_200Hz , 200,  0, 0},
	{Loop_100Hz , 100,  0, 0},
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_10Hz  ,  10,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//��ʼ�������
	for(index=0;index < TASK_NUM;index++)
	{
		//����ÿ���������ʱ������
		sched_tasks[index].interval_ticks = 1000U/sched_tasks[index].rate_hz;
		//�������Ϊ1��Ҳ����1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��
	
	for(index=0;index < TASK_NUM;index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS
		uint32_t tnow = HAL_GetTick();
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{			
				//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();
			 			
		}
			
	}
}

	

