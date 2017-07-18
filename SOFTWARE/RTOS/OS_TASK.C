/********************************
*FileName:	 TASK_C
*Author	 :   JZHG1992
*Versions:	 V1.0
������ռʽ��ʵʱ�ں�
ע�⣺ÿ�����񲻿������κ���ʽ����ʽ����ʽ��whileѭ����forѭ����������ʽ��ѭ��
���򽫵������������޷�ִ��
*********************************/

#include "OS.H"
#include "../TASK/task.h"


/************************************
*		 ����ϵͳ����ִ��Ƶ��
*************************************/

#define TASK_DELAY0 TASK_CLOCK/100  // �ǶȻ�ȡ��ִ��Ƶ��,100hz��
#define TASK_DELAY1 TASK_CLOCK/90  // �ȶ���ִ��Ƶ�� ,90hz
#define TASK_DELAY2 TASK_CLOCK/50  // ģʽѡ���ִ��Ƶ��
#define TASK_DELAY3 TASK_CLOCK/30    // ����ɨ���ִ��Ƶ�� 30hz
#define TASK_DELAY4 TASK_CLOCK/50   // ��������ʾ������ִ��Ƶ��  30hz
#define TASK_DELAY5 TASK_CLOCK/8   // lcdˢ��Ƶ��


//#define OS_DEBUG
/************************************
*		 ����0����
*************************************/
void task0(void)
{
		Task_Delay[0] = TASK_DELAY0;  // ��������ִ�ж�
#ifdef OS_DEBUG
		PrintString1("task0 is on \n");  
#endif
}
/************************************
*		 ����1����
*************************************/

void task1( void )
{
	Task_Delay[1] = TASK_DELAY1;  // ��������ִ�ж�
 
#ifdef OS_DEBUG
		PrintString1("task1 is on \n");  
#endif
  	

}
/************************************
*		 ����2����
*************************************/

void task2( void )
{
	Task_Delay[2] = TASK_DELAY2;  // ��������ִ�ж�
	/* ������� */
#ifdef OS_DEBUG
		PrintString1("task2 is on \n");  
#endif
  
}
/************************************
*		 ����3����
*************************************/

void task3( void )
{
	Task_Delay[3] = TASK_DELAY3;  // ��������ִ�ж�
#ifdef OS_DEBUG
		PrintString1("task3 is on \n");  
#endif
	

}

/************************************
*		 ����4����
*************************************/

void task4( void )
{
	Task_Delay[4] = TASK_DELAY4;  // ��������ִ�ж�
	/* ������� */
#ifdef OS_DEBUG
		PrintString1("task4 is on \n");  
#endif
    sendScopeData();
	
}

 void task5( void )
{
    	Task_Delay[5] = TASK_DELAY5;  // ��������ִ�ж�

	 //	  LCD_UpdateAll();

}





/************************************
*		 �������ָ��
*        �����������ָ��
*************************************/

void ( *const task[] )() = {  task0,task1,task2,task3,task4,task5 };
