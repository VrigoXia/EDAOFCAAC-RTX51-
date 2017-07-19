#include "mode.h"
#include "../ALGORITHM/PID/PID.h"
#include "../../HARDWARE/DEVICES/MOTOR/DC_MOTOR/MOTOR.h"
#include "../../HARDWARE/BSP/timer.h"
#include "../../HARDWARE/BSP/USART1.h"
void mode1(void)
{
					static bit step1=0;//模式一步骤1标志位
          static bit step2=0; //步骤2 标志位
          static bit step3=0;  //步骤三标志位

          if(step1)//步骤一已经完成
            {

              if(step2) //如果步骤2已完成
                {
                  if(step3) //如果步骤三已经完成
                    {
                      //模式一已经完成 ，蜂鸣器开始叫
                //      mode=0;//初始化模式
                      step1=0;
                      step2=0;
                      step3=0;

                    }
                  else//如果步骤三没有完成
                    {
                      close_DC_Motor(LEFT_MOTOR);
									    close_DC_Motor(RIGHT_MOTOR);

                      step3=OK;
                      PrintString1("step3 is ok\n");

                    }

                }
              else //如果步骤二没有完成
                {
                  setTimeout(Timer1,5000); //设置定时器定时长度 ,5秒
                  //////////////////////定时器////////////////////////////////////////
                  if(isExpiredTimer(Timer1))   //如果达到定时时间
                    {

                      setDC_MotorSpeed(LEFT_MOTOR,0.01f);
                      setDC_MotorSpeed(RIGHT_MOTOR,0.99f);
                      close_DC_Motor(RIGHT_MOTOR);
											 close_DC_Motor(LEFT_MOTOR);
                      step2=OK;
                      PrintString1("step2 is ok\n");
                    }
                  else//如果未达到定时时间或定时器未启动
                    {

                      if(isStopped(Timer1)) //只有当定时器是停止状态时才启动定时器。
                        {
                          restartTimer(Timer1);

                        }
                    }
                  ////////////////////////////////////////////////////////////////

                }
            }
          else //如果步骤一没有完成
            {
              PID_setTargetParameter(PID_1,100); //设定稳定角度
              setTimeout(Timer1,5000); //设置定时器定时长度 ,5秒
              open_DC_Motor(LEFT_MOTOR);//电机开始工作
							open_DC_Motor(RIGHT_MOTOR);//电机开始工作
							openPID(PID_1);
              if(abs(PID_getErr(PID_1))<2.0f)//当误差小于2°时，认为达到稳定,定时器就开始计时
                {
                  //////////////////////定时器////////////////////////////////////////
                  if(isExpiredTimer(Timer1))   //如果达到定时时间
                    {
                      stopTimer(Timer1);//达到定时时间后关闭定时器
                      step1=OK; //步骤一完成 ，已经达到稳定状态，蜂鸣器响一下
                      PrintString1("step1 is ok\n");
                    }
                  else//如果未达到定时时间或定时器未启动
                    {

                      if(isStopped(Timer1)) //只有当定时器是停止状态时才启动定时器。
                        {
                          restartTimer(Timer1);

                        }
                    }
                  ////////////////////////////////////////////////////////////////
                }
            }

}

void mode2(void)
{
	 static unsigned int mode2_ERR_times = 0;//误差次数计算
			 static bit mode2_step1 = 0;//步骤1标识位
			 static bit mode2_step2 = 0;//步骤2标识位
			 static bit mode2_step3 = 0;//步骤3标识位
			 static unsigned int mode2_time = 0;//模式执行次数
			 static float mode2_Sta_angle = 0.0;//误差值计算
	while(1)
	{
			 setTimeout(Timer1,10);//设定定时器
////////////////////步骤1范围///////////////////////////
			 if(mode2_step1)//判断步骤1是否完成
			 {
////////////////////步骤2范围///////////////////////////
				 if(mode2_step2)//判断步骤2是否完成
				 {
////////////////////步骤3范围///////////////////////////
					 if(mode2_step3)//判断步骤3是否完成
					 {
						 
					 }
					 else
					 {
						 mode2_time++;//模式2执行次数累计
						 if(mode2_time == 3)//当模式2运行3次后风机停止工作
						 {
							  setDC_MotorSpeed(LEFT_MOTOR,0.01f);
                setDC_MotorSpeed(RIGHT_MOTOR,0.99f);
                close_DC_Motor(RIGHT_MOTOR);
								close_DC_Motor(LEFT_MOTOR);
							 break;
						 }
					 }
////////////////////步骤3///////////////////////////
				 }
				 else
				 {
					 PID_setTargetParameter(PID_1,110.0f);//设定要到达的第二个角度
					 open_DC_Motor(LEFT_MOTOR);//电机开始工作
					 open_DC_Motor(RIGHT_MOTOR);//电机开始工作
					 if(PID_getErr(PID_1) < 2.0f)//判断是否进入误差计算
					 {
						 if(isExpiredTimer(Timer1))//判断是否到达定时器时间
						 {
							 mode2_ERR_times++;//误差次数累计
							 mode2_Sta_angle += abs(PID_getErr(PID_1));//误差累计
							 if(mode2_ERR_times >= 50)//误差累计次数是否到达50次
							 {
								 mode2_Sta_angle /= mode2_ERR_times;//计算平均误差
								 mode2_ERR_times = 0;//误差累计计数器初始化
								 if(abs(mode2_ERR_times) < 5)//判断是否到达稳定
								 {
									 mode2_step2 = 1;//进入步骤3
								 }
							 }
						 }
						 else
						 {
							 restartTimer(Timer1);
						 }
					 }
				 }
////////////////////////步骤2//////////////////////////
			 }
			 else
			 {
				 PID_setTargetParameter(PID_1,70.0f);//设定要达到的第一个角度
				 open_DC_Motor(LEFT_MOTOR);//电机开始工作
				 open_DC_Motor(RIGHT_MOTOR);//电机开始工作
				 if(PID_getErr(PID_1) < 2.0f)//判断是否开始进行误差计算
				 {
					 if(isExpiredTimer(Timer1))//判断定时器是否到达指定时间
					 {
						 mode2_ERR_times++;
						 mode2_Sta_angle += abs(PID_getErr(PID_1));//误差累计
						 if(mode2_ERR_times >= 50)//是否记录了50次误差
						 {
							 mode2_Sta_angle /= mode2_ERR_times;//取误差平均值
							 mode2_ERR_times = 0;//重置误差计数器
							 if(abs(mode2_ERR_times) < 5)//判断是否到达稳定状态，是则进入步骤2
							 {
								 mode2_step1 = 1;
							 }
						 }
					 }
					 else
					 {
						 restartTimer(Timer1);//启动定时器
					 }
				 }
			 }
////////////////////////步骤1/////////////////////////////			 
		 }
}


void mode3(void)
{
	
}

void mode4(void)
{
	
}
void modeLauncher(void)
{
	
}
