#include "mode.h"
#include "../ALGORITHM/PID/PID.h"
#include "../../HARDWARE/DEVICES/MOTOR/DC_MOTOR/MOTOR.h"
#include "../../HARDWARE/BSP/timer.h"
#include "../../HARDWARE/BSP/USART1.h"
void mode1(void)
{
					static bit step1=0;//ģʽһ����1��־λ
          static bit step2=0; //����2 ��־λ
          static bit step3=0;  //��������־λ

          if(step1)//����һ�Ѿ����
            {

              if(step2) //�������2�����
                {
                  if(step3) //����������Ѿ����
                    {
                      //ģʽһ�Ѿ���� ����������ʼ��
                //      mode=0;//��ʼ��ģʽ
                      step1=0;
                      step2=0;
                      step3=0;

                    }
                  else//���������û�����
                    {
                      close_DC_Motor(LEFT_MOTOR);
									    close_DC_Motor(RIGHT_MOTOR);

                      step3=OK;
                      PrintString1("step3 is ok\n");

                    }

                }
              else //��������û�����
                {
                  setTimeout(Timer1,5000); //���ö�ʱ����ʱ���� ,5��
                  //////////////////////��ʱ��////////////////////////////////////////
                  if(isExpiredTimer(Timer1))   //����ﵽ��ʱʱ��
                    {

                      setDC_MotorSpeed(LEFT_MOTOR,0.01f);
                      setDC_MotorSpeed(RIGHT_MOTOR,0.99f);
                      close_DC_Motor(RIGHT_MOTOR);
											 close_DC_Motor(LEFT_MOTOR);
                      step2=OK;
                      PrintString1("step2 is ok\n");
                    }
                  else//���δ�ﵽ��ʱʱ���ʱ��δ����
                    {

                      if(isStopped(Timer1)) //ֻ�е���ʱ����ֹͣ״̬ʱ��������ʱ����
                        {
                          restartTimer(Timer1);

                        }
                    }
                  ////////////////////////////////////////////////////////////////

                }
            }
          else //�������һû�����
            {
              PID_setTargetParameter(PID_1,100); //�趨�ȶ��Ƕ�
              setTimeout(Timer1,5000); //���ö�ʱ����ʱ���� ,5��
              open_DC_Motor(LEFT_MOTOR);//�����ʼ����
							open_DC_Motor(RIGHT_MOTOR);//�����ʼ����
							openPID(PID_1);
              if(abs(PID_getErr(PID_1))<2.0f)//�����С��2��ʱ����Ϊ�ﵽ�ȶ�,��ʱ���Ϳ�ʼ��ʱ
                {
                  //////////////////////��ʱ��////////////////////////////////////////
                  if(isExpiredTimer(Timer1))   //����ﵽ��ʱʱ��
                    {
                      stopTimer(Timer1);//�ﵽ��ʱʱ���رն�ʱ��
                      step1=OK; //����һ��� ���Ѿ��ﵽ�ȶ�״̬����������һ��
                      PrintString1("step1 is ok\n");
                    }
                  else//���δ�ﵽ��ʱʱ���ʱ��δ����
                    {

                      if(isStopped(Timer1)) //ֻ�е���ʱ����ֹͣ״̬ʱ��������ʱ����
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
	 static unsigned int mode2_ERR_times = 0;//����������
			 static bit mode2_step1 = 0;//����1��ʶλ
			 static bit mode2_step2 = 0;//����2��ʶλ
			 static bit mode2_step3 = 0;//����3��ʶλ
			 static unsigned int mode2_time = 0;//ģʽִ�д���
			 static float mode2_Sta_angle = 0.0;//���ֵ����
	while(1)
	{
			 setTimeout(Timer1,10);//�趨��ʱ��
////////////////////����1��Χ///////////////////////////
			 if(mode2_step1)//�жϲ���1�Ƿ����
			 {
////////////////////����2��Χ///////////////////////////
				 if(mode2_step2)//�жϲ���2�Ƿ����
				 {
////////////////////����3��Χ///////////////////////////
					 if(mode2_step3)//�жϲ���3�Ƿ����
					 {
						 
					 }
					 else
					 {
						 mode2_time++;//ģʽ2ִ�д����ۼ�
						 if(mode2_time == 3)//��ģʽ2����3�κ���ֹͣ����
						 {
							  setDC_MotorSpeed(LEFT_MOTOR,0.01f);
                setDC_MotorSpeed(RIGHT_MOTOR,0.99f);
                close_DC_Motor(RIGHT_MOTOR);
								close_DC_Motor(LEFT_MOTOR);
							 break;
						 }
					 }
////////////////////����3///////////////////////////
				 }
				 else
				 {
					 PID_setTargetParameter(PID_1,110.0f);//�趨Ҫ����ĵڶ����Ƕ�
					 open_DC_Motor(LEFT_MOTOR);//�����ʼ����
					 open_DC_Motor(RIGHT_MOTOR);//�����ʼ����
					 if(PID_getErr(PID_1) < 2.0f)//�ж��Ƿ����������
					 {
						 if(isExpiredTimer(Timer1))//�ж��Ƿ񵽴ﶨʱ��ʱ��
						 {
							 mode2_ERR_times++;//�������ۼ�
							 mode2_Sta_angle += abs(PID_getErr(PID_1));//����ۼ�
							 if(mode2_ERR_times >= 50)//����ۼƴ����Ƿ񵽴�50��
							 {
								 mode2_Sta_angle /= mode2_ERR_times;//����ƽ�����
								 mode2_ERR_times = 0;//����ۼƼ�������ʼ��
								 if(abs(mode2_ERR_times) < 5)//�ж��Ƿ񵽴��ȶ�
								 {
									 mode2_step2 = 1;//���벽��3
								 }
							 }
						 }
						 else
						 {
							 restartTimer(Timer1);
						 }
					 }
				 }
////////////////////////����2//////////////////////////
			 }
			 else
			 {
				 PID_setTargetParameter(PID_1,70.0f);//�趨Ҫ�ﵽ�ĵ�һ���Ƕ�
				 open_DC_Motor(LEFT_MOTOR);//�����ʼ����
				 open_DC_Motor(RIGHT_MOTOR);//�����ʼ����
				 if(PID_getErr(PID_1) < 2.0f)//�ж��Ƿ�ʼ����������
				 {
					 if(isExpiredTimer(Timer1))//�ж϶�ʱ���Ƿ񵽴�ָ��ʱ��
					 {
						 mode2_ERR_times++;
						 mode2_Sta_angle += abs(PID_getErr(PID_1));//����ۼ�
						 if(mode2_ERR_times >= 50)//�Ƿ��¼��50�����
						 {
							 mode2_Sta_angle /= mode2_ERR_times;//ȡ���ƽ��ֵ
							 mode2_ERR_times = 0;//������������
							 if(abs(mode2_ERR_times) < 5)//�ж��Ƿ񵽴��ȶ�״̬��������벽��2
							 {
								 mode2_step1 = 1;
							 }
						 }
					 }
					 else
					 {
						 restartTimer(Timer1);//������ʱ��
					 }
				 }
			 }
////////////////////////����1/////////////////////////////			 
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
