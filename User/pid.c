#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Speed_Init(speed_ID *pid)
{
	pid->M3508_ID1.mode=pid->M3508_ID2.mode=pid->M3508_ID3.mode=pid->M3508_ID4.mode=pid->M3508_ID5.mode=pid->M3508_ID6.mode=pid->M3508_ID7.mode=PID_POSITION;
	pid->M3508_ID1.max_out=pid->M3508_ID2.max_out=pid->M3508_ID3.max_out=pid->M3508_ID4.max_out=pid->M3508_ID5.max_out=pid->M3508_ID6.max_out=pid->M3508_ID7.max_out=PID_SPEED_MAX_OUT;
	pid->M3508_ID1.max_iout=pid->M3508_ID2.max_iout=pid->M3508_ID3.max_iout=pid->M3508_ID4.max_iout=pid->M3508_ID5.max_iout=pid->M3508_ID6.max_iout=pid->M3508_ID7.max_iout=400;

	pid->M3508_ID1.kp=10.0;
	pid->M3508_ID1.ki=0.0;
	pid->M3508_ID1.kd=1.5;

	pid->M3508_ID2.kp=10.0;
	pid->M3508_ID2.ki=0.0;
	pid->M3508_ID2.kd=1.5;

	pid->M3508_ID3.kp=18.0;
	pid->M3508_ID3.ki=0.0;
	pid->M3508_ID3.kd=1.5;

	pid->M3508_ID4.kp=15.0;
	pid->M3508_ID4.ki=0.0;
	pid->M3508_ID4.kd=1.5;

	pid->M3508_ID5.kp=10.0;
	pid->M3508_ID5.ki=0.0;
	pid->M3508_ID5.kd=1.5;

	pid->M3508_ID6.kp=10.0;
	pid->M3508_ID6.ki=0.0;
	pid->M3508_ID6.kd=1.5;

	pid->M3508_ID7.kp=10.0;
	pid->M3508_ID7.ki=0.0;
	pid->M3508_ID7.kd=1.5;
}

void PID_Angle_Init(angle_ID *pid)
{
	pid->M3508_ID1.mode=pid->M3508_ID2.mode=pid->M3508_ID3.mode=pid->M3508_ID4.mode=pid->M3508_ID5.mode=pid->M3508_ID6.mode=pid->M3508_ID7.mode=PID_POSITION;
	pid->M3508_ID1.max_out=pid->M3508_ID2.max_out=pid->M3508_ID3.max_out=pid->M3508_ID4.max_out=pid->M3508_ID5.max_out=pid->M3508_ID6.max_out=pid->M3508_ID7.max_out=PID_ANGLE_MAX_OUT;
	pid->M3508_ID1.max_iout=pid->M3508_ID2.max_iout=pid->M3508_ID3.max_iout=pid->M3508_ID4.max_iout=pid->M3508_ID5.max_iout=pid->M3508_ID6.max_iout=pid->M3508_ID7.max_iout=60;

	pid->M3508_ID1.kp=8.2;
	pid->M3508_ID1.ki=0.0;
	pid->M3508_ID1.kd=25.14;

	pid->M3508_ID2.kp=8.2;
	pid->M3508_ID2.ki=0.0;
	pid->M3508_ID2.kd=25.14;

	pid->M3508_ID3.kp=8.2;
	pid->M3508_ID3.ki=0.0;
	pid->M3508_ID3.kd=25.14;

	pid->M3508_ID4.kp=8.2;
	pid->M3508_ID4.ki=0.0;
	pid->M3508_ID4.kd=25.14;

	pid->M3508_ID5.kp=5.0;
	pid->M3508_ID5.ki=0.0;
	pid->M3508_ID5.kd=50.0;

	pid->M3508_ID6.kp=5.0;
	pid->M3508_ID6.ki=0.0;
	pid->M3508_ID6.kd=50.0;

	pid->M3508_ID7.kp=15.0;
	pid->M3508_ID7.ki=0.0;
	pid->M3508_ID7.kd=50.0;
}

float PID_Speed_Calc_3508(M3508_msg *motor_msg, speed_pid_t *pid)	
{
	if(pid == NULL)
	{
		return 0.0f;
	}
	if(pid->mode == PID_POSITION)
	{
		pid->err[2]=pid->err[1];
		pid->err[1]=pid->err[0];
		pid->set=motor_msg->target_speed;
		pid->get=motor_msg->speed;
		pid->err[0]=motor_msg->target_speed-motor_msg->speed;
		pid->pout=pid->kp*pid->err[0];
		pid->iout+=pid->ki*pid->err[0];
		pid->dbuf[2]=pid->dbuf[1];
		pid->dbuf[1]=pid->dbuf[0];
		pid->dbuf[0]=pid->err[0]-pid->err[1];
		pid->dout=pid->kd*pid->dbuf[0];
		LimitMax(pid->iout,pid->max_iout);
		pid->out=pid->pout+pid->iout+pid->dout;
		LimitMax(pid->out,pid->max_out);
	}
	else if(pid->mode == PID_DELAT)
	{
		pid->err[2]=pid->err[1];
		pid->err[1]=pid->err[0];
		pid->set=motor_msg->target_speed;
		pid->get=motor_msg->speed;
		pid->err[0]=motor_msg->target_speed-motor_msg->speed;
		pid->pout=pid->kp*(pid->err[0]-pid->err[1]);
		pid->iout=pid->ki*pid->err[0];
		pid->dout=(pid->err[0]-2.0f*pid->err[1]+pid->err[2]);
		LimitMax(pid->iout,pid->max_iout);
		pid->out+=pid->pout+pid->iout+pid->dout;
		LimitMax(pid->out,pid->max_out);
	}
	return pid->out;
}

float PID_Angle_Calc_3508(M3508_msg *motor_msg, angle_pid_t *pid)	
{
	if(pid == NULL)
	{
		return 0.0f;
	}
	if(pid->mode == PID_POSITION)
	{
		pid->err[2]=pid->err[1];
		pid->err[1]=pid->err[0];
		pid->set=motor_msg->target_ecd;
		pid->get=motor_msg->total_ecd;
		pid->err[0]=motor_msg->target_ecd-motor_msg->total_ecd;
		pid->pout=pid->kp*pid->err[0];
		pid->iout+=pid->ki*pid->err[0];
		pid->dout=pid->kd*(pid->err[0]-pid->err[1]);
		LimitMax(pid->iout,pid->max_iout);
		pid->out=pid->pout+pid->iout+pid->dout;
		LimitMax(pid->out,pid->max_out);
	}
	else if(pid->mode == PID_DELAT)
	{
		pid->err[2]=pid->err[1];
		pid->err[1]=pid->err[0];
		pid->set=motor_msg->target_ecd;
		pid->get=motor_msg->total_ecd;
		pid->err[0]=motor_msg->target_ecd-motor_msg->total_ecd;
		pid->pout=pid->kp*(pid->err[0]-pid->err[1]);
		pid->iout=pid->ki*pid->err[0];
		pid->dout=(pid->err[0]-2.0f*pid->err[1]+pid->err[2]);
		LimitMax(pid->iout,pid->max_iout);
		pid->out+=pid->pout+pid->iout+pid->dout;
		LimitMax(pid->out,pid->max_out);
	}
	return pid->out;
}

void get_total_3508_angle(M3508_msg *p)
{
	int res1=0,res2=0,delta=0;
	if(p->cnt!=0)
   	{
    	if(p->ecd < p->last_ecd)
      	{	
        	res1 = p->ecd + 8192 - p->last_ecd;	
        	res2 = p->ecd - p->last_ecd;				
      	}
    	else
      	{	
        	res1 = p->ecd - 8192 - p->last_ecd ;
        	res2 = p->ecd - p->last_ecd;				
      	}
    	if(abs(res1)<abs(res2))
      		delta = res1;
    	else
      		delta = res2;
      	p->total_ecd += delta;
	}
  	else
		p->cnt=1;
  	p->last_ecd=p->ecd;
}
