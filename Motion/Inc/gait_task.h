#ifndef __GAIT_TASK_H
#define __GAIT_TASK_H

#include "main.h"


extern u8 TROT_TIMES;

void Step_Task(void);       //̤��
void Forward_Task(void);   //ǰ��
void Back_Task(void);       //����
void Turn_Left_Task(void);  //��ת
void Turn_Right_Task(void); //��ת
void Left_Shift_Task(void); //����
void Right_Shift_Task(void);//����
void Dog_Start(void);  //��
void PA_PO_Task(void);
void Po_1_Task(void);
void Po_2_Task(void);
void Po_3_Task(void);
void Shang_Po_Task(void);
	
#endif
