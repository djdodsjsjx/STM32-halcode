#include "oled.h"
#include "oledfont.h"  
#include <math.h>

//��OLEDд��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
static void OLED_WR_Byte(uint8_t dat,uint8_t cmd) {	
	uint8_t i;			  
	if(cmd) OLED_DC_Set();
	else OLED_DC_Clr();		  
	for (i = 0; i < 8; i++) {			  
		OLED_SCL_Clr();
		if (dat & 0x80) OLED_SDA_Set();
		else OLED_SDA_Clr();
		OLED_SCL_Set();
		dat <<= 1;   
	}				 		  
	OLED_DC_Set();   	  
} 


//��ʼ��OLED					    
extern void OLED_Init(void) { 	
	
	OLED_RST_Clr();
	HAL_Delay(100);
	OLED_RST_Set(); 

	OLED_WR_Byte(0xAE, OLED_CMD); //�ر���ʾ
	OLED_WR_Byte(0xD5, OLED_CMD); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WR_Byte(80, OLED_CMD);   //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WR_Byte(0xA8, OLED_CMD); //��������·��
	OLED_WR_Byte(0X3F, OLED_CMD); //Ĭ��0X3F(1/64) 
	OLED_WR_Byte(0xD3, OLED_CMD); //������ʾƫ��
	OLED_WR_Byte(0X00, OLED_CMD); //Ĭ��Ϊ0

	OLED_WR_Byte(0x40, OLED_CMD); //������ʾ��ʼ�� [5:0],����.
													    
	OLED_WR_Byte(0x8D, OLED_CMD); //��ɱ�����
	OLED_WR_Byte(0x14, OLED_CMD); //bit2������/�ر�
	OLED_WR_Byte(0x20, OLED_CMD); //�����ڴ��ַģʽ
	OLED_WR_Byte(0x02, OLED_CMD); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Byte(0xA1, OLED_CMD); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0, OLED_CMD); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Byte(0xDA, OLED_CMD); //����COMӲ����������
	OLED_WR_Byte(0x12, OLED_CMD); //[5:4]����
		 
	OLED_WR_Byte(0x81, OLED_CMD); //�Աȶ�����
	OLED_WR_Byte(0xEF, OLED_CMD); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Byte(0xD9, OLED_CMD); //����Ԥ�������
	OLED_WR_Byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB, OLED_CMD); //����VCOMH ��ѹ����
	OLED_WR_Byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4, OLED_CMD); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Byte(0xA6, OLED_CMD); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ	    						   
	OLED_WR_Byte(0xAF, OLED_CMD); //������ʾ	
	
	
	OLED_Clear();

	
}


uint8_t OLED_GRAM[128][8];	 
extern void OLED_Refresh_Gram(void) {
	uint8_t i,n;		    
	for (i = 0; i < 8; i++) {  
		OLED_WR_Byte(0xb0 + i, OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte(0x00, OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte(0x10, OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for (n = 0; n < 128; n++) OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA); 
	}   
}

//����OLED��ʾ    
extern void OLED_Display_On(void) {
	OLED_WR_Byte(0X8D, OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14, OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF, OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
extern void OLED_Display_Off(void) {
	OLED_WR_Byte(0X8D, OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10, OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE, OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
extern void OLED_Clear(void) {  
	uint8_t i,n;  
	for (i = 0; i < 8; i++) {
		for(n = 0; n < 128; n++) {
			OLED_GRAM[n][i] = 0X00; 
		}
	}  
	OLED_Refresh_Gram();//������ʾ
}
//���� 
//x:0~127
//y:0~63
//t:1 ��� 0,���				   
extern void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t) {
	uint8_t pos, bx, temp = 0;
	if (x > 127 || y > 63) return;//������Χ��.
	pos = 7 - y / 8;
	bx = y % 8;
	temp = 1 << (7 - bx);
	if (t) OLED_GRAM[x][pos] |= temp;
	else OLED_GRAM[x][pos] &= ~temp;	    
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
extern void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode) {      			    
	uint8_t temp, t, t1;
	uint8_t y0 = y;
	chr = chr - ' ';//�õ�ƫ�ƺ��ֵ				   
    for (t = 0; t < size; t++) {   
		if (size == 12) temp = oled_asc2_1206[chr][t];  //����1206����
		else temp = oled_asc2_1608[chr][t];		 //����1608���� 	                          
        for (t1 = 0; t1 < 8; t1++) {
			if (temp & 0x80) OLED_DrawPoint(x, y, mode);
			else OLED_DrawPoint(x, y, !mode);
			temp <<= 1;
			y++;
			if((y - y0) == size) {
				y = y0;
				x++;
				break;
			}
		}  	 
    }          
}
			  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	

//void OLED_ShowNumber(uint8_t x, uint8_t y, int16_t num, uint8_t len, uint8_t size) {         	
//	uint8_t t, temp;
//	uint8_t enshow = 0;
//	if (num < 0) {
//		OLED_ShowChar(x, y, '-', 16, 1);
//		x += size / 2;
//		num = -num;
//	}
//	for (t = 0; t < len; t++) {
//		temp = (num / my_pows(10, len - t - 1)) % 10;
//		if (enshow == 0 && t < len - 1) {
//			if (temp == 0) {
//				OLED_ShowChar(x + size / 2 * t, y, ' ', size, 1);
//				continue;
//			}else enshow = 1; 
//		 	 
//		}
//	 	OLED_ShowChar(x + size / 2 * t, y, temp + '0', size, 1); 
//	}
//} 

extern void OLED_ShowNumber(uint8_t x, uint8_t y, int16_t num, uint8_t len, uint8_t size) {         	
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++) {
		temp = (int16_t)(num / pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < len-1) {
			if (temp == 0) {
				OLED_ShowChar(x + size / 2 * t, y, ' ', size, 1);
				continue;
			}else enshow = 1; 
		 	 
		}
	 	OLED_ShowChar(x + size / 2 * t, y, temp + '0', size, 1); 
	}
} 

//��ʾ�ַ���
//x,y:�������  
//*p:�ַ�����ʼ��ַ
//��16����
extern void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p) {
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while (*p != '\0') {       
        if (x > MAX_CHAR_POSX ) {
			x = 0;
			y += 16;
		}
        if (y > MAX_CHAR_POSY) {
			y = x = 0;
			OLED_Clear();
		}
        OLED_ShowChar(x, y, *p, 16, 1);	 
        x += 8;
        p++;
    }  
}	






