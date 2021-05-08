#ifndef __MOTION_H
#define __MOTION_H

/* �ٶ�ʸ�� */
typedef struct
{
	float module;
	float direction;
}vector_t;

typedef struct
{
	float x;
	float y;
}Speed_t;

/* λ�õ�Ľṹ�� ��λmm */
typedef struct
{
	float x;
	float y;
}Point_t;

/* ��̬�ṹ�壬�Ե�бʽ��������������λ�����ڸ�λ������̬ ��б���ýǶ��ƵĽǶȴ��� */
typedef struct
{
	Point_t point;
	/* �Ƕ��� */
	float   direction;
	/* �ٶ� */
	float vel;
}Pose_t;

typedef struct
{
	Point_t point;
	float vel;
	float direction;
	unsigned short startPtr;
	unsigned short endPtr;
}PointU_t;

/* Exported constants --------------------------------------------------------*/

/* �Ƕ��ƻ����ƻ���ϵ�� */
#define CHANGE_TO_RADIAN    0.01745329251994f   	//�Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_ANGLE     57.29577951308232f		//������ת��Ϊ�Ƕ���ϵ��

#define SQRT_2	1.414213562373f	

#ifndef PI
#define PI                  3.1415926f				//Բ����
#endif

#endif
