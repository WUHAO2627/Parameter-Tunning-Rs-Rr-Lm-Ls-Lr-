
#ifndef AL_MOTOR_H_
#define AL_MOTIR_H_

#include "math.h"
#include "AL_SV.h"
#include "AL_IDENT.h"



#define UP_LIMIT(x,limit)             (((x) > (limit)) ? (limit) : (x))
#define DW_LIMIT(x,limit)             (((x) < (limit)) ? (limit) : (x))


#define V_ITSD        5000                               //开关频率
#define V_ITS         2.0e-4                             //控制周期
#define V_2PIE        6.28318530717958647692528676
#define V_1DS3        0.577350269189626
#define V_ZERO        0.0
#define V_S3DS2       1.224744871391589


                  

typedef struct
{
	float X;
	float Y; 
}GROUP_TWO,* P_GROUP_TWO;

typedef struct
{
	float A;
	float B;
	float C;  
}GROUP_THREE,* P_GROUP_THREE;

typedef struct
{
    int motor_state;                                      //电机运行状态 0：变频启动  1：变频加估算角度启动  2：转速闭环控制
    float Ts;                                             
    float f_run;                                          
    float f_ref;                                          
    float n_run;                                          
    float n_ref;                                          
    float n_g;                                            
    float wr_g;                                               
    float Us_rms;                                         
    float Is_rms;                                         
    
    float wt;                                             
    float we;                                             
    float ws;                                                
    float w_vf;                                           
    float wt_vf;                                              
    float n_rel;
    float Fr_alpha_g;                                     
    float Fr_beta_g;                                      
    float Ialpha_g;                                       
    float Ibeta_g;                                        
    
    float SinTemp;                                        
    float CosTemp;                                        
    float SinTemp_vf;                                     
    float CosTemp_vf;                                     
    float Udc;
    float Ua_out;
    float Ub_out;
    
    GROUP_THREE Us;                                                          
	GROUP_THREE Is;                                       
    GROUP_TWO Isab;                                       
    GROUP_TWO Isdq;                                       
    GROUP_TWO Usab;                                                                         
    SV_PARA sv_para;                                           
}MOTOR_PARA,* P_MOTOR_PARA;

extern MOTOR_PARA motor;

#define MOTOR_DEFAULT    {0,V_ITS,0,3.5,20,1480,0,0,0,0,\
                          0,0,0,0,0,0,0,0,0,0,\
                          0,0,0,0,0,0,0,\
                          {0,0,0},{0,0,0},\
                         {0,0},{0,0},{0,0},\
				         {0,0,0,0,0}\
}

extern void MOTOR_Control(P_MOTOR_PARA p);
extern void MOTOR_Init(P_MOTOR_PARA p);


#endif

