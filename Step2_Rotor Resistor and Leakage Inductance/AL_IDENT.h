

#ifndef AL_IDENT_H_
#define AL_IDENT_H_

#include "AL_SV.h"

#define    V_TS                25.0e-6          // 辨识周期
#define    IDENTIFY_RS         1                // 定子电阻辨识
#define    IDENTIFY_RR_LO      2                // 转子电阻和漏感辨识
#define    IDENTIFY_LM_IO      3                // 互感和空载电流辨识






typedef struct
{
    int IdentMainStep;                       //辨识主步骤
    int IdentSubStep;                        //辨识分步骤
    int WaitCnt;                             //
    int EnPWM;                               //PWM使能位
    
    float RsIuDestination;                   //
    float RsIuAccL;                          //
    float RsIuAccH;                          //
    float RsUdcAccL;                         //
    float RsUdcAccH;                         //
    float RsDutyL;                           //
    float RsDutyH;                           //
    
    
    float RrDuty;                            //
    float RrCMPA;                            //
    float RrUdc;                             // 
    float RrIuDestination;                   //
    float RrIuMax;                           //
    float RrIuData[14];                      //
    float RrUdcdIdt;                         //
    float RrUzerodIdt;                       //
    float RrUdcIu;                           //
    float RrUzeroIu;                         //
    float RrAcc;
    float L0Acc;
    float RrMax;
    float RrMin;
    float L0Max;
    float L0Min;
    int RrIuFullFlag;                        //
    int RrIuDataCnt;                         //
    int RrIdentCnt;                          //
    
    
    
    
    
    float y;         //isab.a                输出
    float err;       //辨识误差
    float err2;      //辨识误差的平方         该值小于一定值时辨识结束
    float Rs;
    float Rr;
    float Ls;
    float Lm;
    float L0;
       
    float Ts;
    float wt1;
    float wt2;
    float Us_alpha_ref;
    
    float IsA;
    float IsB;
    float IsC;
    float UsA;
    float UsB;
    float UsC;
    float Is_alpha;
    float Us_alpha;
    float Is_alpha_pre;
    float Us_alpha_pre;
    float Udc;
    
    SV_PARA sv_para;
    
    
}IDENT_PARA,* P_IDENT_PARA;

extern IDENT_PARA Ident;

#define IDENT_DEFAULT    {0,0,0,0,\
                          0,0,0,0,0,0,0,\
                          0,0,0,0,0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0},0,0,0,0,0,0,0,0,0,0,0,0,0,\
                          0,0,0,0,0,0,0,0,\
                          V_TS,0,0,0,\
                          0,0,0,0,0,0,0,0,0,0,0,\
                          {0,0,0,0,0}\
}


extern void IDENT_CTRL(P_IDENT_PARA p_ident_para);
extern void IDENT_INIT(P_IDENT_PARA p_ident_para);
extern void IDENT_Rs(P_IDENT_PARA p_ident_para);
extern void IDENT_RrL0(P_IDENT_PARA p_ident_para);



#endif
