

#include "AL_MOTOR.h"

int i = 4501;
float x1_old = 0;
float x2_old = 0;
float x3_old = 0;
float x4_old = 0;

//*************************VF控制加状态观测器控制**************************//
void MOTOR_Control(P_MOTOR_PARA p)
{ 
    
    //******************************角度计算******************************//

    p->w_vf   =   30;
    p->wt_vf   +=  p->w_vf  *  p->Ts;  
    if (p->wt_vf  >  V_2PIE)
    {
        p->wt_vf  =  p->wt_vf  -  V_2PIE;
    }
    if(p->wt_vf  <  -V_2PIE)
    {
        p->wt_vf  =  p->wt_vf  +  V_2PIE;
    }    

    p->we      =    10;                                                 
    p->wt     +=  p->we  *  p->Ts;                                                     
    if (p->wt  >  V_2PIE)
    {
        p->wt  =  p->wt - V_2PIE;
    }
    if (p->wt  <  -V_2PIE)
    {
        p->wt  =  p->wt + V_2PIE;
    }
    


    //**************************CLARKE PARK变换***************************//
    p->Isab.X  =   p->Is.A;                                            
    p->Isab.Y  =  (p->Is.B  -  p->Is.C)  *  V_1DS3;          
                                 
  
    //**************************Identification***************************//

    ident.y = p->Isab.X;
    
    ident.w1 = ident.w1*0.9824 + p->Usab.X*0.00011013+x1_old*0.00011013;
    ident.w2 = ident.w2*0.9912 + p->Usab.X*0.00011062+x2_old*0.00011062;
    ident.w3 = ident.w3*0.9824 + p->Isab.X*0.00011013+x3_old*0.00011013;
    ident.w4 = ident.w3*0.9912 + p->Isab.X*0.00011062+x4_old*0.00011062; 
    
    x1_old   = p->Usab.X;
    x2_old   = p->Usab.X;
    x3_old   = p->Isab.X;
    x4_old   = p->Isab.X;    
    
    if(i>4500)
    {
        IDENT_Ctrl(&ident);
    }
    else
    {
        i++;
    }
    
//     ident.w1 = ident.w1*0.9824 + p->Usab.X*0.00022;
//     ident.w2 = ident.w2*0.9912 + p->Usab.X*0.00022;
//     ident.w3 = ident.w3*0.9824 + p->Isab.X*0.00022;
//     ident.w4 = ident.w3*0.9912 + p->Isab.X*0.00022; 

    //******************************SVPWM********************************//        
    p->Ua_out  =  0.03+0.004*sin(p->wt_vf)+0.003*sin(p->wt);
    p->Ub_out  =  0;
    p->sv_para.Ualpha =   -p->Usab.X/p->Udc/V_1DS3;;                                                      //
    p->sv_para.Ubeta  =  -p->Ub_out;                                                      //                                
    SV_Ctrl(&p->sv_para);                                                                 //SVPWM调制
 //   PWM_Deadzone(p->Is.A,p->Is.B,p->Is.C,&p->sv_para);                                  //死区补偿2us
   
}


void MOTOR_Init(P_MOTOR_PARA p)
{
    //******************************初始化********************************//
    p->motor_state = V_ZERO;
    p->n_run  =  V_ZERO;
    p->n_g    =  V_ZERO;
    p->wr_g   =  V_ZERO;
    p->ws     =  V_ZERO;
    p->we     =  V_ZERO;
    p->wt     =  V_ZERO;
    p->wt_vf  =  V_ZERO;
    p->f_run  =  V_ZERO;
    p->Ialpha_g     =  V_ZERO;                                                 
    p->Ibeta_g      =  V_ZERO;
    p->Fr_alpha_g   =  V_ZERO;
    p->Fr_beta_g    =  V_ZERO;
    SV_Init(&p->sv_para);
    IDENT_Init(&ident);
}

