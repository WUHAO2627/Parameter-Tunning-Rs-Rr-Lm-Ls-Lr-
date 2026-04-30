
#include "AL_IDENT.h"
#include "math.h"


IDENT_PARA Ident = IDENT_DEFAULT;



//###################Motor Parameter Identification Algorithm################
void IDENT_CTRL(P_IDENT_PARA p)
{
    switch(p->IdentMainStep)
    {
        case IDENTIFY_RS:
            IDENT_Rs(p);
            break;
        case IDENTIFY_RR_LO:
            ;
            break;
        case IDENTIFY_LM_IO:
            ;
            break;
        default:
            ;
            break;
    }  
    
       
}
//###########################Identify Rs Algorithm###########################
void IDENT_Rs(P_IDENT_PARA p)
{
    switch(p->IdentSubStep)
    {
        case 1:                 //禁止PWM，配置PWM，U相上下管始终关闭
            p->sv_para.Ta = 0;              
            p->sv_para.Tb = 0;
            p->WaitCnt = 0;
            p->IdentSubStep ++;
            break;
        case 2:                //延时0.5s，使能PWM输出
            p->WaitCnt ++;
            if(p->WaitCnt > 250)
            {
               p->WaitCnt = 0; 
               p->IdentSubStep ++;
            }
            break;
        case 3:
            if(p->IsA > p->RsIuDestination)
            {
               p->RsDutyL = p->sv_para.Ta;
               p->WaitCnt = 0; 
               p->IdentSubStep ++;
            }
            else
            {
                p->sv_para.Ta += 1e-3;
                p->sv_para.Tb -= 1e-3;
            }
            break;
        case 4:
            p->WaitCnt ++;
            if(p->WaitCnt > 500)
            {
                p->RsIuAccL += p->IsA;
                p->RsUdcAccL += p->Udc;
            }
            if(p->WaitCnt >= 1000)
            {
               p->WaitCnt = 0; 
               p->IdentSubStep ++; 
            }
            break;
        case 5:
            if(p->IsA > (4.0*p->RsIuDestination))
            {
               p->RsDutyH = p->sv_para.Ta;
               p->WaitCnt = 0; 
               p->IdentSubStep ++;
            }
            else
            {
                p->sv_para.Ta += 1e-3;
                p->sv_para.Tb -= 1e-3;
            }
            break;
        case 6:
            p->WaitCnt ++;
            if(p->WaitCnt > 500)
            {
                p->RsIuAccH += p->IsA;
                p->RsUdcAccH += p->Udc;
            }
            if(p->WaitCnt >= 1000)
            {
               p->Rs = p->RsUdcAccH*p->RsDutyH - p->RsUdcAccL*p->RsDutyL;
               p->Rs = p->Rs/(p->RsIuAccH - p->RsIuAccL);
               p->Rs = 0.5 * p->Rs;
               p->WaitCnt = 0; 
               p->IdentSubStep ++; 
            }
            break;
        case 7:
        default:                 //禁止PWM输出，初始化PWM，进行下一步辨识
            p->sv_para.Ta = 0;              
            p->sv_para.Tb = 0;
            break;
    }
}



//#####################Identification Algorithm Initialize###################
void IDENT_INIT(P_IDENT_PARA p)
{
    p->y        = 0;
    p->err      = 0;
    p->err2     = 0;
    p->Rs       = 0;
    p->Rr       = 0;
    p->Ls       = 0;
    p->Lm       = 0;
    
    p->IdentMainStep = IDENTIFY_RS;
    p->IdentSubStep = 1;
    p->RsIuDestination = 25.0;
       
    p->wt1      = 0;
    p->wt2      = 0;
    p->Us_alpha_pre = 0;
    p->Us_alpha_ref = 0;
    p->Is_alpha_pre = 0;
    SV_Init(&p->sv_para);   
    
    
}

