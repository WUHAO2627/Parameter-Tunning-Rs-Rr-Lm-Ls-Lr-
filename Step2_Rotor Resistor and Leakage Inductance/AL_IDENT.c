

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
            IDENT_RrL0(p);
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


//#######################Identify Rr and L0 Algorithm#######################
void IDENT_RrL0(P_IDENT_PARA p)
{
    switch(p->IdentSubStep)
    {
        case 1:                 //禁止PWM
            p->EnPWM = 0;              
            p->WaitCnt = 0;
            p->IdentSubStep ++;
            break;
        case 2:                //延时2s，配置PWM，U相上下管关闭,V相上管关闭下管开通，W相上管关闭下管开通，无死区,使能PWM输出
            p->WaitCnt ++;
            if(p->WaitCnt > 250)   //0.5s=250   2s=1000
            {
               p->sv_para.Ta = 0.05;
               p->RrDuty = p->sv_para.Ta;
               p->RrCMPA = 5;
               p->EnPWM = 1;
               p->WaitCnt = 0; 
               p->RrIuFullFlag = 0;
               p->RrIuDataCnt = 0;
               p->IdentSubStep ++;
            }
            break;
        case 3:   //调节占空比，直到输出电流达到额定值
            if(p->RrIuFullFlag == 1)
            {
                p->EnPWM = 0;
                p->WaitCnt = 0;
                if(p->RrIuMax < p->RrIuDestination)
                {
                   p->sv_para.Ta += 0.03; 
                   p->RrDuty = p->sv_para.Ta;
                   p->RrCMPA += 3;
                   p->IdentSubStep = 4;
                   p->RrIuFullFlag = 0;
                }
                else
                {
                    p->RrDuty = p->sv_para.Ta;
                    p->RrCMPA = p->RrCMPA;
                    p->RrIuFullFlag == 1;
                    p->IdentSubStep = 5;
                    p->RrIdentCnt = 0;
                    p->RrAcc = 0;
                    p->L0Acc = 0;
                }
                p->RrIuMax = 0;
            }
            break;
        case 4:          //延时100ms
            p->WaitCnt ++;
            if(p->WaitCnt > 50)
            {
                p->EnPWM = 1;
                p->IdentSubStep = 3;
            }
            break;
        case 5:
            if(p->RrIuFullFlag == 1)
            {
                p->EnPWM = 0;
                p->WaitCnt = 0;
                p->RrIdentCnt ++;
                //***************辨识转子电阻和漏感*******************//
                p->RrUzerodIdt = p->RrIuData[13]-p->RrIuData[12]+p->RrIuData[11]-p->RrIuData[10]+p->RrIuData[9]-p->RrIuData[8]+p->RrIuData[7]-p->RrIuData[6]+p->RrIuData[5]-p->RrIuData[4];
                p->RrUzerodIdt = -p->RrUzerodIdt;
                p->RrUzerodIdt = p->RrUzerodIdt / (1 - p->RrDuty + 0.01);
                p->RrUdcdIdt = p->RrIuData[12]-p->RrIuData[11]+p->RrIuData[10]-p->RrIuData[9]+p->RrIuData[8]-p->RrIuData[7]+p->RrIuData[6]-p->RrIuData[5]+p->RrIuData[4]-p->RrIuData[3];
                p->RrUdcdIdt = p->RrUdcdIdt / (p->RrDuty - 0.01);
                p->RrUzeroIu = p->RrIuData[13]+p->RrIuData[12]+p->RrIuData[11]+p->RrIuData[10]+p->RrIuData[9]+p->RrIuData[8]+p->RrIuData[7]+p->RrIuData[6]+p->RrIuData[5]+p->RrIuData[4];
                p->RrUzeroIu = p->RrUzeroIu * 0.1;
                p->RrUdcIu = p->RrIuData[12]+p->RrIuData[11]+p->RrIuData[10]+p->RrIuData[9]+p->RrIuData[8]+p->RrIuData[7]+p->RrIuData[6]+p->RrIuData[5]+p->RrIuData[4]+p->RrIuData[3];
                p->RrUdcIu = p->RrUdcIu * 0.1;
                p->Rr = 0.66666667 * p->RrUdc * p->RrUzerodIdt / (p->RrUzeroIu * p->RrUdcdIdt + p->RrUdcIu * p->RrUzerodIdt); 
                p->L0 = p->RrUzeroIu * p->Rr / (p->RrUzerodIdt/0.0025/5);
                p->L0 = p->L0 * 0.5;
                p->Rr = p->Rr - 0.048;
                //***************辨识结果累加处理*******************//
                if(p->RrIdentCnt == 1)
                {
                    p->RrMax = p->Rr;
                    p->RrMin = p->Rr;
                    p->L0Max = p->L0;
                    p->L0Min = p->L0;
                }
                else
                {
                    if(p->RrMax < p->Rr)
                    {
                        p->RrMax = p->Rr;
                    }
                    if(p->RrMin > p->Rr)
                    {
                        p->RrMin = p->Rr;
                    }
                    if(p->L0Max < p->L0)
                    {
                        p->L0Max = p->L0;
                    }
                    if(p->L0Min > p->L0)
                    {
                        p->L0Min = p->L0;
                    }
                }
                
                p->RrAcc += p->Rr;
                p->L0Acc += p->L0;
                        
                if(p->RrIdentCnt >=6)
                {
                   p->RrIuFullFlag = 0;
                   p->Rr = (p->RrAcc - p->RrMax - p->RrMin)*0.25;
                   p->L0 = (p->L0Acc - p->L0Max - p->L0Min)*0.25;
                }
                else
                {
                    p->RrIuFullFlag = 0;
                    p->IdentSubStep = 6;
                }
            }
            break;
        case 6:         //延时200ms
            p->WaitCnt ++;
            if(p->WaitCnt > 200)
            {
                p->EnPWM = 1;
                p->IdentSubStep = 5;
            }
            break;
        case 7:
        default:                 //禁止PWM输出，初始化PWM，进行下一步辨识
            ;
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
    
//     p->IdentMainStep = IDENTIFY_RS;
//     p->IdentSubStep = 1;
//     p->RsIuDestination = 25.0;
    
    
    p->IdentMainStep = IDENTIFY_RR_LO;
    p->IdentSubStep = 1;
    p->RrIdentCnt = 0;
    p->RrIuDestination = 100.0;
    p->EnPWM = 0;
       
    p->wt1      = 0;
    p->wt2      = 0;
    p->Us_alpha_pre = 0;
    p->Us_alpha_ref = 0;
    p->Is_alpha_pre = 0;
    SV_Init(&p->sv_para);   
    
    
}

