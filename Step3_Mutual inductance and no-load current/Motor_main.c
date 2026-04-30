#define S_FUNCTION_NAME Motor_main
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#include "AL_IDENT.h"


float systime;
static int enPWM=0;
static int KM202=0;
static int Precharge_finish=0;
float Udc;
int timer2ms;
int TBCTR = 0;





FILE* fp=NULL;
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }
    if (!ssSetNumInputPorts(S, 4)) return;
    ssSetInputPortWidth(S, 0, 3); 
    ssSetInputPortDirectFeedThrough(S, 0, 3);
    ssSetInputPortWidth(S, 1, 3); 
    ssSetInputPortDirectFeedThrough(S, 1, 3);
    ssSetInputPortWidth(S, 2, 1); 
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortWidth(S, 3, 1); 
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    
    if (!ssSetNumOutputPorts(S,7)) return;              
    ssSetOutputPortWidth(S, 0, 4);
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortWidth(S, 2, 1);
	ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortWidth(S, 4, 6);   
    ssSetOutputPortWidth(S, 5, 5);              
    ssSetOutputPortWidth(S, 6, 4);  
        
    ssSetNumSampleTimes(S, 1);
    
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, V_TS);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtr0 = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType uPtr1 = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType uPtr2 = ssGetInputPortRealSignalPtrs(S,2);
    InputRealPtrsType uPtr3 = ssGetInputPortRealSignalPtrs(S,3);
    
    real_T            *y0   = ssGetOutputPortRealSignal(S,0);
    real_T            *y1   = ssGetOutputPortRealSignal(S,1);
    real_T            *y2   = ssGetOutputPortRealSignal(S,2);
    real_T            *y3   = ssGetOutputPortRealSignal(S,3);
    real_T            *y4   = ssGetOutputPortRealSignal(S,4);
    real_T            *y5   = ssGetOutputPortRealSignal(S,5);
    real_T            *y6   = ssGetOutputPortRealSignal(S,6);
    

     Ident.IsA    =  (real_T)(*uPtr0[0]);
     Ident.IsB    =  (real_T)(*uPtr0[1]);
     Ident.IsC    =  (real_T)(*uPtr0[2]);
     Ident.UsA    =  (real_T)(*uPtr1[0]);
     Ident.UsB    =  (real_T)(*uPtr1[1]);                           
     Ident.UsC    =  (real_T)(*uPtr1[2]);                             
     Ident.Udc    =  (real_T)(*uPtr2[0]);
     systime      =  (real_T)(*uPtr3[0]);
     Udc = Ident.Udc;
     


     
     

     if (systime  <=  0.01)
     {
         IDENT_INIT(&Ident);
      }
     if ((systime  >=  0.1)&&(Udc >= 80))
     {
         KM202 = 1;
         Precharge_finish = 1;
     }
       
     if (Precharge_finish == 1)
     {
         if(timer2ms>=10)
         {
            IDENT_CTRL(&Ident);
            timer2ms = 0;
         }
         else
         {
             timer2ms++;
         }
         IDENT_VF(&Ident);

     }
//      if(Ident.EnPWM == 0)
//      {
//          enPWM = 0;
//      }
//      
//      if(TBCTR>=100)
//      {
//          TBCTR = 1;
//          if(Ident.EnPWM == 1)
//          {
//              enPWM = 1;
//          }
//      }
//      else
//      {
//          TBCTR ++;
//      }
//      
//      if(enPWM ==1)
//      {
//          Ident.RrUdc = Ident.Udc;
//          if(TBCTR == (Ident.RrCMPA+1))
//          {
//             Ident.RrIuData[Ident.RrIuDataCnt]=Ident.IsA; 
//             if(Ident.RrIuDataCnt ==0)
//             {
//             Ident.RrIuMax = Ident.RrIuData[Ident.RrIuDataCnt];
//             }
//             if(Ident.RrIuMax < Ident.RrIuData[Ident.RrIuDataCnt])
//             {
//              Ident.RrIuMax = Ident.RrIuData[Ident.RrIuDataCnt];
//             }
//             Ident.RrIuDataCnt++;
//          }
//          if(TBCTR == 100)
//          {
//             Ident.RrIuData[Ident.RrIuDataCnt]=Ident.IsA; 
//             if(Ident.RrIuDataCnt ==0)
//             {
//             Ident.RrIuMax = Ident.RrIuData[Ident.RrIuDataCnt];
//             }
//             if(Ident.RrIuMax < Ident.RrIuData[Ident.RrIuDataCnt])
//             {
//              Ident.RrIuMax = Ident.RrIuData[Ident.RrIuDataCnt];
//             }
//             Ident.RrIuDataCnt++;
//          }
//          if(Ident.RrIuDataCnt >=14)
//          {
//             Ident.RrIuFullFlag = 1;
//             Ident.RrIuDataCnt = 0;
//             Ident.EnPWM = 0;
//          }
//          
//      }

     
     
     
     
     
        
     y0[0]  =  (real_T)Ident.sv_para.Ta;
     y0[1]  =  (real_T)Ident.sv_para.Tb;
     y0[2]  =  (real_T)Ident.sv_para.Tc; 
     y0[3]  =  (real_T)Ident.EnPWM; 
     y1[0]  =  (real_T)KM202;
     y2[0]  =  (real_T)Ident.LmIm;
     y3[0]  =  (real_T)Ident.LmTheta;
     
     
     y4[0]  =  (real_T)(Ident.err);
     y4[1]  =  (real_T)(Ident.I0);
     y4[2]  =  (real_T)(Ident.Rs);
     y4[3]  =  (real_T)(Ident.Rr);
     y4[4]  =  (real_T)(Ident.L0);
     y4[5]  =  (real_T)(Ident.Lm);
     
     
     y5[0]  =  (real_T)(0);
     y5[1]  =  (real_T)(0);
     y5[2]  =  (real_T)(0);
     y5[3]  =  (real_T)(0);
     y5[4]  =  (real_T)(0);
     
     
     y6[0]  =  (real_T)(Ident.RrIuMax);
     y6[1]  =  (real_T)(Ident.IsB);
     y6[2]  =  (real_T)(Ident.Is_alpha);
     y6[3]  =  (real_T)(Ident.Us_alpha);
     
     
     
     

}

static void mdlTerminate(SimStruct *S)
{
	if(fp)
	{
		fclose(fp);
		fp=NULL;
	}
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

