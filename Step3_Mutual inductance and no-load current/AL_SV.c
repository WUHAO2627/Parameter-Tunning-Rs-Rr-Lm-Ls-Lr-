#include "AL_SV.h"


//##########################################SVPWM######################################
void SV_Ctrl(P_SV_PARA p)
{	
	 float Va,Vb,Vc,t1,t2;
	 int   Sector = 0;                                       // Sector is treated as Q0 - independently with global Q
																	
     //Inverse clarke transformation
     Va = p->Ubeta;
     Vb = (-0.5)*p->Ubeta + 0.8660254*p->Ualpha ;            // 0.8660254 = sqrt(3)/2
     Vc = (-0.5)*p->Ubeta - 0.8660254*p->Ualpha ;            // 0.8660254 = sqrt(3)/2
    
    // Va = 0.81649658*v->Ualpha;
    // Vb = (-0.40824829)*v->Ualpha + 0.70710678*v->Ubeta ;  // 0.8660254 = sqrt(3)/2
    // Vc = (-0.40824829)*v->Ualpha - 0.70710678*v->Ubeta ;  // 0.8660254 = sqrt(3)/2

    //60 degree Sector determination
    if (Va>0)
       Sector = 1; 
    if (Vb>0)
       Sector = Sector + 2;
    if (Vc>0)   
       Sector = Sector + 4;
       
    // X,Y,Z (Va,Vb,Vc) calculations
    Va = p->Ubeta;                                           // X = Va 
    Vb = 0.5*p->Ubeta + 0.8660254*p->Ualpha;                 // Y = Vb
    Vc = 0.5*p->Ubeta - 0.8660254*p->Ualpha;                 // Z = Vc
    
    if (Sector==0)                                           // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    {
       p->Ta = 0.5;
       p->Tb = 0.5;
       p->Tc = 0.5;
    }
    else if (Sector==1)                                      // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)
    {
       t1 = Vc;
       t2 = Vb;
       p->Tb = 0.5*(1-t1-t2);                               // tbon = (1-t1-t2)/2
       p->Ta = p->Tb+t1;                                    // taon = tbon+t1
       p->Tc = p->Ta+t2;                                    // tcon = taon+t2
    }
    else if (Sector==2)                                     // Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)
    {
       t1 = Vb;
       t2 = -Va;
       p->Ta = 0.5*(1-t1-t2);                              // taon = (1-t1-t2)/2
       p->Tc = p->Ta+t1;                                   // tcon = taon+t1
       p->Tb = p->Tc+t2;                                   // tbon = tcon+t2
    }      
    else if (Sector==3)                                    // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)
    {
       t1 = -Vc;
       t2 = Va;
       p->Ta = 0.5*(1-t1-t2);                              // taon = (1-t1-t2)/2
       p->Tb = p->Ta+t1;                                   // tbon = taon+t1
       p->Tc = p->Tb+t2;                                   // tcon = tbon+t2
    }   
    else if (Sector==4)                                    // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)
    {
       t1 = -Va;
       t2 = Vc;
       p->Tc = 0.5*(1-t1-t2);                             // tcon = (1-t1-t2)/2
       p->Tb = p->Tc+t1;                                  // tbon = tcon+t1
       p->Ta = p->Tb+t2;                                  // taon = tbon+t2
    }   
    else if (Sector==5)                                   // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)
    {
       t1 = Va;
       t2 = -Vb;
       p->Tb = 0.5*(1-t1-t2);                            // tbon = (1-t1-t2)/2
       p->Tc = p->Tb+t1;                                 // tcon = tbon+t1
       p->Ta = p->Tc+t2;                                 // taon = tcon+t2
    }   
    else if (Sector==6)                                  // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)
    {
       t1 = -Vb;
       t2 = -Vc;
       p->Tc = 0.5*(1-t1-t2);                           // tcon = (1-t1-t2)/2
       p->Ta = p->Tc+t1;                                // taon = tcon+t1
       p->Tb = p->Ta+t2;                                // tbon = taon+t2 
    }
    
    //Convert the unsigned GLOBAL_Q format (ranged (0,1)) -> signed GLOBAL_Q format (ranged (-1,1))
    p->Ta = 2.0*(p->Ta-0.5);
    p->Tb = 2.0*(p->Tb-0.5);
    p->Tc = 2.0*(p->Tc-0.5);        
    if (p->Ta >= 0.96)
    {
    	p->Ta = 0.96;
    }
    if (p->Ta <= -0.96)
    {
    	p->Ta = -0.96;
    }
    if (p->Tb >= 0.96)
    {
    	p->Tb = 0.96;
    }
    if (p->Tb <= -0.96)
    {
    	p->Tb = -0.96;
    }
    if (p->Tc >= 0.96)
    {
    	p->Tc =0.96;
    }
    if (p->Tc <= -0.96)
    {
    	p->Tc = -0.96;
    }
}



//####################################SVPWM Dead zone compensation################################
void PWM_Deadzone(float Isa, float Isb, float Isc, P_SV_PARA p)
{
     if(Isa > 0)
     {
         p->Ta = p->Ta + switch_conpensate;
     }
     if(Isa < 0)
     {
         p->Ta = p->Ta - switch_conpensate;
     }
     if(Isb > 0)
     {
         p->Tb = p->Tb + switch_conpensate;
     }
     if(Isb < 0)
     {
         p->Tb = p->Tb - switch_conpensate;
     }
     if(Isc > 0)
     {
         p->Tc = p->Tc + switch_conpensate;
     }
     if(Isc < 0)
     {
         p->Tc = p->Tc - switch_conpensate;
     }
}


//##########################################SVPWM Initialize######################################
void SV_Init(P_SV_PARA p)
{
    p->Ualpha = 0;
    p->Ubeta  = 0;
    p->Ta = 0;
    p->Tb = 0;
    p->Tc = 0;
}




