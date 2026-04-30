



#ifndef AL_SV_H_
#define AL_SV_H_


#define   switch_conpensate   0.0117//0.0297(¼ÓIGBTµ¼Í¨Ñ¹½µ0.9V²¹³¥)//0.0117        //1.3us                    // 2us  0.018

typedef struct 	{ float  Ualpha; 			// Input: reference alpha-axis phase voltage 
				  float  Ubeta;			    // Input: reference beta-axis phase voltage 
				  float  Ta;				// Output: reference phase-a switching function		
				  float  Tb;				// Output: reference phase-b switching function 
				  float  Tc;				// Output: reference phase-c switching function
				}SV_PARA,* P_SV_PARA;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																				
extern void SV_Ctrl(P_SV_PARA p);
extern void SV_Init(P_SV_PARA p);
extern void PWM_Deadzone(float Isa, float Isb, float Isc, P_SV_PARA p);

#endif

