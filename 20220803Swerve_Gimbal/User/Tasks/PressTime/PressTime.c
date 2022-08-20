#include "PressTime.h"

/* USER CODE END Header_PressTime */
extern Shoot_Control_t shoot_control; 
extern volatile int32_t WaitTimeForShoot;

uint8_t Q_mode=0;
uint8_t E_mode=0;
uint8_t Ctrl_mode=0;
uint8_t Shift_mode=0;
uint8_t R_mode=0;
uint8_t G_mode=2;
uint8_t shift_step=1;
uint8_t F_mode=0;
uint8_t X_mode=0;
uint8_t Z_mode=0;
uint8_t V_mode=0;

int Q_cnt=0;
int E_cnt=0;
int Ctrl_cnt=0;
int	Shift_cnt=0;
int	R_cnt=0;
int	G_cnt=0;
int	F_cnt=0;
int	X_cnt=0;
int	Z_cnt=0;
int	V_cnt=0;
void PressTime(void const * argument)
{
  /* USER CODE BEGIN PressTime */
  /* Infinite loop */

  osDelay(2000);
  WaitTimeForShoot = 0;
  for(;;)
  {
		
		
			
		if(IF_KEY_PRESSED_Q){
			
			if(Q_mode==0){
			if(Q_cnt<Enable_Count)
		Q_cnt++;
		 else if(Q_cnt>=Enable_Count)
		Q_cnt=Enable_Count;
	  }
			if(Q_mode==1){
			if(Q_cnt>0)
		Q_cnt--;
		 else if(Q_cnt<=0)
		Q_cnt=0;
	  }	
		
		}
		else{
			
		if(Q_cnt==Enable_Count)
		  Q_mode=1;
		if(Q_cnt==0)
		  Q_mode=0;
		}
		
		if(IF_KEY_PRESSED_E){
			
			if(E_mode==0){
			if(E_cnt<Enable_Count)
		E_cnt++;
		 else if(E_cnt>=Enable_Count)
		E_cnt=Enable_Count;
	  }
			if(E_mode==1){
			if(E_cnt>0)
		E_cnt--;
		 else if(E_cnt<=0)
		E_cnt=0;
	  }	
		
	}
else {
if(E_cnt==Enable_Count)
		  E_mode=1;
		if(E_cnt==0)
		  E_mode=0;
}

	if(IF_KEY_PRESSED_SHIFT)
		{
		 if(Shift_mode==0){
			if(Shift_cnt<Enable_Count)
		Shift_cnt++;
		 else if(Shift_cnt>=Enable_Count)
		Shift_cnt=Enable_Count;
	  }
			if(Shift_mode==1){
			if(Shift_cnt>0)
		Shift_cnt--;
		 else if(Shift_cnt<=0)
		Shift_cnt=0; }	
		
		}	
		
	else {
		if(Shift_cnt==Enable_Count){
		 Shift_mode=1;
		shift_step=2;}
		if(Shift_cnt==0){
		 Shift_mode=0;
		shift_step=1;
		}
		
		}
		

	
		if(IF_KEY_PRESSED_G)
		{
		 if(G_mode==0){
			if(G_cnt<10)
	    G_cnt++;
		 else if(G_cnt>=Enable_Count)
		G_cnt=Enable_Count;
	 }
			if(G_mode==1){
			if(G_cnt<Enable_Count*2)
		G_cnt++;
		 else if(G_cnt>=Enable_Count*2)
		G_cnt=Enable_Count*2; }	
		if(G_mode==2){
			if(G_cnt>0)
	    G_cnt=G_cnt-2;
		 else if(G_cnt<=0)
		G_cnt=0;
	  }

		}
		
	else {
		if(G_cnt==Enable_Count*2){
		 G_mode=2;
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		if(G_cnt==Enable_Count){
		 G_mode=1;
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		if(G_cnt==0){
		 G_mode=0;
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		 }
	  }
	
		if(IF_KEY_PRESSED_F)
		{
		 if(F_mode==0){
			if(F_cnt<Enable_Count)
		F_cnt++;
		 else if(F_cnt>=Enable_Count)
		F_cnt=Enable_Count;
	  }
			if(F_mode==1){
			if(F_cnt>0)
		F_cnt--;
		 else if(F_cnt<=0)
		F_cnt=0; }	
		
		}	
		
	else {
		if(F_cnt==Enable_Count){
		 F_mode=1;
		}
		if(F_cnt==0){
		 F_mode=0;
		}
	}  
		
		if(IF_KEY_PRESSED_X)
		{
		 if(X_mode==0){
			if(X_cnt<Enable_Count)
		X_cnt++;
		 else if(X_cnt==Enable_Count)
		X_cnt=Enable_Count;
	  }
			if(X_mode==1){
			if(X_cnt>0)
		X_cnt--;
		 else if(X_cnt==0)
		X_cnt=0; }	
		
		}	
		
	else {
		if(X_cnt==Enable_Count){
		 X_mode=1;

		}
		if(X_cnt==0){
		 X_mode=0;
			shift_step=1;
		}
	}

 
	
	

	if(IF_KEY_PRESSED_V)
		{
		 if(V_mode==0){
			if(V_cnt<Enable_Count)
		V_cnt++;
		 else if(V_cnt>=Enable_Count)
		V_cnt=Enable_Count;
	  }
			if(V_mode==1){
			if(V_cnt>0)
		V_cnt--;
		 else if(V_cnt<=0)
		V_cnt=0; }	
		
		}	
		
	else {
		if(V_cnt==Enable_Count){
		 V_mode=1;
		}
		if(V_cnt==0){
		 V_mode=0;
		}
	}  	
	
	if(IF_KEY_PRESSED_R)
		{
		 if(R_mode==0){
			if(R_cnt<Enable_Count)
		R_cnt++;
		  if(R_cnt>=Enable_Count){
		  R_mode=1;
			R_cnt=Enable_Count;}}
	  }
		

		if(R_mode==1)
		{
			 Q_cnt=0;
	     E_cnt=0;
       Ctrl_cnt=0;
	     Shift_cnt=0;
	     R_cnt=0;
		   G_mode=0;
			 G_cnt=0;
		 	 Q_mode=0;
       E_mode=0;
       Ctrl_mode=0;
       Shift_mode=0;
			 shift_step=1;
			 X_mode=0;	
			 X_cnt=0;
       R_mode=0;	
			 R_cnt=0;
			 F_mode=0;
			 F_cnt=0;
			 V_mode =0;
			 V_cnt=0;
		 	WaitTimeForShoot = 0;
		}
	
	
      if(rc_ctrl.rc.sleft == RC_UP || rc_ctrl.mouse.press_l == 1)
      {

//              if(G_mode==0){
//                  if(WaitTimeForShoot < 70)
//                  {
//                      WaitTimeForShoot ++;
//                  }
//                  
//                  else if(WaitTimeForShoot >=70)
//                  {
//                      WaitTimeForShoot = 0;
//											
//									
//                  }
//                  shoot_control.fireStatus = CONTINUESHOOT;
//								}
//               else if(G_mode!=0){
                  if(WaitTimeForShoot <Shoot_Count)
                  {
                      WaitTimeForShoot ++;
                  }
                  
                  else if(WaitTimeForShoot >=Shoot_Count)
                  {
                      WaitTimeForShoot = Shoot_Count;
											
                  }
                  shoot_control.fireStatus = CONTINUESHOOT;
//								}
                 
			
			}
//          else
//          {
//              shoot_control.FireStatus = NOT_FIRE;
//          }
//      }
      else
      {
          shoot_control.fireStatus = NOT_FIRE;
          WaitTimeForShoot = 0;
				
      }
      
//      if(press_flag == 0)
//      {
//          if(IF_MOUSE_PRESSED_LEFT || rc_ctrl.rc.sleft == RC_UP)
//          {
//              press_flag = 1;
//              shoot_control.ButtonState = SHORT_PRESS;
//              shoot_control.button_press_time = 0;
//          }
//          else
//          {
//              press_flag = 0;
//              shoot_control.ButtonState = NOT_PRESS;
//              shoot_control.button_press_time = 0;              
//          }
//      }

//      if(press_flag == 1)
//      {     
//         if(!IF_MOUSE_PRESSED_LEFT || rc_ctrl.rc.sleft != RC_UP)
//         {
//             press_flag = 0;
//             shoot_control.ButtonState = NOT_PRESS;
//             shoot_control.button_press_time = 0;
//         }
//      }
      
	  osDelay(2);
  }
}
  /* USER CODE END PressTime */


