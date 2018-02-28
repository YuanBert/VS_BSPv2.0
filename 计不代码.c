		/* 
//		if (1 == gMotorMachine.RunningState)
//		{
//			gStepCnterCurrentRedVal = HAL_GPIO_ReadPin(VerRasterInput_GPIO_Port, VerRasterInput_Pin);
//			if (0 == gStepCnterCurrentRedVal && 0 == gStepCnterLastRedVal)
//			{
//				gMotorMachine.VerFilterCnt++;
//				if (gMotorMachine.VerFilterCnt > 5)
//				{
//				
//					if (0 == gLastStepValue)
//					{
//						gMotorMachine.StepCnt++;
//					}
//					gLastStepValue = 1;
//					gMotorMachine.VerFilterCnt = 0;
//				}
//			}
//			else
//			{
//				gLastStepValue = 0;
//				gMotorMachine.VerFilterCnt = 0;
//			}
//			gStepCnterLastRedVal = gStepCnterCurrentRedVal;
//
//			if (gMotorMachine.StepCnt > 300 && UPDIR == gMotorMachine.RunDir)
//			{
//				gMotorMachine.VerticalRasterState = 1;
//				gMotorMachine.StepCnt = 0;
//				// 此处可以添加对计数的日志信息的处理，此处写标记位
//				gMotorMachine.RunDir = DOWNDIR;
//				gMotorMachine.RunningState = 0;
//				BSP_MotorStop();
//			}
//			
//			if (gMotorMachine.StepCnt < 290 && UPDIR == gMotorMachine.RunDir)
//			{
//				gMotorMachine.VerticalRasterState = 0;
//			}
//		
//			//判断是否离开了垂直位置，如果方向是关闸方向，且超过10步则认为是离开了设计的垂直位置
//			if (gMotorMachine.StepCnt > 10 && DOWNDIR == gMotorMachine.RunDir)
//			{
//				gMotorMachine.VerticalRasterState = 0;
//			}
//		}	
		*/
BSP_StatusTypeDef BSP_MotorCheck(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  
  //使用遥控时，退出
  if(gMotorMachine.RemoteControFlag)
  {
    return state;
  }
  //车未退出时直接，返回
  if(gComingCarFlag)
  {
    return state;
  }
  if(gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
  {
    BSP_MotorStop();
    state = BSP_ERROR;
    return state;
  }
  //垂直位置
  if(gMotorMachine.VerticalRasterState && 0 == gMotorMachine.HorizontalRasterState)
  {
    if(gMotorMachine.GentleSensorFlag || gMotorMachine.RadarSensorFlag) //车辆在地感和雷达的时候，不关闸
    {
      return state;
    }
    
    if(gMotorMachine.OpenFlag)
    {
      gMotorMachine.OpenFlag = 0;
    }
    
    if(1 == gMotorMachine.CloseFlag && 0 == gMotorMachine.RunningState)
    {
      if(DOWNDIR == gMotorMachine.RunDir)
      {
        gMotorMachine.StartFlag = 1;
        return state;
      }
      return state;
    }
    
    gMotorMachine.OpenFlag  = 0;
    gMotorMachine.CloseFlag = 1;
    return state;
  }
  /* 水平位置 */
  if(0 == gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
  {
    if(gMotorMachine.OpenFlag)
    {
      if(UPDIR == gMotorMachine.RunDir)
      {
        gMotorMachine.StartFlag = 1;
        gMotorMachine.CloseFlag = 0;
        return state;
      }
    }
    if(gMotorMachine.CloseFlag)
    {
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.OpenFlag  = 0;
      return state;
    }
  }
  /* 运行位置 */
  if(0 == gMotorMachine.VerticalRasterState && 0 == gMotorMachine.HorizontalRasterState)
  {
    
	  //向关闸方向运行
	  if (gMotorMachine.RunningState && DOWNDIR == gMotorMachine.RunDir)
	  {
		  if (gMotorMachine.GentleSensorFlag || gMotorMachine.RadarSensorFlag || gMotorMachine.AirSensorFlag) //检测到地感和雷达后不关闸机
			  {
				  BSP_MotorStop();
				  gMotorMachine.RunningState = 0;
				  gMotorMachine.OpenFlag = 1;
				  gGentleSensorStatusDetection.GpioCheckedFlag = 0;
				  return state;
			  }
		  return state;
	  }
	  //向开闸的方向运行
	  if(gMotorMachine.RunningState && UPDIR == gMotorMachine.RunDir)
	  {
		  //不作处理
		  return state;
	  }
	  
	  //如果处在非运行状态，且处于需要开启状态
	  if(0 == gMotorMachine.RunningState && gMotorMachine.OpenFlag)
	  {
		  gMotorMachine.RunDir = UPDIR;
		  gMotorMachine.OpenFlag = 0;
		  gMotorMachine.CloseFlag = 1;
		  gMotorMachine.StartFlag = 1; 
		  
	  }
	  
	//////////////////////////////////////////////////////////////////////////////  
	  if(gMotorMachine.OpenFlag)
    {
      if(gMotorMachine.RunningState)
      {
        gMotorMachine.CloseFlag = 0;
        return state;
      }
      
      gMotorMachine.RunDir = UPDIR;
      gMotorMachine.OpenFlag = 1;
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.StartFlag = 1;
      return state;
    }
    
    if(gMotorMachine.CloseFlag)
    {
      if(gMotorMachine.RunningState)
      {
        if(gMotorMachine.GentleSensorFlag ||gMotorMachine.RadarSensorFlag || gMotorMachine.AirSensorFlag) //检测到地感和雷达后不关闸机
        {
          BSP_MotorStop();
          gMotorMachine.RunningState = 0;
          gMotorMachine.OpenFlag = 1;
          gGentleSensorStatusDetection.GpioCheckedFlag = 0;
          return state;
        }
        return state;
      }
      else
      {
        gMotorMachine.RunDir = DOWNDIR;
        gMotorMachine.StartFlag = 1;
        return state;
      }
    }
	  
    if(0 == gMotorMachine.RunningState)
    {
      gMotorMachine.RunDir = DOWNDIR;
      //gMotorMachine.OpenFlag = 1;
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.StartFlag = 1;    
      return state;
    }
  }
  return state;
}

BSP_StatusTypeDef BSP_MotorAction(void)
{
  BSP_StatusTypeDef state = BSP_OK;
  if(gMotorMachine.StartFlag)
  {
    if(gMotorMachine.RunningState)
    {
      gMotorMachine.StartFlag = 0;
      return state;
    }
    gMotorMachine.RunningState = 1;
    BSP_MotorRun(gMotorMachine.RunDir);
    gMotorMachine.StartFlag = 0;
  }
  return state;
}