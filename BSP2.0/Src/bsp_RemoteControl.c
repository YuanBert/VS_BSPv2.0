#include "bsp_RemoteControl.h"
extern WIRLESS gWIRLESS;


BSP_StatusTypeDef BSP_RemoteControlInit(void)
{
	BSP_StatusTypeDef status = BSP_OK;
	gWIRLESS.OpenFlag  = 0;
	gWIRLESS.CloseFlag = 0;
	gWIRLESS.STopFlag  = 0;
	
	
	return BSP_OK;
}
/******************************** END OF FILE *******************************************/
