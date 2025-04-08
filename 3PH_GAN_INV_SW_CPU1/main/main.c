#include "driverlib.h"
#include "device.h"
#include <math.h>
#include "IQmathLib.h"
#include <stdio.h>
#include <string.h>


int main(void)
{
    Device_init();


    Device_initGPIO();

    DINT;

    Interrupt_initModule();

    IER = 0x0000;
    IFR = 0x0000;

    Interrupt_initVectorTable();

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

}
