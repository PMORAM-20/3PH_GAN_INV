################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2010/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./Control Strategies/Model Predictive Control/main.obj" \
"./device/F2837xD_CodeStartBranch.obj" \
"./device/device.obj" \
"./src/common/data_types.obj" \
"./src/common/math_utils.obj" \
"./src/control/foc_control.obj" \
"./src/control/mpc_control.obj" \
"./src/peripherals/adc_driver.obj" \
"./src/peripherals/cla_driver.obj" \
"./src/peripherals/cmpss_driver.obj" \
"./src/peripherals/peripheral_setup.obj" \
"./src/peripherals/pwm_driver.obj" \
"C:/ti/c2000/C2000Ware_5_04_00_00/driverlib/f2837xd/driverlib/ccs/Release/driverlib.lib" \
"../map/2837xD_RAM_CLA_lnk_cpu1.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include Control\ Strategies/Model\ Predictive\ Control/subdir_vars.mk
-include device/subdir_vars.mk
-include drivers/subdir_vars.mk
-include map/subdir_vars.mk
-include src/common/subdir_vars.mk
-include src/control/subdir_vars.mk
-include src/peripherals/subdir_vars.mk
-include Control\ Strategies/Model\ Predictive\ Control/subdir_rules.mk
-include device/subdir_rules.mk
-include drivers/subdir_rules.mk
-include map/subdir_rules.mk
-include src/common/subdir_rules.mk
-include src/control/subdir_rules.mk
-include src/peripherals/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
3PH_GAN_INV_SW_CPU1.out 

EXE_OUTPUTS__QUOTED += \
"3PH_GAN_INV_SW_CPU1.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "3PH_GAN_INV_SW_CPU1.out"

# Tool invocations
3PH_GAN_INV_SW_CPU1.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs2010/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -O4 --opt_for_speed=5 --fp_mode=relaxed --define=CPU1 --define=CLA1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"3PH_GAN_INV_SW_CPU1.map" --stack_size=0x100 --warn_sections -i"C:/ti/ccs2010/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/lib" -i"C:/ti/ccs2010/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --reread_libs --define=CLA_C --diag_wrap=off --display_error_number --xml_link_info="3PH_GAN_INV_SW_CPU1_linkInfo.xml" --entry_point=code_start --rom_model -o "3PH_GAN_INV_SW_CPU1.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "Control Strategies\Model Predictive Control\main.obj" "device\F2837xD_CodeStartBranch.obj" "device\device.obj" "src\common\data_types.obj" "src\common\math_utils.obj" "src\control\foc_control.obj" "src\control\mpc_control.obj" "src\peripherals\adc_driver.obj" "src\peripherals\cla_driver.obj" "src\peripherals\cmpss_driver.obj" "src\peripherals\peripheral_setup.obj" "src\peripherals\pwm_driver.obj" 
	-$(RM) "Control Strategies\Model Predictive Control\main.d" "device\device.d" "src\common\data_types.d" "src\common\math_utils.d" "src\control\foc_control.d" "src\control\mpc_control.d" "src\peripherals\adc_driver.d" "src\peripherals\cla_driver.d" "src\peripherals\cmpss_driver.d" "src\peripherals\peripheral_setup.d" "src\peripherals\pwm_driver.d" 
	-$(RM) "device\F2837xD_CodeStartBranch.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

