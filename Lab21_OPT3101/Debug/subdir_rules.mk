################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/ti/tirslk_max_1_00_02/Lab21_OPT3102_3" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

SSD1306.obj: C:/ti/tirslk_max_1_00_02/inc/SSD1306.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/ti/tirslk_max_1_00_02/Lab21_OPT3102_3" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

blinker.obj: C:/ti/tirslk_max_1_00_02/inc/blinker.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/ti/tirslk_max_1_00_02/Lab21_OPT3102_3" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

fixed.obj: C:/ti/tirslk_max_1_00_02/inc/fixed.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/ti/tirslk_max_1_00_02/Lab21_OPT3102_3" --include_path="C:/Users/hokke/Documents/_cyber/ccs/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


