################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"/opt/ccstudio/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/bin/cl430" -vmsp -O2 --opt_for_speed=0 --use_hw_mpy=none --include_path="/opt/ccstudio/ccs/ccs_base/msp430/include" --include_path="/run/media/HDD/Documents/Superprops_Electronics/Gemini_Stuff/Gemini_Control_Board/SuperProps_GeminiControlBoard_Windows/SevenSeg_Module" --include_path="/run/media/HDD/Documents/Superprops_Electronics/Gemini_Stuff/Gemini_Control_Board/SuperProps_GeminiControlBoard_Windows/MatrixKeypad_Module" --include_path="/run/media/HDD/Documents/Superprops_Electronics/Gemini_Stuff/Gemini_Control_Board/SuperProps_GeminiControlBoard_Windows/SPI_Module" --include_path="/run/media/HDD/Documents/Superprops_Electronics/Gemini_Stuff/Gemini_Control_Board/SuperProps_GeminiControlBoard_Windows" --include_path="/opt/ccstudio/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/include" --advice:power="all" --define=__MSP430G2353__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --enum_type=packed --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


