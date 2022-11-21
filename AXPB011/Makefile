ARM_TOOLCAHIN	:=	${ARM}/bin/arm-none-eabi-
CC  			:=	$(ARM_TOOLCAHIN)gcc
OBJCOPY			:=	$(ARM_TOOLCAHIN)objcopy

############################################################

#-------------Folder paths-------------
APP_FOLDER			:=	Application/
CMSIS_FOLDER		:=	CMSIS/
GD32F350_FOLDER		:=	$(CMSIS_FOLDER)GD/GD32F3x0/
STDPERIPHS_FOLDER	:=	Peripherals/GD32F3x0_standard_peripheral/
USBPERIPHS_FOLDER	:=	Peripherals/GD32F3x0_usbfs_library/
PLAT_FOLDER			:=	Platform/

#-------------Source locations-------------
APP_PATH				:=	$(APP_FOLDER)Source/
GD32F350_PATH			:=	$(GD32F350_FOLDER)Source/
STDPERIPHS_PATH			:=	$(STDPERIPHS_FOLDER)Source/
USB_HID_GENERIC_PATH	:=	$(USBPERIPHS_FOLDER)device/class/hid/Generic/Source/
USB_HID_PRESS_PATH		:=	$(USBPERIPHS_FOLDER)device/class/hid/Press/Source/
USB_HID_DIGITIZER_PATH	:=	$(USBPERIPHS_FOLDER)device/class/hid/Digitizer/Source/
USB_DEV_CORE_PATH		:=	$(USBPERIPHS_FOLDER)device/core/Source/
USB_DRIVER_PATH			:=	$(USBPERIPHS_FOLDER)driver/Source/
PLAT_PATH				:=	$(PLAT_FOLDER)

#-------------Include locations-------------
APP_INC					:= 	$(APP_FOLDER)Include/
CMSIS_INC				:=	$(CMSIS_FOLDER)
GD32F350_INC			:=	$(GD32F350_FOLDER)Include/
STDPERIPHS_INC			:=	$(STDPERIPHS_FOLDER)Include/
USB_HID_GENERIC_INC		:=	$(USBPERIPHS_FOLDER)device/class/hid/Generic/Include/
USB_HID_PRESS_INC		:=	$(USBPERIPHS_FOLDER)device/class/hid/Press/Include/
USB_HID_DIGITIZER_INC	:=	$(USBPERIPHS_FOLDER)device/class/hid/Digitizer/Include/
USB_DEV_CORE_INC		:=	$(USBPERIPHS_FOLDER)device/core/Include/
USB_DRIVER_INC			:=	$(USBPERIPHS_FOLDER)driver/Include/
USB_USTD_HID			:=	$(USBPERIPHS_FOLDER)ustd/class/hid/
USB_USTD_COMMON			:=	$(USBPERIPHS_FOLDER)ustd/common/

INC_PATH	:=	-I$(APP_INC)				\
				-I$(CMSIS_INC)				\
				-I$(GD32F350_INC)			\
				-I$(STDPERIPHS_INC)			\
				-I$(USB_HID_GENERIC_INC)	\
				-I$(USB_HID_PRESS_INC)		\
				-I$(USB_HID_DIGITIZER_INC)	\
				-I$(USB_DEV_CORE_INC)		\
				-I$(USB_DRIVER_INC)			\
				-I$(USB_USTD_HID)			\
				-I$(USB_USTD_COMMON)

#-------------Define build directory-------------
OBJDIR					:=	bins/objects/
APP_OBJS				:=	$(OBJDIR)$(APP_PATH)
GD32F350_OBJS			:=	$(OBJDIR)$(GD32F350_PATH)
STDPERIPHS_OBJ			:=	$(OBJDIR)$(STDPERIPHS_PATH)
USB_HID_GENERIC_OBJ		:=	$(OBJDIR)$(USB_HID_GENERIC_PATH)
USB_HID_PRESS_OBJ		:=	$(OBJDIR)$(USB_HID_PRESS_PATH)
USB_HID_DIGITIZER_OBJ	:=	$(OBJDIR)$(USB_HID_DIGITIZER_PATH)
USB_DEV_CORE_OBJ		:=	$(OBJDIR)$(USB_DEV_CORE_PATH)
USB_DRIVER_OBJ			:=	$(OBJDIR)$(USB_DRIVER_PATH)
PLAT_OBJ				:=	$(OBJDIR)$(PLAT_PATH)

#-------------Locate all source files-------------
APP_SRC					:=	$(wildcard $(APP_PATH)*.c)
GD32F350_SRC			:=	$(wildcard $(GD32F350_PATH)*.c)
STDPERIPHS_SRC			:=	$(wildcard $(STDPERIPHS_PATH)*.c)
USB_GENERIC_HID_SRC		:=	$(wildcard $(USB_HID_GENERIC_PATH)*.c)
USB_PRESS_HID_SRC		:=	$(wildcard $(USB_HID_PRESS_PATH)*.c)
USB_DIGITIZER_HID_SRC	:=	$(wildcard $(USB_HID_DIGITIZER_PATH)*.c)
USB_DEV_CORE_SRC		:=	$(wildcard $(USB_DEV_CORE_PATH)*.c)
USB_DRIVER_SRC			:=	$(wildcard $(USB_DRIVER_PATH)*.c)
ASM_SRC					:=	$(PLAT_FOLDER)startup_gd32f3x0_app.S

# Create a list of object files we want to compile
OBJS	:=	$(addprefix	$(OBJDIR),	$(patsubst $(APP_PATH)%.c,					$(APP_PATH)%.o,					$(APP_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(GD32F350_PATH)%.c,				$(GD32F350_PATH)%.o,			$(GD32F350_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(STDPERIPHS_PATH)%.c,			$(STDPERIPHS_PATH)%.o,			$(STDPERIPHS_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(USB_HID_GENERIC_PATH)%.c,		$(USB_HID_GENERIC_PATH)%.o,		$(USB_GENERIC_HID_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(USB_HID_PRESS_PATH)%.c,		$(USB_HID_PRESS_PATH)%.o,		$(USB_PRESS_HID_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(USB_HID_DIGITIZER_PATH)%.c,	$(USB_HID_DIGITIZER_PATH)%.o,	$(USB_DIGITIZER_HID_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(USB_DEV_CORE_PATH)%.c,			$(USB_DEV_CORE_PATH)%.o,		$(USB_DEV_CORE_SRC)))\
			$(addprefix	$(OBJDIR),	$(patsubst $(USB_DRIVER_PATH)%.c,			$(USB_DRIVER_PATH)%.o,			$(USB_DRIVER_SRC)))	\
			$(addprefix	$(OBJDIR),	$(patsubst $(PLAT_FOLDER)%.S,				$(PLAT_FOLDER)%.o,				$(ASM_SRC)))

TARGET	:=	bins/axpb011_runtime
############################################################

# Compilation and linking flags

OPTIMISTATION	:= -Og
# EDIT THE LINE ABOVE TO USE ONE OF THESE
#    -O0      #default (no optimisation at all)
#    -O1      #optimize
#    -O2      #optimize even more
#    -O3      #optimize yet more
#    -Os      #optimize for size using O2
#    -Ofast   #optimize O3 with cheats
#    -Og      #optimize debugging experience (some optimisation)

PREPROCESSORS_DEFINES	:=	-DGD32F350				\
							-DGD32F3X0				\
							-DUSE_STDPERIPH_DRIVER	\
							-DARM_MATH_CM4			\
							-DUSB_FS_CORE			\
							-DUSE_EMBEDDED_PHY


MCU	:=	-mcpu=cortex-m4		\
		-mthumb				\
		-mfloat-abi=soft

CFLAGS		:=	$(MCU)						\
				$(OPTIMISTATION)			\
				$(PREPROCESSORS_DEFINES)	\
				-std=c99					\
				-g3							\
				-gdwarf-2					\
				-ffunction-sections 		\
				-fdata-sections 			\
				-Wall

AFLAGS		:= 	$(MCU)				\
				-x 					\
				assembler-with-cpp	\
				-Wall				\
				-fdata-sections		\
				-ffunction-sections

LIBS		:=	-lc					\
				-lm					\
				-lnosys

LIBDIR		:=	-LLib/GCC

LDSCRIPT	:=	$(PLAT_FOLDER)GD32F350K8_FLASH_app.ld

LDFLAGS		:=	$(MCU)	\
				-specs=nano.specs	\
				-T$(LDSCRIPT)	\
				$(LIBDIR)	\
				$(LIBS)	\
				-Wl,-Map=$(TARGET).map	\
				-ffunction-sections	\
				-fdata-sections	\
				-Wl,--gc-sections,--print-memory-usage
				
############################################################

# PHONY means they will be made even if directories/files with the same name already exist
.PHONY: all clean

help:	## Show this help.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

all: compile link

compile: $(APP_OBJS) $(GD32F350_OBJS) $(STDPERIPHS_OBJ) $(USB_HID_GENERIC_OBJ) $(USB_HID_PRESS_OBJ) $(USB_HID_DIGITIZER_OBJ) $(USB_DEV_CORE_OBJ) $(USB_DRIVER_OBJ) $(PLAT_OBJ) $(OBJS)  ## Creates folders and compiles source.

link:	## Links all source files and outputs the target in the bins folder.
	@echo ""
	@echo "Linking..."
	@$(CC) -o $(TARGET).elf $(LDFLAGS) $(OBJS)
	@$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex
	@$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	@echo "Created $(TARGET).elf"
	@echo "Created $(TARGET).hex"
	@echo "Created $(TARGET).bin"
	@echo "Created $(TARGET).map"

clean:	## Cleans the entire bins folder of object files and executable.
	@echo ""
	@echo "Cleaning up objects and executable."
	@rm -f $(TARGET).elf
	@rm -f $(TARGET).hex
	@rm -f $(TARGET).bin
	@rm -f $(TARGET).map
	@rm -f $(APP_OBJS)*.o
	@rm -f $(GD32F350_OBJS)*.o
	@rm -f $(STDPERIPHS_OBJ)*.o
	@rm -f $(USB_HID_GENERIC_OBJ)*.o
	@rm -f $(USB_HID_PRESS_OBJ)*.o
	@rm -f $(USB_HID_DIGITIZER_OBJ)*.o
	@rm -f $(USB_DEV_CORE_OBJ)*.o
	@rm -f $(USB_DRIVER_OBJ)*.o
	@rm -f $(PLAT_OBJ)*.o

############################################################

# Rules for source files
# $< is the source file (the prerequisite) and %@ is the object file being created (the target of the rule)
# The -o option lets us define where the object is created, if not used they will be created at the Makefile level

$(OBJDIR)$(APP_PATH)%.o: $(APP_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@

$(OBJDIR)$(GD32F350_PATH)%.o: $(GD32F350_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@
	
$(OBJDIR)$(STDPERIPHS_PATH)%.o: $(STDPERIPHS_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@

$(OBJDIR)$(USB_HID_GENERIC_PATH)%.o: $(USB_HID_GENERIC_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@

$(OBJDIR)$(USB_HID_PRESS_PATH)%.o: $(USB_HID_PRESS_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@

$(OBJDIR)$(USB_HID_DIGITIZER_PATH)%.o: $(USB_HID_DIGITIZER_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@

$(OBJDIR)$(USB_DEV_CORE_PATH)%.o: $(USB_DEV_CORE_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@
	
$(OBJDIR)$(USB_DRIVER_PATH)%.o: $(USB_DRIVER_PATH)%.c
	@echo "    [CC] $<"
	@$(CC) $(CFLAGS) $(INC_PATH) -c $< -o $@
	
# Rule for startup files
$(OBJDIR)$(PLAT_PATH)%.o: $(PLAT_PATH)%.S
	@echo "    [CC] $<"
	@$(CC) $(AFLAGS) $(INC_PATH) -c -o $@ $<

############################################################

# Create the folders to store the outputs
# -p prevents make from complaining if the folder already exists

$(APP_OBJS):
	mkdir -p $(APP_OBJS)
	
$(GD32F350_OBJS):
	mkdir -p $(GD32F350_OBJS)
	
$(STDPERIPHS_OBJ):
	mkdir -p $(STDPERIPHS_OBJ)
	
$(USB_HID_GENERIC_OBJ):
	mkdir -p $(USB_HID_GENERIC_OBJ)
	
$(USB_HID_PRESS_OBJ):
	mkdir -p $(USB_HID_PRESS_OBJ)
	
$(USB_HID_DIGITIZER_OBJ):
	mkdir -p $(USB_HID_DIGITIZER_OBJ)
	
$(USB_DEV_CORE_OBJ):
	mkdir -p $(USB_DEV_CORE_OBJ)
	
$(USB_DRIVER_OBJ):
	mkdir -p $(USB_DRIVER_OBJ)
	
$(PLAT_OBJ):
	mkdir -p $(PLAT_OBJ)