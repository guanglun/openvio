set STM32_INTERFACE_CFG="interface/stlink.cfg"
set STM32_TARGET_CFG="target/stm32h7x.cfg"
set STM32_BIN_FILE_PATH="build/"
set STM32_BIN_FILE=%STM32_BIN_FILE_PATH%"STM32H7VIO.elf"

echo "start flash STM32 ...";
openocd -f %STM32_INTERFACE_CFG% -f %STM32_TARGET_CFG% -c init -c "reset halt" -c "flash write_image erase %STM32_BIN_FILE%" -c "reset run" -c exit 
@REM echo "set stm32 BOR level ..."
@REM openocd -f %STM32_INTERFACE_CFG% -f %STM32_TARGET_CFG% -c init -c "reset halt" -c "stm32h7x options_write 0 0xE8" -c "reset run" -c exit

