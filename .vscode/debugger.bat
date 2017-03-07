set PATH=C:\Users\Hubert\.vscode\HusarionTools\bin\;%PATH%
cd c:\Users\Hubert\Documents\GitHub\hRobot || exit 1
start /wait st-flash write myproject.bin 0x08010000 || exit 1
start st-util
arm-none-eabi-gdb %*