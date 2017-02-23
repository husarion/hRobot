set PATH=C:\Users\gomiksia\.vscode\HusarionTools\bin\;%PATH%
cd c:\Users\gomiksia\Desktop\hRobot\hRobot || exit 1
start /wait st-flash write myproject.bin 0x08010000 || exit 1
start st-util
arm-none-eabi-gdb %*