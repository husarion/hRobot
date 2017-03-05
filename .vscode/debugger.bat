set PATH=C:\Users\asta1_000\.vscode\HusarionTools\bin\;%PATH%
cd c:\Users\asta1_000\Documents\GitHub\hRobot || exit 1
start /wait st-flash write myproject.bin 0x08010000 || exit 1
start st-util
arm-none-eabi-gdb %*