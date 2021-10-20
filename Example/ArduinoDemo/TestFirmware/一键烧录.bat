:start
dfu-util.exe -a 0  -s 0x08000000 -R -D .\firmware.bin

@echo Press any key,keey find device praogram
pause
goto start

