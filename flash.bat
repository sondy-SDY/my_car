@echo off

rem 检查OpenOCD是否可用
where openocd > nul 2> nul
if %errorlevel% neq 0 (
    echo OpenOCD 未找到，请确保已安装并添加到系统路径
    pause
    exit /b 1
)

echo 正在使用OpenOCD烧录固件...

rem 执行烧录命令
openocd -f openocd.cfg -c "program build/Debug/STM32_LineFollower_Car.elf verify reset exit"

if %errorlevel% neq 0 (
    echo 烧录失败！
    pause
    exit /b 1
)

echo 烧录成功！
pause
