@echo off
echo ========================================
echo ControlWorkbench Build and Obfuscate
echo ========================================
echo.

cd /d "%~dp0"

echo [1/4] Building Release...
dotnet build -c Release
if errorlevel 1 (
    echo BUILD FAILED!
    pause
    exit /b 1
)

echo.
echo [2/4] Checking for ConfuserEx...
if not exist "%TEMP%\ConfuserEx\Confuser.CLI.exe" (
    echo Downloading ConfuserEx...
    powershell -Command "Invoke-WebRequest -Uri 'https://github.com/mkaring/ConfuserEx/releases/download/v1.6.0/ConfuserEx-CLI.zip' -OutFile '%TEMP%\ConfuserEx.zip'"
    powershell -Command "Expand-Archive -Path '%TEMP%\ConfuserEx.zip' -DestinationPath '%TEMP%\ConfuserEx' -Force"
)

echo.
echo [3/4] Obfuscating...
"%TEMP%\ConfuserEx\Confuser.CLI.exe" "ControlWorkbench.App\ConfuserEx.crproj"
if errorlevel 1 (
    echo OBFUSCATION FAILED!
    echo.
    echo Note: ConfuserEx may not fully support .NET 8.
    echo Your Release build is still available at:
    echo ControlWorkbench.App\bin\Release\net8.0-windows\
    pause
    exit /b 1
)

echo.
echo [4/4] Done!
echo.
echo Obfuscated files are in: ControlWorkbench.App\Confused\
echo.
pause
