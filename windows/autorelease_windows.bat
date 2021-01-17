:: Sets up the environment for autobuild on Windows

echo "Install Qt..."
:: Install Qt
pip install aqtinstall
echo "Get Qt 64 bit..."
aqt install --outputdir C:\Qt 5.15.2 windows desktop win64_msvc2019_64
echo "Get Qt 32 bit..."
aqt install --outputdir C:\Qt 5.15.2 windows desktop win32_msvc2019

echo "Build installer..."
:: Build the installer
powershell %1\windows\deploy_windows.ps1 C:\Qt\5.15.2

:: Rename the installer
cp %1\deploy\Jamulus*installer-win.exe %1\deploy\Jamulus-installer-win.exe
