# Sets up the environment for autobuild on Windows

# install Qt
powershell pip install aqtinstall
aqt install --outputdir C:\Qt 5.15.2 windows desktop win64_msvc2019_64
aqt install --outputdir C:\Qt 5.15.2 windows desktop win32_msvc2019

# build the installer
powershell %1\windows\deploy_windows.ps1 C:\Qt\5.15.2

# rename the file
cp %1\deploy\Jamulus*installer-win.exe %1\deploy\Jamulus-installer-win.exe
