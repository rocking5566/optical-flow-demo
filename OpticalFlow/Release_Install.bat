echo off

echo copy DLL
if not exist bin\Release md bin\Release

xcopy /q/y ..\packages\OpenCV-3.0.0\bin\opencv_ffmpeg300.dll bin\Release
xcopy /q/y ..\packages\OpenCV-3.0.0\bin\opencv_world300.dll bin\Release


