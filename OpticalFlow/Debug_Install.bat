echo off

echo copy DLL
if not exist bin\Debug md bin\Debug

xcopy /q/y ..\packages\OpenCV-3.0.0\bin\opencv_ffmpeg300.dll bin\Debug
xcopy /q/y ..\packages\OpenCV-3.0.0\bin\opencv_world300d.dll bin\Debug


