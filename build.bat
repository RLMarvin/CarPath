SET pypath="C:\Users\Admin\AppData\Local\Programs\Python"

mkdir build_py36
cd build_py36
cmake .. -DPYTHON_EXECUTABLE="%pypath%\Python36\python.exe" .. -G"Visual Studio 15 2017 Win64"
cmake --build . --config Release
for /R . %%f in (*.pyd) do copy "%%f" ..
cd ..


mkdir build_py37
cd build_py37
cmake .. -DPYTHON_EXECUTABLE="%pypath%\Python37\python.exe" .. -G"Visual Studio 15 2017 Win64"
cmake --build . --config Release
for /R . %%f in (*.pyd) do copy "%%f" ..
cd ..

pause