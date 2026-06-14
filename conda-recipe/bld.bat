@echo on

if "%PYPGO_CMAKE_PRESET%"=="" set "PYPGO_CMAKE_PRESET=pypgo-conda"
if "%CMAKE_BUILD_PARALLEL_LEVEL%"=="" set "CMAKE_BUILD_PARALLEL_LEVEL=%CPU_COUNT%"

if exist build\pypgo-conda rmdir /s /q build\pypgo-conda
if exist build\pypgo-conda-mkl rmdir /s /q build\pypgo-conda-mkl

"%PYTHON%" -m pip install . --no-build-isolation --no-deps -v
if errorlevel 1 exit /b 1
