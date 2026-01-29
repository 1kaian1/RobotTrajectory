@ECHO OFF
REM Command file for Sphinx documentation (Windows)

pushd %~dp0

REM Paths
set SPHINXBUILD=sphinx-build
set SOURCEDIR=docs/source
set BUILDDIR=docs/_build

REM Check if sphinx-build is installed
%SPHINXBUILD% >NUL 2>NUL
if errorlevel 9009 (
    echo.
    echo The 'sphinx-build' command was not found. Make sure you have Sphinx installed.
    echo https://www.sphinx-doc.org/
    exit /b 1
)

REM Check arguments
if "%1"=="" goto help

REM Run Sphinx with the given target
%SPHINXBUILD% -M %1 %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
goto end

:help
%SPHINXBUILD% -M help %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%

:end
popd
