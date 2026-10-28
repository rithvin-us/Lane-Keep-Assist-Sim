@echo off
REM Runs the LKA without opening the MATLAB window (double-click, a terminal, or Windows Task Scheduler).
REM   run_lka.bat            every video in vids\  -> test_res\<video>\ and test_res\summary.csv
REM   run_lka.bat watch      process new videos in vids\ once (schedule this every few minutes)
REM   run_lka.bat checks     quick checks: all unit tests  -> results\checks\lka_checks_<time>.md
REM   run_lka.bat full       full checks: + closed-loop MIL tests + every video (straight-lane alarm)
REM MATLAB must be on the PATH (test: open a terminal and type  matlab -help ).
cd /d "%~dp0\.."
set CMD=addpath('scripts'); run_lka_on_test_videos
if /I "%~1"=="watch"  set CMD=addpath('scripts'); watch_lka_videos(Once=true)
if /I "%~1"=="checks" set CMD=addpath('scripts'); run_lka_checks
if /I "%~1"=="full"   set CMD=addpath('scripts'); run_lka_checks(Level='full')
echo Running: matlab -batch "%CMD%"
matlab -batch "%CMD%"
set RESULT=%ERRORLEVEL%
if not "%RESULT%"=="0" echo FAILED with exit code %RESULT%
if "%~1"=="" pause
exit /b %RESULT%
