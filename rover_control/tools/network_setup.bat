@echo off
rem Double-click to set up the wired Ethernet port for the two-PC test.
rem Windows will ask for administrator permission.
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process powershell -Verb RunAs -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','\"%~dp0network_setup.ps1\"'"
