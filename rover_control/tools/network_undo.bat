@echo off
rem Double-click to undo network_setup (Ethernet back to automatic, firewall rules removed).
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process powershell -Verb RunAs -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','\"%~dp0network_setup.ps1\"','-Undo'"
