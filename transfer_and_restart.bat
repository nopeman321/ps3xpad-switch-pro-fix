@echo off
echo Transfiriendo binarios a la PS3...
REM Ajusta el usuario/pass si es necesario, o usa herramientas como WinSCP en script
curl -T "src\xpad.sprx" ftp://192.168.18.103/dev_hdd0/plugins/xpad.sprx
curl -T "game\xpad_game.sprx" ftp://192.168.18.103/dev_hdd0/plugins/xpad_game.sprx
echo Reiniciando PS3...
curl http://192.168.18.103/restart.ps3
echo Proceso completado.
pause
