@echo off
setlocal
set "CONFIG=%~dp0sdk_config.ini"
set "PROJ=%~dp0DC-Motor-Car.uvprojx"

echo [1/3] Reading sdk_config.ini...

powershell -ExecutionPolicy Bypass -Command "$ini=@{}; Get-Content $env:CONFIG | ForEach-Object { $t=$_.Trim(); if ($t -match '^(\w+)\s*=\s*(.+)$') { $ini[$matches[1]]=$matches[2].Trim() } }; $keys=@('source','cmsis_include','driverlib_lib'); $keys | ForEach-Object { if (-not $ini.ContainsKey($_)) { Write-Host ('ERROR: '+$_+' not found in sdk_config.ini'); exit 1 } }; Write-Host ('  source       = '+$ini.source); Write-Host ('  cmsis_include= '+$ini.cmsis_include); Write-Host ('  driverlib_lib= '+$ini.driverlib_lib); Write-Host '[2/3] Reading .uvprojx paths...'; $c=Get-Content $env:PROJ -Raw; $i=[regex]::Match($c,'<IncludePath>(\.\\app;.*?)</IncludePath>'); $o=[regex]::Match($c,'<Misc>([^<]*)</Misc>'); if (-not $i.Success) { Write-Host 'ERROR: IncludePath not found'; exit 1 }; if (-not $o.Success) { Write-Host 'ERROR: Misc not found'; exit 1 }; Write-Host ('  Current IncludePath: '+$i.Groups[1].Value); Write-Host ('  Current Misc       : '+$o.Groups[1].Value); Write-Host '[3/3] Updating .uvprojx...'; $c=$c.Replace($i.Groups[1].Value,'.\app;.\bsp;.\middleware;.\core;'+$ini.source+';'+$ini.cmsis_include); $c=$c.Replace($o.Groups[1].Value,$ini.driverlib_lib); Set-Content $env:PROJ -Value $c; Write-Host 'Done! .uvprojx paths updated.'"

if %errorlevel% equ 0 (
    echo.
    echo .uvprojx paths updated successfully.
    echo Open project in Keil and rebuild.
) else (
    echo.
    echo Error occurred. Check paths in sdk_config.ini.
)
pause
