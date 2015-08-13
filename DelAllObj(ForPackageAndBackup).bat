rd /Q /S project\MDK-ARM\Flash
rd /Q /S project\MDK-ARM\CpuRAM
rd /Q /S project\MDK-ARM\ExtSRAM
del /Q project\MDK-ARM\*.bak
del /Q project\MDK-ARM\*.dep
del /Q project\MDK-ARM\JLink*
del /Q project\MDK-ARM\project.uvgui.*

del /Q project\EWARM\Project.dep
del /Q project\EWARM\Flash
del /Q project\EWARM\CpuRAM
del /Q project\EWARM\settings
rd  /Q /S project\EWARM\Flash
rd /Q /S project\EWARM\CpuRAM
rd /Q /S project\EWARM\settings

