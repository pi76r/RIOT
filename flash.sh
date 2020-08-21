project=driver_stts751
directory=tests
board=nucleo-f446ze
if [ $# -eq 1 ] && [ $1 = "r" ]; then
    board=nucleo-f446re
fi
bin=tests_$project

mv $directory/$project/bin/$board/$bin.bin /mnt/c/Users/Pierre-PC/Documents/Programmation/StageCSUG/$bin.bin

ST-LINK_CLI.exe -c SWD UR LPM Hrst Freq=4000 -P "C:/Users/Pierre-PC/Documents/Programmation/StageCSUG/$bin.bin" 0x08000000 -V