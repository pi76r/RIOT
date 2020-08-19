project=pkg_libcsp
directory=tests
board=nucleo-f446ze
if [ $# -eq 1 ] && [ $1 = "r" ]; then
    board=nucleo-f446re
fi

make -C $directory/$project BOARD=$board