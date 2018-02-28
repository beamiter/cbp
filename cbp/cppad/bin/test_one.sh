#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the
#                     Eclipse Public License Version 1.0.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
# Run one of the tests
if [ "$0" != 'bin/test_one.sh' ]
then
cat << EOF
usage: bin/test_one.sh dir/file [extra]

dir:   directory in front of file name
file:  name of *.cpp file, with extension, that contains the test
extra: extra source files and/or options for the g++ command
EOF
	exit 1
fi
dir=`echo $1 | sed -e 's|/[^/]*$||'`
file=`echo $1 | sed -e 's|.*/||'`
extra="$2"
if [ "$dir" == 'test_more/general/local' ]
then
	dir='test_more/general'
	file="local/$file"
fi
# ---------------------------------------------------------------------------
if [ "$dir" == '' ]
then
	echo "test_one.sh: cannot find dir before front of file"
	exit 1
fi
if [ ! -e "$dir/$file" ]
then
	echo "test_one.sh: Cannot find the file $dir/$file"
	exit 1
fi
if [ ! -e 'build/CMakeFiles' ]
then
	echo 'test_one.sh: Must first execute bin/run_cmake.sh'
	exit 1
fi
if [ -e test_one.exe ]
then
	rm test_one.exe
fi
if [ -e test_one.cpp ]
then
	rm test_one.cpp
fi
original_dir=`pwd`
# ---------------------------------------------------------------------------
# check for cppad_lib
count=`ls build/cppad_lib/libcppad_lib.* | wc -l`
if [ "$count" == '0' ]
then
	cd build
	make cppad_lib
	cd ..
fi
# ---------------------------------------------------------------------------
# include_flags
include_flags="-I $original_dir"
if [ -e NOTFOUND/include ]
then
	include_flags="$include_flags -I NOTFOUND/include"
fi
if [ -e NOTFOUND/include ]
then
	include_flags="$include_flags -I NOTFOUND/include"
fi
if [ -e NOTFOUND/include ]
then
	include_flags="$include_flags -I NOTFOUND/include"
fi
if [ -e NOTFOUND/include ]
then
	include_flags="$include_flags -isystem NOTFOUND/include"
fi
# --------------------------------------------------------------------------
# determine the function name
fun=`grep "^bool *[a-zA-Z0-9_]* *( *void *)" $dir/$file | tail -1 | \
	sed -e "s/^bool *\([a-zA-Z0-9_]*\) *( *void *)/\1/"`
#
# determine the main program main
main=`echo *$dir | sed -e 's|.*/||' -e 's|$|.cpp|'`
#
# create test_one.cpp
sed < $dir/$main > test_one.cpp \
-e '/^\tRun( /d' \
-e "s/.*This line is used by test_one.sh.*/\tRun( $fun, \"$fun\");/"
#
# compiler flags
cxx_flags=''
#
# library flags
library_flags=`pkg-config --libs ipopt`
library_flags="$library_flags /home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/cppad_lib/libcppad_lib.so"
library_flags="$library_flags -lboost_thread -lpthread"
for lib in lib lib64
do
	if [ -e NOTFOUND/$lib ]
	then
		library_flags="$library_flags -LNOTFOUND/$lib -lColPack"
		export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:NOTFOUND/$lib"
	fi
	if [ -e NOTFOUND/$lib ]
	then
		library_flags="$library_flags -LNOTFOUND/$lib -ladolc"
		export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:NOTFOUND/$lib"
	fi
done
#
# compile command
compile_command="g++ test_one.cpp -o test_one.exe
	$dir/$file $extra
	-g
	$cxx_flags
	$include_flags
	$library_flags
"
echo "$compile_command"
$compile_command
# --------------------------------------------------------------------------
library_path='/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/cppad_lib'
if ! echo $LD_LIBRARY_PATH | grep "$library_path"
then
	echo 'LD_LIBRARY_PATH=\'
	echo "$library_path:\$LD_LIBRARY_PATH"
	export LD_LIBRARY_PATH="$library_path:$LD_LIBRARY_PATH"
fi
# --------------------------------------------------------------------------
echo_eval ./test_one.exe
exit 0
