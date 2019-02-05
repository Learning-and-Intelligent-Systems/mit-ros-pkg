

#Get the path to this executable, which we'll assume is in `rospack find object_survey`/bin/
if test $(expr substr $0 1 1) = /
  then
   echo global path
   exename=$0
  else
   exename=$(echo  $(pwd)/$0 | sed "s%//%/%g" | sed "s%/./%/%g" | sed "s%/[^/]*/../%/%g")
fi
pkgpath=$(echo $exename | sed "s%/bin/[^/]*$%%")

#now make sure we have the required components:
test -d $pkgpath/testlogs || echo "could not find directory testlogs"
test -d $pkgpath/testlogs || exit


test -e $pkgpath/bin/testcluster || echo "could not find testcluster executable"
test -e $pkgpath/bin/testcluster || exit



#usage: testseries seriesnum clustertol
function testseries {
	series=$1
	if test $1 -lt 10
	  then
	  series=0$1
	fi
	#echo "ls $pkgpath/testlogs/testcloud*$(echo $series)_*.pcd"
	for name in $(ls $pkgpath/testlogs/testcloud*$(echo $series)_*.pcd )
	  do
	  #echo $pkgpath/bin/testcluster $name -s1 -s2 -tol $2
	  $pkgpath/bin/testcluster $name -s1 -s2 -tol $2
	done
}

#usage: execthread iter offset max clustertol
function execthread {
	threadnum=$2
	series=$2
	while test $series -lt $3
	  do
	  testseries $series $4
	  series=$(expr $series + $1)
	done
}

#usage: launchthreads numthreads seriesmax clustertol
function launchthreads {
	offset=0
	while test $offset -lt $1
	do
	  echo launching thread execthread $1 $offset $2 $3
	  execthread $1 $offset $2 $3 &
	  offset=$(expr $offset + 1)
	done
	#execute the first one in the forground:
	#echo running "        " execthread $1 0 $2
	#execthread $1 0 $2 $3
	#wait for threads to finish:
	echo sleep for a sec
	sleep 1
	echo now wait
	while test $(ps -ef | grep testcloud | grep testcluster | wc -l) -gt 0
	 do
	 sleep 5
	done
	echo done waiting
	
}

test $1 || echo "USAGE: $0 [debug | <cluster tolerances>]"
test $1 || exit

if test $1 = debug
then
  rosrun object_survey testcluster ../testlogs/testcloud17_2.0.pcd -debug -tol $2
  rosrun object_survey testcluster ../testlogs/testcloud03_0.1.pcd -debug -tol $2
  rosrun object_survey testcluster ../testlogs/testcloud17_1.0.pcd -debug -tol $2
else
  for tol in $@
  do
    launchthreads 7 21 $tol
  done
fi
