# set root to oot of software directory
export OD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Primary software build directoy for externals
export PATH=$OD_ROOT/build/bin:$PATH

export LIBRARY_PATH=$OD_ROOT/build/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=$OD_ROOT/build/lib:$LD_LIBRARY_PATH
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcmtypes_bot2-core.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcmtypes_bot2-frames.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcmtypes_bot2-procman.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcmtypes_bot2-param.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcmtypes_drake.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcmtypes_kinect.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/lcm.jar
export CLASSPATH=$CLASSPATH:$OD_ROOT/build/share/java/jchart2d-3.2.2.jar

# add lcm libs to pythonpath
export PYTHONPATH=$OD_ROOT/build/lib/python2.7/dist-packages:$PYTHONPATH
export PYTHONPATH=$OD_ROOT/build/lib/python2.7/site-packages:$PYTHONPATH

export GUROBI_DIR=$OD_ROOT/drake/externals/gurobi/
