VREP_PATH=~/bin/V-REP_PRO_EDU_V3_2_2_64_Linux
SCRIPT_PATH=`pwd`
cd $VREP_PATH
sh vrep.sh -s $SCRIPT_PATH/matlab_source/matlab_main/human_baxter.ttt&
cd $SCRIPT_PATH/matlab_source/matlab_main/
sleep 1
matlab -r "run main.m"