#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

helpFunction()
{
    echo ""
    echo "Usage: $0 -t type_motion_planning"
    echo -e "\t-t Type of motion plan to be plotted (unconstrained, constrained, stepped)"
    echo ""
    echo -e "${ORANGE}[WARNING] Be sure to run the script inside its directory!${NC}"
    echo ""
    exit 1 # Exit script after printing help
}
while getopts "t:" opt
do
    case "$opt" in
        t ) type_motion_plan="$OPTARG" ;;
	? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
    esac
done

# In case the type of motion plan is empty, take it as stepped
if [ -z "$type_motion_plan" ]
then
    echo -e "${ORANGE}[WARNING] As no type is indicated, by default the stepped motion plan is selected${NC}";
    type_motion_plan="stepped"
fi

# Begin script in case all parameters are correct
unconstrained="unconstrained"
constrained="constrained"
stepped="stepped"

### Saving script directory ###
path_to_utils_MatlabPlots=$PWD

### Move to the log directory ###
cd ../../test/unit/results/
if [ "$type_motion_plan" = "$unconstrained" ]
then
    cp ./unconstrained_planned_state.txt ./planned_state.txt
    cp ./unconstrained_planned_control.txt ./planned_control.txt
    echo Plotting the unconstrained motion plan results
elif [ "$type_motion_plan" = "$constrained" ]
then
    cp ./constrained_planned_state.txt ./planned_state.txt
    cp ./constrained_planned_control.txt ./planned_control.txt
    echo Plotting the constrained motion plan results
elif [ "$type_motion_plan" = "$stepped" ]
then
    cp ./stepped_planned_state.txt ./planned_state.txt
    cp ./stepped_planned_control.txt ./planned_control.txt
    echo Plotting the stepped motion plan results
fi

cd $path_to_utils_MatlabPlots
../../../../MATLAB/R2021b/bin/matlab -nosplash -nodesktop -r "run('./plotMM.m');"

