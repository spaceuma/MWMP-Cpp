#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

helpFunction()
{
    echo ""
    echo "Usage: $0 -t type_motion_planning -s sample"
    echo -e "\t-t Type of motion plan to be plotted (unconstrained, constrained, stepped)."
    echo -e "\t-s Iteration sample to be plotted. The final result will be plotted if not indicated."
    echo ""
    echo -e "${ORANGE}[WARNING] Be sure to run the script inside its directory!${NC}"
    echo ""
    exit 1 # Exit script after printing help
}
while getopts "t:s:" opt
do
    case "$opt" in
        t ) type_motion_plan="$OPTARG" ;;
        s ) sample="$OPTARG" ;;
	? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
    esac
done

# In case the type of motion plan is empty, take it as stepped
if [ -z "$type_motion_plan" ]
then
    echo -e "${ORANGE}[WARNING] As no type is indicated, by default the stepped motion plan is selected${NC}";
    type_motion_plan="stepped"
fi

# In case the sample is empty, take it as the final one
if [ -z "$sample" ]
then
    echo -e "${ORANGE}[WARNING] As no sample is indicated, by default the final result will be plotted${NC}";
    sample=""
else
    sample=${sample}_
fi

# Begin script in case all parameters are correct
echo Plotting the ${type_motion_plan} motion plan results

### Saving script directory ###
path_to_utils_MatlabPlots=$PWD


### Move to the log directory ###
cd ../../test/unit/results/

### Generate paths for the log files ###
full_path_state="${sample}${type_motion_plan}_planned_state.txt"
full_path_control="${sample}${type_motion_plan}_planned_control.txt"

echo Extracting logs from ${full_path_state} and ${full_path_control}

cp $full_path_state ./planned_state.txt
cp $full_path_control ./planned_control.txt

cd $path_to_utils_MatlabPlots
../../../../MATLAB/R2021b/bin/matlab -nosplash -nodesktop -r "run('./plotMM.m');"

