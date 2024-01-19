#!/bin/bash

NUM=20
NUM_DEPT=2
NUM_CUST=8  #1,4,9,16,25, -->flow memory over--> 36,49,64,81,100
#
C_SYM=3 #100(old_3) 50(o) 25 total for 3
D_SYM=1
#################################################

#init
in_dir="./path_input"
out_dir="./path_output/${NUM}scenario_${NUM_DEPT}dept_${NUM_CUST}cust"
if [[ ! -d "${in_dir}" ]]; then
    mkdir "${in_dir}"
    echo "Making ${in_dir}"
fi
if [[ ! -d "${out_dir}" ]]; then
    if [[ ! -d "./path_output" ]]; then
        mkdir ./path_output
        echo "Making ./path_output"
    fi
    mkdir "${out_dir}"
fi
#---
echo "total number of customers (count), ${NUM_CUST}" > conf_path.txt
{
    echo "total number of depots (count), ${NUM_DEPT}"
    echo "customer placement type (number symbol), ${C_SYM}"
    echo "depot placement type (number symbol), ${D_SYM}"
    echo "path type (number symbol), 1"
} >> conf_path.txt
mv conf_path.txt "${in_dir}"
echo "Done: conf_path.txt created.."

#create NUM scenario maps with NUM_DEPT and NUM_CUST
for ((i=0; i<NUM; i++)); do
    out_dir_scenario="${out_dir}/${i}/"
    if [[ ! -d "${out_dir_scenario}" ]]; then
        mkdir "${out_dir_scenario}"
    fi
    python main__path_generator.py
    (
        cd ./plots || exit
        python main_plot__path_generator.py
    )
    mv ./path_output/*.txt "${out_dir_scenario}"
    mv ./path_output/*.csv "${out_dir_scenario}"
    mv ./path_output/*.png "${out_dir_scenario}"
    echo "Done: scenario${i} in ${out_dir_scenario}"
    
done
