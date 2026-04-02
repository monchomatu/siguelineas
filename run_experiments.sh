#!/bin/bash

N_REPLAN=${1:-0}
N_NO_REPLAN=${2:-0}
USE_NOISE=${3:-False}

counter=1

if [ $N_REPLAN -gt 0 ]; then
    for ((i=1;i<=N_REPLAN;i++))
    do
        ros2 launch controllers system.launch.py \
            experiment_id:=$counter \
            use_replan:=True \
            use_noise:=$USE_NOISE
        ((counter++))
    done
fi

if [ $N_NO_REPLAN -gt 0 ]; then
    for ((i=1;i<=N_NO_REPLAN;i++))
    do
        ros2 launch controllers system.launch.py \
            experiment_id:=$counter \
            use_replan:=False \
            use_noise:=$USE_NOISE
        ((counter++))
    done
fi
