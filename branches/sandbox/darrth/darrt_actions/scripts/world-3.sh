#!/bin/bash

python scripts/hierarchical_trials.py -w -3 -n 50 -t 30 -d 1 -r 100 -o hierarchical_results/spatula/world-3_darrtconnect_50.txt

python scripts/hierarchical_trials.py -w -3 -n 50 -t 15 -d 1 -r 100 -l -o hierarchical_results/spatula/world-3_darrthconnect_50.txt

python scripts/hierarchical_trials.py -w -3 -n 50 -t 15 -d 1 -r 100 -l -f -o hierarchical_results/spatula/world-3_darrth_50.txt

python scripts/hierarchical_trials.py -w -3 -n 50 -t 45 -d 1 -r 100 -f -o hierarchical_results/spatula/world-3_darrt_50.txt