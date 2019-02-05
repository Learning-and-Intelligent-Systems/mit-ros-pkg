#!/bin/bash

python scripts/hierarchical_trials.py -w -4 -n 50 -t 45 -d 1 -r 100 -o hierarchical_results/spatula/world-4_darrtconnect_50.txt

python scripts/hierarchical_trials.py -w -4 -n 50 -t 30 -d 1 -r 100 -l -o hierarchical_results/spatula/world-4_darrthconnect_50.txt

python scripts/hierarchical_trials.py -w -4 -n 50 -t 45 -d 1 -r 100 -l -f -o hierarchical_results/spatula/world-4_darrth_50.txt