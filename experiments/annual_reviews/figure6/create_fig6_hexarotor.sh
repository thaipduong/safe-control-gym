#!/bin/bash

# GP-MPC (with 150 points kernel) for quadrotor environment with diagonal constraint.
python3 ./gp_mpc_experiment_hexarotor.py --task hexarotor --algo gp_mpc --overrides ./config_overrides/gp_mpc_quad_hexarotor.yaml
