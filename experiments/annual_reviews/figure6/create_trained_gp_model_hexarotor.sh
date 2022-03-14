#!/bin/bash

# Backup the pretrained GP model.
cp trained_gp_model/best_model_0.pth trained_gp_model/bak_best_model_0.pth
cp trained_gp_model/best_model_1.pth trained_gp_model/bak_best_model_1.pth
cp trained_gp_model/best_model_2.pth trained_gp_model/bak_best_model_2.pth
cp trained_gp_model/best_model_3.pth trained_gp_model/bak_best_model_3.pth
cp trained_gp_model/best_model_4.pth trained_gp_model/bak_best_model_4.pth
cp trained_gp_model/best_model_5.pth trained_gp_model/bak_best_model_5.pth

# Re-create the GP models in 'trained_gp_model/' using 800 samples for hyperparameter optimization.
python3 ./gp_mpc_experiment_hexarotor.py --train_only True --task hexarotor --algo gp_mpc --overrides ./config_overrides/gp_mpc_quad_training_hexarotor.yaml
