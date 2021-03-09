#!/bin/bash
python -m model.train\
    --name=car_match\
    --repeat=1000\
    --batch_size=128\
    --save_model=True
    # --overwrite=True
    # --report_loss_steps=1\
    # --summary_steps=500\
