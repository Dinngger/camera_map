#!/bin/bash
python -m model.train\
    --name=car_match\
    --repeat=5000\
    --batch_size=128\
    --overwrite=True\
    --save_model=False\
    # --report_loss_steps=1\
    # --summary_steps=500\
