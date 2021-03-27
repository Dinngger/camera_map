#!/bin/bash
python -m model.train\
    --name=car_match\
    --repeat=4000\
    --batch_size=256\
    --overwrite=True\
    # --save_model=True\
    # --report_loss_steps=1\
    # --summary_steps=500\
