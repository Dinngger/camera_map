#!/bin/bash
python -m model.train\
    --name=car_match\
    --repeat=4000\
    --batch_size=1\
    --save_model=True\
    # --overwrite=True\
    # --report_loss_steps=1\
    # --summary_steps=500\
