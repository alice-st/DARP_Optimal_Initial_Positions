#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Give only one argument, the name of the virtual environment to create"
  exit 1
fi

echo "Creating virtual environment: $1"
python -m venv $1
export PATH="$(pwd)/darp-venv/bin:$PATH"

echo "Activating virtual environment: $1"
. "$(pwd)/$1/bin/activate" || . "$(pwd)/$1/bin/activate.csh" || source "$(pwd)/$1/bin/activate" || source "$(pwd)/$1/bin/activate.csh"
echo "Activation completed"

echo $PS1
echo $PATH

pip install \
  numpy==1.26.2\
  matplotlib==3.8.2\
  optuna==2.0.0\
  opencv-python\
  pygame\
  scikit-learn\
  numba