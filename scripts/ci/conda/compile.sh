#!/bin/bash

pwd
ls

conda build recipe
conda install recipe
