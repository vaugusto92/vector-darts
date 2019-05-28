#!/bin/bash

# root directory
CURRENT_DIR=$(pwd)

rm -rf $CURRENT_DIR/dist
rm -rf $CURRENT_DIR/build
rm -rf $CURRENT_DIR/__pycache__
rm -rf $CURRENT_DIR/modules/__pycache__
rm $CURRENT_DIR/main.spec
rm $CURRENT_DIR/main