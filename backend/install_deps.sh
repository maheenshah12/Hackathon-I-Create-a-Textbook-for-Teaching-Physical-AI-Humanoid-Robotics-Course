#!/bin/bash

# Script to install project dependencies
echo "Installing backend dependencies..."

# Check if virtual environment exists, create if not
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies from requirements.txt
pip install -r requirements.txt

echo "Dependencies installed successfully!"
echo "To activate the virtual environment, run: source venv/bin/activate"