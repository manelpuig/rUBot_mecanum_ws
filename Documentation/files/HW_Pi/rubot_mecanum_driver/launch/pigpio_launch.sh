#!/bin/bash

# Verifica si pigpiod ya está en ejecución
if pgrep pigpiod > /dev/null; then
    echo "pigpiod ya está en ejecución."
else
    echo "Iniciando pigpiod..."
    sudo pigpiod
    echo "pigpiod iniciado."
fi
