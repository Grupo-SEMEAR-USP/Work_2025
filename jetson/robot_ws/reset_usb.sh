#!/bin/bash

# Lista de portas USB que você quer resetar
ports=(
  "1-2.4"
  "1-2.3.2"
  "1-2.3.4"
  "1-2.3.3"
)

unbind_usb() {
  # Use tee para redirecionar com sudo corretamente
  echo "$1" | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null
}

bind_usb() {
  echo "$1" | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null
}

# Desativar dispositivos
for port in "${ports[@]}"; do
  echo "Desativando $port..."
  unbind_usb "$port"
done

# Pausa opcional
sleep 1

# Reativar dispositivos
for port in "${ports[@]}"; do
  echo "Reativando $port..."
  bind_usb "$port"
done

echo "Processo concluído!"
