#!/bin/bash
lspci | grep xHCI | sed -e 's/ .*$//' | sed -e 's/^/0000:/' | tee /sys/bus/pci/drivers/xhci_hcd/unbind
lspci | grep xHCI | sed -e 's/ .*$//' | sed -e 's/^/0000:/' | tee /sys/bus/pci/drivers/xhci_hcd/bind
