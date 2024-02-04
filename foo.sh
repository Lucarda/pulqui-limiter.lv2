#!/bin/sh

sudo apt install git geany meson ardour -y
cd ~
git clone https://github.com/lv2/lv2.git
cd ~/lv2
meson setup build
cd build
meson configure
meson compile
sudo meson install
