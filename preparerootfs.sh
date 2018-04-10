#!/bin/sh
echo PATH=$1
make modules_install firmware_install INSTALL_MOD_PATH=$1
make headers_install INSTALL_HDR_PATH=$1/usr
