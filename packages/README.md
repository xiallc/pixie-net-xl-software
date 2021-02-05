Packaging
=========

This document describes building a Debian package of the Pixie-Net XL control
software.

Set Up
------

Install as root the required packages:

   apt-get install build-essential fakeroot devscripts debhelper

Building
--------

- Build the software at the top level. Follow the instructions provided.

- Enter the following command:

   ./build

Checking
--------

Check the package contents with:

  dpkg-deb -c pixie-net-xl_1.0.0_armhf.deb

Installing
----------

Install with:

  apt install ./pixie-net-xl_1.0.0_armhf.deb

If there are any package dependence errors run:

  apt install -f

To remove:

  apt remove ./pixie-net-xl_1.0.0_armhf.deb
