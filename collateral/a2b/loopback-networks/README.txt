# A2B Loopback Networks

## Overview
This folder contains a collection of single-node A2B networks configured
for 16 channels of 32-bit audio.  The purpose is quick loopback testing of
A2B audio for different combinations of AD2428 and AD2433 networks.

## Sub-node preparation
Configure the first, and only, sub node for hardware loopback by jumpering
TX0 to RX0 and TX1 to RX1.

A2B Demo board from While(1) Engineering can be used for this purpose by
installing jumper wires between pins 11 <-> 13 and 12 <-> 14 on the
primary connector to loopback TX0 to RX0 and TX1 to RX1.

Demo boards can be found here:

https://while1.io/product/2428_mini
https://while1.io/product/2433_proto
