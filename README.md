# README #

This was inspired by the work of Niclas Forslund http://foogadgets.blogspot.com/2017/09/upgrading-bosch-pll-360-self-levelling.html
He did great to reverse engineer a Bosch PLL360 laser level and rewrite its firmware to add outdoor mode with pulsating laser.
From his pictures I realized I owned a laser level with the exact same hardware! It is the CST/Berger model CL10. So I tested Niclas FW on my level and it worked!, but I wanted more...

I wanted to field-adjust laser pulse frequency to use the laser detector of my choice. So this FW was born, originating on Niclas work and heavily rewriting the code.

I tested this FW on the CL10 using a Bosch LR8 as laser detector, the setup worked up to 40m distance on high power setting (I added a power selector switch to my laser level).

Find further details in Notes.txt and in main.c source file.

### What is this repository for? ###

This alternate Firmware adds Outdoor mode with variable pulse frequency to the Bosch PLL360 or CST/Berger CL10 laser line levellers.

This version was developed using MPLABX 5.45 and XC8 2.32.





