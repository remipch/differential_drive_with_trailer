#!/bin/sh

# This script converts all png images in this folder into an animated gif

# It needs imagemagick V6 to be installed :
# sudo apt install imagemagick

# WARNING : imagemagick bugs if there is too many images
# (some frames are missing at the end of the loop)
# It seems to be ok for 180 images on my config
convert -delay 5 -loop 0 -resize 50% output/*.png anim.gif
