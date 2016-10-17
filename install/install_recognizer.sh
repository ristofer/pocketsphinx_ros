#!/bin/sh


# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
pkg_name="pocketsphinx_ros";
install_path=$(rospack find "$pkg_name")/install;
install_files="$install_path"/files;


# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

roscd ;


## Download and install dependences

# speech recognizer (pocketsphinx)

sudo apt-get install bison;

cd "$install_files";
git clone https://github.com/cmusphinx/sphinxbase.git;
git clone https://github.com/cmusphinx/pocketsphinx.git;


# other tools
sudo apt-get install unzip;
sudo apt-get install autoconf automake


# Installing the speech recognition system
# ==========================================

cd sphinxbase/;
./autoge.sh;
./configure;
make;
sudo make install;

cd ../pocketsphinx
./configure;
make;
sudo make install;





echo ""
echo "Done. ;)"
echo ""
# :)
