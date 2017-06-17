#!/bin/bash
#
# run me like this:
# > sudo apt-get update
# > cdb bender_tts
# > bash install/install.sh
#
# IF SOMETHING FAILS, DELETE THE FOLLOWING DIR, AND REPEAT: install_space
# 


## ----------------------------------------------------------------------------
## Useful Variables
## ----------------------------------------------------------------------------

# package name
#pkg_name="uchile_speech_pocketsphinx"

# package path
#pkg_path=$(rospack find "$pkg_name")


## ----------------------------------------------------------------------------
## User Configuration 
## ----------------------------------------------------------------------------

# where download and mantain the files
install_space="$UCHILE_WS"/deps/bender/hri/speech

# mark as installed 
install_token="$install_space"/INSTALLED


## ----------------------------------------------------------------------------
## Dependencies
## ----------------------------------------------------------------------------

sudo apt-get install gcc automake autoconf libtool bison swig unzip


## ----------------------------------------------------------------------------
## Install
## ----------------------------------------------------------------------------
mkdir -p "$install_space" && cd "$install_space"


## DOWNLOAD sphinxbase or update it if neccesary
echo ""
cd "$install_space"
if [ ! -d "sphinxbase" ]; then
	echo "sphinxbase repository not found.... cloning..."
	git clone https://github.com/uchile-robotics-die/sphinxbase.git
else
	echo "sphinxbase repository already exists on $install_space"
	echo "updating ..."
	
	cd sphinxbase
	git remote update
	LOCAL=$(git rev-parse @)
	REMOTE=$(git rev-parse "@{u}")
	BASE=$(git merge-base @ "@{u}")

	if [ "$LOCAL" = "$REMOTE" ]; then
	    echo "sphinxbase is up-to-date"

	elif [ "$LOCAL" = "$BASE" ]; then
		# update and mark as not ready
		echo "sphinxbase needs to be updated ... "
	    git pull origin master
	    rm -f "$install_space/$install_token"

	elif [ "$REMOTE" = "$BASE" ]; then
	    echo "Need to push"
	    # do nothing
	else
	    echo "Diverged"
	    # do nothing
	fi
fi

## DOWNLOAD pocketsphinx or update it if neccesary
echo ""
cd "$install_space"
if [ ! -d "pocketsphinx" ]; then
	echo "pocketsphinx repository not found.... cloning..."
	git clone https://github.com/uchile-robotics-die/pocketsphinx.git
else
	echo "pocketsphinx repository already exists on $install_space"
	echo "updating ..."

	cd "$install_space"
	cd pocketsphinx
	git remote update
	LOCAL=$(git rev-parse @)
	REMOTE=$(git rev-parse "@{u}")
	BASE=$(git merge-base @ "@{u}")

	if [ "$LOCAL" = "$REMOTE" ]; then
	    echo "pocketsphinx is up-to-date"

	elif [ "$LOCAL" = "$BASE" ]; then
		# update and mark as not ready
		echo "pocketsphinx needs to be updated ... "
	    git pull origin master
	    rm -f "$install_space/$install_token"

	elif [ "$REMOTE" = "$BASE" ]; then
	    echo "Need to push"
	    # do nothing
	else
	    echo "Diverged"
	    # do nothing
	fi
fi

## exit if already installed
cd "$install_space"
if [ -e "$install_token" ]; then
	echo ""
	echo " - ---------------------------------------------------------------------"
	echo " - sphinxbase and pocketsphinx are already installed and up to date. Bye"
	echo " - ---------------------------------------------------------------------"
	exit 0
fi

## Installing sphinxbase
## -------------------------------------
cd "$install_space"/sphinxbase
./autogen.sh
./configure
make clean all
make check
sudo make install

# used meanwhile... see also: shell/setup.sh
export LD_LIBRARY_PATH=/usr/local/lib
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig

## Installing pocketsphinx
## -------------------------------------
cd "$install_space"/pocketsphinx
./autogen.sh
./configure
make clean all
make check
sudo make install

## mark as installed
cd "$install_space"
touch "$install_token"

echo ""
echo "done"
echo ""
