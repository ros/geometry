all: eigen

TARBALL     = build/eigen2-2.0.15.tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/eigen2-2.0.15.tar.bz2
UNPACK_CMD  = tar xfj
SOURCE_DIR  = build/eigen
TARBALL_PATCH = matrix.patch
MD5SUM_FILE = eigen2-2.0.15.tar.bz2.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

HG_URL = https://tfoote@bitbucket.org/eigen/eigen
HG_DIR = build/eigen
HG_PATCH = matrix.patch
HG_REVISION = default

# This file can be used to automate cloning and patching of 3rd 
# party software.
#
# Before including this file, e.g.:
#	include $(shell rospack find mk)/hg_checkout.mk
# define the following make variables:
#	HG_DIR: name of directory into which you want to clone to 
# 	HG_URL: full URL to download
#	HG_PATCH: your (list of) patch file(s) to patch the downloaded software
#	HG_REVISION: -

$(HG_DIR):
	mkdir -p build
	hg clone $(HG_URL) $(HG_DIR)
	cd $(HG_DIR) && hg up $(HG_REVISION)
	touch rospack_nosubdirs

patched: $(HG_PATCH) Makefile
	cd $(HG_DIR) && hg up -C
ifneq ($(strip $(HG_PATCH)),)
	$(foreach PATCH, $(HG_PATCH), patch -d $(HG_DIR) -p1 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched
endif   


download: $(HG_DIR) patched




INSTALL_DIR = include


eigen: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(HG_DIR)
	mkdir -p $(INSTALL_DIR)
	cp -r $(HG_DIR)/Eigen $(INSTALL_DIR)
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(HG_DIR) $(INSTALL_DIR)

wipe: clean
	rm -rf build


#INSTALL_DIR = include


#eigen: $(INSTALL_DIR)/installed

#$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
#	mkdir -p $(INSTALL_DIR)
#	cp -r $(SOURCE_DIR)/Eigen $(INSTALL_DIR)
#	touch $(INSTALL_DIR)/installed

#clean:
#	rm -rf $(SOURCE_DIR) $(INSTALL_DIR)

#wipe: clean
#	rm -rf build
