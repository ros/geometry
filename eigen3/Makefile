all: eigen

TARBALL     = build/eigen3.0-beta2.tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/eigen3.0-beta2.tar.bz2
UNPACK_CMD  = tar xfj
SOURCE_DIR  = build/eigen
#TARBALL_PATCH = eigen3_namespace.patch 
MD5SUM_FILE = eigen3.0-beta2.tar.bz2.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = include

eigen: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(INSTALL_DIR)
	ln -s Eigen $(SOURCE_DIR)/Eigen3
	cp -r $(SOURCE_DIR)/Eigen $(INSTALL_DIR)
	mv $(INSTALL_DIR)/Eigen $(INSTALL_DIR)/Eigen3
	scripts/rename_eigen_namespace $(INSTALL_DIR)/Eigen3
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) $(INSTALL_DIR)

wipe: clean
	rm -rf build
