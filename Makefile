all: BUILD_LIB
	@cd MAIN               && $(MAKE)
	
BUILD_LIB:
	@cd Log              && $(MAKE)
	@cd G2ArgHandle	     && $(MAKE)
	@cd G2DeviceHandle   && $(MAKE)
	@cd G2Procedure      && $(MAKE)
	
	
CLEAN_LIB:
	@cd Log              && $(MAKE) clean
	@cd G2ArgHandle	     && $(MAKE) clean
	@cd G2DeviceHandle   && $(MAKE) clean
	@cd G2Procedure      && $(MAKE) clean

clean:CLEAN_LIB
	@cd MAIN             && $(MAKE) clean
	
CLEAN_LIB_ALL:
	@cd Log              && $(MAKE) clean_all
	@cd G2ArgHandle	     && $(MAKE) clean_all
	@cd G2DeviceHandle   && $(MAKE) clean_all
	@cd G2Procedure      && $(MAKE) clean_all
	
clean_all:CLEAN_LIB_ALL
	@cd MAIN             && $(MAKE) clean_all
	@rm -vfr bin lib
