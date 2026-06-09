all: BUILD_OBJS
	@cd MAIN             && $(MAKE)

BUILD_OBJS:
	@cd Log              && $(MAKE)
	@cd G2ArgHandle	     && $(MAKE)
	@cd G2DeviceHandle   && $(MAKE)
	@cd G2Procedure      && $(MAKE)

CLEAN_OBJS:
	@cd Log              && $(MAKE) clean
	@cd G2ArgHandle	     && $(MAKE) clean
	@cd G2DeviceHandle   && $(MAKE) clean
	@cd G2Procedure      && $(MAKE) clean

clean:CLEAN_OBJS
	@cd MAIN             && $(MAKE) clean

CLEAN_OBJS_ALL:
	@cd Log              && $(MAKE) clean_all
	@cd G2ArgHandle	     && $(MAKE) clean_all
	@cd G2DeviceHandle   && $(MAKE) clean_all
	@cd G2Procedure      && $(MAKE) clean_all

clean_all:CLEAN_OBJS_ALL
	@cd MAIN             && $(MAKE) clean_all
	@rm -vfr bin out