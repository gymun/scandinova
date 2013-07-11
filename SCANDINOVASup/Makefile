TOP=..
include $(TOP)/configure/CONFIG
#=======================================

# Build devSCANDINOVA as a library for an IOC:
LIBRARY_IOC += devSCANDINOVA

# Library Source files
devSCANDINOVA_SRCS += devSCANDINOVA.c

# Link with the asyn and base libraries
devSCANDINOVA_LIBS += asyn
devSCANDINOVA_LIBS += $(EPICS_BASE_IOC_LIBS)

# Install .dbd and .db files
DBD += devSCANDINOVA.dbd
DB_INSTALLS += devSCANDINOVA.db
DB_INSTALLS += autodrive.db
#=======================================
include $(TOP)/configure/RULES
