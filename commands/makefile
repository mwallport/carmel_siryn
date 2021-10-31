SRCDIR:=./src
HDRDIR:=./h
BINDIR:=./bin
OBJDIR:=./obj
LIBDIR:=./lib
LIBSDIR:=../libs
DEPDIR:=./.d/
DIRS:=${BINDIR} ${OBJDIR} ${DEPDIR} ${LIBDIR}
OBJFILES:=$(addprefix ${OBJDIR}/, $(addsuffix .o, $(basename $(notdir $(wildcard ${SRCDIR}/*.cpp)))))
DEPFILES:=$(addprefix ${DEPDIR}/, $(addsuffix .d, $(basename $(notdir $(wildcard ${SRCDIR}/*.cpp)))))
SRCS:=$(wildcard ${SRCDIR}/*.cpp)
HDRS:=$(wildcard ${HDRDIR}/*.h)
LIBNAME:=libcommands.so
CXX:=g++
CFLAGS+=-Wall -ggdb -I${HDRDIR} -I../h -I../CRC16/h
LDFLAGS=

${OBJDIR}/%.o : ${SRCDIR}/%.cpp
	${CXX} -c $< -fpic -o $@ ${CFLAGS} -MT $@ -MMD -MP -MF ${DEPDIR}/$*.D
	@mv ${DEPDIR}/$*.D ${DEPDIR}/$*.d

all : ${OBJFILES}
	${CXX} -shared -o ${LIBDIR}/${LIBNAME} ${LDFLAGS} ${OBJFILES}
	@cp ${LIBDIR}/${LIBNAME} ${LIBSDIR}/${LIBNAME}

${OBJFILES} : | SUBDIRS

.PHONY: SUBDIRS clean dep_clean ${BINDIR} ${OBJDIR} ${DEPDIR} ${LIBDIR}

SUBDIRS : ${DIRS}
	@mkdir -p $?

clean:
	@rm -rf ${OBJDIR}/*
	@rm -rf ${BINDIR}/*
	@rm -f ${LIBDIR}/*
	@rm -f ./core*
	@rm -f ./a.out
	@rm -rf ${EXEC}
	@rm -f ${LIBSDIR}/${LIBNAME}

dep_clean : clean
	@rm -rf ${DEPDIR}/*

#include ${DEPDIR}/*.d

