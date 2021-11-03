SRCDIR:=./src
HDRDIR:=./h
BINDIR:=./bin
OBJDIR:=./obj
LIBDIR:=./lib
DEPDIR:=./.d
DIRS:=${BINDIR} ${OBJDIR} ${DEPDIR} ${LIBDIR}
OBJFILES:=$(addprefix ${OBJDIR}/, $(addsuffix .o, $(basename $(notdir $(wildcard ${SRCDIR}/*.cpp)))))
DEPFILES:=$(addprefix ${DEPDIR}/, $(addsuffix .d, $(basename $(notdir $(wildcard ${SRCDIR}/*.cpp)))))
SRCS:=$(wildcard ${SRCDIR}/*.cpp)
HDRS:=$(wildcard ${HDRDIR}/*.h)
CXX:=g++
CFLAGS+=-Wall -ggdb -I${HDRDIR}
LDFLAGS=
EXEC:=test

${OBJDIR}/%.o : ${SRCDIR}/%.cpp
	${CXX} -c $< -o $@ ${CFLAGS} -MT $@ -MMD -MP -MF ${DEPDIR}/$*.D
	@mv ${DEPDIR}/$*.D ${DEPDIR}/$*.d

all : ${OBJFILES}
	${CXX} -o ${BINDIR}/${EXEC} ${LDFLAGS} ${OBJFILES}

${OBJFILES} : | SUBDIRS

.PHONY: SUBDIRS clean dep_clean ${BINDIR} ${OBJDIR} ${DEPDIR} ${LIBDIR}

SUBDIRS : ${DIRS}
	@mkdir -p $?

clean:
	@rm -rf ${OBJDIR}/*
	@rm -rf ${BINDIR}/*
	@rm -f ${LIBDIR}/*
	@rm -rf ./core*
	@rm -rf ./a.out

dep_clean : clean
	@rm -rf ${DEPDIR}/*

#include ${DEPDIR}/*.d

