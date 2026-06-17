# -*- mode: makefile; indent-tabs-mode: t -*-
# vi: set noet ci pi sts=0 sw=8 ts=8 :
#
# Copyright 2026 Steffen Persvold
# SPDX-License-Identifier: Apache-2.0

AT		:= @
NPROC		:= $(shell nproc)

VCS		:= $(VCS_HOME)/bin/vcs

VCS_FLAGS	:= -q -nc -debug_acc+pp -timescale=1ns/1ps +lint=all -sverilog -assert svaext +define+HAVE_VCDPLUSON +verilog2001ext+.v -top tb
# -xlrm uniq_prior_final: defer unique/priority case evaluation until
# signals have stabilised at the end of the time step. Filters out
# VCS's delta-cycle RT-MTOCMUCS false positives without silencing
# real overlap detection.
VCS_FLAGS	+= -xlrm uniq_prior_final
VCS_FLAGS	+= +lint=all,noUI,noNS

VERILATOR	:= $(VERILATOR_HOME)/bin/verilator
VERI_OPTS	:= --timing --trace-fst --top-module tb --timescale 1ns/1ps -j $(NPROC) -O3

SIMBIN		:= simv
VERIBIN		:= obj_dir/Vtb

DESIGN_ROOT	:= $(PROJECT_ROOT)
export DESIGN_ROOT

.PHONY:		all
all:		simulate

.PHONY:		simulate
simulate:	compile
		@echo "#     Running simulation (VCS)..."
		$(eval SIMEXE=$(shell realpath $(SIMBIN)))
		$(AT)$(SIMEXE) $(RUNARGS) -l simv.log -no_save

.PHONY:		verisim
verisim:	verilate
		@echo "#     Running simulation (Verilator)..."
		$(eval SIMEXE=$(shell realpath $(VERIBIN)))
		$(AT)$(SIMEXE) $(RUNARGS) 2>&1 | tee verisim.log

.PHONY:		compile
compile:	$(PRELUDE) Makefile sim_files.f
		@echo "#     Compiling simulation binary (VCS) ..."
		$(AT)$(VCS) $(VCS_FLAGS) $(BUILDARGS) -F sim_files.f -o $(SIMBIN) -l compile.log

.PHONY:		verilate
verilate:	$(PRELUDE) Makefile sim_files.f
		@echo "#     Compiling simulation binary (Verilator) ..."
		$(AT)$(VERILATOR) --binary $(VERI_OPTS) $(BUILDARGS) -F sim_files.f

.PHONY:		lint
lint:		$(PRELUDE) Makefile sim_files.f
		@echo "#     Linting code (HDL) ..."
		$(AT)$(VERILATOR) --lint-only $(VERI_OPTS) $(BUILDARGS) -F sim_files.f
