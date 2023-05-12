all: stls svgs
.PHONY: all

STLS=stl/body.stl stl/body_led.stl stl/body_tp.stl stl/lid.stl
stls: $(STLS)
.PHONY: stls

SVGS=svg/preview.svg svg/case_top.svg svg/plate.svg
svgs: $(SVGS)
.PHONY: svgs

ERGOGEN_OUTS=$(patsubst stl/%.stl,output/cases/%.jscad,$(STLS)) $(patsubst svg/%,output/outlines/%,$(SVGS))

ERGOGEN=$(PWD)/tools/node_modules/.bin/ergogen
JSCAD=$(PWD)/tools/node_modules/.bin/openjscad

$(info $(OUTS))

$(ERGOGEN_OUTS): eskarp.yaml $(ERGOGEN)
	$(ERGOGEN) $< -o output -d

output/cases/%.stl: output/cases/%.jscad $(JSCAD)
	$(JSCAD) -o $@ $<

stl/%.stl: output/cases/%.stl
	mkdir -p stl
	cp $< $@

svg/%.svg: output/outlines/%.svg
	mkdir -p svg
	cp $< $@

$(ERGOGEN):
	mkdir -p tools
	cd tools && npm install ergogen

$(JSCAD):
	mkdir -p tools
	cd tools && npm install git+https://github.com/ergogen/oldjscad