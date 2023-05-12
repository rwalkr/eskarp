all: stls svgs
.PHONY: all

STLS=stl/body.stl stl/body_led.stl stl/body_tp.stl stl/lid.stl
stls: $(STLS)
.PHONY: stls

SVGS=svg/preview.svg svg/case_top.svg svg/plate.svg
svgs: $(SVGS)
.PHONY: svgs

OUTS=$(patsubst stl/%,output/cases/%,$(STLS)) $(patsubst svg/%,output/outlines/%,$(SVGS))

$(info $(OUTS))

$(OUTS): eskarp.yaml
	ergogen $< -o output -d

stl/%.stl: output/cases/%.stl
	mkdir -p stl
	cp $< $@

svg/%.svg: output/outlines/%.svg
	mkdir -p svg
	cp $< $@

ergogen_init:
	npm install -g ergogen