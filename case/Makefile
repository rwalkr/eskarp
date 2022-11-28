all: stl/body.stl stl/body_led.stl stl/body_tp.stl stl/lid.stl svg/preview.svg svg/case_top.svg svg/plate.svg

output/cases/body.stl output/cases/body_led.stl output/cases/body_tp.stl output/cases/lid.stl: eskarp.yaml
	ergogen $< -o output -d

stl/%.stl: output/cases/%.stl
	mkdir -p stl
	cp $< $@

svg/%.svg: output/outlines/%.svg
	mkdir -p svg
	cp $< $@

ergogen_init:
	npm install -g ergogen