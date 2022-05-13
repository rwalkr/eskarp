all: stl/body.stl stl/lid.stl svg/preview.svg svg/case_top.svg

output/cases/body.stl output/cases/lid.stl: eskarp.yaml
	ergogen $< -o output -d

stl/%.stl: output/cases/%.stl
	mkdir -p stl
	cp $< $@

svg/%.svg: output/outlines/%.svg
	mkdir -p svg
	cp $< $@

ergogen_init:
	npm install -g ergogen