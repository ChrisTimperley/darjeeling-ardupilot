SCENARIOS = red-poster

clean:
	rm -f mav.tlog* mav.parm darjeeling.log* bugzood.log*

.ardupilot:
	test -d .ardupilot || git clone https://gitlab.eecs.umich.edu/wrg-code/ardupilot .ardupilot
	git -C .ardupilot pull

$(SCENARIOS): .ardupilot
	git -C .ardupilot checkout $@
	docker build -t trmo/ardupilot:$@ .ardupilot

src/darjeeling_ardupilot/data/mavproxy:
	./build-mavproxy.sh

.PHONY: $(SCENARIOS) .ardupilot
