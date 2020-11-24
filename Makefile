SCENARIOS = \
  red-poster \
  ais-updated-floating_point_exception \
  ais-improved-infinite_loop \
  ais-updated-use_after_free \
  ais-updated-logic_bug \
  ais-updated-heap_overflow_data_overwrite \
  ais-updated-arc_injection \
  ais-updated-int_overflow

all: scenarios src/darjeeling_ardupilot/data/mavproxy

scenarios: $(SCENARIOS)

$(SCENARIOS): .ardupilot
	git -C .ardupilot checkout $@
	docker build -t trmo/ardupilot:$@ .ardupilot

clean:
	rm -f mav.tlog* mav.parm darjeeling.log* bugzood.log*

.ardupilot:
	test -d .ardupilot || git clone https://gitlab.eecs.umich.edu/wrg-code/ardupilot .ardupilot
	git -C .ardupilot pull

src/darjeeling_ardupilot/data/mavproxy:
	./build-mavproxy.sh

.PHONY: all scenarios $(SCENARIOS) .ardupilot clean
